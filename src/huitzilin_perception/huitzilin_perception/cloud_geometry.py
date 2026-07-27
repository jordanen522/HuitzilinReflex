"""
cloud_geometry.py — pure-numpy/scipy point-cloud math for the W3 detector.

No ROS imports: everything here is unit-testable on any machine
(test/test_cloud_geometry.py). detector_node.py is the only other consumer.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


# ── Quaternions / rigid transforms ────────────────────────────────────────────

def is_valid_quat(x: float, y: float, z: float, w: float,
                  tol: float = 1e-3) -> bool:
    """True if (x,y,z,w) is a unit quaternion. ROS message defaults are all
    zeros (invalid) — bags recorded before the odom-orientation fix carry
    exactly that, and must not be interpreted as a pose."""
    return abs((x * x + y * y + z * z + w * w) - 1.0) <= tol


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def make_transform(t, q) -> np.ndarray:
    """Translation (3,) + quaternion (x,y,z,w) -> 4x4 homogeneous transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(*q)
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def apply_transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array. Returns float32 (N, 3)."""
    if pts.shape[0] == 0:
        return pts
    return (pts @ T[:3, :3].T + T[:3, 3]).astype(np.float32)


# ── Voxel grid (pure-numpy, no PCL / open3d dep) ─────────────────────────────

def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    """
    Down-sample an (N, 3) float32 xyz array to one point per voxel.
    Returns an (M, 3) array with M ≤ N.
    """
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    # unique voxels → take centroid of points in each
    unique_keys, inv = np.unique(keys, axis=0, return_inverse=True)
    centroids = np.zeros((len(unique_keys), 3), dtype=np.float32)
    counts = np.bincount(inv, minlength=len(unique_keys)).reshape(-1, 1)
    np.add.at(centroids, inv, pts)
    centroids /= counts
    return centroids


# ── Frame differencing ────────────────────────────────────────────────────────

def foreground_mask(current: np.ndarray, background: np.ndarray,
                    threshold: float) -> np.ndarray:
    """
    Bool mask of current points farther than `threshold` from ANY background
    point. cKDTree NN query — the old (N, M, 3) broadcast was ~40 GB at
    30k x 120k points and OOM-killed the node (see W3-13 commit e4e1086).
    """
    if background.shape[0] == 0:
        return np.ones(len(current), dtype=bool)
    tree = cKDTree(background)
    min_dists, _ = tree.query(current, k=1, workers=-1)
    return min_dists > threshold


# ── Euclidean clustering (single-linkage radius, cKDTree) ────────────────────

def cluster_all(pts: np.ndarray, tol: float, min_pts: int) -> list[np.ndarray]:
    """
    Greedy radius-based Euclidean clustering on (N, 3) float32 xyz, with NO
    upper size bound. Returns a list of (k, 3) arrays, each being one cluster.

    Split out from euclidean_cluster() because an upper point cap cannot be
    applied at this layer without destroying information: a projectile touching
    a large surface lands in one oversized cluster, and discarding it here loses
    the ball. cluster_and_split() re-clusters such a blob instead.
    """
    if pts.shape[0] == 0:
        return []

    tree = cKDTree(pts)
    assigned = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []

    for seed_idx in range(len(pts)):
        if assigned[seed_idx]:
            continue
        # BFS from this seed
        assigned[seed_idx] = True
        queue = [seed_idx]
        members = []
        while queue:
            idx = queue.pop()
            members.append(idx)
            for nb in tree.query_ball_point(pts[idx], tol):
                if not assigned[nb]:
                    assigned[nb] = True
                    queue.append(nb)
        if len(members) >= min_pts:
            clusters.append(pts[np.array(members)])

    return clusters


def euclidean_cluster(pts: np.ndarray, tol: float,
                      min_pts: int, max_pts: int) -> list[np.ndarray]:
    """
    Greedy radius-based Euclidean clustering on (N, 3) float32 xyz.
    Returns a list of (k, 3) arrays, each being one cluster.

    Clusters outside [min_pts, max_pts] are dropped. Prefer
    cluster_and_split() in the live pipeline — see its docstring for why the
    max_pts drop is the wrong behaviour when the scene is cluttered.
    """
    return [c for c in cluster_all(pts, tol, min_pts) if c.shape[0] <= max_pts]


def cluster_extent(cluster: np.ndarray) -> float:
    """Largest axis-aligned side length of a cluster's bounding box, in metres."""
    if cluster.shape[0] == 0:
        return 0.0
    return float(np.max(cluster.max(axis=0) - cluster.min(axis=0)))


def cluster_and_split(pts: np.ndarray, tol: float, min_pts: int,
                      max_extent: float, split_tol: float,
                      max_split_points: int) -> list[np.ndarray]:
    """
    Cluster at `tol`; any cluster too physically large to be the projectile is
    re-clustered ONCE at the tighter `split_tol` instead of being discarded.
    Returns only clusters with extent <= max_extent.

    Why this exists (W4, 2026-07-26): the old path clustered once at
    cluster_tolerance_m and then *discarded* anything bigger than the ball. Under
    patrol that threw the ball away with the clutter — funnel evidence from a
    throw that passed 0.484 m from the drone:

        fg=303 clusters=2 ball_sized=1 (size,extent)=[(291, 1.42), (12, 0.18)]

    The 12-point / 0.18 m cluster *is* the ball; on the frames that produced
    `fg=2946 clusters=0` it had merged into a blob over cluster_max_points and
    vanished. A compact object near a large surface must stay separable.

    Why ONE pass and not recursive shrinking, which was the obvious first
    implementation: recursion manufactures false positives. Measured in
    test_background_map.py — shrinking 0.20 -> 0.10 -> 0.05 against a surface
    whose own point spacing is 0.05 m shatters it into 29 fragments of 0.10 m
    extent, every one of which passes the max_extent ball gate. A single
    intermediate tolerance separates a detached object from a surface without
    ever reaching the surface's internal spacing. Pick split_tol above the
    voxel leaf and below the object's standoff from the scene.

    max_split_points bounds the cost — a blob bigger than that is an egomotion
    flood, handled upstream by fg_max_points, not a scene object worth splitting.
    """
    out: list[np.ndarray] = []
    for c in cluster_all(pts, tol, min_pts):
        if cluster_extent(c) <= max_extent:
            out.append(c)
            continue
        if split_tol >= tol or c.shape[0] > max_split_points:
            continue
        out.extend(sub for sub in cluster_all(c, split_tol, min_pts)
                   if cluster_extent(sub) <= max_extent)
    return out
