"""
cloud_geometry.py — pure-numpy/scipy point-cloud math for the W3 detector.

No ROS imports: everything here is unit-testable on any machine
(test/test_cloud_geometry.py). detector_node.py is the only other consumer.
"""

from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
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


def rotate_covariance(R: np.ndarray, Rot: np.ndarray) -> np.ndarray:
    """Re-express a 3x3 covariance under rotation `Rot` (child frame -> parent
    frame). A congruence transform, not a similarity one -- covariance is a
    bilinear form, so applying `Rot` the way a vector would be rotated (a bare
    `Rot @ R`) would not even stay symmetric. Callers should pass the SAME
    `Rot` used to move the point the covariance belongs to, e.g. the rotation
    block of the transform apply_transform() just applied to it."""
    return Rot @ R @ Rot.T


# ── Voxel grid (pure-numpy, no PCL / open3d dep) ─────────────────────────────

# Voxel indices are packed into one int64 so np.unique can take its fast 1-D
# sort path. Measured 2026-07-27 on the Dell, this function was THE detector
# latency overrun: 66-74 ms per call on patrol clouds against a 77 ms/frame wall
# budget (15 Hz at RTF 0.864), 48-62% of all measured stage time. Two numpy
# anti-patterns, both in one function:
#   * np.unique(keys, axis=0) builds a void-dtype view of each row and does a
#     lexicographic row sort — far slower than sorting plain integers.
#   * np.add.at is the unbuffered scatter-add; np.bincount(weights=...) does the
#     same reduction in optimised C.
# 21 bits per axis => +-2^20 voxels, i.e. +-21 km at a 0.02 m leaf. Points are
# voxelised in the CAMERA frame where roi_max_range_m (5 m) bounds them, so this
# window is never approached in production; out-of-window clouds fall back to
# the row-wise path rather than silently wrapping two voxels into one.
_VOXEL_BITS = 21
_VOXEL_HALF = 1 << (_VOXEL_BITS - 1)


def _voxel_centroids(pts: np.ndarray, inv: np.ndarray,
                     n_voxels: int) -> np.ndarray:
    """Mean of the points in each voxel, from a group-index array.

    Accumulates in float64 (bincount's weight dtype) and narrows at the end,
    which is strictly more accurate than the old float32 in-place scatter-add.
    """
    counts = np.bincount(inv, minlength=n_voxels)
    sums = np.stack(
        [np.bincount(inv, weights=pts[:, i], minlength=n_voxels)
         for i in range(3)],
        axis=1)
    return (sums / counts[:, None]).astype(np.float32)


def _voxel_downsample_rowwise(pts: np.ndarray,
                              keys: np.ndarray) -> np.ndarray:
    """Fallback for coordinates outside the packable key window."""
    unique_keys, inv = np.unique(keys, axis=0, return_inverse=True)
    return _voxel_centroids(pts, inv.ravel(), len(unique_keys))


def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    """
    Down-sample an (N, 3) float32 xyz array to one point per voxel.
    Returns an (M, 3) array with M ≤ N.

    Rows come back in lexicographic voxel order (x, then y, then z), which the
    packed key preserves because x occupies the most significant bits. That
    ordering is part of the contract — see
    test_voxel_downsample_matches_reference_on_random_cloud.
    """
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int64)
    if keys.min() < -_VOXEL_HALF or keys.max() >= _VOXEL_HALF:
        return _voxel_downsample_rowwise(pts, keys)
    # Bias to non-negative before packing: shifting a negative index would put
    # sign bits into the neighbouring axis's field. The bias is monotonic, so
    # sort order is unaffected.
    shifted = keys + _VOXEL_HALF
    packed = ((shifted[:, 0] << (2 * _VOXEL_BITS))
              | (shifted[:, 1] << _VOXEL_BITS)
              | shifted[:, 2])
    unique_packed, inv = np.unique(packed, return_inverse=True)
    return _voxel_centroids(pts, inv.ravel(), len(unique_packed))


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

# Cap on materialised neighbour pairs before clustering falls back to the
# per-point traversal. 4M pairs is ~64 MB of int64 indices. The voxel stage
# guarantees points are >= voxel_leaf_m apart and fg_max_points caps the input at
# 5000, so a depth *surface* yields roughly pi*(tol/leaf)^2 ~ 300 neighbours per
# point (~750k pairs at the production 0.20 m / 0.02 m); only a solid volume
# would approach this cap. Checked with count_neighbors, which returns the pair
# count from C without building the array.
_MAX_CLUSTER_PAIRS = 4_000_000


def _cluster_components(pts: np.ndarray, tree: cKDTree, tol: float,
                        min_pts: int) -> list[np.ndarray]:
    """Single-linkage radius clustering as one graph problem.

    Identical partition to the per-point traversal below — both are the
    connected components of the graph joining points within `tol` — but the
    neighbour search and the traversal each happen once, in C, instead of one
    Python-level KD-tree query per point.
    """
    n = pts.shape[0]
    pairs = tree.query_pairs(tol, output_type="ndarray")
    if pairs.shape[0]:
        graph = coo_matrix(
            (np.ones(pairs.shape[0], dtype=np.int8), (pairs[:, 0], pairs[:, 1])),
            shape=(n, n))
    else:
        # Every point isolated; still needs an n x n graph so each gets a label.
        graph = coo_matrix((n, n), dtype=np.int8)
    n_comp, labels = connected_components(graph, directed=False)

    # Order components by their lowest member index, reproducing the seed-scan
    # order of the original loop. Downstream picks the largest cluster with
    # max(), which returns the FIRST maximum, so this ordering is observable.
    # Reverse-order assignment leaves the minimum index without ufunc.at.
    first_member = np.empty(n_comp, dtype=np.int64)
    rev = np.arange(n - 1, -1, -1)
    first_member[labels[rev]] = rev

    counts = np.bincount(labels, minlength=n_comp)
    members_by_label = np.split(np.argsort(labels, kind="stable"),
                                np.cumsum(counts)[:-1])
    return [pts[members_by_label[lab]]
            for lab in np.argsort(first_member)
            if counts[lab] >= min_pts]


def _cluster_traversal(pts: np.ndarray, tree: cKDTree, tol: float,
                       min_pts: int) -> list[np.ndarray]:
    """Per-point fallback: no pair array, so memory stays O(N).

    Only reached when the neighbour graph would be too large to materialise —
    a case fg_max_points is meant to prevent upstream.
    """
    assigned = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []

    for seed_idx in range(len(pts)):
        if assigned[seed_idx]:
            continue
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


def cluster_all(pts: np.ndarray, tol: float, min_pts: int) -> list[np.ndarray]:
    """
    Radius-based Euclidean clustering on (N, 3) float32 xyz, with NO upper size
    bound. Returns a list of (k, 3) arrays, each being one cluster, ordered by
    each cluster's lowest member index.

    Split out from euclidean_cluster() because an upper point cap cannot be
    applied at this layer without destroying information: a projectile touching
    a large surface lands in one oversized cluster, and discarding it here loses
    the ball. cluster_and_split() re-clusters such a blob instead.

    Measured 2026-07-27 on the Dell: this was the second-hottest detector stage
    after voxel_downsample was fixed — 12-71 ms per call, up to 38% of stage
    time — because the original implementation issued one Python-level
    tree.query_ball_point per point. It is now one query_pairs plus one
    connected_components, both in C, with the per-point version kept as a
    memory fallback.
    """
    n = pts.shape[0]
    if n == 0:
        return []
    # count_neighbors counts ordered pairs including each point with itself.
    tree = cKDTree(pts)
    n_pairs = (int(tree.count_neighbors(tree, tol)) - n) // 2
    if n_pairs > _MAX_CLUSTER_PAIRS:
        return _cluster_traversal(pts, tree, tol, min_pts)
    return _cluster_components(pts, tree, tol, min_pts)


def euclidean_cluster(pts: np.ndarray, tol: float,
                      min_pts: int, max_pts: int) -> list[np.ndarray]:
    """
    Greedy radius-based Euclidean clustering on (N, 3) float32 xyz.
    Returns a list of (k, 3) arrays, each being one cluster.

    Clusters outside [min_pts, max_pts] are dropped.

    NOT on any live path — detector_node calls cluster_and_split(), which
    re-clusters an over-large cluster instead of dropping it (see its docstring
    for why the max_pts drop is wrong in a cluttered scene). This is kept
    deliberately, as the Week 3 reference the A/B tests compare against:
    test_background_map.test_euclidean_cluster_behaviour_unchanged pins its
    output, and test_cloud_geometry uses it as the known-good baseline. Deleting
    it removes the only independent implementation those comparisons have.
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
