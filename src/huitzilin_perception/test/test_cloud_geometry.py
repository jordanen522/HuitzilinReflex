"""Unit tests for cloud_geometry.py — the pure-numpy math under detector_node.

Runs without ROS (numpy + scipy only), so it works on any dev box:
    python -m pytest src/huitzilin_perception/test/test_cloud_geometry.py

The egomotion tests encode the W3 recall root cause: background differencing
in the moving camera frame floods (whole scene = foreground) under patrol
translation, while the same differencing in the fixed odom frame stays clean.
"""

import math

import numpy as np
from scipy.spatial import cKDTree

from huitzilin_perception.cloud_geometry import (
    _cluster_traversal,
    apply_transform,
    cluster_all,
    euclidean_cluster,
    foreground_mask,
    is_valid_quat,
    make_transform,
    quat_to_rot,
    rotate_covariance,
    voxel_downsample,
)


# quaternion / transform primitives

def test_quat_identity():
    R = quat_to_rot(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)


def test_quat_yaw_90():
    # 90° about +z: x-axis -> y-axis
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    R = quat_to_rot(0.0, 0.0, s, c)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_invalid_quat_rejected():
    # ROS message default (all zeros) — what the pre-fix bags contain.
    assert not is_valid_quat(0.0, 0.0, 0.0, 0.0)
    assert is_valid_quat(0.0, 0.0, 0.0, 1.0)
    assert not is_valid_quat(0.0, 0.0, 0.0, 0.5)  # non-unit


def test_transform_round_trip():
    rng = np.random.default_rng(7)
    pts_world = rng.uniform(-5, 5, (100, 3)).astype(np.float32)
    # camera pose in world: translated + yawed 30°
    yaw = math.radians(30)
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    t = np.array([1.5, -2.0, 0.8])
    T_world_cam = make_transform(t, q)
    # points as the camera sees them
    R = quat_to_rot(*q)
    pts_cam = (pts_world - t) @ R  # == R.T applied from the left
    back = apply_transform(T_world_cam, pts_cam)
    np.testing.assert_allclose(back, pts_world, atol=1e-4)


# covariance rotation (evasion_node's anisotropic-R fusion hook)

def test_rotate_covariance_identity_is_a_no_op():
    R = np.diag([1.0, 4.0, 9.0])
    np.testing.assert_allclose(rotate_covariance(R, np.eye(3)), R)


def test_rotate_covariance_isotropic_is_rotation_invariant():
    # A round sensor's covariance doesn't care which way it is pointed.
    R = np.eye(3) * 0.25
    yaw = math.radians(37)
    Rot = quat_to_rot(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    np.testing.assert_allclose(rotate_covariance(R, Rot), R, atol=1e-12)


def test_rotate_covariance_round_trip_via_transpose_restores_original():
    rng = np.random.default_rng(3)
    A = rng.uniform(-1, 1, (3, 3))
    R = A @ A.T + np.eye(3) * 0.01  # a real (SPD) covariance
    yaw = math.radians(52)
    Rot = quat_to_rot(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    rotated = rotate_covariance(R, Rot)
    back = rotate_covariance(rotated, Rot.T)  # Rot.T undoes Rot for a rotation
    np.testing.assert_allclose(back, R, atol=1e-9)


def test_rotate_covariance_stays_symmetric_positive_definite():
    rng = np.random.default_rng(11)
    A = rng.uniform(-1, 1, (3, 3))
    R = A @ A.T + np.eye(3) * 0.01
    yaw = math.radians(81)
    Rot = quat_to_rot(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    rotated = rotate_covariance(R, Rot)
    np.testing.assert_allclose(rotated, rotated.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(rotated) > 0.0)


# egomotion compensation: the actual W3 failure mode

def _static_scene(n=2000, seed=1):
    """Ground-plane-ish static scene in world frame, 4-12 m ahead."""
    rng = np.random.default_rng(seed)
    pts = np.empty((n, 3), dtype=np.float32)
    pts[:, 0] = rng.uniform(4, 12, n)       # ahead
    pts[:, 1] = rng.uniform(-4, 4, n)       # lateral
    pts[:, 2] = rng.uniform(-2.0, -1.8, n)  # ground below drone
    return pts


def _view_from(pose_t, pose_q, world_pts):
    """World points as seen from a camera at (t, q) in world frame."""
    R = quat_to_rot(*pose_q)
    return ((world_pts - pose_t) @ R).astype(np.float32)


def test_camera_frame_differencing_floods_under_translation():
    """Uncompensated: 2 m/s patrol over 5 frames -> most of scene 'moves'."""
    world = _static_scene()
    qi = (0.0, 0.0, 0.0, 1.0)
    prev = _view_from(np.array([0.0, 0, 0]), qi, world)
    curr = _view_from(np.array([0.66, 0, 0]), qi, world)  # 5 frames @ 2 m/s, 15 Hz
    fg = foreground_mask(curr, prev, threshold=0.15)
    # >20% of the scene turns "foreground" from egomotion alone. At real cloud
    # sizes (50-90k ROI points) that is >>5000, i.e. the fg_max_points skip
    # that threw away half of all frames in the regression run.
    assert fg.mean() > 0.2


def test_odom_frame_differencing_is_clean_and_keeps_ball():
    """Compensated: same motion, differenced in world frame -> only ball is fg."""
    world = _static_scene()
    qi = (0.0, 0.0, 0.0, 1.0)
    poses = [np.array([0.132 * i, 0, 0]) for i in range(6)]  # 2 m/s @ 15 Hz

    bg_world = []
    for t in poses[:5]:
        cam = _view_from(t, qi, world)
        T = make_transform(t, qi)
        bg_world.append(apply_transform(T, cam))
    bg_world = np.vstack(bg_world)

    ball_world = np.array([[6.0, 0.1, 0.0],
                           [6.03, 0.1, 0.0],
                           [6.0, 0.13, 0.03]], dtype=np.float32)
    cam = _view_from(poses[5], qi, np.vstack([world, ball_world]))
    curr_world = apply_transform(make_transform(poses[5], qi), cam)

    fg = foreground_mask(curr_world, bg_world, threshold=0.15)
    assert fg[:-3].mean() < 0.01          # static scene suppressed
    assert fg[-3:].all()                  # ball survives differencing

    clusters = euclidean_cluster(curr_world[fg], tol=0.20, min_pts=3, max_pts=500)
    assert len(clusters) == 1
    assert clusters[0].shape[0] == 3


def test_voxel_downsample_keeps_small_object():
    ball = np.array([[6.0, 0.0, 0.0], [6.02, 0.0, 0.01], [6.0, 0.03, 0.0]],
                    dtype=np.float32)
    out = voxel_downsample(ball, leaf=0.02)
    assert out.shape[0] >= 2  # 80 mm ball must survive 0.02 m voxels


# voxel_downsample: pinned before the speed rewrite
# Measured hot stage: 66-74 ms per call on patrol clouds against a 77 ms/frame
# wall budget, so one stage was the whole latency overrun. These tests pin the
# CONTRACT (one point per occupied voxel, member centroid, lexicographic row
# order, float32 out) so the int64-key rewrite is provably behaviour-preserving
# rather than merely faster.

def _voxel_reference(pts, leaf):
    """The pre-rewrite implementation, kept here as the oracle."""
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    unique_keys, inv = np.unique(keys, axis=0, return_inverse=True)
    centroids = np.zeros((len(unique_keys), 3), dtype=np.float32)
    counts = np.bincount(inv.ravel(), minlength=len(unique_keys)).reshape(-1, 1)
    np.add.at(centroids, inv.ravel(), pts)
    centroids /= counts
    return centroids


def test_voxel_downsample_collapses_one_voxel_to_its_centroid():
    # Three points inside the single voxel [0.00,0.02) on every axis.
    pts = np.array([[0.000, 0.000, 0.000],
                    [0.010, 0.010, 0.010],
                    [0.002, 0.004, 0.006]], dtype=np.float32)
    out = voxel_downsample(pts, leaf=0.02)
    assert out.shape == (1, 3)
    np.testing.assert_allclose(out[0], pts.mean(axis=0), atol=1e-6)


def test_voxel_downsample_separates_adjacent_voxels():
    pts = np.array([[0.01, 0.0, 0.0],
                    [0.03, 0.0, 0.0]], dtype=np.float32)
    out = voxel_downsample(pts, leaf=0.02)
    assert out.shape[0] == 2


def test_voxel_downsample_handles_negative_coordinates():
    # floor() must be used, not truncation: -0.01 belongs to voxel -1, not 0,
    # and the odom frame the map lives in has points on both sides of origin.
    pts = np.array([[-0.01, 0.0, 0.0],
                    [0.01, 0.0, 0.0]], dtype=np.float32)
    out = voxel_downsample(pts, leaf=0.02)
    assert out.shape[0] == 2, out


def test_voxel_downsample_matches_reference_on_random_cloud():
    """The safety net for the rewrite: identical output, not just similar.

    Random floats over a metre-scale box at the production 0.02 m leaf, which
    produces both singleton and heavily-populated voxels.
    """
    rng = np.random.default_rng(20260727)
    pts = (rng.random((20000, 3), dtype=np.float32) * 4.0 - 2.0)
    got = voxel_downsample(pts, leaf=0.02)
    want = _voxel_reference(pts, leaf=0.02)
    assert got.shape == want.shape
    # Row order must match too — not just the set of centroids. The detector
    # picks the LARGEST cluster downstream, so a reordering would be silent
    # here and only show up as a flaky detection.
    np.testing.assert_allclose(got, want, rtol=0, atol=2e-6)


def test_voxel_downsample_matches_reference_when_clustered_and_duplicated():
    # Exact duplicates and tight clusters are the realistic case: a depth
    # camera quantises to pixel rays, so many points share a voxel exactly.
    rng = np.random.default_rng(7)
    seeds = rng.random((300, 3), dtype=np.float32) * 2.0
    pts = np.repeat(seeds, 7, axis=0)
    pts[::3] += np.float32(0.001)
    got = voxel_downsample(pts, leaf=0.02)
    want = _voxel_reference(pts, leaf=0.02)
    np.testing.assert_allclose(got, want, rtol=0, atol=2e-6)


def test_voxel_downsample_returns_float32_and_is_never_larger():
    rng = np.random.default_rng(11)
    pts = (rng.random((5000, 3), dtype=np.float32) * 0.5)
    out = voxel_downsample(pts, leaf=0.02)
    assert out.dtype == np.float32
    assert out.shape[0] <= pts.shape[0]


def test_voxel_downsample_far_coordinates_still_correct():
    """Beyond the packable key window the result must stay right, not wrap.

    Key packing is only valid inside +-2^20 voxels (+-21 km at 0.02 m). The ROI
    caps camera-frame points at 5 m so production never approaches this, but a
    silent integer wrap would merge two distant voxels into one bogus centroid,
    and that is exactly the class of bug that is invisible until it matters.
    """
    far = 1.0e6
    pts = np.array([[far, 0.0, 0.0],
                    [far + 0.005, 0.0, 0.0],
                    [-far, 0.0, 0.0]], dtype=np.float64)
    out = voxel_downsample(pts, leaf=0.02)
    # Two distinct locations survive; the near-duplicate pair merges.
    assert out.shape[0] == 2, out
    xs = np.sort(out[:, 0])
    assert xs[0] < 0 < xs[1], out


def test_voxel_downsample_empty_input():
    empty = np.zeros((0, 3), dtype=np.float32)
    assert voxel_downsample(empty, leaf=0.02).shape[0] == 0


# cluster_all: graph path vs per-point traversal
# The second hot stage, 12-71 ms per call (up to 38% of stage time) once
# voxel_downsample was fixed, because it ran one Python-level query_ball_point
# per point. Replaced by one query_pairs + one connected_components. Both are
# the connected components of the same radius graph, so these tests demand an
# IDENTICAL partition, not merely a similar one — the fallback path is still
# reachable, and a divergence between them would be a detection difference
# that only appears on dense clouds.

def _partition(clusters):
    """Clusters as a comparable structure: order preserved, members sorted."""
    return [sorted(map(tuple, np.round(c, 6).tolist())) for c in clusters]


def test_cluster_paths_agree_on_random_cloud():
    rng = np.random.default_rng(20260727)
    pts = (rng.random((900, 3), dtype=np.float32) * 1.0)
    fast = cluster_all(pts, tol=0.08, min_pts=3)
    slow = _cluster_traversal(pts, cKDTree(pts), tol=0.08, min_pts=3)
    assert len(fast) == len(slow)
    assert _partition(fast) == _partition(slow)


def test_cluster_paths_agree_on_separated_blobs():
    rng = np.random.default_rng(3)
    blobs = [rng.normal(c, 0.02, size=(40, 3)).astype(np.float32)
             for c in ([0.0, 0.0, 2.0], [1.0, 0.0, 2.0], [0.0, 1.0, 2.0])]
    pts = np.vstack(blobs)
    fast = cluster_all(pts, tol=0.10, min_pts=5)
    slow = _cluster_traversal(pts, cKDTree(pts), tol=0.10, min_pts=5)
    assert len(fast) == 3, [c.shape for c in fast]
    assert _partition(fast) == _partition(slow)


def test_cluster_order_is_by_lowest_member_index():
    """Downstream uses max(..., key=size), which returns the FIRST maximum.

    So cluster order is observable behaviour, not an implementation detail: two
    equal-sized clusters must resolve the same way as before the rewrite.
    """
    # Second blob (rows 3-5) is far from the first (rows 0-2).
    pts = np.array([[5.0, 0.0, 0.0], [5.01, 0.0, 0.0], [5.02, 0.0, 0.0],
                    [0.0, 0.0, 0.0], [0.01, 0.0, 0.0], [0.02, 0.0, 0.0]],
                   dtype=np.float32)
    out = cluster_all(pts, tol=0.05, min_pts=3)
    assert len(out) == 2
    # The cluster seeded at row 0 (x~5.0) must come first despite larger coords.
    assert out[0][:, 0].min() > 4.0, out


def test_cluster_singletons_are_dropped_by_min_pts():
    pts = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=np.float32)
    assert cluster_all(pts, tol=0.05, min_pts=2) == []
    assert len(cluster_all(pts, tol=0.05, min_pts=1)) == 2


def test_cluster_isolated_points_get_their_own_labels():
    # No pairs at all: the graph has no edges, and every point must still be
    # labelled rather than collapsing into one component.
    pts = (np.arange(5, dtype=np.float32)[:, None] * 10.0) * np.ones((1, 3),
                                                                    np.float32)
    assert len(cluster_all(pts, tol=0.05, min_pts=1)) == 5


def test_cluster_empty_input():
    assert cluster_all(np.zeros((0, 3), dtype=np.float32), 0.2, 3) == []


def test_dense_blob_falls_back_instead_of_materialising_pairs():
    """The memory guard, exercised by lowering the cap rather than by faking.

    A dense blob at production tol would materialise a pair array far larger
    than the cloud; the guard must route it to the O(N)-memory traversal and
    still return the same partition.
    """
    import huitzilin_perception.cloud_geometry as cg

    rng = np.random.default_rng(5)
    pts = rng.normal(0.0, 0.05, size=(600, 3)).astype(np.float32)
    expected = _partition(cluster_all(pts, tol=0.30, min_pts=3))
    saved = cg._MAX_CLUSTER_PAIRS
    try:
        cg._MAX_CLUSTER_PAIRS = 10      # force the fallback
        got = _partition(cg.cluster_all(pts, tol=0.30, min_pts=3))
    finally:
        cg._MAX_CLUSTER_PAIRS = saved
    assert got == expected
