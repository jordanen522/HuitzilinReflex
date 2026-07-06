"""Unit tests for cloud_geometry.py — the pure-numpy math under detector_node.

Runs without ROS (numpy + scipy only), so it works on any dev box:
    python -m pytest src/huitzilin_perception/test/test_cloud_geometry.py

The egomotion tests encode the W3 recall root cause: background differencing
in the moving camera frame floods (whole scene = foreground) under patrol
translation, while the same differencing in the fixed odom frame stays clean.
"""

import math

import numpy as np

from huitzilin_perception.cloud_geometry import (
    apply_transform,
    euclidean_cluster,
    foreground_mask,
    is_valid_quat,
    make_transform,
    quat_to_rot,
    voxel_downsample,
)


# ── quaternion / transform primitives ────────────────────────────────────────

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


# ── egomotion compensation: the actual W3 failure mode ───────────────────────

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
    # that threw away half of all frames in the 2026-07-06 regression run.
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
