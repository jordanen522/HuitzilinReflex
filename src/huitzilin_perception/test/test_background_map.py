"""Unit tests for the W4 background rework — persistent voxel map + cluster split.

Runs without ROS (numpy + scipy only):
    python -m pytest src/huitzilin_perception/test/test_background_map.py

The two headline tests encode the measured Week-4 failure (
"the detector goes blind while patrolling"):

  test_revisited_region_stays_background — the rolling deque forgets a region
      after 0.267 s, so re-entering it floods the foreground; the persistent map
      reports it as background.
  test_split_recovers_ball_merged_into_wall — the ball merges into a metre-scale
      blob at cluster_tolerance_m and the old max_points filter discarded it;
      splitting recovers it.
"""

import numpy as np

from huitzilin_perception.background_map import VoxelBackgroundMap
from huitzilin_perception.cloud_geometry import (
    cluster_all,
    cluster_and_split,
    cluster_extent,
    euclidean_cluster,
    foreground_mask,
)


# helpers

def _grid(x0, x1, y0, y1, z, step):
    """Planar point grid in the z = const plane."""
    xs = np.arange(x0, x1, step)
    ys = np.arange(y0, y1, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel(),
                     np.full(gx.size, z)], axis=1).astype(np.float32)


def _ball(centre, step=0.02, n=4):
    """Compact blob ~ (n-1)*step across — stands in for the 80 mm projectile."""
    r = np.arange(n) * step
    gx, gy, gz = np.meshgrid(r, r, r[:3])
    pts = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    return (pts + np.asarray(centre, dtype=np.float32)).astype(np.float32)


# voxel map basics

def test_empty_map_reports_no_foreground():
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=10.0)
    pts = _grid(0, 1, 0, 1, 0.0, 0.1)
    assert not bg.foreground(pts).any()
    assert len(bg) == 0


def test_inserted_points_are_background():
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=10.0)
    pts = _grid(0, 1, 0, 1, 0.0, 0.1)
    bg.insert(pts, 1.0)
    assert len(bg) > 0
    assert not bg.foreground(pts).any()


def test_novel_point_is_foreground():
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=10.0)
    bg.insert(_grid(0, 1, 0, 1, 0.0, 0.1), 1.0)
    far = np.array([[5.0, 5.0, 5.0]], dtype=np.float32)
    assert bg.foreground(far).all()


def test_neighbour_dilation_absorbs_voxel_boundary_jitter():
    # A point one voxel away from an observed voxel must NOT be foreground:
    # without dilation, a static surface point jittering across a voxel edge
    # would read as a new object on every crossing.
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=10.0)
    bg.insert(np.array([[0.05, 0.05, 0.05]], dtype=np.float32), 1.0)
    adjacent = np.array([[0.15, 0.05, 0.05],    # +1 voxel on x
                         [0.15, 0.15, 0.15]],   # +1 on all three
                        dtype=np.float32)
    assert not bg.foreground(adjacent).any()
    # Two voxels out is beyond the dilation and must still be novel.
    assert bg.foreground(np.array([[0.35, 0.05, 0.05]], dtype=np.float32)).all()


def test_ttl_prune_forgets_stale_voxels():
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=5.0, prune_every=1)
    old = _grid(0, 1, 0, 1, 0.0, 0.1)
    bg.insert(old, 1.0)
    assert len(bg) > 0
    # A later insert elsewhere, well past the TTL, must evict the old region.
    bg.insert(_grid(9, 10, 9, 10, 0.0, 0.1), 100.0)
    bg.prune(100.0)
    assert bg.foreground(old).all()


def test_max_voxels_caps_growth():
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=1e6, max_voxels=50, prune_every=1)
    for i in range(6):
        bg.insert(_grid(i * 2.0, i * 2.0 + 1.0, 0, 1, 0.0, 0.1), float(i))
        bg.prune(float(i))
    assert len(bg) <= 50


def test_backwards_time_clears_map():
    # Sim time running backwards = the stack restarted; carrying voxels across
    # that would diff against geometry from a previous flight.
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=10.0)
    pts = _grid(0, 1, 0, 1, 0.0, 0.1)
    bg.insert(pts, 100.0)
    bg.insert(np.array([[0.0, 0.0, 0.0]], dtype=np.float32), 1.0)
    assert bg.foreground(pts).any()


def test_insert_keeps_keys_sorted():
    # Lookups are searchsorted, so an unsorted key array silently reports
    # occupied voxels as novel.
    rng = np.random.default_rng(7)
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=1e6)
    seen = []
    for i in range(10):
        pts = rng.uniform(-5.0, 5.0, size=(200, 3)).astype(np.float32)
        bg.insert(pts, float(i))
        seen.append(pts)
        assert np.all(np.diff(bg._keys) > 0)
    for pts in seen:
        assert not bg.foreground(pts).any()


# the measured failure: a revisited region must stay background

def test_revisited_region_stays_background():
    """
    Sweep a view window forward across a static wall, then sweep back.

    The rolling deque only holds bg_history_frames, so on the return leg the
    scene it already saw is novel again and the foreground floods — that is the
    patrol blindness. The persistent map reports ~nothing.
    """
    wall = _grid(0.0, 8.0, 0.0, 1.0, 0.0, 0.05)
    window = 2.0

    def view(x0):
        m = (wall[:, 0] >= x0) & (wall[:, 0] < x0 + window)
        return wall[m]

    starts = [round(0.4 * i, 2) for i in range(15)]             # forward 0 -> 5.6
    starts += [round(5.6 - 0.4 * i, 2) for i in range(1, 15)]   # and back

    bg_map = VoxelBackgroundMap(leaf_m=0.10, ttl_s=1e6)
    deque_frames: list[np.ndarray] = []
    map_fg_return = 0
    deque_fg_return = 0

    for i, x0 in enumerate(starts):
        cur = view(x0)
        is_return_leg = i >= 15

        map_fg = int(bg_map.foreground(cur).sum())
        bg_map.insert(cur, float(i))

        if deque_frames:
            hist = np.vstack(deque_frames)
            deque_fg = int(foreground_mask(cur, hist, 0.10).sum())
        else:
            deque_fg = 0
        deque_frames.append(cur)
        if len(deque_frames) > 5:
            deque_frames.pop(0)

        if is_return_leg:
            map_fg_return += map_fg
            deque_fg_return += deque_fg

    # On the return leg every point has been seen before: the map must be quiet.
    assert map_fg_return == 0, f"persistent map flagged {map_fg_return} points"
    # The deque, having forgotten, floods on the very same geometry.
    assert deque_fg_return > 500, f"deque only flagged {deque_fg_return}"


def test_map_still_flags_a_ball_over_seen_ground():
    """Coverage must not cost sensitivity: a projectile over already-observed
    ground is still foreground."""
    ground = _grid(0.0, 3.0, 0.0, 3.0, 0.0, 0.05)
    bg = VoxelBackgroundMap(leaf_m=0.10, ttl_s=1e6)
    for t in range(5):
        bg.insert(ground, float(t))
    ball = _ball((1.5, 1.5, 1.20))
    assert bg.foreground(ball).all()


# the measured failure: the ball must survive a merge with a big surface

def test_split_recovers_ball_merged_into_wall():
    """
    Funnel evidence being reproduced (throw that passed 0.484 m from the drone):
        fg=303 clusters=2 ball_sized=1 (size,extent)=[(291, 1.42), (12, 0.18)]
    On the frames that mattered the 12-point ball had merged into the 1.42 m blob
    at tol=0.20 and the max_points filter dropped the lot.
    """
    wall = _grid(0.0, 1.4, 0.0, 1.4, 0.0, 0.05)
    ball = _ball((0.70, 0.70, 0.15))            # 0.15 m off the wall
    fg = np.vstack([wall, ball])

    # Old path: one oversized cluster, discarded by max_pts -> ball lost.
    old = euclidean_cluster(fg, tol=0.20, min_pts=5, max_pts=500)
    assert old == [] or all(cluster_extent(c) > 0.35 for c in old)

    # New path: one re-cluster at 0.10 detaches the ball; the wall stays whole
    # (and oversized, so it is still rejected).
    got = cluster_and_split(fg, tol=0.20, min_pts=5, max_extent=0.35,
                            split_tol=0.10, max_split_points=5000)
    assert len(got) == 1, [(c.shape[0], round(cluster_extent(c), 2)) for c in got]
    np.testing.assert_allclose(got[0].mean(axis=0), ball.mean(axis=0), atol=1e-5)


def test_recursive_shrinking_would_shatter_the_wall():
    """
    Why cluster_and_split makes exactly ONE pass.

    The first implementation recursed (tol *= 0.5 until the extent fit). At a
    radius near the surface's own 0.05 m point spacing the wall breaks into many
    fragments that all pass the ball-extent gate — pure false positives. This
    pins the behaviour that motivated the single-pass design.
    """
    wall = _grid(0.0, 1.4, 0.0, 1.4, 0.0, 0.05)
    shattered = [c for c in cluster_all(wall, 0.05, 5)
                 if cluster_extent(c) <= 0.35]
    assert len(shattered) > 5, len(shattered)
    # One pass at a radius above the spacing keeps it whole and rejectable.
    assert cluster_and_split(wall, tol=0.20, min_pts=5, max_extent=0.35,
                             split_tol=0.10, max_split_points=5000) == []


def test_split_leaves_already_small_clusters_alone():
    a = _ball((0.0, 0.0, 0.0))
    b = _ball((2.0, 0.0, 0.0))
    got = cluster_and_split(np.vstack([a, b]), tol=0.20, min_pts=5,
                            max_extent=0.35, split_tol=0.10,
                            max_split_points=5000)
    assert len(got) == 2
    assert all(cluster_extent(c) <= 0.35 for c in got)


def test_split_refuses_oversized_blobs():
    # Cost guard: a flood is not worth re-clustering, and fg_max_points is the
    # layer that is supposed to catch it.
    wall = _grid(0.0, 2.0, 0.0, 2.0, 0.0, 0.05)
    fg = np.vstack([wall, _ball((1.0, 1.0, 0.15))])
    got = cluster_and_split(fg, tol=0.20, min_pts=5, max_extent=0.35,
                            split_tol=0.10, max_split_points=10)
    assert got == []


def test_euclidean_cluster_behaviour_unchanged():
    # The Week-3 entry point keeps its exact semantics (both bounds inclusive),
    # since the recorded W3 recall figures were scored through it.
    a = _ball((0.0, 0.0, 0.0))            # 48 points
    got = euclidean_cluster(a, tol=0.05, min_pts=5, max_pts=500)
    assert len(got) == 1 and got[0].shape[0] == a.shape[0]
    assert euclidean_cluster(a, tol=0.05, min_pts=5, max_pts=10) == []
    assert euclidean_cluster(a, tol=0.05, min_pts=100, max_pts=500) == []
