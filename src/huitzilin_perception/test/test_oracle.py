"""The synthetic sensor's honesty checks.

This harness exists to answer questions the real detector cannot reach, which
makes it exactly the kind of tool that can prove whatever you wanted. These
tests pin the properties that stop it: it must refuse to see things a real
camera could not see, its noise must be the noise it claims, and its geometry
must be the geometry evasion_node will invert.
"""

import math

import numpy as np
import pytest

from huitzilin_perception.cloud_geometry import quat_to_rot
from huitzilin_perception.oracle import (
    DEFAULT_BALL_PREFIXES,
    DelayLine,
    apply_noise,
    in_view,
    is_ball_name,
    measurement_covariance,
    select_ball,
    synthesize,
    to_body,
)

IDENTITY = np.eye(3)


def _yaw(rad):
    """body->world rotation for a yaw of `rad` about +z."""
    return quat_to_rot(0.0, 0.0, math.sin(rad / 2.0), math.cos(rad / 2.0))


# --- which model is the ball -------------------------------------------------
#
# These exist because getting this wrong is SILENT. The oracle matched only
# "projectile_" while dodge_battery names its ball "ball_...", so the oracle
# published zero centroids, every scenario scored NO-FIRE, and the run looked
# like a trigger that never fires rather than a sensor that never saw. The
# names below are built exactly as the two spawners build them.

def test_the_battery_ball_is_recognised():
    """dodge_battery.py: name = f"ball_{rid}_r{rep}_{int(time.time())}".

    THE regression. This is the harness that scores every number we report,
    so an oracle that cannot see its ball can only ever produce 0/N.
    """
    assert is_ball_name("ball_B08_r0_1765432100", DEFAULT_BALL_PREFIXES)


def test_the_spawn_projectile_ball_is_recognised():
    """spawn_projectile.py: f"projectile_{scenario_id}_{int(time.time())}"."""
    assert is_ball_name("projectile_S01_1765432100", DEFAULT_BALL_PREFIXES)


def test_both_spawner_conventions_ship_by_default():
    """Neither may be dropped: the two spawners are both still in use, and
    whichever is missing fails silently rather than loudly."""
    assert "ball_" in DEFAULT_BALL_PREFIXES
    assert "projectile_" in DEFAULT_BALL_PREFIXES


def test_the_drone_is_never_mistaken_for_a_ball():
    assert not is_ball_name("iris_depth", DEFAULT_BALL_PREFIXES)
    assert not is_ball_name("ground_plane", DEFAULT_BALL_PREFIXES)


def test_the_nearest_matching_ball_wins():
    """A ball that failed to despawn leaves a stale model under the same
    prefix. First-match would hand the tracker whichever the message listed
    first; nearest picks the live incoming one."""
    got = select_ball(
        [("ball_B08_r0_1765432100", (9.0, 0.0, 0.0)),    # stale, far
         ("ball_B08_r1_1765432200", (3.0, 0.0, 0.0))],   # live, closing
        drone_world=(0.0, 0.0, 0.0))
    np.testing.assert_allclose(got, [3.0, 0.0, 0.0])


def test_a_nearer_non_ball_model_does_not_steal_the_match():
    """Nearest is applied AFTER the prefix filter, never before."""
    got = select_ball(
        [("ground_plane", (0.1, 0.0, 0.0)),
         ("projectile_S01_1765432100", (4.0, 0.0, 0.0))],
        drone_world=(0.0, 0.0, 0.0))
    np.testing.assert_allclose(got, [4.0, 0.0, 0.0])


def test_no_ball_in_the_world_is_not_an_error():
    """Between runs there is legitimately no ball; the caller must read this
    as 'nothing to report this frame'."""
    assert select_ball([("iris_depth", (0.0, 0.0, 0.0))],
                       drone_world=(0.0, 0.0, 0.0)) is None
    assert select_ball([], drone_world=(0.0, 0.0, 0.0)) is None


def test_an_empty_prefix_list_matches_nothing():
    """Pins the behaviour the node turns into a startup failure: matching
    nothing must not quietly look like 'the ball was not there'."""
    assert not is_ball_name("ball_B08_r0_1765432100", [])
    assert select_ball([("ball_B08_r0_1765432100", (3.0, 0.0, 0.0))],
                       drone_world=(0.0, 0.0, 0.0), prefixes=[]) is None


# --- geometry ----------------------------------------------------------------

def test_level_drone_sees_a_ball_ahead_straight_ahead():
    rel = to_body([5.0, 0.0, 0.0], IDENTITY)
    np.testing.assert_allclose(rel, [5.0, 0.0, 0.0], atol=1e-12)


def test_yaw_is_undone_into_the_body_frame():
    """A ball due east of a drone facing north is on the drone's RIGHT, which
    in FLU is negative y."""
    rel = to_body([5.0, 0.0, 0.0], _yaw(math.pi / 2))   # nose points +y
    np.testing.assert_allclose(rel, [0.0, -5.0, 0.0], atol=1e-9)


def test_the_round_trip_evasion_node_performs_is_exact():
    """evasion_node lifts the centroid back into odom with the SAME rotation.
    If these two disagree, the oracle injects an attitude error the real
    pipeline would never have had."""
    R = _yaw(0.7)
    rel_world = np.array([3.0, -2.0, 1.5])
    back = R @ to_body(rel_world, R)
    np.testing.assert_allclose(back, rel_world, atol=1e-12)


# --- visibility gates --------------------------------------------------------

def test_a_ball_beyond_the_configured_range_is_not_reported():
    assert not in_view([12.1, 0.0, 0.0], detection_range_m=12.0)
    assert in_view([11.9, 0.0, 0.0], detection_range_m=12.0)


def test_a_ball_closer_than_the_near_limit_is_not_reported():
    """A real depth camera has a near limit too (roi_min_range_m 0.30);
    without one the oracle would keep feeding the tracker measurements right
    through the moment of impact."""
    assert not in_view([0.2, 0.0, 0.0], detection_range_m=12.0, min_range_m=0.30)


def test_a_ball_behind_the_drone_is_never_reported():
    """The most important refusal. Warning from outside the frustum is warning
    no sensor could have given, and a dodge planned on it would be a result
    about nothing."""
    assert not in_view([-5.0, 0.0, 0.0], detection_range_m=12.0)


def test_the_field_of_view_edge_is_where_it_is_configured():
    r = 5.0
    inside = math.radians(44.0)
    outside = math.radians(46.0)
    assert in_view([r * math.cos(inside), r * math.sin(inside), 0.0],
                   detection_range_m=12.0, fov_half_angle_deg=45.0)
    assert not in_view([r * math.cos(outside), r * math.sin(outside), 0.0],
                       detection_range_m=12.0, fov_half_angle_deg=45.0)


def test_a_ball_exactly_on_the_drone_is_not_reported():
    """Guards a divide-by-zero in the bearing test."""
    assert not in_view([0.0, 0.0, 0.0], detection_range_m=12.0)


# --- noise -------------------------------------------------------------------

def test_zero_sigma_is_exactly_noiseless():
    rng = np.random.default_rng(1)
    truth = np.array([4.0, 1.0, -0.5])
    np.testing.assert_array_equal(apply_noise(truth, (0.0, 0.0, 0.0), rng), truth)


def test_per_axis_sigma_lands_on_the_right_axis():
    """A stereo sensor's depth error is not its lateral error. Getting these
    transposed would make the oracle model a sensor nobody has."""
    rng = np.random.default_rng(7)
    truth = np.zeros(3)
    samples = np.array([apply_noise(truth, (2.0, 0.01, 0.01), rng)
                        for _ in range(4000)])
    std = samples.std(axis=0)
    assert std[0] == pytest.approx(2.0, rel=0.1)
    assert std[1] == pytest.approx(0.01, rel=0.2)
    assert std[2] == pytest.approx(0.01, rel=0.2)


def test_the_reported_covariance_matches_the_noise_actually_applied():
    R = measurement_covariance((2.0, 0.01, 0.01))
    np.testing.assert_allclose(np.diag(R), [4.0, 1e-4, 1e-4])


def test_a_noiseless_oracle_still_yields_an_invertible_covariance():
    """R = 0 makes the innovation covariance singular and the filter raises."""
    R = measurement_covariance((0.0, 0.0, 0.0))
    assert np.linalg.det(R) > 0.0


# --- dropout and determinism -------------------------------------------------

def test_dropout_removes_roughly_the_requested_fraction():
    rng = np.random.default_rng(3)
    kept = sum(synthesize([5.0, 0.0, 0.0], IDENTITY,
                          detection_range_m=12.0, dropout_prob=0.25,
                          rng_gen=rng) is not None
               for _ in range(2000))
    assert 0.70 < kept / 2000 < 0.80


def test_the_same_seed_replays_the_same_battery():
    def run(seed):
        rng = np.random.default_rng(seed)
        return [synthesize([5.0, 0.0, 0.0], IDENTITY, detection_range_m=12.0,
                           noise_std_xyz=(0.1, 0.1, 0.1), dropout_prob=0.2,
                           rng_gen=rng) for _ in range(50)]

    a, b = run(11), run(11)
    assert len(a) == len(b)
    for x, y in zip(a, b):
        if x is None or y is None:
            assert x is None and y is None
        else:
            np.testing.assert_array_equal(x, y)


def test_gates_are_judged_on_truth_not_on_the_noisy_reading():
    """Visibility is a property of where the ball IS. If noise were applied
    first, a ball just outside the range gate could be jittered into view --
    the oracle would then report detections its own configuration says are
    impossible."""
    rng = np.random.default_rng(5)
    out_of_range = [20.0, 0.0, 0.0]
    for _ in range(200):
        assert synthesize(out_of_range, IDENTITY, detection_range_m=12.0,
                          noise_std_xyz=(5.0, 5.0, 5.0), rng_gen=rng) is None


def test_a_visible_ball_is_reported_in_the_body_frame():
    got = synthesize([0.0, 6.0, 0.0], _yaw(math.pi / 2),   # dead ahead of a
                     detection_range_m=12.0)               # north-facing drone
    assert got is not None
    np.testing.assert_allclose(got, [6.0, 0.0, 0.0], atol=1e-9)


# --- pipeline delay ----------------------------------------------------------
#
# The oracle without this hands the tracker a measurement the instant the world
# moves, which no camera does. The delay comes straight off tca, the term the
# whole envelope is bounded by, so the disabled path must be EXACTLY the old
# behaviour and the enabled path must not leak or reorder frames.

def test_zero_delay_returns_the_payload_immediately():
    """The default must be byte-identical to having no delay line at all.

    Every recorded result was flown against a zero-latency oracle. If the
    disabled path ever buffered even one tick, all of them would silently stop
    being reproducible.
    """
    line = DelayLine(0.0)
    assert not line.enabled
    assert line.push(10.0, "a") == ["a"]
    assert line.push(10.5, "b") == ["b"]
    assert len(line) == 0
    assert line.due(99.0) == []


def test_a_delayed_frame_is_held_then_released_once():
    line = DelayLine(0.030)
    assert line.push(1.000, "a") == []       # not due yet
    assert line.due(1.020) == []             # still inside the pipeline
    assert line.due(1.030) == ["a"]          # due exactly at the boundary
    assert line.due(1.100) == []             # and never delivered twice
    assert len(line) == 0


def test_frames_come_out_in_order_and_the_queue_drains():
    line = DelayLine(0.050)
    for i, t in enumerate([1.00, 1.01, 1.02]):
        assert line.push(t, i) == []
    assert len(line) == 3
    assert line.due(1.055) == [0]            # only the one that is due
    assert line.due(1.080) == [1, 2]         # oldest first
    assert len(line) == 0


def test_frames_in_flight_survive_the_ball_leaving_the_frustum():
    """due() is called every tick, not only when a detection is made.

    A real pipeline still delivers what it already captured after the ball goes
    out of view. If release only happened on push, the last frames before the
    ball left would be lost — exactly the ones closest to impact.
    """
    line = DelayLine(0.040)
    line.push(2.000, "last-frame-before-it-vanished")
    assert line.due(2.041) == ["last-frame-before-it-vanished"]


def test_a_sim_time_rewind_does_not_strand_the_queue():
    """Relaunching against the same clock must not silence the node forever.

    Without this the release times sit in an unreachable future, due() returns
    nothing for the rest of the run, and the log says nothing about why.
    """
    line = DelayLine(0.050)
    line.push(500.0, "stale")
    assert line.due(0.0) == []               # clock went backwards; dropped
    assert len(line) == 0
    assert line.push(0.0, "fresh") == []
    assert line.due(0.050) == ["fresh"]      # and the line still works after


def test_a_negative_delay_is_refused():
    """Negative latency is time travel, and would release frames early — a
    sensor that is BETTER than ground truth, which is the one direction this
    harness must never be able to flatter itself in."""
    with pytest.raises(ValueError):
        DelayLine(-0.001)
