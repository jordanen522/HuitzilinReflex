"""Unit tests for kalman.py — pure math, no ROS."""
import numpy as np
import pytest

from huitzilin_perception.ballistics import G_MPS2, ballistic_positions
from huitzilin_perception.kalman import (
    CHI2_GATE_3DOF_99,
    GRAVITY_ENU,
    ProjectileTracker,
)

RATE_HZ = 15.0  # detector centroid rate in sim


def _feed_ballistic(tracker, p0, v0, *, n, noise_std=0.0, seed=42, t0=0.0):
    """Feed n samples of an exact ballistic track (+ optional noise)."""
    rng = np.random.default_rng(seed)
    ts = t0 + np.arange(n) / RATE_HZ
    pts = ballistic_positions(p0, v0, ts - t0)
    if noise_std > 0.0:
        pts = pts + rng.normal(0.0, noise_std, pts.shape)
    for t, z in zip(ts, pts):
        tracker.process(float(t), z)
    return float(ts[-1])


def test_first_measurement_initialises_track():
    tr = ProjectileTracker()
    assert not tr.has_track
    assert tr.process(0.0, [6.0, 0.0, 2.0])
    assert tr.has_track
    assert tr.n_updates == 1
    pos, vel = tr.state()
    np.testing.assert_allclose(pos, [6.0, 0.0, 2.0])
    np.testing.assert_allclose(vel, [0.0, 0.0, 0.0])


def test_state_raises_without_track():
    with pytest.raises(RuntimeError):
        ProjectileTracker().state()


def test_velocity_converges_on_noisy_ballistic_track():
    tr = ProjectileTracker(meas_std_m=0.05)
    p0, v0 = [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9]
    t_last = _feed_ballistic(tr, p0, v0, n=10, noise_std=0.05)
    pos, vel = tr.state()
    truth_pos = ballistic_positions(p0, v0, t_last)[0]
    truth_vel = np.asarray(v0) + t_last * GRAVITY_ENU
    assert np.linalg.norm(pos - truth_pos) < 0.15
    assert np.linalg.norm(vel - truth_vel) < 1.0


def test_gravity_is_modelled_not_estimated():
    # Free-fall from rest, exact measurements: velocity must track -g*t
    tr = ProjectileTracker()
    t_last = _feed_ballistic(tr, [0.0, 0.0, 30.0], [0.0, 0.0, 0.0], n=8)
    _, vel = tr.state()
    assert vel[2] == pytest.approx(-G_MPS2 * t_last, abs=0.5)


def test_outlier_is_gated_but_track_survives():
    tr = ProjectileTracker(meas_std_m=0.05)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=6)
    n_before = tr.n_updates
    accepted = tr.process(t_last + 1.0 / RATE_HZ, [50.0, 50.0, 50.0])
    assert not accepted
    assert tr.has_track
    assert tr.n_updates == n_before


def test_persistent_disagreement_reseeds_track():
    tr = ProjectileTracker(meas_std_m=0.05, max_consecutive_rejects=3)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=6)
    for k in range(1, 4):
        tr.process(t_last + k / RATE_HZ, [80.0, 80.0, 10.0])
    assert tr.has_track
    assert tr.n_updates == 1                      # reseeded from the new object
    pos, _ = tr.state()
    np.testing.assert_allclose(pos, [80.0, 80.0, 10.0])


def test_track_times_out_and_reseeds():
    tr = ProjectileTracker(track_timeout_s=0.5)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=5)
    assert tr.process(t_last + 1.0, [1.0, 2.0, 3.0])   # 1 s silence > timeout
    assert tr.n_updates == 1
    pos, _ = tr.state()
    np.testing.assert_allclose(pos, [1.0, 2.0, 3.0])


def test_out_of_order_stamp_rejected():
    tr = ProjectileTracker()
    tr.process(1.0, [6.0, 0.0, 2.0])
    assert not tr.process(0.9, [5.9, 0.0, 2.0])
    assert tr.n_updates == 1


# ── Task 3: dodge planning ────────────────────────────────────────────────

from huitzilin_perception.kalman import (  # noqa: E402
    dodge_direction,
    plan_dodge,
    predict_closest_approach,
    should_dodge,
)


def test_tca_and_miss_without_gravity():
    tca, miss, miss_vec = predict_closest_approach(
        [6.0, 0.5, 0.0], [-8.0, 0.0, 0.0], gravity=np.zeros(3))
    assert tca == pytest.approx(0.75, abs=0.02)
    assert miss == pytest.approx(0.5, abs=0.02)
    assert miss_vec[1] == pytest.approx(0.5, abs=0.02)


def test_tca_gravity_compensated_throw_is_a_hit():
    # Lofted throw: vz0 = 0.5*g*t_flight returns to aim altitude at t=0.75 s.
    v_rel = np.array([-8.0, 0.0, 0.5 * G_MPS2 * 0.75])
    tca, miss, _ = predict_closest_approach([6.0, 0.0, 0.0], v_rel)
    assert tca == pytest.approx(0.75, abs=0.02)
    assert miss < 0.06  # bounded by the 10 ms sampling step


def test_miss_beyond_horizon_reports_horizon_edge():
    # Ball flying AWAY: closest approach is now (t=0).
    tca, miss, _ = predict_closest_approach([6.0, 0.0, 0.0], [8.0, 0.0, 0.0],
                                            gravity=np.zeros(3))
    assert tca == pytest.approx(0.0)
    assert miss == pytest.approx(6.0)


def test_dodge_direction_moves_away_from_pass_side():
    # Ball passes 0.4 m to the drone's +y: dodge must be -y.
    d = dodge_direction([0.0, 0.4, 0.0], [-8.0, 0.0, 0.0])
    np.testing.assert_allclose(d, [0.0, -1.0, 0.0], atol=1e-9)
    assert np.linalg.norm(d) == pytest.approx(1.0)


def test_dodge_direction_dead_centre_falls_back_to_lateral():
    d = dodge_direction([0.01, 0.0, 0.0], [-8.0, 0.0, 0.0])
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert abs(d @ np.array([-1.0, 0.0, 0.0])) < 1e-9   # perpendicular to approach
    assert d[2] == pytest.approx(0.0)                    # horizontal fallback


def test_dodge_direction_vertical_approach_still_defined():
    d = dodge_direction([0.0, 0.0, 0.01], [0.0, 0.0, -10.0])
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert abs(d[2]) < 1e-9


@pytest.mark.parametrize("n,miss,tca,expected", [
    (3, 0.5, 1.0, True),
    (2, 0.5, 1.0, False),   # not enough confirmations
    (3, 0.9, 1.0, False),   # passes outside threat radius
    (3, 0.5, 2.0, False),   # too far in the future
    (3, 0.5, -0.1, False),  # already passed
])
def test_should_dodge_thresholds(n, miss, tca, expected):
    assert should_dodge(
        n, miss, tca,
        min_updates=3, threat_radius_m=0.75, trigger_horizon_s=1.5,
    ) is expected


def test_plan_dodge_end_to_end_hit_geometry():
    v0 = np.array([-8.0, 0.0, 0.5 * G_MPS2 * 0.75])
    plan = plan_dodge([6.0, 0.0, 2.0], v0, [0.0, 0.0, 2.0], [0.0, 0.0, 0.0])
    assert plan.tca_s == pytest.approx(0.75, abs=0.02)
    assert plan.miss_m < 0.06
    assert np.linalg.norm(plan.direction) == pytest.approx(1.0)


# ── Ground clearance (W4 live bring-up, 2026-07-26) ───────────────────────
# The drone dodged itself into the runway from 2 m: a gravity-compensated
# throw arrives descending, so the perpendicular escape points DOWN.

from huitzilin_perception.kalman import clamp_dodge_to_clearance  # noqa: E402


def test_clearance_leaves_upward_dodge_untouched():
    d = clamp_dodge_to_clearance([0.0, 0.6, 0.8], altitude_m=0.2,
                                 floor_m=1.0, descent_len_m=1.5)
    np.testing.assert_allclose(d, [0.0, 0.6, 0.8], atol=1e-9)


def test_clearance_leaves_affordable_descent_untouched():
    # 2 m up, 1 m floor -> 1 m of headroom; a 1.5 m dodge descending at 0.5
    # of full speed travels 0.75 m. Affordable, so pass it through.
    d = clamp_dodge_to_clearance([0.0, 0.866, -0.5], altitude_m=2.0,
                                 floor_m=1.0, descent_len_m=1.5)
    np.testing.assert_allclose(d, [0.0, 0.866, -0.5], atol=1e-9)


def test_clearance_reaims_steep_dodge_horizontally_keeping_unit_speed():
    # The measured crash case: dir_body z = -0.83 at 2 m altitude.
    d = clamp_dodge_to_clearance([0.03, 0.56, -0.83], altitude_m=2.0,
                                 floor_m=1.0, descent_len_m=1.5)
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert d[2] == pytest.approx(-1.0 / 1.5)      # exactly the headroom
    assert d[1] > 0.0                              # same escape side
    assert d[0] / d[1] == pytest.approx(0.03 / 0.56)


def test_clearance_at_the_floor_forbids_descent_entirely():
    d = clamp_dodge_to_clearance([0.0, 0.56, -0.83], altitude_m=0.8,
                                 floor_m=1.0, descent_len_m=1.5)
    assert d[2] == pytest.approx(0.0)
    np.testing.assert_allclose(d, [0.0, 1.0, 0.0], atol=1e-9)


def test_clearance_straight_down_escape_becomes_horizontal():
    d = clamp_dodge_to_clearance([0.0, 0.0, -1.0], altitude_m=0.5,
                                 floor_m=1.0, descent_len_m=1.5)
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert d[2] == pytest.approx(0.0)


def test_plan_dodge_applies_clearance_when_altitude_given():
    # Descending throw from ahead-and-above: unclamped escape points down.
    v0 = np.array([-8.0, 0.0, -2.0])
    kw = dict(altitude_m=1.2, floor_m=1.0, descent_len_m=1.5)
    free = plan_dodge([6.0, 0.0, 1.6], v0, [0.0, 0.0, 1.2], [0.0, 0.0, 0.0])
    clamped = plan_dodge([6.0, 0.0, 1.6], v0, [0.0, 0.0, 1.2],
                         [0.0, 0.0, 0.0], **kw)
    assert free.direction[2] < -0.3                     # would fly downward
    assert clamped.direction[2] >= -0.2 / 1.5 - 1e-9    # 0.2 m of headroom
    assert np.linalg.norm(clamped.direction) == pytest.approx(1.0)
    assert clamped.tca_s == pytest.approx(free.tca_s)   # geometry unchanged
