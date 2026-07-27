"""Unit tests for kalman.py — pure math, no ROS."""
import math

import numpy as np
import pytest

from huitzilin_perception.ballistics import G_MPS2, ballistic_positions
from huitzilin_perception.kalman import (
    CHI2_GATE_3DOF_99,
    GRAVITY_ENU,
    MultiHypothesisTracker,
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


# ── Reseed counters ─────────────────────────────────────────────────────────
# A reseed puts the velocity state back to ZERO, and a zero-velocity estimate
# makes predict_closest_approach report tca ~ 0 with a miss equal to the current
# range, which the trigger reads as "not a threat". Measured 2026-07-27: the
# published predicted miss tracks the ball's current range on inbound throws —
# exactly that signature. These counters make a mid-flight reseed visible instead
# of inferred. They are lifetime totals, so reset() must NOT clear them.

def test_reseed_counters_start_at_zero():
    tr = ProjectileTracker()
    assert tr.n_reseeds_reject == 0
    assert tr.n_reseeds_timeout == 0


def test_reject_reseed_is_counted_and_separated_from_timeout():
    tr = ProjectileTracker(meas_std_m=0.05, max_consecutive_rejects=3)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=6)
    for k in range(1, 4):
        tr.process(t_last + k / RATE_HZ, [80.0, 80.0, 10.0])
    assert tr.n_reseeds_reject == 1
    assert tr.n_reseeds_timeout == 0


def test_timeout_reseed_is_counted_and_separated_from_rejects():
    tr = ProjectileTracker(track_timeout_s=0.5)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=5)
    tr.process(t_last + 1.0, [1.0, 2.0, 3.0])
    assert tr.n_reseeds_timeout == 1
    assert tr.n_reseeds_reject == 0


def test_reseed_counters_survive_an_explicit_reset():
    """Lifetime diagnostics: if reset() cleared them, the very event they exist
    to record would erase its own evidence."""
    tr = ProjectileTracker(track_timeout_s=0.5)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=5)
    tr.process(t_last + 1.0, [1.0, 2.0, 3.0])
    tr.reset()
    assert tr.n_reseeds_timeout == 1


def test_track_age_spans_the_measurements_and_restarts_with_the_track():
    """Age is the discriminator between a RESTARTED track and merely SPARSE
    detections — lifetime reseed totals cannot separate those, because the
    false-positive stream reseeds constantly between throws."""
    tr = ProjectileTracker(track_timeout_s=0.5)
    assert math.isnan(tr.track_age_s)                  # no track yet
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=5)
    assert tr.track_age_s == pytest.approx(4.0 / RATE_HZ)   # 5 updates = 4 gaps
    tr.process(t_last + 1.0, [1.0, 2.0, 3.0])          # timeout -> fresh track
    assert tr.track_age_s == pytest.approx(0.0)


def test_a_clean_track_never_reseeds():
    tr = ProjectileTracker()
    _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=10)
    assert tr.n_reseeds_reject == 0
    assert tr.n_reseeds_timeout == 0
    assert tr.n_updates == 10


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
    d = clamp_dodge_to_clearance([0.0, math.sqrt(3.0) / 2.0, -0.5],
                                 altitude_m=2.0, floor_m=1.0,
                                 descent_len_m=1.5)
    np.testing.assert_allclose(d, [0.0, math.sqrt(3.0) / 2.0, -0.5], atol=1e-9)


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
    # Lofted enough to still be ABOVE the drone at closest approach, so the
    # "away" side is downward — the live crash geometry.
    v0 = np.array([-8.0, 0.0, 4.5])
    kw = dict(altitude_m=1.2, floor_m=1.0, descent_len_m=1.5)
    free = plan_dodge([6.0, 0.0, 1.2], v0, [0.0, 0.0, 1.2], [0.0, 0.0, 0.0])
    clamped = plan_dodge([6.0, 0.0, 1.2], v0, [0.0, 0.0, 1.2],
                         [0.0, 0.0, 0.0], **kw)
    assert free.direction[2] < -0.3                     # would fly downward
    assert clamped.direction[2] >= -0.2 / 1.5 - 1e-9    # 0.2 m of headroom
    assert np.linalg.norm(clamped.direction) == pytest.approx(1.0)
    assert clamped.tca_s == pytest.approx(free.tca_s)   # geometry unchanged


# ── The dodge-authority mechanism (2026-07-27) ───────────────────────────────
# Measured live: the detector publishes the TRUE ball on 5-6 consecutive frames
# from ~4.7 m inwards, yet the track that fires a dodge is invariably exactly 3
# updates and 0.132 s old. These tests pin why, in the tracker rather than the
# detector.
#
# ProjectileTracker is SINGLE-TARGET, and the detector emits ~1.4 false-positive
# centroids per second, so a stale FP track is usually alive when a ball
# arrives. What follows is not simply "the ball is rejected": after one
# rejection the incumbent's covariance has inflated enough (init_vel_std 15 m/s)
# to SWALLOW the ball as a legitimate update, and the filter then fits a
# velocity along the line from the false positive to the ball. Only when that
# fiction is itself contradicted three times does the track reseed from the
# ball — by which point several true detections have been spent.

def _ball_stream(n, *, t0=0.0, start=(4.7, 0.0, 2.0), speed=8.0):
    """n frames of a head-on ball closing at `speed`, at the detector rate."""
    ts = t0 + np.arange(n) / RATE_HZ
    return [(float(t), np.array([start[0] - speed * (t - t0), start[1], start[2]]))
            for t in ts]


TRUE_BALL_VEL = np.array([-8.0, 0.0, 0.0])


def _vel_err(tracker):
    _, v = tracker.state()
    return float(np.linalg.norm(v - TRUE_BALL_VEL))


def test_ball_alone_pins_velocity_by_the_second_frame():
    """Control: with no incumbent, two frames are enough to know the velocity."""
    tr = ProjectileTracker()
    stream = _ball_stream(6)
    tr.process(*stream[0])
    assert _vel_err(tr) == pytest.approx(8.0, abs=1e-6)  # seeded at zero
    tr.process(*stream[1])
    assert _vel_err(tr) < 0.5
    assert tr.n_updates == 2


def test_an_incumbent_false_positive_corrupts_the_balls_velocity_estimate():
    """A stale FP track makes the ball's opening detections actively harmful.

    The filter does not merely ignore them: it fits a velocity along the line
    from the false positive to the ball, which is wildly wrong, and then spends
    further true detections rejecting the fiction it just built.
    """
    tr = ProjectileTracker()
    tr.process(0.0, np.array([1.0, 3.0, 2.0]))  # false positive, 3 m off path

    accepted, errs = [], []
    for t, z in _ball_stream(6, t0=1 / RATE_HZ):
        accepted.append(tr.process(t, z))
        errs.append(_vel_err(tr))

    # Frame 1 is swallowed by the inflated covariance, not rejected...
    assert accepted[:2] == [False, True]
    # ...and the velocity it produces is far worse than the zero it started at.
    assert errs[1] > 30.0
    # Frames 2-3 then contradict that fiction, and frame 4 reseeds from the ball.
    assert accepted[2:5] == [False, False, True]
    assert tr.n_reseeds_reject == 1
    # After SIX true detections the ball's own track holds just two updates —
    # the control has six by now, and is one short of the confirmation gate.
    assert tr.n_updates == 2

    # Only the sixth true detection restores what the control had at the second.
    assert errs[5] < 0.5


def test_the_incumbent_costs_four_frames_of_warning():
    """Quantifies the dodge-authority loss: 4 frames = 0.267 s at 15 Hz.

    This is the whole of the missing dodge warning — `tca` at commit measures
    0.18-0.26 s, and closing this gap is worth more than any detector threshold
    that has been tried.
    """
    def frames_to_converge(with_fp):
        tr = ProjectileTracker()
        t0 = 0.0
        if with_fp:
            tr.process(0.0, np.array([1.0, 3.0, 2.0]))
            t0 = 1 / RATE_HZ
        for i, (t, z) in enumerate(_ball_stream(10, t0=t0)):
            tr.process(t, z)
            if _vel_err(tr) < 0.5:
                return i
        return None

    clean, contaminated = frames_to_converge(False), frames_to_converge(True)
    assert clean == 1
    assert contaminated == 5
    assert (contaminated - clean) / RATE_HZ == pytest.approx(0.267, abs=0.01)


# ── MultiHypothesisTracker: the fix for the above ────────────────────────────
# One filter forced to represent whatever arrived last is the whole defect. Keep
# a small set of candidate tracks instead, associate each centroid to the one
# that explains it best, and start a new track for anything unexplained; the
# ball's opening detections then accumulate on their OWN hypothesis instead of
# being spent correcting a false positive's.


def _mht_vel_err(track):
    _, v = track.state()
    return float(np.linalg.norm(v - TRUE_BALL_VEL))


def test_mht_converges_as_fast_with_an_incumbent_as_without():
    """The headline: the false positive no longer costs the ball any frames.

    This is the same experiment as test_the_incumbent_costs_four_frames_of_
    warning, which measures a 4-frame (0.267 s) penalty on the single-target
    filter. Here the penalty must be zero.
    """
    def frames_to_converge(with_fp):
        mht = MultiHypothesisTracker()
        t0 = 0.0
        if with_fp:
            mht.process(0.0, np.array([1.0, 3.0, 2.0]))
            t0 = 1 / RATE_HZ
        for i, (t, z) in enumerate(_ball_stream(10, t0=t0)):
            mht.process(t, z)
            if any(_mht_vel_err(tr) < 0.5 for tr in mht.tracks):
                return i
        return None

    assert frames_to_converge(True) == frames_to_converge(False) == 1


def test_an_unexplained_centroid_starts_its_own_track():
    """A detection no live hypothesis can explain is a new object, not an
    outlier to be argued with."""
    mht = MultiHypothesisTracker()
    mht.process(0.0, np.array([1.0, 3.0, 2.0]))          # false positive
    assert mht.n_tracks == 1
    mht.process(1 / RATE_HZ, np.array([4.7, 0.0, 2.0]))  # ball, 4.7 m away
    assert mht.n_tracks == 2
    assert mht.n_spawned == 2


def test_a_detection_the_ball_track_explains_is_not_stolen_by_a_bloated_one():
    """Association ranks by likelihood, not raw Mahalanobis distance.

    A track that has coasted without evidence has a huge covariance, so raw
    Mahalanobis distance makes it look like a GOOD explanation for anything
    nearby — it would quietly steal the mature ball track's own detections and
    re-create the defect one level up. Constructed so the two rank oppositely:
    the ball track sees a real 0.15 m noise residual against a converged
    covariance (d2 ~ 1.7), the bloated one a 2.5 m residual against a 6 m sigma
    (d2 ~ 0.2). Likelihood includes log|S| and prefers the confident track.
    """
    rng = np.random.default_rng(7)
    mht = MultiHypothesisTracker(meas_std_m=0.15)
    stream = _ball_stream(8)
    for t, z in stream[:6]:
        mht.process(t, z + rng.normal(0.0, 0.15, 3))
    ball = mht.tracks[0]
    assert ball.n_updates == 6

    # A false positive 1.5 m off the path, left to coast for 0.3 s: bloated
    # enough that its gate covers the ball's next frame.
    t_fp = stream[5][0] - 0.3
    mht.process(t_fp, np.array([2.0, 1.5, 2.0]))
    assert mht.n_tracks == 2

    t, z = stream[6]
    mht.process(t, z + rng.normal(0.0, 0.15, 3))
    assert ball.n_updates == 7          # the ball kept its own detection
    assert mht.n_tracks == 2            # and no third hypothesis was invented


def test_a_silent_track_is_pruned_after_the_timeout():
    mht = MultiHypothesisTracker(track_timeout_s=0.5)
    mht.process(0.0, np.array([1.0, 3.0, 2.0]))
    assert mht.n_tracks == 1
    mht.process(0.6, np.array([4.7, 0.0, 2.0]))   # 0.6 s of silence for the FP
    assert mht.n_tracks == 1
    assert mht.n_pruned == 1


def test_confirmed_excludes_immature_tracks():
    mht = MultiHypothesisTracker()
    for t, z in _ball_stream(3):
        mht.process(t, z)
    assert mht.confirmed(min_updates=3) == mht.tracks
    assert mht.confirmed(min_updates=4) == []


def test_track_count_is_bounded_and_evicts_the_weakest():
    """A false-positive storm must not push the mature ball track out."""
    mht = MultiHypothesisTracker(max_tracks=3)
    for t, z in _ball_stream(5):
        mht.process(t, z)
    ball = mht.tracks[0]

    t_fp = _ball_stream(5)[-1][0]
    for i in range(6):  # six scattered one-shot detections
        mht.process(t_fp + 1e-3 * (i + 1),
                    np.array([-3.0 - 2.0 * i, 4.0 + 2.0 * i, 2.0]))

    assert mht.n_tracks == 3
    assert ball in mht.tracks
    assert ball.n_updates == 5


def test_reset_drops_every_hypothesis():
    mht = MultiHypothesisTracker()
    for t, z in _ball_stream(4):
        mht.process(t, z)
    mht.process(0.5, np.array([0.0, 5.0, 2.0]))   # inside track_timeout_s
    assert mht.n_tracks == 2
    mht.reset()
    assert mht.n_tracks == 0
    assert mht.confirmed(min_updates=1) == []
