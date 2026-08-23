"""Truth-attributed scoring: was a detection OF THE BALL? ROS-free.

WHY THIS EXISTS ALONGSIDE test_score_bags_logic.py. That file covers
`attribute_closing_ball`, which decides whether a run of detections MOVES like a
ball: in-band closing rate, floored at the airframe's own maximum ground speed
so egomotion over static terrain cannot produce it. It is a good discriminator
and it stays. It is not ground truth. It can only ever say "something closed at
a ball-like rate", never "that was the ball", and on a library whose dominant
false-positive class is newly-explored terrain along a patrol leg the difference
is exactly the thing under test.

This module answers the other question, and can only answer it because the bag
now records /gz/dynamic_poses (scripts/capture_scenario.sh). The match radius,
K and window here should not be re-chosen after seeing a result on the split
they are meant to score.

The control that matters most is `test_a_ball_free_scenario_matches_nothing`:
feed real detections and an empty projectile track, and the scorer must return
zero matches. A scorer that cannot fail cannot measure.
"""

import math

import pytest

from huitzilin_perception.truth_attribution import (
    DEFAULT_MIN_MATCHED,
    MATCH_FLOOR_M,
    MATCH_SIGMAS,
    STAMP_SKEW_S,
    interpolate_track,
    match_radius_m,
    score_scenario,
    void_reason,
)

# A hovering drone at the origin, sampled far faster than the cloud, as the gz
# pose stream is. Held still so a detection's range is just its own norm.
DRONE = [(t / 100.0, (0.0, 0.0, 2.0)) for t in range(0, 1000)]
WINDOW = (1.0, 4.0)


def _ball_closing(speed_mps, t0=0.0, t1=5.0, step=0.01, start_m=30.0):
    """Truth track for a ball flying straight down +x toward the drone."""
    out = []
    t = t0
    while t <= t1 + 1e-9:
        out.append((t, (start_m - speed_mps * (t - t0), 0.0, 2.0)))
        t += step
    return out


def _at(track, t):
    return interpolate_track(track, t)


# -- the match radius --------------------------------------------------------

def test_match_radius_at_the_reference_range_is_three_sigma():
    """sigma(26) = 0.30 by definition of the lane's error model, so 3 sigma is
    0.90 m and the 0.50 m floor does not bind."""
    assert match_radius_m(26.0, 0.0) == pytest.approx(MATCH_SIGMAS * 0.30)


def test_match_radius_at_short_range_is_the_floor():
    """sigma(5) is ~0.011 m, so 3 sigma is ~0.033 m. Without the floor the gate
    would be tighter than voxelization (0.02 m) and the ball's own 0.08 m
    extent, and would reject correct detections as unmatched."""
    assert match_radius_m(5.0, 0.0) == pytest.approx(MATCH_FLOOR_M)


def test_match_radius_adds_one_frame_of_stamp_skew():
    """20 m/s x 20 ms = 0.40 m. Not negligible at the speed under study, and a
    constant chosen up front rather than fitted to a miss."""
    assert (match_radius_m(26.0, 20.0) - match_radius_m(26.0, 0.0)
            == pytest.approx(20.0 * STAMP_SKEW_S))


def test_match_radius_grows_with_range():
    radii = [match_radius_m(z, 0.0) for z in (5.0, 15.0, 26.0, 34.0)]
    assert radii == sorted(radii)
    assert radii[0] < radii[-1]


# -- truth interpolation -----------------------------------------------------

def test_interpolation_hits_a_sample_exactly():
    track = [(0.0, (0.0, 0.0, 0.0)), (1.0, (10.0, 0.0, 0.0))]
    assert _at(track, 1.0) == pytest.approx((10.0, 0.0, 0.0))


def test_interpolation_is_linear_between_samples():
    track = [(0.0, (0.0, 0.0, 0.0)), (1.0, (10.0, 20.0, -4.0))]
    assert _at(track, 0.25) == pytest.approx((2.5, 5.0, -1.0))


def test_interpolation_refuses_to_extrapolate():
    """A detection outside the truth track's span has NO truth, and inventing
    one by holding the last sample would manufacture a match for a detection of
    a ball that has already come to rest and left the dynamic-pose stream."""
    track = [(1.0, (0.0, 0.0, 0.0)), (2.0, (1.0, 0.0, 0.0))]
    assert _at(track, 0.999) is None
    assert _at(track, 2.001) is None


def test_interpolation_of_an_empty_track_is_none():
    assert _at([], 1.0) is None


# -- scoring a positive ------------------------------------------------------

def _score(detections, ball, speed=20.0, **kw):
    return score_scenario(detections=detections, ball_track=ball,
                          drone_track=DRONE, ball_speed_mps=speed,
                          window=WINDOW, **kw)


def test_a_perfectly_tracked_ball_is_recalled():
    ball = _ball_closing(20.0)
    dets = [(t, _at(ball, t)) for t in (1.5, 1.6, 1.7, 1.8)]
    out = _score(dets, ball)
    assert out["matched"] == 4
    assert out["unmatched"] == 0
    assert out["recalled"] is True
    assert max(out["errors_m"]) == pytest.approx(0.0, abs=1e-9)


def test_a_ball_free_scenario_matches_nothing():
    """THE CONTROL. Real detections, no projectile in the world: every one of
    them must land in `unmatched`, and recall must be false. If this ever
    passes with matches, every recall number the scorer has produced is void,
    because it means detections match something that is not there."""
    dets = [(t, (10.0, 0.0, 2.0)) for t in (1.5, 1.6, 1.7, 1.8, 1.9)]
    out = _score(dets, [])
    assert out["matched"] == 0
    assert out["unmatched"] == 5
    assert out["recalled"] is False


def test_detections_of_the_wrong_object_do_not_match():
    """Terrain 8 m off the ball's path, closing at a ball-like rate because the
    drone is flying toward it. This is the case the kinematic attribution
    cannot separate and truth can."""
    ball = _ball_closing(20.0)
    dets = [(t, (_at(ball, t)[0], 8.0, 2.0)) for t in (1.5, 1.6, 1.7)]
    out = _score(dets, ball)
    assert out["matched"] == 0
    assert out["unmatched"] == 3
    assert out["recalled"] is False


def test_a_detection_of_the_drone_itself_is_a_false_positive():
    """Amendment A1. The original wording -- "not matched to any truth entity"
    -- would have excused a detector reporting the drone's own standoffs as an
    incoming threat, because the airframe IS a truth entity. Only the
    projectile counts as a match."""
    # Times chosen while the ball is still 6-10 m out. At t = 1.5 s this ball
    # is AT the origin, and a detection there would match it correctly -- the
    # ball really is at the drone at that instant. The failure mode under test
    # is the detector reporting its own airframe while the ball is elsewhere.
    ball = _ball_closing(20.0)
    dets = [(t, (0.05, 0.0, 2.0)) for t in (1.0, 1.1, 1.2)]
    out = _score(dets, ball)
    assert out["matched"] == 0
    assert out["unmatched"] == 3


def test_the_match_boundary_is_inclusive_and_bites():
    """A detection at exactly R matches; a hair beyond does not.

    R IS COMPUTED FROM THE DETECTION'S OWN RANGE, not from truth's -- section
    3.2 says so, because at scoring time truth is what you are testing against
    and the range you can always attribute to a detection is its own. That has
    a consequence worth stating: displacing a detection away from truth also
    grows its gate slightly, so a naive "truth + 0.1 % more than R" point can
    still match. The boundary is therefore found as the fixed point of
    d = R(z_truth + d) along the line of sight, which is the displacement at
    which error and gate are exactly equal.
    """
    ball = _ball_closing(20.0, start_m=52.0)
    t = 1.5
    truth = _at(ball, t)
    drone = (0.0, 0.0, 2.0)
    z_truth = math.dist(truth, drone)
    # Range-dependent arm of the gate, not the 0.50 m floor.
    assert MATCH_SIGMAS * 0.30 * (z_truth / 26.0) ** 2 > MATCH_FLOOR_M

    unit = tuple((a - b) / z_truth for a, b in zip(truth, drone))
    d = match_radius_m(z_truth, 20.0)
    for _ in range(50):
        d = match_radius_m(z_truth + d, 20.0)

    def along(dist):
        return tuple(a + dist * u for a, u in zip(truth, unit))

    assert math.dist(along(d), truth) == pytest.approx(
        match_radius_m(math.dist(along(d), drone), 20.0), rel=1e-9)
    assert _score([(t, along(d))], ball)["matched"] == 1
    assert _score([(t, along(d * 1.01))], ball)["matched"] == 0


def test_detections_outside_the_window_are_not_counted_at_all():
    """The strict window exists because every replay opens with a cold
    background map and a false-positive burst. A detection outside it is
    neither a match nor a false positive HERE -- it belongs to a different
    measurement."""
    ball = _ball_closing(20.0)
    dets = [(t, _at(ball, t)) for t in (0.2, 0.5, 4.5)]
    out = _score(dets, ball)
    assert out["in_window"] == 0
    assert out["matched"] == 0
    assert out["unmatched"] == 0
    assert out["recalled"] is False


# -- K, and the two verdicts -------------------------------------------------

def test_k_comes_from_the_dodge_trigger():
    """min_track_updates is 3 in evasion.yaml: three confirmations before a
    dodge may fire. A recall metric with K=1 would count a detection the
    aircraft cannot act on."""
    assert DEFAULT_MIN_MATCHED == 3


def test_two_matches_are_not_a_recall_and_three_are():
    ball = _ball_closing(20.0)
    two = [(t, _at(ball, t)) for t in (1.5, 1.6)]
    three = [(t, _at(ball, t)) for t in (1.5, 1.6, 1.7)]
    assert _score(two, ball)["recalled"] is False
    assert _score(three, ball)["recalled"] is True


def test_a_negative_fires_on_k_unmatched_detections():
    """The false-positive verdict, and it uses the same K so that "enough to
    act on" means one thing across both metrics."""
    dets = [(t, (10.0, 0.0, 2.0)) for t in (1.5, 1.6, 1.7)]
    out = _score(dets, [])
    assert out["fired"] is True
    assert _score(dets[:2], [])["fired"] is False


def test_recall_and_false_positives_are_reported_separately():
    """A scenario can be BOTH recalled and emitting spurious centroids, and
    that is a real defect the artifact has to be able to show. Neither verdict
    may cancel the other."""
    ball = _ball_closing(20.0)
    dets = ([(t, _at(ball, t)) for t in (1.5, 1.6, 1.7)]
            + [(t, (2.0, 9.0, 2.0)) for t in (1.55, 1.65, 1.75)])
    out = _score(dets, ball)
    assert out["recalled"] is True
    assert out["fired"] is True
    assert out["matched"] == 3
    assert out["unmatched"] == 3


# -- void --------------------------------------------------------------------

def test_a_positive_without_a_projectile_track_is_void():
    """Void is neither a pass nor a fail: it means the bag cannot answer the
    question. It is re-captured, and it stays in the denominator."""
    assert void_reason("positive", ball_track=[], drone_track=DRONE)


def test_a_negative_without_a_projectile_track_is_not_void():
    """Most negatives have no projectile ON PURPOSE. Voiding them would delete
    the false-positive measurement entirely."""
    assert void_reason("negative", ball_track=[], drone_track=DRONE) is None


def test_any_scenario_without_a_drone_track_is_void():
    """Amendment A2. The match radius needs the detection's range from the
    camera, which comes from the drone's own transform. gz publishes MOVING
    entities, so a sufficiently still airframe can drop out of the stream and
    take the whole scoring basis with it."""
    ball = _ball_closing(20.0)
    assert void_reason("positive", ball_track=ball, drone_track=[])
    assert void_reason("negative", ball_track=[], drone_track=[])


def test_a_complete_positive_is_not_void():
    assert void_reason("positive", ball_track=_ball_closing(20.0),
                       drone_track=DRONE) is None


# -- the accuracy distribution, which is not the match gate ------------------

def test_errors_are_reported_for_matched_detections_only():
    """R is a MATCHING tolerance, not an accuracy claim. The artifact carries
    the error distribution so a system that scrapes in under R on every frame
    cannot hide behind a pass -- but an unmatched detection has no meaningful
    error to the ball, so it contributes none."""
    ball = _ball_closing(20.0)
    dets = ([(t, _at(ball, t)) for t in (1.5, 1.6)]
            + [(1.7, (2.0, 9.0, 2.0))])
    out = _score(dets, ball)
    assert len(out["errors_m"]) == out["matched"] == 2
    assert all(e == pytest.approx(0.0, abs=1e-9) for e in out["errors_m"])
