"""
Unit tests for score_bags_logic.py — Task 5b, extended by Task 5c.

Runs without ROS (pure Python, no rclpy import anywhere in the module under
test): part of the ROS-free CI subset picked up automatically by
.github/workflows/tests.yml's deny-list (it imports nothing that needs ROS).

Three regression guards here are load-bearing, one per defect this file's
history is made of:

  test_build_bag_play_cmd_never_omits_exclude_topics — Defect 1 (5b). The
  original _replay_bag built a `ros2 bag play` command with no
  --exclude-topics at all, so a T-bag's own recorded /threat/centroid track
  replayed straight back onto the bus and score_bags counted it as a
  detection by the detector under test.

  test_compute_exclude_topics_rejects_partial_discovery — HIGH-1 (5c). The
  same defect, restored silently: a publisher enumeration of
  ['/parameter_events', '/rosout'] is non-empty and contains nothing
  forbidden, so it passed every guard while excluding nothing that matters.

  test_strict_window_rejects_cold_map_burst_at_bag_start — CRITICAL-1 (5c).
  5b's symmetric gate opened at (or within half a second of) bag start for
  every positive in the library, so the cold-background-map FP burst
  detector.yaml documents in the first ~1-2 s of every replay could pass a
  positive whose ball was never detected at all.
"""

import pytest

from huitzilin_perception.score_bags_logic import (
    DEFAULT_MIN_CLOSING_RUN,
    FORBIDDEN_EXCLUDE_TOPICS,
    LIBRARY_MAX_BALL_SPEED_MPS,
    MAX_DETECTION_GAP_S,
    POST_EVENT_TOLERANCE_S,
    REQUIRED_EXCLUDE_TOPICS,
    SPAWN_LEAD_S,
    V_AIRFRAME_MAX_MPS,
    attribute_closing_ball,
    build_bag_play_cmd,
    clock_msg_to_sim_t,
    closing_runs,
    compute_exclude_topics,
    is_in_window_loose,
    is_in_window_strict,
    is_in_window_symmetric_legacy,
    longest_closing_run,
    run_closure_rate,
    strict_window_bounds,
)


# ── compute_exclude_topics ──────────────────────────────────────────────────

def test_compute_exclude_topics_returns_sorted_deduped_list():
    result = compute_exclude_topics(["/threat/marker", "/threat/centroid",
                                      "/threat/centroid"])
    assert result == ["/threat/centroid", "/threat/marker"]


def test_compute_exclude_topics_rejects_clock():
    with pytest.raises(ValueError, match=r"/clock"):
        compute_exclude_topics(["/threat/centroid", "/clock"])


def test_compute_exclude_topics_rejects_oak_points():
    with pytest.raises(ValueError, match=r"/oak/points"):
        compute_exclude_topics(["/threat/centroid", "/oak/points"])


def test_compute_exclude_topics_rejects_empty_enumeration():
    # An empty exclude list is exactly the pre-fix behaviour (Defect 1) —
    # must never be produced silently.
    with pytest.raises(ValueError):
        compute_exclude_topics([])


def test_compute_exclude_topics_rejects_partial_discovery():
    # HIGH-1: the exact list rclpy's Node.__init__ produces before
    # detector_node.py has created /threat/centroid. Non-empty, nothing
    # forbidden — and useless as an exclusion.
    with pytest.raises(ValueError, match=r"/threat/centroid"):
        compute_exclude_topics(["/rosout", "/parameter_events"])


def test_compute_exclude_topics_rejects_any_list_without_centroid():
    # Not just the rclpy pair: ANY non-empty list missing the scored topic.
    with pytest.raises(ValueError, match=r"PARTIAL"):
        compute_exclude_topics(["/threat/marker"])


def test_forbidden_topics_are_exactly_clock_and_oak_points():
    # Locks the two topics the task-5b-brief's ambiguity resolution names
    # explicitly — nothing more, nothing less.
    assert FORBIDDEN_EXCLUDE_TOPICS == frozenset({"/clock", "/oak/points"})


def test_required_topics_are_exactly_the_scored_topic():
    assert REQUIRED_EXCLUDE_TOPICS == frozenset({"/threat/centroid"})


# ── build_bag_play_cmd — the regression test for Defect 1 itself ───────────

def test_build_bag_play_cmd_never_omits_exclude_topics():
    cmd = build_bag_play_cmd(
        "/data/huitzilin_bags/week3_T05_20260816.mcap",
        ["/threat/centroid", "/threat/marker"],
    )
    assert "--exclude-topics" in cmd
    idx = cmd.index("--exclude-topics")
    excluded = cmd[idx + 1:]
    assert "/threat/centroid" in excluded
    assert "/threat/marker" in excluded
    # A bag replaying its own recorded track is exactly Defect 1: assert the
    # command never plays with zero exclusions, which is what the original
    # _replay_bag always did.
    assert cmd.count("--exclude-topics") == 1


def test_build_bag_play_cmd_refuses_empty_exclude_list():
    with pytest.raises(ValueError):
        build_bag_play_cmd("/data/huitzilin_bags/week3_T05.mcap", [])


def test_build_bag_play_cmd_includes_bag_path_and_clock_flag():
    cmd = build_bag_play_cmd("/tmp/bag.mcap", ["/threat/centroid"])
    assert "/tmp/bag.mcap" in cmd
    assert "--clock" in cmd


# ── clock_msg_to_sim_t (Task 5c MEDIUM-4) ──────────────────────────────────
#
# This one line fixes the absolute position of every window in the library.
# A nanosecond-scaling slip would move all seventeen in lockstep.

def test_clock_msg_to_sim_t_combines_sec_and_nanosec():
    assert clock_msg_to_sim_t(1036, 480_000_000) == pytest.approx(1036.48)


def test_clock_msg_to_sim_t_handles_zero():
    # A legitimate 0.0 bag start must not be confused with "no clock".
    assert clock_msg_to_sim_t(0, 0) == 0.0


def test_clock_msg_to_sim_t_nanosec_scale_is_1e9_not_1e6():
    # 1 ms expressed in nanoseconds must read as 0.001 s, not 1.0 s.
    assert clock_msg_to_sim_t(0, 1_000_000) == pytest.approx(0.001)


# ── strict_window_bounds (Task 5c CRITICAL-1) ──────────────────────────────

def test_strict_window_lower_bound_never_precedes_spawn():
    # THE required guard. window_s (4.0) exceeds every time_to_closest_s in
    # the library (0.4-1.5), so an unfloored `event - window_s` lands at or
    # before bag start for every single positive. The floor must hold for
    # all of them.
    bag_start = 100.0
    for ttc in (0.4, 0.43, 0.5, 0.7, 0.75, 0.875, 1.1, 1.25, 1.5):
        lower, _upper = strict_window_bounds(bag_start, ttc, window_s=4.0)
        assert lower >= bag_start + SPAWN_LEAD_S, (
            f"ttc={ttc}: window opens {bag_start + SPAWN_LEAD_S - lower:.3f}s "
            f"before the projectile exists"
        )


def test_strict_window_floor_holds_for_any_window_width():
    for window_s in (0.5, 1.0, 4.0, 40.0):
        lower, _u = strict_window_bounds(100.0, 0.75, window_s=window_s)
        assert lower >= 100.0 + SPAWN_LEAD_S


def test_strict_window_is_asymmetric_about_the_event():
    # Generous before closest approach (a closing ball is legitimately
    # visible there), tight after (it has passed, or hit the ground).
    lower, upper = strict_window_bounds(100.0, 1.5, window_s=4.0)
    event = 100.0 + SPAWN_LEAD_S + 1.5
    assert event - lower == pytest.approx(1.5)        # floored at spawn
    assert upper - event == pytest.approx(POST_EVENT_TOLERANCE_S)
    assert (upper - lower) < 2 * 4.0                  # narrower than 5b's 8 s


def test_strict_window_is_a_subset_of_the_symmetric_gate():
    # The new gate can only remove passes, never add them: every scenario's
    # bounds sit inside the old +/- window_s interval.
    for ttc in (0.43, 0.75, 1.5):
        lower, upper = strict_window_bounds(100.0, ttc, window_s=4.0)
        event = 100.0 + SPAWN_LEAD_S + ttc
        assert lower >= event - 4.0
        assert upper <= event + 4.0


def test_strict_window_upper_never_exceeds_sidecar_window():
    # A sidecar that ships a window_s TIGHTER than POST_EVENT_TOLERANCE_S
    # still binds — min(), not a fixed 2 s.
    _lower, upper = strict_window_bounds(100.0, 0.75, window_s=0.5)
    assert upper == pytest.approx(100.0 + SPAWN_LEAD_S + 0.75 + 0.5)


def test_strict_window_requires_bag_start():
    with pytest.raises(ValueError):
        strict_window_bounds(None, 0.7, window_s=4.0)


def test_strict_window_requires_time_to_closest_s():
    # This is the exact shape of Defect 2's silent fallback: a positive
    # sidecar with no time_to_closest_s must error, not quietly pass.
    with pytest.raises(ValueError):
        strict_window_bounds(100.0, None, window_s=4.0)


# ── is_in_window_strict ─────────────────────────────────────────────────────

def test_strict_window_true_near_closest_approach():
    # bag_start=100, spawn_lead=3 (default), ttc=0.7 -> event at 103.7
    assert is_in_window_strict(
        [103.9], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is True


def test_strict_window_true_for_detection_slightly_before_closest_approach():
    # A detection just before the ball reaches closest approach is a
    # legitimate early catch, not a miss. event=103.7, floor=103.0.
    assert is_in_window_strict(
        [103.5], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is True


def test_strict_window_rejects_cold_map_burst_at_bag_start():
    # CRITICAL-1, stated as the failure it prevents. detector.yaml:229-231
    # documents this library's dominant FP class as a cold-background-map
    # burst in the first ~1-2 s of a replay. Here the detector NEVER sees
    # the ball; all it emits is that burst. Under Task 5b's symmetric gate
    # this scored a true positive and read as 100% recall.
    cold_burst = [100.05, 100.31, 100.62, 100.94, 101.27, 101.40]
    assert is_in_window_symmetric_legacy(
        cold_burst, bag_start=100.0, time_to_closest_s=0.75, window_s=4.0
    ) is True                                     # the bug, reproduced
    assert is_in_window_strict(
        cold_burst, bag_start=100.0, time_to_closest_s=0.75, window_s=4.0
    ) is False                                    # the fix


def test_strict_window_rejects_detection_one_tick_before_spawn():
    just_before_spawn = [100.0 + SPAWN_LEAD_S - 0.001]
    assert is_in_window_strict(
        just_before_spawn, bag_start=100.0, time_to_closest_s=0.75,
        window_s=4.0,
    ) is False


def test_strict_window_rejects_late_background_the_old_gate_accepted():
    # 3.5 s after the nominal event: inside 5b's +/-4 s tolerance, outside
    # the 2 s the ball's own physics can justify.
    late = [100.0 + SPAWN_LEAD_S + 0.75 + 3.5]
    assert is_in_window_symmetric_legacy(
        late, bag_start=100.0, time_to_closest_s=0.75, window_s=4.0
    ) is True
    assert is_in_window_strict(
        late, bag_start=100.0, time_to_closest_s=0.75, window_s=4.0
    ) is False


def test_strict_window_uses_spawn_lead_s_by_default():
    # Without the spawn lead, event_t would be 100.7 and the true detection
    # at 103.7 would read as 3 s away. Use a window small enough to
    # discriminate: window_s=1.0.
    detection_at_true_event = [100.0 + SPAWN_LEAD_S + 0.7]  # = 103.7 exactly
    assert is_in_window_strict(
        detection_at_true_event, bag_start=100.0, time_to_closest_s=0.7,
        window_s=1.0,
    ) is True
    assert abs(103.7 - 100.7) > 1.0


def test_strict_window_false_far_from_closest_approach_but_inside_loose_window():
    # The discriminator the 5b brief asked for: a detection that passes the
    # loose (bag-start-anchored) gate but is nowhere near when the ball was
    # actually present must fail the strict gate.
    detections = [100.5]  # 0.5s after bag_start=100 -> loose passes trivially
    assert is_in_window_loose(detections, bag_start=100.0, window_s=4.0) is True
    assert is_in_window_strict(
        detections, bag_start=100.0, time_to_closest_s=8.0, window_s=4.0
    ) is False


def test_strict_window_false_when_no_detections():
    assert is_in_window_strict(
        [], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is False


# ── superseded gates, kept for artifact comparison only ────────────────────

def test_loose_window_true_when_detection_within_window_of_bag_start():
    assert is_in_window_loose([102.0], bag_start=100.0, window_s=4.0) is True


def test_loose_window_false_when_detection_after_window():
    assert is_in_window_loose([106.0], bag_start=100.0, window_s=4.0) is False


def test_loose_window_requires_bag_start():
    with pytest.raises(ValueError):
        is_in_window_loose([102.0], bag_start=None, window_s=4.0)


def test_loose_window_false_when_no_detections():
    assert is_in_window_loose([], bag_start=100.0, window_s=4.0) is False


def test_loose_window_has_no_lower_bound_as_documented():
    # LOW-4: the expression accepts t < bag_start. Locked so the docstring
    # and the computation cannot drift apart again.
    assert is_in_window_loose([99.0], bag_start=100.0, window_s=4.0) is True


def test_symmetric_legacy_gate_reproduces_task_5b_exactly():
    assert is_in_window_symmetric_legacy(
        [100.0], bag_start=100.0, time_to_closest_s=0.43, window_s=4.0
    ) is True   # -0.57 s relative to bag start: 5b's real lower edge


# ── closing_runs / longest_closing_run ─────────────────────────────────────

def test_longest_closing_run_empty_is_zero():
    assert longest_closing_run([]) == 0


def test_longest_closing_run_single_point_is_one():
    assert longest_closing_run([(100.0, 3.0)]) == 1


def test_longest_closing_run_detects_decreasing_sequence():
    ranges = [(100.0, 4.0), (100.067, 3.5), (100.134, 3.0), (100.201, 2.5)]
    assert longest_closing_run(ranges) == 4


def test_longest_closing_run_ignores_out_of_order_input():
    ranges = [(100.201, 2.5), (100.0, 4.0), (100.134, 3.0), (100.067, 3.5)]
    assert longest_closing_run(ranges) == 4


def test_longest_closing_run_breaks_on_non_decrease():
    ranges = [(100.0, 4.0), (100.067, 4.2), (100.134, 4.5), (100.201, 4.8)]
    assert longest_closing_run(ranges) == 1


def test_longest_closing_run_ties_break_the_run():
    ranges = [(100.0, 3.0), (100.067, 3.0), (100.134, 2.5)]
    assert longest_closing_run(ranges) == 2  # only the 3.0 -> 2.5 step


def test_longest_closing_run_finds_longest_among_several():
    ranges = [
        (100.0, 5.0), (100.067, 4.0),
        (101.0, 6.0), (101.067, 5.5), (101.134, 5.0), (101.201, 4.5),
    ]
    assert longest_closing_run(ranges) == 4


def test_closing_run_is_broken_by_a_time_gap():
    # Task 5c: without a gap bound, two detections seconds apart counted as
    # one "closing step" — Task 5b's 8 s window made that routine.
    ranges = [(100.0, 5.0), (100.067, 4.0), (106.0, 3.0)]
    assert longest_closing_run(ranges) == 2


def test_max_gap_is_never_the_operative_constraint_on_an_in_band_step():
    # The gap bound is derived so that it CANNOT decide an attribution: any
    # step closing faster than the airframe maximum, between two ranges
    # inside a roi_max_range_m-deep ROI, is necessarily shorter than it.
    # Locking this is what stops the constant from ever being quietly
    # retuned into a discriminator. (An earlier draft derived it from
    # cloud_queue_depth as 0.4 s, which modelled dropped clouds rather than
    # missed detections and WAS the operative constraint on eight of ten
    # tune positives.)
    assert MAX_DETECTION_GAP_S == pytest.approx(5.00 / V_AIRFRAME_MAX_MPS)
    longest_possible_in_band_step = 5.00 / V_AIRFRAME_MAX_MPS
    assert MAX_DETECTION_GAP_S >= longest_possible_in_band_step


def test_closing_runs_partitions_every_point_exactly_once():
    ranges = [(100.0, 5.0), (100.067, 4.0), (100.134, 4.5), (100.201, 3.0)]
    runs = closing_runs(ranges)
    assert sum(len(r) for r in runs) == len(ranges)


def test_run_closure_rate_is_end_to_end():
    run = [(100.0, 5.0), (100.067, 4.5), (100.134, 4.0)]
    assert run_closure_rate(run) == pytest.approx(1.0 / 0.134, rel=1e-3)


def test_run_closure_rate_of_short_run_is_zero():
    assert run_closure_rate([(100.0, 5.0)]) == 0.0
    assert run_closure_rate([]) == 0.0


# ── attribute_closing_ball (Task 5c CRITICAL-2) ────────────────────────────
#
# The refuted rule tested only the SIGN of successive range differences.
# Under patrol the drone translates toward newly-explored terrain, whose
# base_link range falls monotonically frame after frame — so the sign test
# selects FOR this library's dominant FP class. These tests pin the two
# populations apart by RATE.

def _sequence(start_range, rate_mps, n, hz=15.0, t0=100.0):
    """n points closing at exactly rate_mps, sampled at the depth-cloud rate."""
    dt = 1.0 / hz
    return [(t0 + i * dt, start_range - rate_mps * i * dt) for i in range(n)]


def test_airframe_speed_closing_run_is_not_attributed():
    # THE required negative case: a textbook clean, strictly decreasing,
    # time-contiguous run of 5 points — which the 5b rule attributes with no
    # hesitation — closing at 2.09 m/s, the MEDIAN measured patrol speed.
    # This is a rock the drone is flying at, and it must not read as a ball.
    egomotion = _sequence(4.5, 2.09, 5)
    assert longest_closing_run(egomotion) == 5          # 5b would attribute
    result = attribute_closing_ball(egomotion, ball_speed_mps=8.0)
    assert result["attributable"] is False
    assert "m/s" in result["reason"]


def test_the_decision_boundary_sits_at_the_airframe_maximum():
    # 3.49 m/s is the measured rolling-max airframe ground speed, so
    # anything at or below it is achievable by egomotion over static
    # terrain and must be excluded; anything above it is not. Probed either
    # side rather than exactly on it — a knife-edge float comparison at the
    # boundary is not a meaningful physical claim.
    just_below = _sequence(4.5, V_AIRFRAME_MAX_MPS - 0.05, 5)
    just_above = _sequence(4.5, V_AIRFRAME_MAX_MPS + 0.05, 5)
    assert attribute_closing_ball(just_below, ball_speed_mps=8.0)[
        "attributable"] is False
    assert attribute_closing_ball(just_above, ball_speed_mps=8.0)[
        "attributable"] is True


def test_ball_speed_closing_run_is_attributed():
    # THE required positive case: same shape, same length, closing at 8 m/s
    # — the library's primary design-point ball speed (S02/S07/S09).
    ball = _sequence(4.8, 8.0, 5)
    result = attribute_closing_ball(ball, ball_speed_mps=8.0)
    assert result["attributable"] is True
    assert result["best_run"] >= DEFAULT_MIN_CLOSING_RUN
    assert result["closure_rate"] == pytest.approx(8.0, rel=1e-6)


def test_slow_ball_against_a_closing_airframe_is_attributed():
    # The thin-margin end of the library (S01/S10, 4 m/s). A head-on 4 m/s
    # ball seen from a drone flying toward it closes faster than either
    # alone, which is why these scenarios can still separate.
    slow = _sequence(4.8, 4.0 + 2.0, 4)
    assert attribute_closing_ball(slow, ball_speed_mps=4.0)[
        "attributable"] is True


def test_range_jumps_faster_than_physics_are_not_attributed():
    # An FP chain: successive detections on unrelated objects at unrelated
    # depths. Sign-only says "closing"; the rate says 30 m/s, which no ball
    # in this library plus the airframe can produce.
    jumpy = _sequence(6.0, 30.0, 4)
    result = attribute_closing_ball(jumpy, ball_speed_mps=8.0)
    assert result["attributable"] is False


def test_one_wild_step_does_not_carry_a_static_run_into_the_band():
    # Every STEP must be in band, not just the endpoints: two near-static
    # steps plus one impossible jump average into the band but are not a
    # ball.
    ranges = [(100.0, 5.00), (100.067, 4.99), (100.134, 1.00),
              (100.201, 0.99)]
    assert attribute_closing_ball(ranges, ball_speed_mps=8.0)[
        "attributable"] is False


def test_short_run_at_ball_speed_is_not_attributed():
    # Two points is one step — below DEFAULT_MIN_CLOSING_RUN.
    assert attribute_closing_ball(_sequence(4.8, 8.0, 2), 8.0)[
        "attributable"] is False


def test_attribution_needs_contiguous_points():
    # Same three ranges, same rate on paper, but spread over seconds: not
    # one object tracked frame to frame.
    spread = [(100.0, 5.0), (102.0, 4.0), (104.0, 3.0)]
    assert attribute_closing_ball(spread, ball_speed_mps=8.0)[
        "attributable"] is False


def test_attribution_reports_the_numbers_the_artifact_needs():
    # HIGH-2: everything the run artifact prints must come out of here.
    result = attribute_closing_ball(_sequence(4.8, 8.0, 5), 8.0)
    for key in ("attributable", "longest_run", "best_run", "closure_rate",
                "rate_band", "n_points", "reason"):
        assert key in result
    assert result["n_points"] == 5
    assert result["rate_band"] == (V_AIRFRAME_MAX_MPS,
                                   8.0 + V_AIRFRAME_MAX_MPS)


def test_attribution_of_empty_detection_set():
    result = attribute_closing_ball([], ball_speed_mps=8.0)
    assert result["attributable"] is False
    assert result["longest_run"] == 0
    assert result["n_points"] == 0


def test_negative_control_band_is_the_library_maximum():
    # A ball-free bag is scored against the most permissive band any
    # scenario gets, so the control cannot be passed by making the test
    # narrow.
    assert LIBRARY_MAX_BALL_SPEED_MPS == 17.0
    result = attribute_closing_ball([], LIBRARY_MAX_BALL_SPEED_MPS)
    assert result["rate_band"] == (V_AIRFRAME_MAX_MPS,
                                   17.0 + V_AIRFRAME_MAX_MPS)


def test_out_of_band_step_splits_a_run_instead_of_destroying_it():
    # A qualifying 3-point run, one impossible jump, then another
    # qualifying 3-point run. Defining runs BY the band (rather than by
    # sign and then rate-checking whole sign-runs) keeps both halves, so a
    # single bad frame in the middle of a long track cannot veto a real
    # detection.
    first = _sequence(4.8, 8.0, 3, t0=100.0)
    second = _sequence(2.0, 8.0, 3, t0=100.30)
    assert attribute_closing_ball(first + second, 8.0)["attributable"] is True


def test_sign_only_run_at_airframe_speed_never_becomes_attributable():
    # The refuted 5b statistic and the 5c verdict must disagree here — that
    # disagreement is the whole point of the fix.
    ego = _sequence(4.9, 2.09, 8)
    result = attribute_closing_ball(ego, ball_speed_mps=17.0)
    assert result["longest_run"] == 8      # what 5b reported and trusted
    assert result["best_run"] < DEFAULT_MIN_CLOSING_RUN
    assert result["attributable"] is False


def test_min_closing_run_stays_at_three():
    # Raising it would make the hardest positives unattributable by
    # construction: the matrix's own dwell arithmetic gives 2.8 clouds for
    # T08 and 3.4 for the thinnest original.
    assert DEFAULT_MIN_CLOSING_RUN == 3
