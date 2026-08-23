"""Unit tests for score_bags_logic.py.

Runs without ROS (pure Python, no rclpy import anywhere in the module under
test): part of the ROS-free CI subset picked up automatically by
.github/workflows/tests.yml's deny-list (it imports nothing that needs ROS).

Three guards here are load-bearing, one per way this scorer has been caught
scoring itself:

  test_build_bag_play_cmd_never_omits_exclude_topics — _replay_bag once built
  a `ros2 bag play` command with no --exclude-topics at all, so a T-bag's own
  recorded /threat/centroid track replayed straight back onto the bus and
  score_bags counted it as a detection by the detector under test.

  test_compute_exclude_topics_rejects_partial_discovery — the same defect,
  restored silently: a publisher enumeration of ['/parameter_events',
  '/rosout'] is non-empty and contains nothing forbidden, so it passes every
  guard while excluding nothing that matters.

  test_strict_window_rejects_cold_map_burst_at_bag_start — the older symmetric
  gate opened at (or within half a second of) bag start for every positive in
  the library, so the cold-background-map false-positive burst in the first
  ~1-2 s of every replay could pass a positive whose ball was never detected.
"""

import math
import os
import sys

import pytest

from huitzilin_perception.score_bags_logic import (
    DEFAULT_MIN_CLOSING_RUN,
    FLAT_THROW_FALL_TIME_S,
    FORBIDDEN_EXCLUDE_TOPICS,
    G_MPS2,
    HOVER_ALTITUDE_M,
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
    count_in_window,
    flat_throw_fall_time_s,
    format_closure_rate,
    is_in_window_loose,
    is_in_window_strict,
    is_in_window_symmetric_legacy,
    longest_closing_run,
    run_closure_rate,
    strict_window_bounds,
    symmetric_window_bounds,
)

# The Monte Carlo harness lives beside this file and is not a pytest module.
# Imported by explicit path rather than as a sibling so the import does not
# depend on pytest's import mode (the ROS-free CI subset runs it with a
# PYTHONPATH, colcon runs it another way).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import null_attribution_mc  # noqa: E402

# Every time_to_closest_s in the shipped library (scenario_matrix.yaml),
# positives and negatives, originals and tune set. Several tests below are
# properties that must hold across all of them.
LIBRARY_TTCS = (0.0, 0.4, 0.43, 0.5, 0.7, 0.75, 0.875, 1.1, 1.25, 1.5)


# --- compute_exclude_topics --------------------------------------------------

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
    # Locks the forbidden set exactly: excluding either of these breaks the
    # replay itself rather than isolating the detector. Nothing more, nothing
    # less.
    assert FORBIDDEN_EXCLUDE_TOPICS == frozenset({"/clock", "/oak/points"})


def test_required_topics_are_exactly_the_scored_topic():
    assert REQUIRED_EXCLUDE_TOPICS == frozenset({"/threat/centroid"})


# --- build_bag_play_cmd: the replay must never feed the bag's own track back -

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


# --- clock_msg_to_sim_t -------------------------------------------------------
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


# --- strict_window_bounds -----------------------------------------------------

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
    spawn = 100.0 + SPAWN_LEAD_S
    event = spawn + 1.5
    assert event - lower == pytest.approx(1.5)        # floored at spawn
    # ttc 1.5 s > the 0.639 s flat-throw fall time, so the tolerance hangs
    # off ground impact, not off an event that never happens (HIGH-3).
    assert upper - spawn == pytest.approx(
        FLAT_THROW_FALL_TIME_S + POST_EVENT_TOLERANCE_S)
    assert upper < event + POST_EVENT_TOLERANCE_S
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
    # still binds — min(), not a fixed 2 s. (Vacuous for the shipped
    # library, where every sidecar carries 4.0; kept because a sidecar is
    # data. See strict_window_bounds' docstring.)
    _lower, upper = strict_window_bounds(100.0, 0.5, window_s=0.25)
    assert upper == pytest.approx(100.0 + SPAWN_LEAD_S + 0.5 + 0.25)


# --- the late edge is clamped at ground impact -------------------------------

def test_flat_throw_fall_time_is_the_documented_physics():
    # sqrt(2h/g) from a 2 m hover, the same arithmetic scenario_matrix.yaml's
    # own T01 note quotes. Not a tuned number: h is bridge.yaml's
    # takeoff_alt_m and g is g.
    assert FLAT_THROW_FALL_TIME_S == pytest.approx(
        math.sqrt(2.0 * HOVER_ALTITUDE_M / G_MPS2))
    assert FLAT_THROW_FALL_TIME_S == pytest.approx(0.639, abs=5e-4)


def test_fall_time_is_none_for_the_vertical_offset_scenarios():
    # N04 (-10.0 m) and T13 (-7.5 m) are not thrown from the standard 2 m
    # hover at all, so the flat-throw model does not describe them and the
    # caller must fall back rather than invent a drop height.
    assert flat_throw_fall_time_s(-10.0) is None
    assert flat_throw_fall_time_s(-7.5) is None
    assert flat_throw_fall_time_s(0.0) == pytest.approx(FLAT_THROW_FALL_TIME_S)


def test_late_edge_hangs_off_ground_impact_when_the_ball_lands_first():
    # S01 (ttc 1.50) and S10 (1.25): the ball is on the ground at 0.639 s,
    # so the nominal closest approach never happens and the tolerance must
    # not be measured from it.
    spawn = 100.0 + SPAWN_LEAD_S
    for ttc in (1.5, 1.25, 1.1):
        _l, upper = strict_window_bounds(100.0, ttc, window_s=4.0)
        assert upper - spawn == pytest.approx(
            FLAT_THROW_FALL_TIME_S + POST_EVENT_TOLERANCE_S)


def test_late_edge_is_unchanged_when_closest_approach_precedes_impact():
    # S03/S06/S08 (ttc 0.43) and S12 (0.50) reach closest approach while
    # still airborne, so the clamp is inert for them — the rule is not a
    # blanket shortening.
    spawn = 100.0 + SPAWN_LEAD_S
    for ttc in (0.4, 0.43, 0.5):
        _l, upper = strict_window_bounds(100.0, ttc, window_s=4.0)
        assert upper - spawn == pytest.approx(ttc + POST_EVENT_TOLERANCE_S)


def test_the_new_late_edge_is_a_strict_tightening_and_must_stay_one():
    # The clamped window must be a SUBSET of the unclamped one, for every
    # scenario in the library and for anything anyone adds -- a rule that
    # could ever admit a detection the previous rule rejected would be a
    # loosening, and a loosening cannot be justified by physics that only
    # ever removes flight time.
    # Fails loudly if min() is ever turned into max(), dropped, or given a
    # larger fall time.
    for ttc in LIBRARY_TTCS + (0.05, 2.0, 3.9, 12.0):
        for window_s in (0.25, 1.0, 4.0, 40.0):
            spawn = 100.0 + SPAWN_LEAD_S
            try:
                lower, upper = strict_window_bounds(
                    100.0, ttc, window_s=window_s)
            except ValueError:
                # An inverted window is refused outright. Refusing admits
                # nothing, so it cannot possibly be a loosening -- but
                # assert independently that it only
                # happens where the un-guarded arithmetic really does
                # invert, so a raise can never become a way to hide a bug.
                assert (min(ttc, FLAT_THROW_FALL_TIME_S)
                        + min(window_s, POST_EVENT_TOLERANCE_S)
                        < max(ttc - window_s, 0.0)), (
                    f"ttc={ttc} window_s={window_s}: raised, but the window "
                    f"is not inverted"
                )
                continue
            previous_upper = (
                spawn + ttc + min(window_s, POST_EVENT_TOLERANCE_S)
            )
            assert upper <= previous_upper + 1e-12, (
                f"ttc={ttc} window_s={window_s}: late edge LOOSENED by "
                f"{upper - previous_upper:.4f}s"
            )
            # ...and the floor and the anchor did not move with it.
            assert lower == pytest.approx(
                max(spawn + ttc - window_s, spawn))


def test_fall_time_none_restores_the_nominal_event_edge():
    # The documented per-scenario fallback: a geometry the flat-throw model
    # does not describe gets exactly the previous rule, not a guess.
    spawn = 100.0 + SPAWN_LEAD_S
    _l, upper = strict_window_bounds(100.0, 1.5, window_s=4.0,
                                     fall_time_s=None)
    assert upper == pytest.approx(spawn + 1.5 + POST_EVENT_TOLERANCE_S)


def test_late_clamp_never_pushes_the_upper_edge_below_the_lower():
    # This test used to sweep ttc at window_s = 4.0
    # ONLY, so it asserted a property over a grid on which the property is
    # unconditionally true and passed without ever visiting the region where
    # it is false. Same shape as the M-7/F-5 defect. The grid now sweeps
    # window_s down through the crossover.
    #
    # The property below the crossover is NOT "the window is still ordered";
    # it is "the function refuses, loudly". Anything else — an empty
    # interval, a degenerate point — scores the positive FN with no error.
    exercised_a_raise = False
    for ttc in LIBRARY_TTCS:
        for window_s in (0.1, 0.2, 0.25, 0.3, 0.43, 0.5, 1.0, 2.0, 4.0):
            inverts = (min(ttc, FLAT_THROW_FALL_TIME_S)
                       + min(window_s, POST_EVENT_TOLERANCE_S)
                       < max(ttc - window_s, 0.0))
            if inverts:
                exercised_a_raise = True
                with pytest.raises(ValueError):
                    strict_window_bounds(100.0, ttc, window_s=window_s)
                continue
            lower, upper = strict_window_bounds(100.0, ttc,
                                                window_s=window_s)
            assert upper >= lower, f"ttc={ttc} window_s={window_s}"
    # The grid must actually reach the region where the property can fail —
    # otherwise this test degrades back into the vacuous one it replaced.
    assert exercised_a_raise


def test_an_inverted_strict_window_raises_instead_of_admitting_nothing():
    # The concrete case from the a later revision review, verified on this box:
    #   strict_window_bounds(100.0, 1.5, window_s=0.25)
    #     -> lower = +4.250, upper = +3.889   (width -0.361 s)
    # The old (pre-clamp) rule could not produce it: both of its edges hung
    # off the same event_t. An empty window admits no detection, so a
    # positive scored against it would read FN with no error anywhere.
    with pytest.raises(ValueError, match="INVERTED"):
        strict_window_bounds(100.0, 1.5, window_s=0.25)
    # ...and it propagates through the production predicate rather than
    # being swallowed into a quiet False.
    with pytest.raises(ValueError, match="INVERTED"):
        is_in_window_strict([104.0], bag_start=100.0,
                            time_to_closest_s=1.5, window_s=0.25)


def test_a_large_time_to_closest_also_inverts_the_window():
    # The SECOND route, found by running the guard rather than reading the
    # algebra (a later revision). Once min(window_s, post_event_s) saturates at
    # post_event_s, the late edge stops growing with window_s while the
    # floor keeps reaching forward, so a large enough ttc inverts the window
    # at the SHIPPED window_s = 4.0:
    #   ttc > fall_time_s + POST_EVENT_TOLERANCE_S + window_s  ->  6.639 s
    # The a later revision review characterised the hazard as "small window_s"
    # only; it is a surface, not a single crossover.
    threshold = FLAT_THROW_FALL_TIME_S + POST_EVENT_TOLERANCE_S + 4.0
    assert threshold == pytest.approx(6.639, abs=1e-3)
    lower, upper = strict_window_bounds(100.0, threshold - 0.01,
                                        window_s=4.0)
    assert upper >= lower
    with pytest.raises(ValueError, match="INVERTED"):
        strict_window_bounds(100.0, threshold + 0.01, window_s=4.0)


def test_the_window_inversion_crossover_is_where_the_physics_puts_it():
    # (ttc - min(ttc, fall)) / 2 — not a tuned bound, just where the floor
    # reaching forward from the event overtakes the late edge reaching back
    # from ground impact. At the library's largest ttc that is ~0.4307 s.
    ttc = 1.5
    crossover = (ttc - min(ttc, FLAT_THROW_FALL_TIME_S)) / 2.0
    assert crossover == pytest.approx(0.4307, abs=1e-3)
    lower, upper = strict_window_bounds(100.0, ttc,
                                        window_s=crossover + 1e-3)
    assert upper >= lower
    with pytest.raises(ValueError):
        strict_window_bounds(100.0, ttc, window_s=crossover - 1e-3)


def test_the_shipped_library_never_reaches_the_inversion():
    # Why this is a latent hazard and not a live bug: capture_scenario.sh:82
    # writes detection_window_s: 4.0 into every sidecar, and the largest
    # time_to_closest_s in scenario_matrix.yaml is 1.5 s. Nothing in the
    # shipped library is near EITHER route into the inversion.
    for ttc in LIBRARY_TTCS:
        lower, upper = strict_window_bounds(100.0, ttc, window_s=4.0)
        # 2.0 s exactly at ttc = 0.0 (every negative); 2.639 s at ttc >= fall.
        assert upper - lower >= POST_EVENT_TOLERANCE_S
        assert ttc < FLAT_THROW_FALL_TIME_S + POST_EVENT_TOLERANCE_S + 4.0
        assert 4.0 > (ttc - min(ttc, FLAT_THROW_FALL_TIME_S)) / 2.0


def test_strict_window_requires_bag_start():
    with pytest.raises(ValueError):
        strict_window_bounds(None, 0.7, window_s=4.0)


def test_strict_window_requires_time_to_closest_s():
    # This is the exact shape of Defect 2's silent fallback: a positive
    # sidecar with no time_to_closest_s must error, not quietly pass.
    with pytest.raises(ValueError):
        strict_window_bounds(100.0, None, window_s=4.0)


# --- is_in_window_strict -----------------------------------------------------

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
    # the ball; all it emits is that burst. Under the earlier revision's symmetric gate
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
    # ttc is the library MAXIMUM (S01, 1.5 s). It used to be 8.0 here, which
    # is four times anything scenario_matrix.yaml contains and — as the
    # inversion guard now shows — is not merely unrealistic but an
    # inverted window under the clamped late edge (see
    # test_an_inverted_strict_window_raises_instead_of_admitting_nothing).
    # A real library value makes this discriminator STRONGER, not weaker: it
    # demonstrates the property at a ttc the harness actually scores.
    detections = [100.5]  # 0.5s after bag_start=100 -> loose passes trivially
    assert is_in_window_loose(detections, bag_start=100.0, window_s=4.0) is True
    assert is_in_window_strict(
        detections, bag_start=100.0, time_to_closest_s=1.5, window_s=4.0
    ) is False


def test_strict_window_false_when_no_detections():
    assert is_in_window_strict(
        [], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is False


# --- superseded gates, kept for artifact comparison only ---------------------

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


def test_symmetric_window_bounds_are_the_old_gate_written_as_an_interval():
    lower, upper = symmetric_window_bounds(100.0, 0.43, window_s=4.0)
    event = 100.0 + SPAWN_LEAD_S + 0.43
    assert (lower, upper) == pytest.approx((event - 4.0, event + 4.0))
    # 5b's real lower edge for S03/S06/S08: 0.57 s BEFORE bag start.
    assert lower == pytest.approx(100.0 - 0.57)


def test_count_in_window_counts_rather_than_just_deciding():
    # MEDIUM-3: how many detections each gate ADMITS is the number that says
    # whether the new gate is doing anything, and a boolean cannot carry it.
    # 99.80 and 100.50 are PRE-SPAWN (spawn is at 103.0) — cold-map burst
    # territory, which the old gate admitted and the new one floors out.
    detections = [99.80, 100.5, 103.2, 103.6, 105.0, 109.0]
    sym = symmetric_window_bounds(100.0, 0.75, window_s=4.0)
    strict = strict_window_bounds(100.0, 0.75, window_s=4.0)
    assert count_in_window(detections, *sym) == 5      # 99.80 .. 105.0
    assert count_in_window(detections, *strict) == 3   # 103.2, 103.6, 105.0
    assert count_in_window([], *strict) == 0


# --- format_closure_rate (a later revision, LOW-2) --------------------------------

def test_format_closure_rate_says_when_there_was_no_in_band_run():
    # A bare "rate=0.00" reads as a measured zero closure. It is not: it is
    # what run_closure_rate() returns for a single point, i.e. no in-band
    # run existed at all.
    none_in_band = attribute_closing_ball(_sequence(4.5, 2.09, 5), 8.0)
    assert none_in_band["best_run"] < 2
    assert none_in_band["closure_rate"] == 0.0
    assert format_closure_rate(none_in_band) == "n/a(no in-band run)"


def test_format_closure_rate_prints_a_real_rate_when_one_exists():
    fired = attribute_closing_ball(_sequence(4.8, 8.0, 5), 8.0)
    assert format_closure_rate(fired) == "8.00m/s"


# --- closing_runs / longest_closing_run --------------------------------------

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
    # the earlier revision: without a gap bound, two detections seconds apart counted as
    # one "closing step" — the earlier revision's 8 s window made that routine.
    ranges = [(100.0, 5.0), (100.067, 4.0), (106.0, 3.0)]
    assert longest_closing_run(ranges) == 2


def test_max_detection_gap_is_still_derived_from_the_roi_and_airframe_max():
    # Narrowed (a later revision, LOW-4): this locks the CONSTANT against silent
    # retuning and nothing more. The second assertion it used to carry
    # (MAX_DETECTION_GAP_S >= 5.00 / V_AIRFRAME_MAX_MPS) was implied by the
    # first and exercised no code at all. The property it was gesturing at
    # — that the gap can never decide an attribution — is exercised for
    # real in the next test.
    # (An earlier draft derived this constant from cloud_queue_depth as
    # 0.4 s, which modelled dropped clouds rather than missed detections and
    # WAS the operative constraint on eight of ten tune positives.)
    assert MAX_DETECTION_GAP_S == pytest.approx(5.00 / V_AIRFRAME_MAX_MPS)


def test_max_gap_never_changes_an_attribution_verdict():
    # The real property, exercised against attribute_closing_ball rather
    # than asserted about the constant: any step that is IN BAND closes
    # faster than V_AIRFRAME_MAX_MPS between two ranges inside a 5 m ROI, so
    # it cannot span more than MAX_DETECTION_GAP_S — which means removing
    # the gap bound entirely must not change a single verdict. If a future
    # edit retunes the constant downward into a discriminator, these stop
    # agreeing.
    # Only ROI-valid sequences: every range inside [roi_min, roi_max]. That
    # constraint is the whole argument — an in-band step closes at
    # > 3.49 m/s over at most (5.00 - 0.30) m, so it spans at most
    # 4.70 / 3.49 = 1.347 s, under the 1.433 s bound.
    cases = [
        pts for pts in (
            _sequence(5.0, rate, n, hz=hz)
            for rate in (2.09, 3.4, 3.6, 8.0, 11.0, 20.5, 30.0)
            for n in (2, 3, 5, 6)
            for hz in (15.0, 6.0, 3.5, 1.2, 0.6)
        )
        if all(0.30 <= r <= 5.00 for _t, r in pts)
    ]
    assert len(cases) > 30          # the grid must not filter itself empty
    for points in cases:
        bounded = attribute_closing_ball(points, ball_speed_mps=17.0)
        unbounded = attribute_closing_ball(
            points, ball_speed_mps=17.0, max_gap_s=float("inf"))
        assert bounded["attributable"] == unbounded["attributable"]
        assert bounded["best_run"] == unbounded["best_run"]


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


# --- attribute_closing_ball -----------------------------
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
    #
    # Fixed in a later revision (MEDIUM-7): the previous version's "impossible
    # jump" (t 100.1333 -> 100.30, r 3.7333 -> 2.0) was 10.40 m/s, INSIDE
    # the (3.49, 11.49] band, so the two halves were simply joined into one
    # 6-point run and the assertion passed without ever exercising the
    # behaviour it names. The joining step is now genuinely out of band, and
    # both halves are asserted to survive AS SEPARATE RUNS.
    first = _sequence(4.8, 8.0, 3, t0=100.0)        # ends (100.1333, 3.7333)
    second = _sequence(2.0, 8.0, 3, t0=100.15)      # starts (100.15, 2.0)
    joining_rate = (first[-1][1] - second[0][1]) / (second[0][0] - first[-1][0])
    assert joining_rate > 8.0 + V_AIRFRAME_MAX_MPS   # ~104 m/s: out of band

    result = attribute_closing_ball(first + second, 8.0)
    assert result["attributable"] is True
    # 3, not 6: the halves were kept apart, not merged. A rule that
    # rate-checked whole sign-runs would report False here instead, because
    # all six points form one sign-run whose end-to-end rate is out of band.
    assert result["best_run"] == 3
    assert result["longest_run"] == 6                # sign-only sees one run


def test_out_of_band_step_still_fails_when_neither_half_is_long_enough():
    # The complement, so "splits" cannot silently become "ignores": two
    # 2-point halves either side of the same out-of-band step is 4 in-band
    # points and still not an attribution.
    first = _sequence(4.8, 8.0, 2, t0=100.0)
    second = _sequence(2.0, 8.0, 2, t0=100.15)
    result = attribute_closing_ball(first + second, 8.0)
    assert result["attributable"] is False
    assert result["best_run"] == 2


# --- the null model (a later revision, MEDIUM-5) ----------------------------------

def test_monte_carlo_harness_reproduces_the_published_5b_noise_rate():
    # The harness is shipped so this number can be re-derived rather than
    # believed. The 5b sign rule is distribution-free (it reads only the
    # sign of successive differences), so this figure is an exact check on
    # the harness: the the earlier revision reviewer published 0.91 at n=15 over 200 000
    # trials, and it is the number that explains 5b's "11 of 12
    # attributable" as noise. Seeded, so this does not flake.
    p = null_attribution_mc.p_fires(
        null_attribution_mc.fires_sign_rule, n=15, trials=20_000)
    assert p == pytest.approx(0.91, abs=0.015)


def test_monte_carlo_harness_reproduces_the_published_5c_noise_rate():
    # The replacement rule is 2.5-4x better and still not decisive: at the
    # in-window counts this library actually produces, pure noise attributes
    # a ball roughly a third of the time.
    p = null_attribution_mc.p_fires(
        null_attribution_mc.fires_rate_rule, n=15, trials=20_000)
    assert p == pytest.approx(0.36, abs=0.03)


def test_the_published_null_figures_still_match_the_measured_n_win():
    # THE STALENESS GUARD (a later revision, F2-1). The defect this catches is not
    # arithmetic — it is that detector.yaml's three null figures were computed
    # at one n_win vector, the a later revision window change moved that vector,
    # and nothing anywhere noticed. The figures were prose; prose cannot fail.
    #
    # Recomputed here from null_attribution_mc.MEASURED_N_WIN, so any future
    # change to those counts that does not also move the published figures
    # fails THIS test instead of shipping a stale claim into the yaml.
    #
    # Reduced trials (20 000 vs the published 200 000) with tolerances wide
    # enough for the sampling error and no wider. Seeded, so it does not flake.
    s = null_attribution_mc.null_summary(trials=20_000)
    assert s["rate_min"] == pytest.approx(
        null_attribution_mc.PUBLISHED_RATE_MIN, abs=0.01)
    assert s["rate_max"] == pytest.approx(
        null_attribution_mc.PUBLISHED_RATE_MAX, abs=0.01)
    assert s["p_none"] == pytest.approx(
        null_attribution_mc.PUBLISHED_P_NONE, abs=0.015)
    assert s["expected"] == pytest.approx(
        null_attribution_mc.PUBLISHED_EXPECTED, abs=0.06)


def test_the_null_figures_move_when_the_n_win_vector_does():
    # The guard above is only worth having if it is sensitive. The PREVIOUS
    # round's vector — S01 13 not 10, S10 16 not 12 — must produce figures
    # that visibly fail the published pins, otherwise the staleness it exists
    # to catch would slip through it too.
    stale = null_attribution_mc.null_summary(
        n_win=(13, 3, 8, 11, 5, 4, 3, 7, 4, 16, 4, 5), trials=20_000)
    assert stale["rate_max"] > null_attribution_mc.PUBLISHED_RATE_MAX + 0.01
    assert stale["p_none"] < null_attribution_mc.PUBLISHED_P_NONE - 0.015
    assert stale["expected"] > null_attribution_mc.PUBLISHED_EXPECTED + 0.06


def test_the_null_floor_sits_at_the_smallest_in_window_count():
    # The interval detector.yaml used to ship opened at 0.10, which was never
    # right at either vector. The floor is set by the SMALLEST n_win in the
    # library — n = 3, S02 and S07 — and is a third of what was published.
    assert min(null_attribution_mc.MEASURED_N_WIN) == 3
    p3 = null_attribution_mc.p_fires(
        null_attribution_mc.fires_rate_rule, n=3, trials=20_000)
    assert p3 == pytest.approx(null_attribution_mc.PUBLISHED_RATE_MIN,
                               abs=0.01)
    assert p3 < 0.10


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
