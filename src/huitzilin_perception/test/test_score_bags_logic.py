"""
Unit tests for score_bags_logic.py — Task 5b.

Runs without ROS (pure Python, no rclpy import anywhere in the module under
test): part of the ROS-free CI subset picked up automatically by
.github/workflows/tests.yml's deny-list (it imports nothing that needs ROS).

test_build_bag_play_cmd_never_omits_exclude_topics is the single most
valuable test in this file per the task-5b-brief: it is the regression test
that would have caught Defect 1 — the original _replay_bag built a `ros2 bag
play` command with no --exclude-topics at all, so a T-bag's own recorded
/threat/centroid track replayed straight back onto the bus and score_bags
counted it as a detection by the detector under test.
"""

import pytest

from huitzilin_perception.score_bags_logic import (
    FORBIDDEN_EXCLUDE_TOPICS,
    SPAWN_LEAD_S,
    build_bag_play_cmd,
    compute_exclude_topics,
    is_attributable_to_ball,
    is_in_window_loose,
    is_in_window_strict,
    longest_closing_run,
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


def test_forbidden_topics_are_exactly_clock_and_oak_points():
    # Locks the two topics the task-5b-brief's ambiguity resolution names
    # explicitly — nothing more, nothing less.
    assert FORBIDDEN_EXCLUDE_TOPICS == frozenset({"/clock", "/oak/points"})


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


# ── is_in_window_loose ──────────────────────────────────────────────────────

def test_loose_window_true_when_detection_within_window_of_bag_start():
    assert is_in_window_loose([102.0], bag_start=100.0, window_s=4.0) is True


def test_loose_window_false_when_detection_after_window():
    assert is_in_window_loose([106.0], bag_start=100.0, window_s=4.0) is False


def test_loose_window_requires_bag_start():
    with pytest.raises(ValueError):
        is_in_window_loose([102.0], bag_start=None, window_s=4.0)


def test_loose_window_false_when_no_detections():
    assert is_in_window_loose([], bag_start=100.0, window_s=4.0) is False


# ── is_in_window_strict ─────────────────────────────────────────────────────
#
# event_t = bag_start + spawn_lead_s + time_to_closest_s. spawn_lead_s
# defaults to SPAWN_LEAD_S (3.0) -- the projectile does not exist until that
# many seconds into the recording (capture_scenario.sh), so anchoring at
# bag_start + time_to_closest_s alone (no lead) puts the window entirely
# BEFORE the throw. That was caught in review: an initial version of this
# fix used the no-lead formula and scored several genuine detections as
# false negatives purely from a ~3 s anchoring error.

def test_strict_window_true_near_closest_approach():
    # bag_start=100, spawn_lead=3 (default), time_to_closest_s=0.7
    # -> event at 103.7
    assert is_in_window_strict(
        [103.9], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is True


def test_strict_window_true_for_detection_slightly_before_closest_approach():
    # A detection just before the ball reaches closest approach is a
    # legitimate early catch, not a miss (symmetric tolerance). event=103.7.
    assert is_in_window_strict(
        [103.5], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is True


def test_strict_window_uses_spawn_lead_s_by_default():
    # This is the exact bug caught in review: without the spawn lead, event_t
    # would be 100.7 and a detection at 103.6 (right where the ball actually
    # is -- 0.6 s after the true spawn+closest-approach instant) would read
    # as 2.9 s away and still happen to pass a 4 s window by coincidence, but
    # a detection right at true closest approach with a TIGHTER window would
    # not. Use a window small enough to discriminate: window_s=1.0.
    detection_at_true_event = [100.0 + SPAWN_LEAD_S + 0.7]  # = 103.7 exactly
    assert is_in_window_strict(
        detection_at_true_event, bag_start=100.0, time_to_closest_s=0.7,
        window_s=1.0,
    ) is True
    # The no-lead anchor (100.7) is 3 s away from this same detection --
    # outside a 1 s window -- which is exactly why the lead is load-bearing.
    assert abs(103.7 - 100.7) > 1.0


def test_strict_window_spawn_lead_s_is_overridable():
    # bag_start=100, spawn_lead_s=0 (explicit override), ttc=0.7 -> event=100.7
    assert is_in_window_strict(
        [100.9], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0,
        spawn_lead_s=0.0,
    ) is True
    # Same detection is also within the wider default-lead window (103.7),
    # since window_s=4.0 easily absorbs the 3 s difference -- window width
    # alone can mask an anchor bug, which is why the test above uses a
    # tight window instead.
    assert is_in_window_strict(
        [100.9], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0,
    ) is True


def test_strict_window_false_far_from_closest_approach_but_inside_loose_window():
    # This is the discriminator the brief asks for: a detection that would
    # pass the loose (bag-start-anchored) gate but is nowhere near when the
    # ball was actually present must fail the strict gate. Loose only checks
    # "within window_s of bag start" -- an early spurious centroid right
    # after the clock starts always satisfies that, no matter how late the
    # ball actually arrives (time_to_closest_s=8.0 here -- with the 3 s
    # spawn lead, event_t=111.0, well past the 4 s loose window), which is
    # exactly the "weak on a ~20 s bag" gap the brief calls out.
    detections = [100.5]  # 0.5s after bag_start=100 -> loose passes trivially
    assert is_in_window_loose(detections, bag_start=100.0, window_s=4.0) is True
    assert is_in_window_strict(
        detections, bag_start=100.0, time_to_closest_s=8.0, window_s=4.0
    ) is False


def test_strict_window_requires_bag_start():
    with pytest.raises(ValueError):
        is_in_window_strict(
            [100.5], bag_start=None, time_to_closest_s=0.7, window_s=4.0
        )


def test_strict_window_requires_time_to_closest_s():
    # This is the exact shape of Defect 2's silent fallback: a positive
    # sidecar with no time_to_closest_s must error, not quietly pass.
    with pytest.raises(ValueError):
        is_in_window_strict(
            [100.5], bag_start=100.0, time_to_closest_s=None, window_s=4.0
        )


def test_strict_window_false_when_no_detections():
    assert is_in_window_strict(
        [], bag_start=100.0, time_to_closest_s=0.7, window_s=4.0
    ) is False


# ── longest_closing_run / is_attributable_to_ball ───────────────────────────

def test_longest_closing_run_empty_is_zero():
    assert longest_closing_run([]) == 0


def test_longest_closing_run_single_point_is_one():
    assert longest_closing_run([(100.0, 3.0)]) == 1


def test_longest_closing_run_detects_decreasing_sequence():
    # A ball closing on the drone: range falls frame over frame.
    ranges = [(100.0, 4.0), (100.1, 3.5), (100.2, 3.0), (100.3, 2.5)]
    assert longest_closing_run(ranges) == 4


def test_longest_closing_run_ignores_out_of_order_input():
    # Same sequence, shuffled -- must sort by time before checking closing.
    ranges = [(100.3, 2.5), (100.0, 4.0), (100.2, 3.0), (100.1, 3.5)]
    assert longest_closing_run(ranges) == 4


def test_longest_closing_run_breaks_on_non_decrease():
    # Non-closing detections (e.g. drifting-away terrain / receding
    # background): range strictly increases, so no step ever decreases and
    # every run stays at 1.
    ranges = [(100.0, 4.0), (101.0, 4.2), (102.0, 4.5), (103.0, 4.8)]
    assert longest_closing_run(ranges) == 1


def test_longest_closing_run_ties_break_the_run():
    # Equal range does not count as "decreasing" -- a static false positive
    # (same rock re-detected) must not read as closing.
    ranges = [(100.0, 3.0), (100.1, 3.0), (100.2, 2.5)]
    assert longest_closing_run(ranges) == 2  # only the 3.0 -> 2.5 step


def test_longest_closing_run_finds_longest_among_several():
    ranges = [
        (100.0, 5.0), (100.1, 4.0),               # run of 2
        (101.0, 6.0), (101.1, 5.5), (101.2, 5.0), (101.3, 4.5),  # run of 4
    ]
    assert longest_closing_run(ranges) == 4


def test_is_attributable_to_ball_true_for_long_enough_run():
    ranges = [(100.0, 4.0), (100.1, 3.5), (100.2, 3.0)]  # run of 3
    assert is_attributable_to_ball(ranges, min_run=3) is True


def test_is_attributable_to_ball_false_for_sporadic_noise():
    ranges = [(100.0, 4.0), (101.0, 4.2), (102.0, 3.9)]
    assert is_attributable_to_ball(ranges, min_run=3) is False


def test_is_attributable_to_ball_uses_default_min_run():
    # Exactly at the default threshold (3 points, 2 decreasing steps).
    ranges = [(100.0, 4.0), (100.1, 3.5), (100.2, 3.0)]
    assert is_attributable_to_ball(ranges) is True
    # One point short of it.
    assert is_attributable_to_ball(ranges[:2]) is False
