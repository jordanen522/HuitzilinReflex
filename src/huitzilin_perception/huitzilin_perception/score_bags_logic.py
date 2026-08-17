"""
score_bags_logic.py — pure-Python scoring logic for score_bags.py, split out
so it is unit-testable without rclpy/ROS (see test/test_score_bags_logic.py,
which the ROS-free CI subset picks up automatically).

Covers the two defects Task 5b repairs:

Defect 1 — the harness was replaying a bag's own recorded /threat/centroid
(and /threat/marker) track back onto the bus, and score_bags' subscriber
counted that as a detection by the detector UNDER TEST. compute_exclude_topics
turns "every topic the node under test publishes" into the --exclude-topics
list for `ros2 bag play`, and refuses (loudly) to ever exclude /clock or
/oak/points — /clock is the harness's own time source (excluding it kills the
detector's clock guard and reads as a dead-node 0% recall, not a detection
regression) and /oak/points is the detector's INPUT, never its output.

Defect 2 — `detection_window_s` was never enforced because no sidecar carried
`bag_start_sim_t`, so `in_window = detected` (any centroid, anywhere in the
whole replay) was the only code path ever exercised. is_in_window_loose and
is_in_window_strict both require a real bag_start (the first /clock stamp of
that bag's own replay, per score_bags.py's ScorerNode) and both raise rather
than silently pass when an input they need is missing.
"""

from __future__ import annotations

# Topics that must never appear in an --exclude-topics list, no matter what a
# (possibly buggy) enumeration produces. /clock is the harness's sole time
# source once the warm-up player hands off (run_regression.sh); excluding it
# starves the detector's clock guard and produces a dead-node 0% recall that
# reads as a detection regression. /oak/points is the detector's subscribed
# INPUT — it never appears in the detector's own publisher list, so its
# presence here would itself be a bug in whatever built the topic list.
FORBIDDEN_EXCLUDE_TOPICS = frozenset({"/clock", "/oak/points"})


def compute_exclude_topics(published_topics: list[str]) -> list[str]:
    """
    Turn "topics the node under test publishes" into a safe, deduplicated,
    sorted --exclude-topics list.

    Raises ValueError (never silently no-ops) if:
      - the caller's enumeration is empty — an empty exclude list would
        silently reopen the self-scoring defect this function exists to
        prevent, and a detector that publishes nothing is itself a sign the
        node under test was never actually discovered.
      - the enumeration contains /clock or /oak/points — that means whatever
        discovered "topics the node under test publishes" is looking at the
        wrong node, since detector_node.py neither publishes /clock nor
        /oak/points (it SUBSCRIBES to /oak/points).
    """
    topics = set(published_topics)
    forbidden_hit = topics & FORBIDDEN_EXCLUDE_TOPICS
    if forbidden_hit:
        raise ValueError(
            f"refusing to exclude {sorted(forbidden_hit)} from bag replay — "
            "these are never legitimate outputs of the node under test; the "
            "topic enumeration is looking at the wrong node"
        )
    if not topics:
        raise ValueError(
            "no published topics discovered for the node under test — "
            "excluding nothing would silently reopen the harness's "
            "self-scoring defect (a bag replaying its own recorded "
            "/threat/centroid track back onto the bus)"
        )
    return sorted(topics)


def build_bag_play_cmd(
    bag_path: str, exclude_topics: list[str], rate: float = 1.0
) -> list[str]:
    """
    Build the `ros2 bag play` argv. Centralised so the exclusion cannot be
    forgotten by a future call site the way the original _replay_bag was.
    """
    if not exclude_topics:
        raise ValueError(
            "refusing to build a bag-play command with no --exclude-topics — "
            "see compute_exclude_topics"
        )
    return [
        "ros2", "bag", "play",
        str(bag_path),
        "--clock",
        "--rate", str(rate),
        "--exclude-topics", *exclude_topics,
    ]


def is_in_window_loose(
    detections: list[float], bag_start: float, window_s: float
) -> bool:
    """
    Original documented semantics: at least one detection within window_s of
    bag start (i.e. within the first window_s seconds of the replay). This is
    the LOOSER of the two gates — "weak when bags run ~20 s" (task-5b-brief)
    — kept only for side-by-side comparison against the stricter gate below.
    """
    if bag_start is None:
        raise ValueError(
            "bag_start is required — a missing bag-start sim time must fail "
            "loudly, never silently pass as 'detected somewhere'"
        )
    return any((t - bag_start) <= window_s for t in detections)


# capture_scenario.sh:21 fires the projectile SPAWN_LEAD seconds INTO the
# recording, not at bag start -- the script writes the label sidecar, starts
# `ros2 bag record`, then only after this many seconds calls
# spawn_projectile. `time_to_closest_s` in the matrix/sidecar is defined
# relative to SPAWN ("estimated time from spawn to closest approach",
# scenario_matrix.yaml:18), not relative to bag start. This constant is the
# same literal `SPAWN_LEAD=3` capture_scenario.sh applies to EVERY scenario
# it records -- there is exactly one capture script, used for both the T
# family and the 17 originals, with no per-family branch on the value.
#
# Empirically validated on the Dell 2026-08-17 by reading each bag's own
# recorded /clock and /threat/centroid directly (bypassing replay): T01's
# real ball-detection run lands at bag_start+[3.351, 3.615]s, matching the
# matrix's own independently-derived range-gate-entry/ground-impact window
# (spawn+[0.27, 0.64]s) to within 0.03 s once SPAWN_LEAD is added back in.
# T10 shows the same pattern (a real run at bag_start+[3.73, 4.19]s against a
# nominal spawn+1.1s closest-approach). T05 shows a genuine ABSENCE of any
# detection in the corresponding window (bag_start+[3.26, 3.53]s empty on
# both sides out to +-1s) -- even at capture time, independent evidence T05
# is a genuinely hard throw rather than only a scoring artefact. S05, the
# only one of the 17 originals with any recorded capture-time centroid
# (n=1), is consistent within this same tolerance (measured lead 4.108 s
# against a nominal 3.0 s -- inside window_s regardless). The 17 originals
# otherwise carry no capture-time centroid track to check further (Task 5b
# Defect 1), so S05 is a single-point spot-check, not a family-wide
# confirmation -- but the SAME capture_scenario.sh recorded both families
# with the same hardcoded constant, which is why the value is applied
# uniformly here rather than only to the T split.
SPAWN_LEAD_S = 3.0


def is_in_window_strict(
    detections: list[float],
    bag_start: float,
    time_to_closest_s: float,
    window_s: float,
    spawn_lead_s: float = SPAWN_LEAD_S,
) -> bool:
    """
    Stricter gate: at least one detection within window_s of the ball's own
    closest-approach time. That time is bag_start + spawn_lead_s +
    time_to_closest_s, NOT bag_start + time_to_closest_s -- the projectile
    does not exist until spawn_lead_s into the recording (see SPAWN_LEAD_S
    above), and time_to_closest_s is defined relative to spawn, not to bag
    start. Symmetric tolerance: a detection slightly BEFORE closest approach
    (the ball still closing) is a legitimate early detection, not a miss.

    window_s is deliberately NOT narrowed to the ball's much shorter true
    dwell time (often well under 1 s -- see the matrix's per-scenario
    acquisition-window notes): time_to_closest_s is an estimate that
    excludes gravity drop and patrol egomotion (scenario_matrix.yaml's own
    caveat), so the tolerance has to absorb that uncertainty. This does mean
    a dense-enough false-positive background within window_s of the true
    event can still pass this gate on a nearby but wrong detection -- a
    known limitation, not silently hidden (see the Task 5b report).
    """
    if bag_start is None:
        raise ValueError(
            "bag_start is required — a missing bag-start sim time must fail "
            "loudly, never silently pass as 'detected somewhere'"
        )
    if time_to_closest_s is None:
        raise ValueError(
            "time_to_closest_s is required for the strict window gate — a "
            "positive-scenario sidecar missing it must fail loudly, never "
            "silently fall back to 'detected somewhere'"
        )
    event_t = bag_start + spawn_lead_s + time_to_closest_s
    return any(abs(t - event_t) <= window_s for t in detections)


# ── Attribution (Task 5b) ───────────────────────────────────────────────────
#
# The bags do NOT carry ground-truth ball position (no /gz/dynamic_poses —
# see the task-5b-brief), so attribution cannot compare a detection against
# a known trajectory. What IS available: a real ball closing on the drone
# produces a RUN of detections whose range (distance from the sensor) falls
# across consecutive frames, while first-sight terrain / patrol-motion noise
# produces sporadic, non-closing detections — the same signal Task 5 flagged
# as the fix for this FP class (cross-frame persistence /
# min_track_updates), reused here purely for offline attribution.

DEFAULT_MIN_CLOSING_RUN = 3  # points; i.e. >= 2 consecutive decreasing steps


def longest_closing_run(ranges_by_time: list[tuple[float, float]]) -> int:
    """
    ranges_by_time: [(t, range_m), ...], any order.

    Returns the length (in points) of the longest run of TIME-CONSECUTIVE
    points whose range strictly decreases step to step. A single point is a
    run of 1 (no closing behaviour); two points where range[1] < range[0] is
    a run of 2; and so on. Ties (equal range) break the run, matching
    "decreases" literally rather than "non-increases" -- a genuinely static
    false positive (the same rock re-detected) should not count as closing.
    """
    if not ranges_by_time:
        return 0
    ordered = [r for _t, r in sorted(ranges_by_time, key=lambda p: p[0])]
    best = 1
    cur = 1
    for prev, cur_r in zip(ordered, ordered[1:]):
        if cur_r < prev:
            cur += 1
            best = max(best, cur)
        else:
            cur = 1
    return best


def is_attributable_to_ball(
    ranges_by_time: list[tuple[float, float]],
    min_run: int = DEFAULT_MIN_CLOSING_RUN,
) -> bool:
    """
    True if `ranges_by_time` contains a closing run of at least `min_run`
    points (see longest_closing_run). This is a NECESSARY-evidence check,
    not a proof — a real ball obscured to a single frame is NOT
    attributable by this method (see the task-5b-report.md's honest count
    of how many TPs this could not attribute either way).
    """
    return longest_closing_run(ranges_by_time) >= min_run
