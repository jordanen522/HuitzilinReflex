"""
score_bags_logic.py — pure-Python scoring logic for score_bags.py, split out
so it is unit-testable without rclpy/ROS (see test/test_score_bags_logic.py,
which the ROS-free CI subset picks up automatically).

Covers the two defects Task 5b repaired and the two CONFIRMED CRITICAL
findings its review returned against those repairs (Task 5c).

Defect 1 (5b) — the harness was replaying a bag's own recorded
/threat/centroid (and /threat/marker) track back onto the bus, and
score_bags' subscriber counted that as a detection by the detector UNDER
TEST. compute_exclude_topics turns "every topic the node under test
publishes" into the --exclude-topics list for `ros2 bag play`. It refuses
(loudly) to ever exclude /clock or /oak/points, and — Task 5c HIGH-1 — it
now also refuses a discovery result that does NOT contain /threat/centroid,
because a partial publisher list like ['/parameter_events', '/rosout']
passed every earlier guard and silently restored Defect 1 in full.

Defect 2 (5b) — `detection_window_s` was never enforced because no sidecar
carried `bag_start_sim_t`, so `in_window = detected` (any centroid, anywhere
in the whole replay) was the only code path ever exercised.

Task 5c CRITICAL-1 — 5b's replacement gate was `abs(t - event_t) <=
window_s`. Symmetric, so at the shipped window_s of 4.0 it spanned EIGHT
seconds of a ~20 s bag, and its lower edge landed between -0.57 s and
+0.50 s of bag start for all twelve of the 17 originals' positives. The
cold-background-map false-positive burst that detector.yaml documents in
the first ~1-2 s of every replay therefore sat inside the gate, so a
positive whose ball was never seen could pass on background alone — exactly
the failure Defect 2 existed to make impossible. strict_window_bounds()
replaces it with an asymmetric, physics-floored window; see its docstring.

Task 5c CRITICAL-2 — 5b attributed a TP to a ball by finding a run of >= 3
in-window detections of strictly DECREASING range, using only the SIGN of
each range difference. That is a stationary sensor's null hypothesis. These
bags were captured under patrol, and detector.yaml identifies this library's
dominant FP class as newly-explored static terrain along a patrol leg —
terrain the drone translates TOWARD, whose base_link range therefore falls
monotonically frame after frame. The sign test selects FOR that class. The
magnitude was in hand and discarded; attribute_closing_ball() now uses it.
"""

from __future__ import annotations

# ── Bag-replay exclusion (Defect 1) ────────────────────────────────────────
#
# Topics that must never appear in an --exclude-topics list, no matter what a
# (possibly buggy) enumeration produces. /clock is the harness's sole time
# source once the warm-up player hands off (run_regression.sh); excluding it
# starves the detector's clock guard and produces a dead-node 0% recall that
# reads as a detection regression. /oak/points is the detector's subscribed
# INPUT — it never appears in the detector's own publisher list, so its
# presence here would itself be a bug in whatever built the topic list.
FORBIDDEN_EXCLUDE_TOPICS = frozenset({"/clock", "/oak/points"})

# Topics that MUST appear, or the exclusion is not doing its job (Task 5c
# HIGH-1). score_bags subscribes to exactly one topic — /threat/centroid —
# and that is the topic whose recorded capture-time track contaminated every
# pre-5b measurement. A live publisher enumeration that comes back with
# ['/parameter_events', '/rosout'] (both created inside rclpy's Node.__init__,
# i.e. BEFORE detector_node.py creates its own publishers, and propagated
# per-endpoint and asynchronously by DDS) satisfies "non-empty" and "nothing
# forbidden" while excluding nothing that matters. That is Defect 1 restored
# in full, with no error anywhere.
REQUIRED_EXCLUDE_TOPICS = frozenset({"/threat/centroid"})


def compute_exclude_topics(published_topics: list[str]) -> list[str]:
    """
    Turn "topics the node under test publishes" into a safe, deduplicated,
    sorted --exclude-topics list.

    Raises ValueError (never silently no-ops) if:
      - the enumeration contains /clock or /oak/points — that means whatever
        discovered "topics the node under test publishes" is looking at the
        wrong node, since detector_node.py neither publishes /clock nor
        /oak/points (it SUBSCRIBES to /oak/points).
      - the caller's enumeration is empty — an empty exclude list would
        silently reopen the self-scoring defect this function exists to
        prevent, and a detector that publishes nothing is itself a sign the
        node under test was never actually discovered.
      - the enumeration is missing /threat/centroid (Task 5c HIGH-1) — a
        partial DDS graph read is not a safe basis for an exclusion, and the
        one topic the harness actually scores is the one that must be gone.
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
    missing = REQUIRED_EXCLUDE_TOPICS - topics
    if missing:
        raise ValueError(
            f"publisher enumeration {sorted(topics)} is missing "
            f"{sorted(missing)} — this is a PARTIAL read of the ROS graph, "
            "not the node's real publisher set. Proceeding would replay the "
            "bag's own recorded /threat/centroid track back onto the bus and "
            "score it as the detector's output (Task 5b Defect 1)"
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


def clock_msg_to_sim_t(sec: int, nanosec: int) -> float:
    """
    rosgraph_msgs/Clock payload -> float sim seconds.

    Split out of score_bags._read_bag_start_sim_t (Task 5c MEDIUM-4) purely
    so it can be tested with no bag fixture and no ROS: this one line fixes
    the absolute position of every scenario's detection window, and a
    nanosecond-scaling slip here would shift all seventeen windows in
    lockstep and be nearly invisible in the output.
    """
    return float(sec) + float(nanosec) * 1e-9


# ── Detection window (Defect 2, and Task 5c CRITICAL-1) ────────────────────
#
# capture_scenario.sh:21 fires the projectile SPAWN_LEAD seconds INTO the
# recording, not at bag start -- the script writes the label sidecar, starts
# `ros2 bag record`, then only after this many seconds calls
# spawn_projectile. `time_to_closest_s` in the matrix/sidecar is defined
# relative to SPAWN ("estimated time from spawn to closest approach",
# scenario_matrix.yaml:18), not relative to bag start. This constant is the
# same literal `SPAWN_LEAD=3` capture_scenario.sh applies to EVERY scenario
# it records -- there is exactly one capture script, used for both the T
# family and the 17 originals, with no per-family branch on the value, and
# no revision of the script in git history has ever carried another value
# (verified across all five: 70d6680, ceff20d, 5992359, 8d86873, fd330b8).
SPAWN_LEAD_S = 3.0

# How far AFTER the ball's last airborne instant a genuine detection can
# still land (Task 5c CRITICAL-1: the width of the window).
#
# The asymmetry is not a preference, it is the physics. Everything in the
# ball's own flight puts real detections EARLIER than the nominal event:
#   - Week 3 throws are flat (compensate_gravity defaults False,
#     ballistics.py), so a ball thrown level from a 2 m hover reaches the
#     ground sqrt(2*2.0/9.81) = 0.639 s after spawn regardless of its speed.
#     For five of the twelve original positives (S01 1.5 s, S10 1.25 s) and
#     four of the ten tune positives (ttc 1.1 s) the nominal closest
#     approach is never reached at all -- scenario_matrix.yaml:73-77 says so
#     for T01 explicitly. That is what FLAT_THROW_FALL_TIME_S below is for.
#   - an oblique throw leaves the 45 deg ROI cone before closest approach
#     (the matrix's own T04/T06 notes).
# So the ONLY mechanism that can put a genuine detection later is
# spawn-command latency: capture_scenario.sh:90-97 runs `sleep 3` and then
# `ros2 run huitzilin_perception spawn_projectile`, and that `ros2 run`
# costs real time before the projectile exists. The whole flight -- spawn,
# closest approach, ground impact -- is therefore shifted LATER by that
# latency relative to bag_start + SPAWN_LEAD_S, which is exactly what this
# constant absorbs, and it is added to the ball's last airborne instant
# rather than to a nominal event that may never happen (Task 5c fix round 1,
# HIGH-3).
#
# Budget, measured on the Dell rather than assumed:
#   ros2 CLI startup                0.25 - 0.49 s  (3 runs, idle machine)
#   python + rclpy init + Node()    0.41 - 0.74 s  (3 runs, idle machine)
#   -> `ros2 run` floor             0.7  - 1.2  s, before spawn_projectile
#      does any work of its own (ballistics + a gz create service round
#      trip, on a machine simultaneously running gz, SITL and rosbag2).
# PROVENANCE, so a later reader can weigh it: the one end-to-end figure the
# repo contains is S05's recorded capture-time centroid, which puts the
# effective lead at 4.108 s against the nominal 3.0, i.e. 1.108 s of
# latency. S05 is a TRAIN-SPLIT ORIGINAL, not a tune bag. It was used as
# CONFIRMATION ONLY of a constant derived from the Dell timing measurement
# and capture_scenario.sh (the Task 5b review's Q4 rule sanctions exactly
# that), but this is a scoring constant partly corroborated by an original
# and should be read as such.
#
# 2.0 s is ~1.8x that end-to-end offset and ~1.7-2.9x the measured `ros2
# run` floor, so the late edge cannot manufacture a false negative out of
# spawn latency -- while cutting the gate from the 8.0 s that Task 5b
# shipped down to 2.43-2.64 s, and (with the floor below) removing the
# documented cold-map FP burst entirely.
POST_EVENT_TOLERANCE_S = 2.0

# ── The ball's last airborne instant (Task 5c fix round 1, HIGH-3) ─────────
#
# The window used to be `event_t + min(window_s, POST_EVENT_TOLERANCE_S)`
# with event_t the NOMINAL closest approach, bag_start + spawn_lead +
# time_to_closest_s. That contradicted the physics three paragraphs above:
# for S01 (ttc 1.50 s) the same docstring says the ball has been on the
# ground since 0.639 s after spawn, so the nominal event never happens and
# the window ran to spawn + 3.50 s -- 2.86 s after impact. The gate was
# WIDEST exactly where the ball is present for the smallest fraction of it,
# and the two widest windows (S01 3.50 s, S10 3.25 s) returned the two
# largest in-window detection counts in the library (13 and 16, against 4-8
# for the 2.43 s windows).
#
# The corrected rule derives the late edge from the same flat-throw model:
#
#   t_last_airborne = min(time_to_closest_s, FLAT_THROW_FALL_TIME_S)
#   upper           = spawn_t + t_last_airborne + min(window_s, post_event_s)
#
# min(), because a flat throw that has not reached closest approach by
# ground impact is still closing when it lands: its ACTUAL closest approach
# in flight is at impact, not at the nominal ttc. When ttc <= the fall time
# the nominal event does happen and the term is unchanged.
#
# This is a STRICT TIGHTENING by construction -- min(ttc, T) <= ttc for
# every ttc and every T >= 0, so the new upper edge is <= the old one for
# every scenario in the library and for every scenario anyone could add.
# The floor and the ANCHOR are untouched: lower is still
# max(event_t - window_s, spawn_t) and event_t is still bag_start +
# spawn_lead_s + time_to_closest_s.
#
# Sources, none of them a measured outcome of this harness:
#   - 2.0 m: bridge.yaml / hw_bridge.yaml takeoff_alt_m, the altitude every
#     Week 3 capture flew (capture_scenario.sh raises it only for the
#     vertical-offset negatives N04/T13, handled below).
#   - flat throw: spawn_projectile.py:362 declares compensate_gravity False
#     and capture_scenario.sh:92-96 never passes it, so vz(0) = 0.
#   - 9.81 m/s^2 and h = 1/2 g t^2.
HOVER_ALTITUDE_M = 2.0
G_MPS2 = 9.81
FLAT_THROW_FALL_TIME_S = (2.0 * HOVER_ALTITUDE_M / G_MPS2) ** 0.5  # 0.639 s


def flat_throw_fall_time_s(
    offset_vertical_m: float = 0.0,
    hover_alt_m: float = HOVER_ALTITUDE_M,
) -> float | None:
    """
    Seconds from spawn to ground impact for a flat Week-3 throw, or None
    when the flat-throw model does not describe that scenario and the caller
    must fall back to the nominal event.

    Returns None for any scenario with a non-zero offset_vertical_m (N04 at
    -10.0 m, T13 at -7.5 m). Those two are NOT captured from the standard
    2 m hover -- capture_scenario.sh:54-71 refuses to spawn unless the drone
    has climbed to >= 10.5 m / 8.0 m first -- so neither the drop height nor
    the hover altitude in this model is the one they flew, and guessing a
    substitute would be inventing physics for a scenario nobody measured.
    Both are negatives carrying time_to_closest_s: 0, so the fallback is
    numerically identical to the clamp for them anyway; the None exists so
    that stays true if a future positive ever carries a vertical offset.

    A lobbed or gravity-compensated throw (compensate_gravity=True, which is
    the Week 4/6 battery configuration and no part of this bag library)
    would need its own flight-time model and is not covered here either --
    it does not reach the ground at sqrt(2h/g).
    """
    if offset_vertical_m:
        return None
    return (2.0 * hover_alt_m / G_MPS2) ** 0.5


def strict_window_bounds(
    bag_start: float,
    time_to_closest_s: float,
    window_s: float,
    spawn_lead_s: float = SPAWN_LEAD_S,
    post_event_s: float = POST_EVENT_TOLERANCE_S,
    fall_time_s: float | None = FLAT_THROW_FALL_TIME_S,
) -> tuple[float, float]:
    """
    Absolute [lower, upper] sim-time bounds of the strict detection window.

    lower = max(event_t - window_s, bag_start + spawn_lead_s)

        The floor is a PHYSICS floor, not a tuned one: the projectile does
        not exist before bag_start + spawn_lead_s, so a centroid earlier
        than that is definitionally not the ball, whatever else it is. Task
        5b's symmetric gate had no floor at all, which is why its lower edge
        landed within half a second of bag start for every positive in the
        library and swallowed the cold-background-map FP burst that
        detector.yaml:229-231 documents in the first ~1-2 s of every replay.

    upper = spawn_t + min(time_to_closest_s, fall_time_s)
                    + min(window_s, post_event_s)

        Asymmetric and much tighter on the late side, because the ball's own
        physics puts genuine detections BEFORE the nominal event and only
        spawn-command latency can put one after it (see
        POST_EVENT_TOLERANCE_S).

        min(time_to_closest_s, fall_time_s) is the ball's LAST AIRBORNE
        INSTANT relative to spawn, and is where the tolerance is added --
        not to a nominal closest approach the flat throw may never reach
        (Task 5c fix round 1, HIGH-3; see FLAT_THROW_FALL_TIME_S). Pass
        fall_time_s=None for a scenario whose geometry the flat-throw model
        does not describe, which restores the nominal-event edge for that
        scenario alone; flat_throw_fall_time_s() decides that.

        `min(window_s, ...)` keeps the sidecar's detection_window_s as a
        hard ceiling. For THIS library that term is entirely non-binding and
        the property is true but vacuous -- capture_scenario.sh:82 writes
        detection_window_s: 4.0 into every sidecar and post_event_s is 2.0,
        so `min` always returns post_event_s and the gate has exactly one
        free parameter. It is kept because a sidecar is data and a future
        capture could ship a tighter one, but nobody should read the
        ceiling as doing work here.

        Both edges together make this window a strict SUBSET of the one
        Task 5b shipped AND of the one Task 5c originally shipped -- recall
        measured under it can only fall or stay equal, never rise.

    event_t = bag_start + spawn_lead_s + time_to_closest_s is UNCHANGED and
    deliberately so: it was independently verified across every historical
    revision of capture_scenario.sh, and breaking a correct anchor while
    fixing the width is the most likely way to turn a ~1 s calibration
    offset into manufactured false negatives. It still sets the lower edge;
    only the late edge now clamps at ground impact.
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
    spawn_t = bag_start + spawn_lead_s
    event_t = spawn_t + time_to_closest_s
    last_airborne_s = (
        time_to_closest_s if fall_time_s is None
        else min(time_to_closest_s, fall_time_s)
    )
    lower = max(event_t - window_s, spawn_t)
    upper = spawn_t + last_airborne_s + min(window_s, post_event_s)
    return lower, upper


def is_in_window_strict(
    detections: list[float],
    bag_start: float,
    time_to_closest_s: float,
    window_s: float,
    spawn_lead_s: float = SPAWN_LEAD_S,
    post_event_s: float = POST_EVENT_TOLERANCE_S,
    fall_time_s: float | None = FLAT_THROW_FALL_TIME_S,
) -> bool:
    """
    At least one detection inside strict_window_bounds().

    THIS is the production predicate (Task 5c fix round 1, LOW-3):
    score_bags._score_one used to inline `any(lower <= t <= upper ...)`
    itself, so the eight unit tests below covered a function the harness
    never called. The predicate that decides pass/fail must be the one with
    regression tests.
    """
    lower, upper = strict_window_bounds(
        bag_start, time_to_closest_s, window_s, spawn_lead_s, post_event_s,
        fall_time_s,
    )
    return any(lower <= t <= upper for t in detections)


def symmetric_window_bounds(
    bag_start: float,
    time_to_closest_s: float,
    window_s: float,
    spawn_lead_s: float = SPAWN_LEAD_S,
) -> tuple[float, float]:
    """
    Absolute bounds of Task 5b's symmetric unfloored gate, [event - w,
    event + w]. Exists so the artifact can report how many detections that
    gate admitted (n_win_sym) beside how many this one does (Task 5c fix
    round 1, MEDIUM-3) -- the ratio is the only number that answers "the
    strict, sym and loose verdicts still agree, so is the new gate inert?".
    """
    if bag_start is None or time_to_closest_s is None:
        raise ValueError("bag_start and time_to_closest_s are both required")
    event_t = bag_start + spawn_lead_s + time_to_closest_s
    return event_t - window_s, event_t + window_s


def count_in_window(detections: list[float], lower: float, upper: float) -> int:
    """How many detection stamps fall in [lower, upper]."""
    return sum(1 for t in detections if lower <= t <= upper)


def is_in_window_symmetric_legacy(
    detections: list[float],
    bag_start: float,
    time_to_closest_s: float,
    window_s: float,
    spawn_lead_s: float = SPAWN_LEAD_S,
) -> bool:
    """
    Task 5b's gate exactly as it shipped: `abs(t - event_t) <= window_s`,
    symmetric and unfloored. Kept ONLY so every run artifact can print the
    old verdict beside the new one and make the CRITICAL-1 change visible
    per scenario. It is never what decides pass/fail.
    """
    lower, upper = symmetric_window_bounds(
        bag_start, time_to_closest_s, window_s, spawn_lead_s
    )
    return count_in_window(detections, lower, upper) > 0


def is_in_window_loose(
    detections: list[float], bag_start: float, window_s: float
) -> bool:
    """
    The PRE-5b semantics, reproduced verbatim for comparison only: at least
    one detection with (t - bag_start) <= window_s.

    Note what that expression actually computes (Task 5c LOW-4): it has no
    lower bound, so any t before bag_start passes too. That is faithful to
    the original — the point of this function is to reproduce the gate whose
    numbers older reports quoted, not to be a defensible gate — but the
    docstring used to claim "within the first window_s seconds of the
    replay", which is not the same thing. It is never authoritative.
    """
    if bag_start is None:
        raise ValueError(
            "bag_start is required — a missing bag-start sim time must fail "
            "loudly, never silently pass as 'detected somewhere'"
        )
    return any((t - bag_start) <= window_s for t in detections)


# ── Attribution (Task 5b, rebuilt for Task 5c CRITICAL-2) ──────────────────
#
# The bags do NOT carry ground-truth ball position (no /gz/dynamic_poses —
# see the task-5b-brief), so attribution cannot compare a detection against a
# known trajectory. What IS available is the range from base_link that
# /threat/centroid already carries (detector_node.py stamps it
# frame_id="base_link", so sqrt(x^2+y^2+z^2) is range from the airframe, not
# from a world origin — no frame conversion is involved anywhere below).
#
# Task 5b tested only the SIGN of successive range differences. Under patrol
# that is inverted: the drone translates toward newly-explored terrain, whose
# base_link range then decreases monotonically frame after frame, and
# detector.yaml:226-231 names exactly that terrain as this library's dominant
# FP mechanism. Independently quantified by the Task 5b reviewer over 200 000
# iid-noise trials per point through the shipped function: P(spurious run >=
# 3) = 0.77 at n=10, 0.91 at n=15, 0.97 at n=20 detections in the window. The
# reported 11-of-12 attribution rate sat exactly on the n~15 noise
# prediction, so the statistic measured how many detections were in the
# window, not whether a ball was there.
#
# The fix uses the magnitude the old test discarded, against two MEASURED
# populations rather than a chosen number.

# Null population: static terrain closing only because the airframe
# translates toward it. Range from base_link to a stationary point changes at
# exactly the radial component of the drone's own velocity, so |dr/dt| is
# bounded above by the drone's ground speed. This repo's measured patrol
# speed distribution (CLAUDE.md's hover_mode note, from the throw-window
# gate's own rolling statistics) is a 3.49 m/s rolling maximum against a
# 2.09 m/s median. 3.49 m/s is therefore the UPPER EDGE OF THE NULL
# POPULATION, not a tuned threshold: a closure faster than the drone can
# itself fly cannot be produced by egomotion on a static object.
#
# PROVENANCE (Task 5c fix round 1, MEDIUM-6): CLAUDE.md records this figure
# from the WEEK 6 dodge-battery throw-window gate, while this bag library
# was captured in WEEK 3. It is a real measurement of this repo's own
# patrol, but nothing establishes that the two runs used the same patrol
# speed configuration, so treat it as "a measured patrol maximum for this
# airframe", not "the measured maximum of these bags". It is immaterial to
# the outcome recorded in detector.yaml -- the falsification control shows
# the null is not bounded by egomotion at all -- but anyone who revives this
# rule should re-measure the null population on the Week 3 captures rather
# than importing this number again.
V_AIRFRAME_MAX_MPS = 3.49
V_AIRFRAME_MEDIAN_MPS = 2.09

# Signal population ceiling: a ball's range rate in base_link cannot exceed
# its own speed plus the airframe's, so (speed_mps + V_AIRFRAME_MAX_MPS) is
# the fastest closure that scenario can physically produce. Ranges jumping
# faster than that are not one object being tracked frame to frame; they are
# unrelated detections at unrelated depths, which is what an FP chain looks
# like. This upper edge is what stops the rate test from being satisfied by
# large random range jumps at frame rate.
LIBRARY_MAX_BALL_SPEED_MPS = 17.0  # scenario_matrix.yaml: T03, T07, T08

# Two detections of the SAME object cannot be arbitrarily far apart in time
# (the Task 5b reviewer's point: with no bound at all, two detections six
# seconds apart inside an 8 s window counted as one "closing step").
#
# The bound is set by the ROI, not by the frame rate. A first attempt at this
# constant used (cloud_queue_depth + 1) / 15 Hz = 0.4 s, reasoning from how
# many clouds the detector can DROP. That models the wrong mechanism and was
# wrong by a factor of ~3.5: the gap between two detections of one ball is
# dominated by clouds the detector PROCESSED and did not detect the ball in,
# which no queue depth bounds — the matrix's own dwell arithmetic (2.8-9.0
# depth clouds per positive) says detections are sparse by construction. The
# tune split showed that constant to be the operative constraint on eight of
# ten positives, which is how the error surfaced.
#
# What genuinely bounds it: a run that qualifies below closes at more than
# V_AIRFRAME_MAX_MPS, and the whole ROI is only roi_max_range_m deep, so no
# qualifying step can span more than roi_max_range_m / V_AIRFRAME_MAX_MPS.
# Setting the gap there makes it provably NEVER the operative constraint on
# an attribution — the closure-rate band does all the discriminating, which
# is what it is for — while still splitting the descriptive run statistic on
# anything absurd.
DEPTH_CLOUD_HZ = 15.0
ROI_MAX_RANGE_M = 5.00  # detector.yaml
MAX_DETECTION_GAP_S = ROI_MAX_RANGE_M / V_AIRFRAME_MAX_MPS  # 1.433 s

# Points, i.e. >= 2 consecutive closing steps. NOT raised above 3: the
# matrix's own dwell arithmetic gives 2.8 (T08) to 5.5 (T01) depth clouds
# per tune positive and 3.4-9.0 across the twelve originals, so a threshold
# of 4 would make the hardest positives unattributable by construction
# rather than by evidence.
DEFAULT_MIN_CLOSING_RUN = 3


def closing_runs(
    ranges_by_time: list[tuple[float, float]],
    max_gap_s: float = MAX_DETECTION_GAP_S,
) -> list[list[tuple[float, float]]]:
    """
    ranges_by_time: [(t, range_m), ...], any order.

    Split into MAXIMAL runs of time-consecutive points that are both
    strictly closing (range decreases step to step) and time-contiguous (no
    step longer than max_gap_s). Ties break a run, matching "decreases"
    literally. Maximal runs only — no sub-run search — so the rate test
    below cannot cherry-pick the steepest two steps out of a long shallow
    sequence.
    """
    if not ranges_by_time:
        return []
    ordered = sorted(ranges_by_time, key=lambda p: p[0])
    runs: list[list[tuple[float, float]]] = []
    cur = [ordered[0]]
    for prev, point in zip(ordered, ordered[1:]):
        closing = point[1] < prev[1]
        contiguous = (point[0] - prev[0]) <= max_gap_s
        if closing and contiguous:
            cur.append(point)
        else:
            runs.append(cur)
            cur = [point]
    runs.append(cur)
    return runs


def longest_closing_run(
    ranges_by_time: list[tuple[float, float]],
    max_gap_s: float = MAX_DETECTION_GAP_S,
) -> int:
    """
    Length in points of the longest closing run: the sign-only statistic,
    reported as a descriptive figure beside the rate verdict — but on its
    own it is the refuted statistic (see CRITICAL-2 above) and must never
    again be used as an attribution verdict by itself.

    It is NOT Task 5b's number and must not be compared with one (Task 5c
    fix round 1, LOW-7): 5b's run had no gap bound at all, so two detections
    six seconds apart inside its 8 s window counted as one closing step,
    whereas this one is bounded by max_gap_s. Same sign test, different
    statistic; it can only read lower than 5b's, never higher.
    """
    if not ranges_by_time:
        return 0
    return max(len(r) for r in closing_runs(ranges_by_time, max_gap_s))


def run_closure_rate(run: list[tuple[float, float]]) -> float:
    """
    End-to-end closure rate of one run, m/s: (range_first - range_last) /
    (t_last - t_first). End-to-end rather than per-step so per-frame range
    jitter averages out over the run instead of being amplified by the
    ~0.067 s frame period. Returns 0.0 for a run of fewer than 2 points or
    zero elapsed time.
    """
    if len(run) < 2:
        return 0.0
    dt = run[-1][0] - run[0][0]
    if dt <= 0.0:
        return 0.0
    return (run[0][1] - run[-1][1]) / dt


def attribute_closing_ball(
    ranges_by_time: list[tuple[float, float]],
    ball_speed_mps: float,
    min_run: int = DEFAULT_MIN_CLOSING_RUN,
    max_gap_s: float = MAX_DETECTION_GAP_S,
    v_airframe_max: float = V_AIRFRAME_MAX_MPS,
) -> dict:
    """
    Decide whether a set of in-window detections is attributable to a ball.

    A run here is a maximal sequence of time-consecutive detections in which
    EVERY step both closes (range strictly decreases) and does so at a rate
    inside the band (v_airframe_max, ball_speed_mps + v_airframe_max].
    Attributable iff some such run is >= min_run points long.

      - The band's LOWER edge is the measured maximum ground speed of the
        airframe, so a qualifying run is one that no amount of egomotion
        over static terrain could have produced: range from base_link to a
        stationary point changes at exactly the radial component of the
        drone's own velocity, which never exceeded 3.49 m/s.
      - The band's UPPER edge is that scenario's ball speed plus the same
        airframe maximum — the fastest closure that ball can physically
        produce. A chain of unrelated detections at unrelated depths, which
        is what an FP burst looks like at frame rate, jumps faster than
        that and does not qualify.
      - The band is applied per STEP and the run is DEFINED by it, rather
        than the run being defined by sign and then rate-checked. Both
        matter: per-step rejects a run whose average lands in band because
        one wild jump was offset by two near-static steps, and defining the
        run by the same criterion stops an out-of-band step in the middle
        of a long sign-only sequence from destroying two qualifying halves.
      - max_gap_s is a belt-and-braces bound only. Since an in-band step
        closes at > v_airframe_max and both its ranges lie inside a
        roi_max_range_m-deep ROI, no in-band step can span more than
        roi_max_range_m / v_airframe_max seconds anyway.

    Returns a dict — every field goes into the run artifact (Task 5c
    HIGH-2), because "11 of 12 attributable" reaching only a console log is
    part of why the refuted version survived to be quoted:
      attributable   bool
      longest_run    int    points, longest closing run (any rate), the
                            sign-only descriptive statistic
      best_run       int    points, longest IN-BAND run — the one the
                            verdict is made on
      closure_rate   float  m/s, end-to-end rate of that same longest
                            in-band run, whether or not it qualified
      rate_band      (float, float)
      n_points       int    detections considered
      reason         str    why it failed, when it failed

    On closure_rate when nothing qualified (Task 5c fix round 1, LOW-2):
    the fallback is the longest IN-BAND run, `max(in_band_runs, key=len)`
    — NOT "the longest closing run", which is what this docstring used to
    claim and is not what the code below does. When no step is in band
    every in-band "run" is a single point, run_closure_rate() of a 1-point
    run is 0.0 by definition, and the row prints rate=0.00 (S01, S05, S06,
    S07, S10 all did). That 0.00 means "there was no in-band run at all",
    NOT "the closure rate was measured and came out zero" — completely
    different findings that the bare number cannot tell apart. Render it
    with format_closure_rate(), which says which one it is in words.
    """
    band_lo = v_airframe_max
    band_hi = ball_speed_mps + v_airframe_max
    # The sign-only statistic: descriptive only, never a verdict, and NOT
    # comparable with Task 5b's run length (it is gap-bounded; 5b's was
    # not). See longest_closing_run's docstring.
    longest = longest_closing_run(ranges_by_time, max_gap_s)

    ordered = sorted(ranges_by_time, key=lambda p: p[0])
    in_band_runs: list[list[tuple[float, float]]] = []
    cur = ordered[:1]
    for prev, point in zip(ordered, ordered[1:]):
        dt = point[0] - prev[0]
        rate = (prev[1] - point[1]) / dt if dt > 0.0 else float("inf")
        if dt > 0.0 and dt <= max_gap_s and band_lo < rate <= band_hi:
            cur.append(point)
        else:
            in_band_runs.append(cur)
            cur = [point]
    if ordered:
        in_band_runs.append(cur)

    qualifying = [r for r in in_band_runs if len(r) >= min_run]
    if qualifying:
        best = max(qualifying, key=len)
        return {
            "attributable": True,
            "longest_run": longest,
            "best_run": len(best),
            "closure_rate": run_closure_rate(best),
            "rate_band": (band_lo, band_hi),
            "n_points": len(ranges_by_time),
            "reason": "",
        }

    best_in_band = max((len(r) for r in in_band_runs), default=0)
    fallback = max(in_band_runs, key=len) if in_band_runs else []
    reason = (
        f"longest in-band closing run is {best_in_band} point(s), needs "
        f"{min_run} (sign-only run would be {longest}); band "
        f"({band_lo:.2f}, {band_hi:.2f}] m/s over {len(ranges_by_time)} "
        f"detection(s)"
    )
    return {
        "attributable": False,
        "longest_run": longest,
        "best_run": best_in_band,
        "closure_rate": run_closure_rate(fallback),
        "rate_band": (band_lo, band_hi),
        "n_points": len(ranges_by_time),
        "reason": reason,
    }


def format_closure_rate(attr: dict) -> str:
    """
    Render attribute_closing_ball()'s closure_rate for a human (Task 5c fix
    round 1, LOW-2).

    A closure rate needs at least one in-band STEP to exist, i.e. an
    in-band run of >= 2 points. Below that the dict carries 0.0 purely
    because run_closure_rate() of a single point is 0.0, and a bare
    "rate=0.00" in an artifact row reads as a measured zero closure. Say
    which it is instead.
    """
    if attr.get("best_run", 0) < 2:
        return "n/a(no in-band run)"
    return f"{attr['closure_rate']:.2f}m/s"
