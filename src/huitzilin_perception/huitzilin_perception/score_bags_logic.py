"""
Pure-Python scoring logic for score_bags.py, split out so it is unit-testable
without rclpy (see test/test_score_bags_logic.py, which the ROS-free CI subset
picks up automatically).

Three guards live here, each closing a way the harness could score itself.

Bag-replay exclusion. A bag records the detector's own /threat/centroid track,
and replaying it puts that track back on the bus, where score_bags' subscriber
counts it as a detection by the detector under test. compute_exclude_topics
turns "every topic the node under test publishes" into the --exclude-topics
list for `ros2 bag play`. It refuses (loudly) to exclude /clock or /oak/points,
and refuses a discovery result that does not contain /threat/centroid: a
partial publisher list like ['/parameter_events', '/rosout'] passes every other
guard and silently restores the contamination in full.

The detection window. Enforcing it needs bag_start_sim_t from the sidecar;
without it `in_window = detected` -- any centroid anywhere in the replay -- is
the only path ever taken. A symmetric `abs(t - event_t) <= window_s` gate is
not enough either: at the shipped window_s of 4.0 it spans eight seconds of a
~20 s bag, and its lower edge lands within 0.6 s of bag start, which admits the
cold background map's false-positive burst from the first 1-2 s of every
replay. A positive whose ball was never seen then passes on background alone.
strict_window_bounds() replaces it with an asymmetric, physics-floored window.

Attribution. Finding a run of >= 3 in-window detections of decreasing range
tests only the SIGN of each range difference, which is a stationary sensor's
null hypothesis. These bags were captured under patrol, where the dominant
false-positive class is newly-explored static terrain along a leg -- terrain
the drone translates toward, whose base_link range therefore falls
monotonically frame after frame. The sign test selects for that class, so
attribute_closing_ball() uses the magnitude as well.
"""

from __future__ import annotations

# Bag-replay exclusion
#
# Topics that must never appear in an --exclude-topics list, no matter what a
# (possibly buggy) enumeration produces. /clock is the harness's sole time
# source once the warm-up player hands off (run_regression.sh); excluding it
# starves the detector's clock guard and produces a dead-node 0% recall that
# reads as a detection regression. /oak/points is the detector's subscribed
# INPUT — it never appears in the detector's own publisher list, so its
# presence here would itself be a bug in whatever built the topic list.
FORBIDDEN_EXCLUDE_TOPICS = frozenset({"/clock", "/oak/points"})

# Topics that MUST appear, or the exclusion is not doing its job. score_bags
# subscribes to exactly one topic — /threat/centroid — and that is the topic
# whose recorded capture-time track contaminated every earlier measurement.
# A live publisher enumeration that comes back with
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
      - the enumeration is missing /threat/centroid — a
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
            "score it as the detector's output"
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

    Split out of score_bags._read_bag_start_sim_t purely
    so it can be tested with no bag fixture and no ROS: this one line fixes
    the absolute position of every scenario's detection window, and a
    nanosecond-scaling slip here would shift all seventeen windows in
    lockstep and be nearly invisible in the output.
    """
    return float(sec) + float(nanosec) * 1e-9


# Detection window
#
# capture_scenario.sh fires the projectile SPAWN_LEAD seconds INTO the
# recording, not at bag start. `time_to_closest_s` in the matrix/sidecar is
# relative to SPAWN, not to bag start. One capture script serves both the T
# family and the 17 originals, with no per-family branch, and no revision in
# git history has carried another value (checked across 70d6680, ceff20d,
# 5992359, 8d86873, fd330b8).
SPAWN_LEAD_S = 3.0

# How far AFTER the ball's last airborne instant a genuine detection can land.
#
# The asymmetry is physics, not preference. Everything in the ball's flight
# puts real detections EARLIER than the nominal event: a flat throw from a 2 m
# hover is on the ground 0.639 s after spawn whatever its speed, and an oblique
# throw leaves the 45 deg ROI cone before closest approach. For nine of the
# library's positives the nominal closest approach never happens at all.
#
# The ONLY mechanism that puts a genuine detection LATER is spawn-command
# latency: `ros2 run` costs real time before the projectile exists. Measured on
# the Dell over 3 idle runs -- ros2 CLI startup 0.25-0.49 s, python + rclpy
# init + Node() 0.41-0.74 s, so a 0.7-1.2 s floor before spawn_projectile does
# any work of its own.
#
# PROVENANCE, so a later reader can weigh it: the one end-to-end figure in the
# repo is S05's recorded capture-time centroid, putting the effective lead at
# 4.108 s against the nominal 3.0 -- 1.108 s of latency. S05 is a TRAIN-SPLIT
# ORIGINAL, used as confirmation only of a constant derived from the Dell
# timing and capture_scenario.sh. Read it as partly corroborated, not measured.
#
# 2.0 s is ~1.8x that end-to-end offset and ~1.7-2.9x the `ros2 run` floor, so
# the late edge cannot manufacture a false negative out of spawn latency --
# while cutting the gate from the legacy 8.0 s down to 2.43-2.64 s and removing
# the cold-map FP burst entirely.
POST_EVENT_TOLERANCE_S = 2.0

# The ball's last airborne instant.
#
#   t_last_airborne = min(time_to_closest_s, FLAT_THROW_FALL_TIME_S)
#   upper           = spawn_t + t_last_airborne + min(window_s, post_event_s)
#
# min(), because a flat throw that has not reached closest approach by ground
# impact is still closing when it lands: its ACTUAL closest approach is at
# impact, not at the nominal ttc. When ttc <= the fall time the nominal event
# does happen and the term is unchanged.
#
# This replaced a rule anchored on the NOMINAL closest approach, which was
# widest exactly where the ball is present for the smallest fraction of the
# window: S01 (ttc 1.50 s) ran to spawn + 3.50 s, 2.86 s after impact. The two
# widest windows (S01 3.50 s, S10 3.25 s) returned the two largest in-window
# detection counts in the library, 13 and 16 against 4-8 for the 2.43 s
# windows. The new edge is a STRICT TIGHTENING: min(ttc, T) <= ttc for every
# ttc and T >= 0. Floor and anchor are untouched.
#
# Sources, none of them a measured outcome of this harness:
#   - 2.0 m: bridge.yaml / hw_bridge.yaml takeoff_alt_m, the altitude every
#     Week 3 capture flew (capture_scenario.sh raises it only for the
#     vertical-offset negatives N04/T13, handled below).
#   - flat throw: spawn_projectile.py declares compensate_gravity False and
#     capture_scenario.sh's spawn subshell never passes it, so vz(0) = 0.
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
    2 m hover -- capture_scenario.sh's SPAWN_Z check refuses to spawn unless
    the drone has climbed to >= 10.5 m / 8.0 m first -- so neither the drop height nor
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
        than that is definitionally not the ball, whatever else it is.
        The legacy symmetric gate had no floor at all, which is why its lower
        edge landed within half a second of bag start for every positive in
        the library and swallowed the cold-background-map FP burst that
        detector.yaml documents under cluster_max_extent_m, in the first
        ~1-2 s of every replay.

    upper = spawn_t + min(time_to_closest_s, fall_time_s)
                    + min(window_s, post_event_s)

        Asymmetric and much tighter on the late side, because the ball's own
        physics puts genuine detections BEFORE the nominal event and only
        spawn-command latency can put one after it (see
        POST_EVENT_TOLERANCE_S).

        min(time_to_closest_s, fall_time_s) is the ball's LAST AIRBORNE
        INSTANT relative to spawn, and is where the tolerance is added --
        not to a nominal closest approach the flat throw may never reach. Pass
        fall_time_s=None for a scenario whose geometry the flat-throw model
        does not describe, which restores the nominal-event edge for that
        scenario alone; flat_throw_fall_time_s() decides that.

        `min(window_s, ...)` keeps the sidecar's detection_window_s as a
        hard ceiling. For THIS library that term is entirely non-binding and
        the property is true but vacuous -- capture_scenario.sh writes
        detection_window_s: 4.0 into every sidecar and post_event_s is 2.0,
        so `min` always returns post_event_s and the gate has exactly one
        free parameter. It is kept because a sidecar is data and a future
        capture could ship a tighter one, but nobody should read the
        ceiling as doing work here.

        Both edges together make this window a strict SUBSET of the legacy
        gate and of its first replacement -- recall
        measured under it can only fall or stay equal, never rise.

        RAISES on an inverted window. The
        two edges move in OPPOSITE directions -- the floor reaches FORWARD
        from the event, the clamped late edge reaches BACK from ground
        impact -- so the pair can cross and leave an empty interval. The old
        (pre-clamp) rule could never do that: its upper edge hung off the
        same event_t the floor did. An empty window admits no detection, so
        the positive would have scored FN with no error raised anywhere.

        The exact condition is

            min(ttc, fall_time_s) + min(window_s, post_event_s)
                < max(ttc - window_s, 0)

        which is a SURFACE, not a single crossover -- two independent routes
        reach it, and the second was found by running the guard
        rather than by reading the algebra:

          - small window_s: below (ttc - min(ttc, fall_time_s)) / 2. At
            ttc = 1.5 s that is 0.4307 s, and window_s = 0.25 s gives
            lower = spawn+4.250, upper = spawn+3.889, width -0.361 s.
          - large ttc: above fall_time_s + post_event_s + window_s, once
            min(window_s, post_event_s) has saturated at post_event_s. At
            the shipped window_s = 4.0 that is ttc > 6.639 s.

        The shipped library reaches neither: capture_scenario.sh writes
        detection_window_s: 4.0 into every sidecar and scenario_matrix.yaml's
        largest time_to_closest_s is 1.5 s. Both routes become reachable for
        a future capture -- the first via a tighter sidecar, which is exactly
        the eventuality the retained min(window_s, ...) exists for.

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
    if upper < lower:
        raise ValueError(
            "strict detection window is INVERTED "
            f"(lower={lower - bag_start:+.4f}s upper={upper - bag_start:+.4f}s "
            f"relative to bag_start, width={upper - lower:+.4f}s) for "
            f"time_to_closest_s={time_to_closest_s} window_s={window_s} "
            f"post_event_s={post_event_s} fall_time_s={fall_time_s}. "
            "An inverted window admits nothing, so the positive would score "
            "FN with no error at all — the exact silent failure this gate "
            "exists to prevent. The condition is "
            "min(ttc, fall_time_s) + min(window_s, post_event_s) "
            "< max(ttc - window_s, 0): the floor reaching FORWARD from the "
            "event overtakes the late edge reaching BACK from ground "
            "impact. Two routes reach it — a small window_s (below "
            "(ttc - min(ttc, fall_time_s)) / 2, i.e. ~0.43 s at ttc = 1.5 s) "
            "or a large ttc (above fall_time_s + post_event_s + window_s, "
            "i.e. ~6.6 s at the shipped window_s = 4.0). The shipped library "
            "reaches neither: every sidecar carries detection_window_s: 4.0 "
            "and scenario_matrix.yaml's largest time_to_closest_s is 1.5 s. "
            "Fix the sidecar, or widen window_s; do NOT paper over this by "
            "clamping the edges together."
        )
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

    THIS is the production predicate:
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
    Absolute bounds of the legacy symmetric unfloored gate, [event - w,
    event + w]. Exists so the artifact can report how many detections that
    gate admitted (n_win_sym) beside how many this one does -- the ratio is the only number that answers "the
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
    The legacy gate exactly as it shipped: `abs(t - event_t) <= window_s`,
    symmetric and unfloored. Kept ONLY so every run artifact can print the
    old verdict beside the new one and make the strict-window change
    visible per scenario. It is never what decides pass/fail.
    """
    lower, upper = symmetric_window_bounds(
        bag_start, time_to_closest_s, window_s, spawn_lead_s
    )
    return count_in_window(detections, lower, upper) > 0


def is_in_window_loose(
    detections: list[float], bag_start: float, window_s: float
) -> bool:
    """
    The legacy semantics, reproduced verbatim for comparison only: at least
    one detection with (t - bag_start) <= window_s.

    Note what that expression actually computes: it has no
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


# Attribution
#
# The bags do NOT carry ground-truth ball position (no /gz/dynamic_poses), so
# attribution cannot compare a detection against a known trajectory. What IS
# available is the range from base_link that
# /threat/centroid already carries (detector_node.py stamps it
# frame_id="base_link", so sqrt(x^2+y^2+z^2) is range from the airframe, not
# from a world origin — no frame conversion is involved anywhere below).
#
# The original test used only the SIGN of successive range differences. Under patrol
# that is inverted: the drone translates toward newly-explored terrain, whose
# base_link range then decreases monotonically frame after frame, and
# detector.yaml's cluster_max_extent_m note names exactly that terrain as this
# library's dominant FP mechanism. Independently quantified over 200 000
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
# Provenance: CLAUDE.md records this figure
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

# Two detections of the SAME object cannot be arbitrarily far apart in time.
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
    own it is the refuted statistic (see the rate verdict above) and must
    never again be used as an attribution verdict by itself.

    It is not the legacy number and must not be compared with one: the legacy
    run had no gap bound at all, so two detections six seconds apart inside
    its 8 s window counted as one closing step,
    whereas this one is bounded by max_gap_s. Same sign test, different
    statistic; it can only read lower than the legacy statistic's, never higher.
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

    Returns a dict — every field goes into the run artifact, because "11 of 12 attributable" reaching only a console log is
    part of why the refuted version survived to be quoted:
      attributable bool
      longest_run int points, longest closing run (any rate), the
                            sign-only descriptive statistic
      best_run int points, longest IN-BAND run — the one the
                            verdict is made on
      closure_rate float m/s, end-to-end rate of that same longest
                            in-band run, whether or not it qualified
      rate_band (float, float)
      n_points int detections considered
      reason str why it failed, when it failed

    On closure_rate when nothing qualified:
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
    # comparable with the legacy run length (it is gap-bounded; the legacy one was
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
    Render attribute_closing_ball()'s closure_rate for a human.

    A closure rate needs at least one in-band STEP to exist, i.e. an
    in-band run of >= 2 points. Below that the dict carries 0.0 purely
    because run_closure_rate() of a single point is 0.0, and a bare
    "rate=0.00" in an artifact row reads as a measured zero closure. Say
    which it is instead.
    """
    if attr.get("best_run", 0) < 2:
        return "n/a(no in-band run)"
    return f"{attr['closure_rate']:.2f}m/s"
