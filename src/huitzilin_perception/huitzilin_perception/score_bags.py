"""
score_bags.py — HuitzilinReflex Week 3, W3-16 + W3-17.

Offline regression harness: replays each labeled rosbag through the detector
and scores detection recall and false-positive rate against the label sidecars.

USAGE
-----
  ros2 run huitzilin_perception score_bags \
      --ros-args \
      -p bag_dir:=/data/huitzilin_bags \
      -p scenario_matrix:=/path/to/scenario_matrix.yaml \
      -p split:=test \
      -p recall_floor:=0.95 \
      -p output_file:=/tmp/week3_regression.txt

  # Or via the helper script (sets use_sim_time automatically):
  ./scripts/run_regression.sh

ALGORITHM
---------
For each scenario in the chosen split:
  1. Find the matching .mcap bag in bag_dir (by scenario id prefix).
  2. Read the scenario label from the bag's sidecar .label.yaml.
  3. Replay: ros2 bag play <bag> --clock --exclude-topics <detector's own
     output topics> (we drive replay via subprocess, node reads published
     topics). The exclusion is load-bearing: T-bags were captured with a
     live perception stack running, so each bag CONTAINS that stack's own
     recorded /threat/centroid (and /threat/marker) track. Replaying it
     without excluding those topics republishes that recorded track and
     score_bags' subscriber counts it as a detection by the detector UNDER
     TEST. Exclusion topics are discovered from the
     LIVE detector node's own publisher list (see _discover_exclude_topics),
     not hardcoded here, so a future debug publisher cannot reopen this hole
     silently.
  4. Listen on /threat/centroid for the duration of the bag.
  5. Score:
     - Positive scenario: centroid published inside the strict window
       [max(event - detection_window_s, bag_start + SPAWN_LEAD_S),
        event + min(detection_window_s, POST_EVENT_TOLERANCE_S)],
       event = bag_start + SPAWN_LEAD_S + time_to_closest_s (all three
       required — see the detection-window note below and score_bags_logic's
       strict_window_bounds) → TP else FN
     - Negative scenario: centroid published at any point → FP else TN
  5b. Attribute: decide whether each TP is a ball or background that merely
     landed inside the window, from the closure RATE of a contiguous
     closing run of in-window centroids (attribute_closing_ball). The same
     test runs over every negative as a falsification control.

     TWO KINDS OF NEGATIVE, and only one of them is a null control:
       - GENUINELY BALL-FREE (speed_mps 0.0 — N01, N02, N05, T11, T14): no
         spawn command is ever issued (capture_scenario.sh spawns only when
         the sidecar's speed is > 0), so
         the bag contains no projectile. Attributing a ball in one of these
         refutes the method, full stop. This is the falsification control.
       - BALL PRESENT, MUST NOT BE DETECTED (N03/T12 behind the camera at
         8/11 m/s, N04/T13 spawned 10.0/7.5 m below): a projectile IS
         spawned. They are negatives in the sense of "wrong bearing, wrong
         elevation, the gates must suppress it", not "contains no ball".
         Whether attribution fires on these is interesting — it is not a
         null result either way, because there is a real ball in the bag.
     Both kinds are scored, which is correct; the aggregate reports the
     ball-free subset separately because only that subset can falsify.
  6. Aggregate TP/FP/TN/FN → recall, precision, FP rate.

All timing uses message stamps (use_sim_time), never wall-clock.

The detection window — it was never actually enforced: no sidecar ever
carried `bag_start_sim_t`, so every run silently fell back to "detected
somewhere in the whole replay", which the FP background alone can satisfy.
bag_start is now measured directly — but NOT by subscribing to the live
/clock topic during replay, despite that being the obvious reading of "the
first /clock stamp is the honest bag-start in sim time." Verified on the
`ros2 bag play --clock` does not republish a bag's own
recorded /clock payload during replay. It SYNTHESISES its own /clock stream
from real wall-clock playback progress (`ros2 topic echo /clock --once`
during an active replay returned `sec: 1786950555`, matching that day's
wall-clock date, against the bag's own recorded /clock payload of ~1036 —
Gazebo sim time from the original capture session). That stream exists to
satisfy downstream nodes' use_sim_time / clock_guard machinery, not to
reproduce the bag's original sim-time epoch — an earlier version of this fix
subscribed to it live and scored T01-T03 as false negatives purely from
comparing a wall-clock bag_start against sim-time detection stamps, caught
before being reported. bag_start is instead read directly from the bag
FILE's own recorded /clock payload, decoupled from replay entirely (see
_read_bag_start_sim_t) — the sidecar's `bag_start_sim_t` key is no longer
read either way. A bag with no recorded /clock, or a positive sidecar with
no `time_to_closest_s`, fails loudly (see _score_one) rather than silently
passing.

The anchor is bag_start + SPAWN_LEAD_S + time_to_closest_s, NOT bag_start +
time_to_closest_s: the projectile does not exist until SPAWN_LEAD_S seconds
into the recording (capture_scenario.sh), and time_to_closest_s is defined
relative to spawn, not bag start (scenario_matrix.yaml:18). An initial
version of this fix anchored at bag_start + time_to_closest_s directly and
scored T01-T04 as false negatives that were purely a ~3 s anchoring error,
not a detector regression — caught before being reported. See
score_bags_logic.SPAWN_LEAD_S for the empirical validation.

OUTPUT
------
Prints a per-scenario table plus aggregate recall + FP rate.
Writes the same to output_file.
Exits with code 1 if recall < recall_floor (for CI/regression gating).

NOTES
-----
- Must be run with use_sim_time:=true (set in launch or via --ros-args).
- Requires ros2 bag play and the detector node in the same session.
  The launch file week3_perception.launch.py handles this setup.
"""

from __future__ import annotations

import math
import subprocess
import sys
import time
import threading
import traceback
from pathlib import Path
from typing import Optional

import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock

from huitzilin_perception.score_bags_logic import (
    LIBRARY_MAX_BALL_SPEED_MPS,
    POST_EVENT_TOLERANCE_S,
    REQUIRED_EXCLUDE_TOPICS,
    SPAWN_LEAD_S,
    attribute_closing_ball,
    build_bag_play_cmd,
    clock_msg_to_sim_t,
    compute_exclude_topics,
    count_in_window,
    flat_throw_fall_time_s,
    format_closure_rate,
    is_in_window_loose,
    is_in_window_strict,
    is_in_window_symmetric_legacy,
    strict_window_bounds,
    symmetric_window_bounds,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Label sidecar format
#
# Each bag <bag_dir>/week3_<id>_<ts>.mcap has a sidecar:
#   <bag_dir>/week3_<id>.label.yaml
#
# week3_S01.label.yaml:
#   scenario_id: S01
#   label: positive           # positive | negative
#   closest_approach_m: 0.0
#   time_to_closest_s: 1.5    # sim time from spawn to closest approach
#   detection_window_s: 3.0   # centroid must fire within this sim-time window
#
# bag_start_sim_t is intentionally NOT a sidecar field: score_bags reads it
# itself from each bag's own recorded /clock payload (see
# _read_bag_start_sim_t) rather than trusting a recorded value that no
# sidecar has ever actually carried. It is read from the
# BAG FILE directly, not from a live /clock subscription during replay —
# `ros2 bag play --clock` synthesises its own wall-clock-anchored /clock
# stream rather than republishing the bag's recorded payload; see the module
# docstring for how that was discovered.

DETECTION_WINDOW_DEFAULT_S = 4.0  # sim seconds; override per scenario in sidecar

# How many times / how long to retry ROS-graph discovery of the detector
# node's publisher list before giving up. The detector is started by
# run_regression.sh before score_bags, but graph propagation can lag by a
# few hundred ms after the node comes up.
TOPIC_DISCOVERY_RETRIES = 10
TOPIC_DISCOVERY_RETRY_S = 0.5


class ScorerNode(Node):
    """Listens on /threat/centroid and records detections during bag replay."""

    def __init__(self) -> None:
        super().__init__("score_bags")

        self.declare_parameter("bag_dir", "/data/huitzilin_bags")
        self.declare_parameter("scenario_matrix",
                               "config/scenario_matrix.yaml")
        self.declare_parameter("split", "test")   # train|test|tune|heldout|all
        # Filename prefix of the bags to score. "week3_" is every bag captured
        # in earlier runs and is the default so no existing invocation
        # changes. The held-out set uses "heldout_", because a bag named
        # week3_H07 carrying the headline recall number is exactly the kind of
        # mislabelling a sceptical reader is right to distrust -- those bags
        # are not Week 3 data, were captured on a different world, against a
        # different camera, under a fixed protocol.
        self.declare_parameter("bag_prefix", "week3_")
        self.declare_parameter("recall_floor", 0.95)
        self.declare_parameter("output_file", "/tmp/week3_regression.txt")
        # The node under test. Its own publisher list — discovered live from
        # the ROS graph, not hardcoded — is what gets excluded from bag
        # replay.
        self.declare_parameter("detector_node_name", "detector")

        self._bag_dir   = Path(self.get_parameter("bag_dir").value)
        self._matrix_f  = Path(self.get_parameter("scenario_matrix").value)
        self._split     = self.get_parameter("split").value
        self._floor     = self.get_parameter("recall_floor").value
        self._out_file  = Path(self.get_parameter("output_file").value)
        self._detector_node_name = self.get_parameter("detector_node_name").value
        self._bag_prefix = self.get_parameter("bag_prefix").value

        # Track detections during current bag replay. bag_start is no longer
        # tracked as live subscriber state — see _read_bag_start_sim_t.
        self._detection_times: list[float] = []
        # Full (t, x, y, z) alongside the timestamps above — kept separate so
        # is_in_window_strict/loose's pure-function signature (list[float])
        # never has to change. Only used for attribution: a real
        # ball produces a closing sequence of centroids with DECREASING
        # range across frames, first-sight terrain/patrol-motion noise does
        # not — see _attributable_to_ball.
        self._detection_positions: list[tuple[float, float, float, float]] = []
        self._listening = False

        # Topics to exclude from every `ros2 bag play` — discovered once in
        # run(), before the scoring loop, via _discover_exclude_topics().
        self._exclude_topics: list[str] = []

        self._sub = self.create_subscription(
            PointStamped,
            "/threat/centroid",
            self._centroid_cb,
            RELIABLE_QOS,
        )
        self.get_logger().info(
            f"score_bags ready | bag_dir={self._bag_dir} split={self._split}"
        )

    def _centroid_cb(self, msg: PointStamped) -> None:
        if not self._listening:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._detection_times.append(t)
        self._detection_positions.append(
            (t, msg.point.x, msg.point.y, msg.point.z)
        )

    def start_listening(self) -> None:
        self._detection_times = []
        self._detection_positions = []
        self._listening = True

    def stop_listening(self) -> list[float]:
        self._listening = False
        return list(self._detection_times)

    # Main scoring logic

    def run(self) -> int:
        """
        Iterate the scenario matrix, replay bags, score.
        Returns exit code (0 = pass, 1 = regression).
        """
        if not self._matrix_f.exists():
            self.get_logger().error(f"Scenario matrix not found: {self._matrix_f}")
            return 1

        try:
            self._exclude_topics = self._discover_exclude_topics()
        except RuntimeError as e:
            # Loud and fatal on purpose: a run that cannot
            # establish what to exclude must not fall back to excluding
            # nothing, which is exactly the self-scoring defect this harness
            # exists to prevent.
            self.get_logger().error(str(e))
            return 1
        self.get_logger().info(
            f"excluding from every bag replay: {self._exclude_topics}"
        )

        with open(self._matrix_f) as f:
            matrix = yaml.safe_load(f)

        split_ids: list[str]
        if self._split == "all":
            split_ids = [s["id"] for s in matrix["scenarios"]]
        else:
            splits = matrix.get("split", {})
            if self._split not in splits:
                # A typo scored ZERO scenarios and reported a clean run: the
                # loop below simply never executed, and the artifact came out
                # with an empty table and no error anywhere. That is the same
                # silent-empty failure class as the exclusion-topic bug this
                # module already guards, and it matters more now that there
                # is a `heldout` split whose whole value is being scored once.
                raise ValueError(
                    f"split {self._split!r} is not in "
                    f"{self._matrix_f} — known splits are "
                    f"{sorted(splits)} plus the literal 'all'. Scoring an "
                    "unknown split would report an empty run as a clean one.")
            split_ids = splits[self._split]

        scenarios = {s["id"]: s for s in matrix["scenarios"]}
        results: list[dict] = []

        for sid in split_ids:
            if sid not in scenarios:
                self.get_logger().warn(f"Scenario {sid} not in matrix; skipping.")
                continue
            scen = scenarios[sid]
            result = self._score_one(scen)
            results.append(result)
            status = "✓" if result["pass"] else "✗"
            self.get_logger().info(
                f"  [{status}] {sid:4s} {scen['label']:8s} | "
                f"detected={result['detected']} | {result['note']}"
            )

        return self._report(results)

    def _discover_exclude_topics(self) -> list[str]:
        """
        Ask the live ROS graph what the node under test actually publishes,
        and turn that into an --exclude-topics list. Retries briefly because
        the detector may have just come up (run_regression.sh starts it
        immediately before score_bags) and this node's own rmw participant
        can take a beat to match it even after `ros2 node list` elsewhere
        already sees it — get_publisher_names_and_types_by_node RAISES
        (rather than returning an empty list) while that graph cache is
        still catching up, so the raise itself must be treated as "not
        found yet, retry", not as an immediate fatal error. Raises
        RuntimeError — never returns an empty list — only once every retry
        is exhausted: run() treats that as fatal rather than silently
        scoring with no exclusion at all. the loop used to break on the first NON-EMPTY
        result. rclpy creates /parameter_events and /rosout inside
        Node.__init__, i.e. before detector_node.py creates
        /threat/centroid, and DDS propagates endpoints asynchronously — so
        a graph read of ['/parameter_events', '/rosout'] is a real
        intermediate state, it satisfied every guard, and it produced an
        exclusion list without the one topic that matters. That silently
        restored the self-scoring contamination in full. The break now requires the topic the
        harness actually subscribes to; compute_exclude_topics is the
        second, load-bearing check on the same condition.
        """
        last_seen: list[str] = []
        last_exc: Optional[Exception] = None
        for attempt in range(TOPIC_DISCOVERY_RETRIES):
            try:
                pubs = self.get_publisher_names_and_types_by_node(
                    self._detector_node_name, "/"
                )
                last_seen = [name for name, _types in pubs]
                last_exc = None
                if REQUIRED_EXCLUDE_TOPICS.issubset(set(last_seen)):
                    break
                if last_seen:
                    self.get_logger().warn(
                        f"partial publisher list for "
                        f"'{self._detector_node_name}' on attempt "
                        f"{attempt + 1}: {sorted(last_seen)} — still waiting "
                        f"for {sorted(REQUIRED_EXCLUDE_TOPICS)}"
                    )
            except Exception as e:  # node not yet visible in this graph cache
                last_exc = e
                last_seen = []
            time.sleep(TOPIC_DISCOVERY_RETRY_S)
        try:
            return compute_exclude_topics(last_seen)
        except ValueError as e:
            detail = f"; last discovery error: {last_exc!r}" if last_exc else ""
            raise RuntimeError(
                f"could not establish a safe --exclude-topics list for node "
                f"'{self._detector_node_name}' after "
                f"{TOPIC_DISCOVERY_RETRIES} attempts{detail}: {e}"
            ) from e

    def _unusable_window_row(
        self, sid: str, label: str, detected: bool,
        window_s, time_to_closest_s, exc: Exception,
    ) -> dict:
        """
        Turn a refused detection window into a per-bag error row. strict_window_bounds() raises on an
        inverted window and that raise is CORRECT and stays:
        an inverted window admits nothing, so scoring under it would post a
        silent FN — the exact failure the gate exists to prevent. What was
        wrong was the blast radius. The raise reached run()'s scenario loop
        unguarded, so ONE broken sidecar aborted the entire run: _report()
        was never reached, so no artifact was written even for the bags
        already scored, and every later bag went unreplayed.

        A refused window is the same kind of thing as the three conditions
        already guarded in _score_one — a broken input, discovered per bag,
        at scoring time. So it gets the same treatment: error=True, which
        _report() partitions out of TP/FN/TN/FP and out of recall before it
        computes anything, and which forces a FAIL verdict naming this
        scenario. It must NOT enter the confusion matrix (counting a harness
        error as a detection result is what once put "FP rate 25%" on a run
        with zero actual false positives).

        The note is deliberately compact — it lands in a fixed-width table
        row AND in the verdict line — so the exception's full text (which
        names both inversion routes and the arithmetic) goes to the log
        instead. The two numbers a reader needs to act are in the row.
        """
        self.get_logger().error(f"    {sid} strict window refused: {exc}")
        return {
            "id": sid, "label": label, "detected": detected,
            "pass": False, "error": True,
            "note": (
                f"STRICT WINDOW REFUSED — detection_window_s={window_s} "
                f"time_to_closest_s={time_to_closest_s} give an empty "
                f"window; fix the sidecar or widen the window (full "
                f"condition in the node log)"
            ),
        }

    def _score_one(self, scen: dict) -> dict:
        sid = scen["id"]
        label = scen["label"]

        bag_path = self._find_bag(sid)
        if bag_path is None:
            # A missing bag is a harness error, NOT a detection result: it
            # must not enter the confusion matrix (counting N04 as FP put
            # "FP rate 25%" on a run with zero actual false positives).
            return {
                "id": sid, "label": label, "detected": False,
                "pass": False, "error": True,
                "note": "BAG MISSING — capture it (scripts/capture_scenario.sh)",
            }

        sidecar = self._load_sidecar(sid)
        window_s = sidecar.get("detection_window_s", DETECTION_WINDOW_DEFAULT_S)

        # Read straight from the bag FILE, before replay — not from a live
        # /clock subscription during replay (see the module docstring and
        # _read_bag_start_sim_t for why that is the wrong source).
        bag_start = self._read_bag_start_sim_t(bag_path)
        if bag_start is None:
            # "No window could be established" must fail loudly, never
            # quietly pass as "detected somewhere". A bag with
            # no recorded /clock is a harness/capture error, not a detector
            # result — exclude it from the confusion matrix the same way a
            # missing bag is excluded, rather than crashing the whole run.
            return {
                "id": sid, "label": label, "detected": False,
                "pass": False, "error": True,
                "note": "NO /clock IN BAG FILE — cannot establish bag-start "
                        "sim time; window cannot be enforced",
            }

        self.start_listening()
        bag_duration = self._replay_bag(bag_path)
        detections = self.stop_listening()

        detected = len(detections) > 0

        # Ball speed for the attribution rate band. Positives use their own
        # scenario's speed. Negatives are scored against the LIBRARY MAXIMUM
        # band — the most permissive test any scenario gets — because the
        # ball-free ones carry speed_mps 0.0, which would make their band
        # empty and the control unfalsifiable by construction.
        speed_mps = float(scen.get("speed_mps") or 0.0)
        ctl_speed = LIBRARY_MAX_BALL_SPEED_MPS
        # A negative is "ball-free" only when no projectile was spawned at
        # all. capture_scenario.sh spawns iff speed > 0, so speed_mps
        # 0.0 IS the ball-free test. N03/N04/T12/T13 carry 8.0/8.0/11.0/11.0
        # — a real ball, aimed where the detector must not see it.
        ball_free = speed_mps == 0.0
        # The flat-throw ground-impact clamp on the window's late edge only
        # describes a throw launched level from the standard 2 m hover;
        # flat_throw_fall_time_s returns None for the vertical-offset
        # scenarios (N04, T13), which restores the nominal-event edge for
        # those alone. See its docstring.
        fall_time_s = flat_throw_fall_time_s(
            float(scen.get("offset_vertical_m") or 0.0)
        )
        all_ranges = [
            (t, math.sqrt(x * x + y * y + z * z))
            for (t, x, y, z) in self._detection_positions
        ]

        if label == "positive":
            time_to_closest_s = sidecar.get("time_to_closest_s")
            if time_to_closest_s is None:
                # Same principle: a positive sidecar missing the field the
                # strict window needs must fail loudly, not silently fall
                # back to "any detection in the whole replay counts".
                return {
                    "id": sid, "label": label, "detected": detected,
                    "pass": False, "error": True,
                    "note": "sidecar missing time_to_closest_s — strict "
                            "window cannot be enforced",
                }
            # The window is asymmetric: floored at spawn, and clamped at
            # ground impact on the late side.
            #   lower = max(event - window_s, bag_start + SPAWN_LEAD_S)
            #   upper = spawn + min(ttc, fall_time)
            #                 + min(window_s, POST_EVENT_TOLERANCE_S)
            # See strict_window_bounds().
            window_kwargs = dict(
                spawn_lead_s=SPAWN_LEAD_S,
                post_event_s=POST_EVENT_TOLERANCE_S,
                fall_time_s=fall_time_s,
            )
            try:
                lower, upper = strict_window_bounds(
                    bag_start, time_to_closest_s, window_s, **window_kwargs
                )
                # Call the tested predicate rather than re-inlining it: the
                # comparison that decides pass/fail is the one that must
                # carry the regression tests.
                in_window_strict = is_in_window_strict(
                    detections, bag_start, time_to_closest_s, window_s,
                    **window_kwargs
                )
            except ValueError as e:
                # NARROW on purpose: only the two calls that can refuse a
                # window are inside the try. See _unusable_window_row.
                return self._unusable_window_row(
                    sid, label, detected, window_s, time_to_closest_s, e
                )
            # Both superseded gates, computed for the artifact only so the
            # strict-window change is visible per scenario rather than only
            # in aggregate: `sym` is the legacy symmetric unfloored gate, `loose`
            # is the pre-5b "anywhere in the first window_s" gate.
            in_window_sym = is_in_window_symmetric_legacy(
                detections, bag_start, time_to_closest_s, window_s,
                spawn_lead_s=SPAWN_LEAD_S,
            )
            in_window_loose = is_in_window_loose(detections, bag_start, window_s)
            passed = in_window_strict
            # How many detections each gate ADMITS, not just whether it
            # fires. n_win/n_win_sym is the only
            # number that answers "strict, sym and loose still agree on
            # every positive — is the new gate doing anything at all?", and
            # it belongs in the artifact where it can be recomputed rather
            # than in a report's prose.
            sym_lower, sym_upper = symmetric_window_bounds(
                bag_start, time_to_closest_s, window_s,
                spawn_lead_s=SPAWN_LEAD_S,
            )
            n_win_sym = count_in_window(detections, sym_lower, sym_upper)

            event_rel = SPAWN_LEAD_S + time_to_closest_s
            det_rel = sorted(round(t - bag_start, 3) for t in detections)
            self.get_logger().info(
                f"    {sid} detections_rel_s={det_rel} "
                f"event_rel_s={event_rel:.3f} "
                f"strict_window_rel=[{lower - bag_start:.3f}, "
                f"{upper - bag_start:.3f}]"
            )

            # Attribution: is this TP a real ball, or
            # background that merely landed inside the window? Restricted to
            # detections actually inside the strict window — a closing run
            # elsewhere in a 20-45 s bag says nothing about THIS TP.
            in_window_ranges = [
                (t, r) for (t, r) in all_ranges if lower <= t <= upper
            ]
            attr = attribute_closing_ball(in_window_ranges, speed_mps)
            attributable = attr["attributable"]
            closing_run = attr["longest_run"]
            # Raw evidence behind the verdict. The (time, range) pairs are
            # what let an attribution verdict be re-derived offline without
            # another
            # ~13-minute Dell run — which is exactly what was needed to
            # catch a mis-derived constant in this very function.
            self.get_logger().info(
                f"    {sid} in_window_t_range="
                + str([(round(t - bag_start, 3), round(r, 3))
                       for (t, r) in sorted(in_window_ranges)])
            )
            # t_last_rel makes the "POST_EVENT_TOLERANCE_S is non-binding"
            # claim checkable from the artifact instead of from a console
            # log nobody keeps: compare it against
            # win_rel's upper edge.
            in_win_times = [t for t in detections if lower <= t <= upper]
            t_last_rel = (
                f"{max(in_win_times) - bag_start:.2f}" if in_win_times else "-"
            )
            note = (
                f"win_rel=[{lower - bag_start:.2f},{upper - bag_start:.2f}] "
                f"strict={in_window_strict} sym={in_window_sym} "
                f"loose={in_window_loose} n_det={len(detections)} "
                f"n_win={attr['n_points']} n_win_sym={n_win_sym} "
                f"t_last_rel={t_last_rel} run={attr['longest_run']} "
                f"rate={format_closure_rate(attr)} "
                f"band=({attr['rate_band'][0]:.2f},{attr['rate_band'][1]:.2f}] "
                f"attributable={attributable}"
            )
            self.get_logger().info(
                f"    {sid} attributable={attributable} {attr['reason']}"
            )
        else:  # negative
            # TN: no centroid at all, anywhere in the replay. Unaffected by
            # the window logic above — a negative has no closest-approach
            # time to anchor to.
            passed = not detected

            # NEGATIVE CONTROL. Every negative carries
            # tens of live false positives, so these are the control
            # population an attribution test must survive. The original test set
            # attributable=None here and so could never falsify its own
            # method.
            #
            # READ THE `ball_free` FLAG BEFORE QUOTING ANY COUNT FROM THIS
            # BRANCH. Only the speed_mps 0.0 bags
            # (N01, N02, N05, T11, T14) contain no projectile and can
            # therefore falsify. N03/N04/T12/T13 have a real ball in them,
            # thrown away from or below the camera; a firing there is a
            # different and weaker finding — see the module docstring.
            #
            # Two reads, both against the most permissive band in the
            # library (ctl_speed), i.e. the easiest test any scenario gets:
            #   ctl_win  — the same window rule a positive gets, with this
            #              scenario's own time_to_closest_s (0 for every
            #              negative), so the window width is comparable.
            #   ctl_all  — EVERY detection in the whole replay. This is the
            #              decisive one: if the rule attributes a ball
            #              anywhere in a ball-free bag, it is refuted.
            ctl_ttc = float(sidecar.get("time_to_closest_s") or 0.0)
            try:
                ctl_lower, ctl_upper = strict_window_bounds(
                    bag_start, ctl_ttc,
                    window_s, spawn_lead_s=SPAWN_LEAD_S,
                    post_event_s=POST_EVENT_TOLERANCE_S,
                    fall_time_s=fall_time_s,
                )
            except ValueError as e:
                # Guarded too, though it needs a worse sidecar to reach: at
                # the ttc = 0.0 every negative carries, the window inverts
                # only for a NEGATIVE detection_window_s (lower = spawn +
                # |window_s|, upper = spawn + window_s). Nothing validates
                # that field, so the route is real, and an unguarded raise
                # here would take the whole run down exactly as the positive
                # one did.
                return self._unusable_window_row(
                    sid, label, detected, window_s, ctl_ttc, e
                )
            ctl_win_ranges = [
                (t, r) for (t, r) in all_ranges if ctl_lower <= t <= ctl_upper
            ]
            attr_win = attribute_closing_ball(ctl_win_ranges, ctl_speed)
            attr_all = attribute_closing_ball(all_ranges, ctl_speed)
            attributable = attr_all["attributable"]
            closing_run = attr_all["longest_run"]
            note = (
                # win_rel on negatives too: ctl_win is
                # uninterpretable without the width of the window it read.
                f"win_rel=[{ctl_lower - bag_start:.2f},"
                f"{ctl_upper - bag_start:.2f}] "
                f"ball_free={ball_free} "
                f"fp_count={len(detections)} "
                f"n_win={attr_win['n_points']} "
                f"ctl_win={attr_win['attributable']}"
                f"(run={attr_win['longest_run']},"
                f"rate={format_closure_rate(attr_win)}) "
                f"ctl_all={attr_all['attributable']}"
                f"(run={attr_all['longest_run']},"
                f"rate={format_closure_rate(attr_all)}) "
                f"band=({attr_all['rate_band'][0]:.2f},"
                f"{attr_all['rate_band'][1]:.2f}]"
            )
            self.get_logger().info(
                f"    {sid} NEGATIVE CONTROL ctl_all="
                f"{attr_all['attributable']} {attr_all['reason']}"
            )
            self.get_logger().info(
                f"    {sid} all_t_range="
                + str([(round(t - bag_start, 3), round(r, 3))
                       for (t, r) in sorted(all_ranges)])
            )

        return {
            "id": sid, "label": label,
            "detected": detected, "pass": passed,
            "detections": len(detections),
            "bag_duration_s": bag_duration,
            "note": note,
            "attributable": attributable,
            "closing_run": closing_run,
            "ball_free": ball_free,
        }

    def _find_bag(self, sid: str) -> Optional[Path]:
        """First bag in bag_dir whose name starts with <bag_prefix><sid>."""
        prefix = f"{self._bag_prefix}{sid}"
        for f in sorted(self._bag_dir.glob(f"{prefix}*.mcap")):
            return f
        # Also check subdirectory bags (ros2 bag play uses a directory)
        for d in sorted(self._bag_dir.glob(f"{prefix}*/")):
            return d
        return None

    def _load_sidecar(self, sid: str) -> dict:
        sidecar_path = (self._bag_dir / f"{self._bag_prefix}{sid}.label.yaml")
        if sidecar_path.exists():
            with open(sidecar_path) as f:
                return yaml.safe_load(f) or {}
        return {}

    def _read_bag_start_sim_t(self, bag_path: Path) -> Optional[float]:
        """
        Read the bag's OWN recorded first /clock message directly from the
        mcap file, decoupled from replay entirely. Returns None if the bag
        has no recorded /clock topic (or can't be opened) — callers must
        treat that as fatal for the bag, never fall back silently.

        Why not just subscribe to /clock during replay (the seemingly
        obvious reading of "the first /clock stamp is the honest bag-start
        in sim time")? Verified on the `ros2 bag play
        --clock` does NOT republish a bag's recorded /clock payload. It
        SYNTHESISES its own /clock stream from real wall-clock playback
        progress, to satisfy downstream nodes' use_sim_time / clock_guard
        machinery — `ros2 topic echo /clock --once` during an active T01
        replay returned `sec: 1786950555` (that day's wall-clock date)
        while the bag's own recorded /clock payload (read here, from the
        file) is ~1036 (Gazebo sim time from the original capture session).
        A live subscription during replay therefore measures the WRONG
        clock; only the file's own payload is honest bag-start sim time.
        This does not affect detection scoring itself: /threat/centroid
        stamps are copied from the incoming cloud's own header.stamp
        (detector_node.py), which IS the bag's original recorded Gazebo
        sim-time payload — only bag_start needed this fix. storage_id is left empty so rosbag2 reads it from
        the bag's own metadata.yaml. It used to be hardcoded "mcap" while
        _find_bag will happily return a DIRECTORY bag, so a directory bag
        with sqlite3 storage failed the whole run on a bag `ros2 bag play`
        would have replayed perfectly. The reader is also closed explicitly
        rather than left to refcounting (31 opens per run), and the
        sec/nanosec arithmetic moved to score_bags_logic.clock_msg_to_sim_t
        so it is covered by the ROS-free test suite.
        """
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path))
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            self.get_logger().error(f"could not open {bag_path} for bag_start "
                                    f"read: {e}")
            return None
        try:
            reader.set_filter(rosbag2_py.StorageFilter(topics=["/clock"]))
            if not reader.has_next():
                return None
            _topic, data, _t = reader.read_next()
            msg = deserialize_message(data, Clock)
            return clock_msg_to_sim_t(msg.clock.sec, msg.clock.nanosec)
        finally:
            try:
                reader.close()
            except Exception:
                pass

    def _replay_bag(self, bag_path: Path) -> float:
        """
        Replay a bag with ros2 bag play --clock, EXCLUDING every topic the
        detector under test publishes (self._exclude_topics, discovered once
        in run() — see _discover_exclude_topics), and wait for it to finish.
        Returns approximate sim duration (seconds). Non-blocking spin runs
        in background via thread.

        The exclusion is fix: T-bags were captured with a
        live perception stack running, so they contain that stack's own
        recorded /threat/centroid (and /threat/marker) track. Without
        --exclude-topics, replaying the bag republishes that recorded track
        and _centroid_cb counts it as a detection by the detector under test
        — wired into build_bag_play_cmd so it cannot be forgotten by a future
        caller the way the bare cmd list here used to allow.
        """
        cmd = build_bag_play_cmd(str(bag_path), self._exclude_topics, rate=1.0)
        t_start = time.monotonic()
        try:
            proc = subprocess.run(cmd, timeout=120, capture_output=True)
            if proc.returncode not in (0, -2):  # -2 = SIGINT (normal end)
                self.get_logger().warn(
                    f"bag play returned {proc.returncode}: {proc.stderr.decode()[:200]}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"bag replay timed out for {bag_path.name}")
        return time.monotonic() - t_start

    # Reporting

    def _report(self, results: list[dict]) -> int:
        errors    = [r for r in results if r.get("error")]
        scored    = [r for r in results if not r.get("error")]
        positives = [r for r in scored if r["label"] == "positive"]
        negatives = [r for r in scored if r["label"] == "negative"]

        tp = sum(1 for r in positives if r["pass"])
        fn = sum(1 for r in positives if not r["pass"])
        tn = sum(1 for r in negatives if r["pass"])
        fp = sum(1 for r in negatives if not r["pass"])

        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        fp_rate = fp / (fp + tn) if (fp + tn) > 0 else 0.0

        lines = [
            "",
            "═" * 60,
            f"  HuitzilinReflex Week 3 — Detection Regression",
            f"  Split: {self._split}   Bag dir: {self._bag_dir}",
            "═" * 60,
            "",
            f"  {'ID':5s}  {'Label':8s}  {'Det':3s}  {'Pass':4s}  Note",
            "  " + "─" * 55,
        ]
        for r in results:
            lines.append(
                f"  {r['id']:5s}  {r['label']:8s}  "
                f"{'Y' if r['detected'] else 'N':3s}  "
                f"{'✓' if r['pass'] else '✗':4s}  "
                f"{r.get('note','')}"
            )
        passed = recall >= self._floor and not errors
        verdict = "PASS ✓"
        if errors:
            # name the actual errors. The old string
            # hardcoded "bag(s) missing" for all three error conditions, so
            # a sidecar with no time_to_closest_s produced a verdict that
            # misdirected the investigation to a bag that was never missing.
            detail = "; ".join(f"{r['id']}: {r.get('note','')}" for r in errors)
            verdict = (
                f"FAIL ✗ — {len(errors)} scenario(s) excluded from metrics "
                f"[{detail}]"
            )
        elif not passed:
            verdict = "FAIL ✗ — recall below floor"
        # always print the denominator. "Recall: 100.0%"
        # over a shrunken positive set, with no denominator, is copyable
        # straight out of the artifact as an overstatement.
        n_pos = len(positives)
        n_excluded_pos = sum(1 for r in errors if r["label"] == "positive")
        recall_line = (
            f"  Recall (TP rate): {recall*100:.1f}%  ({tp}/{n_pos} positives "
            f"scored"
            + (f", {n_excluded_pos} excluded" if n_excluded_pos else "")
            + f")  (floor: {self._floor*100:.0f}%)"
        )
        # attribution belongs in the artifact, not only in a
        # console log that nobody can re-read six months later.
        attributed = sum(1 for r in positives if r.get("attributable"))
        ctl_hits = [r["id"] for r in negatives if r.get("attributable")]
        # Only the speed_mps 0.0 negatives are
        # ball-free, and only they can falsify an attribution rule. Emitting
        # the two counts separately is what stops the next reader repeating
        # the error of calling all nine "ball-free" — N03/N04/T12/T13 have a
        # projectile in them, aimed where the detector must not see it.
        ball_free_negs = [r for r in negatives if r.get("ball_free")]
        with_ball_negs = [r for r in negatives if not r.get("ball_free")]
        bf_hits = [r["id"] for r in ball_free_negs if r.get("attributable")]
        wb_hits = [r["id"] for r in with_ball_negs if r.get("attributable")]
        lines += [
            "",
            "  ─" + "─" * 55,
            f"  TP={tp}  FN={fn}  TN={tn}  FP={fp}"
            + (f"  EXCLUDED={len(errors)}" if errors else ""),
            recall_line,
            f"  Precision:        {precision*100:.1f}%",
            f"  FP rate:          {fp_rate*100:.1f}%",
            f"  Attributable to a closing ball: {attributed}/{n_pos} positives",
            f"  Negative control, ALL negatives attributed to a ball: "
            f"{len(ctl_hits)}/{len(negatives)}"
            + (f"  {ctl_hits}" if ctl_hits else ""),
            f"    of which BALL-FREE (speed_mps 0.0 — the only true null): "
            f"{len(bf_hits)}/{len(ball_free_negs)}"
            + (f"  {bf_hits}" if bf_hits else ""),
            f"    of which BALL PRESENT but must-not-detect (behind/below): "
            f"{len(wb_hits)}/{len(with_ball_negs)}"
            + (f"  {wb_hits}" if wb_hits else ""),
            "",
            f"  {verdict}",
            "═" * 60,
            "",
        ]

        report = "\n".join(lines)
        # ARTIFACT FIRST, CONSOLE SECOND.
        # print() used to run HERE, ahead of the write and outside any
        # try. On a non-UTF-8 stdout (Windows cp1252, or LANG=C) the
        # report's box-drawing characters raise UnicodeEncodeError out of
        # _report(), so run() aborted and the artifact was never created
        # at all -- an exception escaping run() before the artifact is
        # written, and strictly worse than the zero-byte file this
        # ordering originally fixed. The write below is already
        # guarded and already pins its encoding, so ordering it first
        # costs nothing and cannot itself lose the artifact.
        try:
            self._out_file.parent.mkdir(parents=True, exist_ok=True)
            # encoding is pinned because the report ALWAYS
            # contains box-drawing characters and em-dashes, so write_text's
            # locale default silently truncates the artifact to zero bytes
            # under any non-UTF-8 locale (Windows cp1252, or a container
            # with LANG=C) — the file is opened and truncated before the
            # encode fails, and the only trace is a warn. Same class of
            # defect as the guard above: the artifact must survive.
            self._out_file.write_text(report, encoding="utf-8")
            self.get_logger().info(f"Report written → {self._out_file}")
        except Exception as e:
            self.get_logger().warn(f"Could not write report: {e}")

        # The console is the SECONDARY sink, so it may not veto the
        # verdict. The exit code IS the scoring result and
        # run_regression.sh/CI gate on it; letting an encoding failure
        # propagate turns a PASS into a non-zero exit plus a traceback
        # (main() catches it and returns 1) -- a manufactured FAIL on a
        # run that passed. Guarded, but NOT silenced: the report is
        # re-printed with the unmappable glyphs substituted (every number
        # and the verdict are ASCII; only the rules and ticks are lost)
        # and the substitution is logged. The exact text is on disk.
        try:
            print(report)
        except UnicodeEncodeError as e:
            enc = getattr(sys.stdout, "encoding", None) or "ascii"
            print(
                report.encode(enc, errors="replace").decode(
                    enc, errors="replace"
                )
            )
            self.get_logger().warn(
                f"console encoding {enc} cannot render the report; "
                f"printed it with substitutions ({e}). The artifact at "
                f"{self._out_file} is exact."
            )

        return 0 if passed else 1


# Entry point

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScorerNode()

    # Run scoring in a background thread so rclpy.spin can process callbacks
    exit_code = [0]

    def _run():
        # _done in a finally, not after the call. run() raising left the flag
        # unset, so the loop below spun on a dead thread forever with no
        # output and no traceback -- a hung regression rather than a failed
        # one, which under run_regression.sh looks like a slow scoring pass.
        try:
            exit_code[0] = node.run()
        except Exception:
            exit_code[0] = 1
            traceback.print_exc()
        finally:
            node._done = True

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()

    while rclpy.ok() and not getattr(node, "_done", False):
        rclpy.spin_once(node, timeout_sec=0.05)

    thread.join(timeout=5.0)
    node.destroy_node()
    # A SIGTERM during scoring already shut the context down; shutting down
    # twice raises RCLError, which would replace the scoring exit code with a
    # traceback and make a failed run indistinguishable from a crashed one.
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
