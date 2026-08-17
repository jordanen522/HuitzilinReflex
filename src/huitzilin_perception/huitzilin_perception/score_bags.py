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
     TEST — Task 5b Defect 1. Exclusion topics are discovered from the
     LIVE detector node's own publisher list (see _discover_exclude_topics),
     not hardcoded here, so a future debug publisher cannot reopen this hole
     silently.
  4. Listen on /threat/centroid for the duration of the bag.
  5. Score:
     - Positive scenario: centroid published within detection_window_s of
       the ball's own closest-approach time (bag_start + SPAWN_LEAD_S +
       time_to_closest_s — all three required — see Defect 2 below) → TP
       else FN
     - Negative scenario: centroid published at any point → FP else TN
  6. Aggregate TP/FP/TN/FN → recall, precision, FP rate.

All timing uses message stamps (use_sim_time), never wall-clock.

Defect 2 (Task 5b) — the window was never actually enforced: no sidecar ever
carried `bag_start_sim_t`, so every run silently fell back to "detected
somewhere in the whole replay", which the FP background alone can satisfy.
bag_start is now measured directly — but NOT by subscribing to the live
/clock topic during replay, despite that being the obvious reading of "the
first /clock stamp is the honest bag-start in sim time." Verified on the
Dell 2026-08-17: `ros2 bag play --clock` does not republish a bag's own
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
    SPAWN_LEAD_S,
    build_bag_play_cmd,
    compute_exclude_topics,
    is_attributable_to_ball,
    is_in_window_loose,
    is_in_window_strict,
    longest_closing_run,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Label sidecar format ──────────────────────────────────────────────────────
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
# sidecar has ever actually carried (Task 5b Defect 2). It is read from the
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
        self.declare_parameter("split", "test")   # train | test | tune | all
        self.declare_parameter("recall_floor", 0.95)
        self.declare_parameter("output_file", "/tmp/week3_regression.txt")
        # The node under test. Its own publisher list — discovered live from
        # the ROS graph, not hardcoded — is what gets excluded from bag
        # replay (Task 5b Defect 1).
        self.declare_parameter("detector_node_name", "detector")

        self._bag_dir   = Path(self.get_parameter("bag_dir").value)
        self._matrix_f  = Path(self.get_parameter("scenario_matrix").value)
        self._split     = self.get_parameter("split").value
        self._floor     = self.get_parameter("recall_floor").value
        self._out_file  = Path(self.get_parameter("output_file").value)
        self._detector_node_name = self.get_parameter("detector_node_name").value

        # Track detections during current bag replay. bag_start is no longer
        # tracked as live subscriber state — see _read_bag_start_sim_t.
        self._detection_times: list[float] = []
        # Full (t, x, y, z) alongside the timestamps above — kept separate so
        # is_in_window_strict/loose's pure-function signature (list[float])
        # never has to change. Only used for attribution (Task 5b): a real
        # ball produces a closing sequence of centroids with DECREASING
        # range across frames, first-sight terrain/patrol-motion noise does
        # not — see _attributable_to_ball and the task-5b-report.md.
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

    # ── Main scoring logic ────────────────────────────────────────────────────

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
            # Loud and fatal on purpose (Task 5b Defect 1): a run that cannot
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
            split_ids = matrix.get("split", {}).get(self._split, [])

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
        scoring with no exclusion at all.
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
                if last_seen:
                    break
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
            # Task 5b Defect 2: "no window could be established" must fail
            # loudly, never quietly pass as "detected somewhere". A bag with
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
            # spawn_lead_s: the projectile does not exist until SPAWN_LEAD_S
            # seconds into the recording (capture_scenario.sh), and
            # time_to_closest_s is defined relative to SPAWN, not bag start
            # (scenario_matrix.yaml:18) — see SPAWN_LEAD_S's docstring in
            # score_bags_logic.py for the empirical validation.
            in_window_strict = is_in_window_strict(
                detections, bag_start, time_to_closest_s, window_s,
                spawn_lead_s=SPAWN_LEAD_S,
            )
            in_window_loose = is_in_window_loose(detections, bag_start, window_s)
            # Strict (anchored to the ball's own closest-approach time) is
            # the authoritative gate — "within window_s of bag start" is weak
            # on a ~20 s bag (task-5b-brief). Loose is reported alongside it
            # for comparison only; it is never what decides pass/fail.
            passed = in_window_strict
            note = (
                f"window={window_s:.1f}s lead={SPAWN_LEAD_S:.1f}s "
                f"strict={in_window_strict} loose={in_window_loose}"
            )
            # Detection audit trail: every FN root-cause investigation this
            # task has done needed raw detection times relative to bag_start
            # (see task-5b-report.md) — logging them unconditionally here
            # means a future FN never needs a special diagnostic run to see
            # WHERE the detector actually fired vs. the strict window.
            event_rel = SPAWN_LEAD_S + time_to_closest_s
            det_rel = sorted(round(t - bag_start, 3) for t in detections)
            self.get_logger().info(
                f"    {sid} detections_rel_s={det_rel} event_rel_s={event_rel:.3f} "
                f"window=[{event_rel - window_s:.3f}, {event_rel + window_s:.3f}]"
            )

            # Attribution (Task 5b): is this TP a real ball, or a background
            # detection that merely landed inside the (generous) strict
            # window? The bags carry no ground-truth ball pose, so the only
            # available discriminator is a CLOSING RUN — consecutive
            # in-window detections whose range from the sensor decreases —
            # vs. sporadic non-closing ones. Restricted to detections
            # actually inside the strict window: a closing run elsewhere in
            # a 20-45 s bag says nothing about THIS TP.
            attributable = None
            closing_run = 0
            if passed:
                event_t = bag_start + event_rel
                in_window_ranges = [
                    (t, math.sqrt(x * x + y * y + z * z))
                    for (t, x, y, z) in self._detection_positions
                    if abs(t - event_t) <= window_s
                ]
                closing_run = longest_closing_run(in_window_ranges)
                attributable = is_attributable_to_ball(in_window_ranges)
                self.get_logger().info(
                    f"    {sid} attributable_to_ball={attributable} "
                    f"(longest_closing_run={closing_run}, "
                    f"n_in_window={len(in_window_ranges)})"
                )
        else:  # negative
            # TN: no centroid at all, anywhere in the replay. Unaffected by
            # the window logic above — a negative has no closest-approach
            # time to anchor to. Attribution does not apply to negatives.
            passed = not detected
            note = f"fp_count={len(detections)}"
            attributable = None
            closing_run = 0

        return {
            "id": sid, "label": label,
            "detected": detected, "pass": passed,
            "detections": len(detections),
            "bag_duration_s": bag_duration,
            "note": note,
            "attributable": attributable,
            "closing_run": closing_run,
        }

    def _find_bag(self, sid: str) -> Optional[Path]:
        """Find first .mcap file in bag_dir whose name starts with week3_<sid>."""
        prefix = f"week3_{sid}"
        for f in sorted(self._bag_dir.glob(f"{prefix}*.mcap")):
            return f
        # Also check subdirectory bags (ros2 bag play uses a directory)
        for d in sorted(self._bag_dir.glob(f"{prefix}*/")):
            return d
        return None

    def _load_sidecar(self, sid: str) -> dict:
        sidecar_path = self._bag_dir / f"week3_{sid}.label.yaml"
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
        in sim time")? Verified on the Dell 2026-08-17: `ros2 bag play
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
        sim-time payload — only bag_start needed this fix.
        """
        storage_options = rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id="mcap"
        )
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
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/clock"]))
        if not reader.has_next():
            return None
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, Clock)
        return msg.clock.sec + msg.clock.nanosec * 1e-9

    def _replay_bag(self, bag_path: Path) -> float:
        """
        Replay a bag with ros2 bag play --clock, EXCLUDING every topic the
        detector under test publishes (self._exclude_topics, discovered once
        in run() — see _discover_exclude_topics), and wait for it to finish.
        Returns approximate sim duration (seconds). Non-blocking spin runs
        in background via thread.

        The exclusion is Task 5b Defect 1's fix: T-bags were captured with a
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

    # ── Reporting ──────────────────────────────────────────────────────────────

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
            verdict = f"FAIL ✗ — {len(errors)} bag(s) missing (excluded from metrics)"
        elif not passed:
            verdict = "FAIL ✗ — recall below floor"
        lines += [
            "",
            "  ─" + "─" * 55,
            f"  TP={tp}  FN={fn}  TN={tn}  FP={fp}"
            + (f"  MISSING={len(errors)}" if errors else ""),
            f"  Recall (TP rate): {recall*100:.1f}%  "
            f"(floor: {self._floor*100:.0f}%)",
            f"  Precision:        {precision*100:.1f}%",
            f"  FP rate:          {fp_rate*100:.1f}%",
            "",
            f"  {verdict}",
            "═" * 60,
            "",
        ]

        report = "\n".join(lines)
        print(report)

        try:
            self._out_file.parent.mkdir(parents=True, exist_ok=True)
            self._out_file.write_text(report)
            self.get_logger().info(f"Report written → {self._out_file}")
        except Exception as e:
            self.get_logger().warn(f"Could not write report: {e}")

        return 0 if passed else 1


# ── Entry point ───────────────────────────────────────────────────────────────

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
