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
  3. Replay: ros2 bag play <bag> --clock --start-paused
     (we drive replay via subprocess, node reads published topics).
  4. Listen on /threat/centroid for the duration of the bag.
  5. Score:
     - Positive scenario: centroid published within detection_window_s → TP else FN
     - Negative scenario: centroid published at any point → FP else TN
  6. Aggregate TP/FP/TN/FN → recall, precision, FP rate.

All timing uses message stamps (use_sim_time), never wall-clock.

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

import os
import subprocess
import sys
import time
import threading
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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
#   time_to_closest_s: 1.5    # sim time from bag start to closest approach
#   detection_window_s: 3.0   # centroid must fire within this sim-time window
#

DETECTION_WINDOW_DEFAULT_S = 4.0  # sim seconds; override per scenario in sidecar


class ScorerNode(Node):
    """Listens on /threat/centroid and records detections during bag replay."""

    def __init__(self) -> None:
        super().__init__("score_bags")

        self.declare_parameter("bag_dir", "/data/huitzilin_bags")
        self.declare_parameter("scenario_matrix",
                               "config/scenario_matrix.yaml")
        self.declare_parameter("split", "test")   # train | test | all
        self.declare_parameter("recall_floor", 0.95)
        self.declare_parameter("output_file", "/tmp/week3_regression.txt")

        self._bag_dir   = Path(self.get_parameter("bag_dir").value)
        self._matrix_f  = Path(self.get_parameter("scenario_matrix").value)
        self._split     = self.get_parameter("split").value
        self._floor     = self.get_parameter("recall_floor").value
        self._out_file  = Path(self.get_parameter("output_file").value)

        # Track detections during current bag replay
        self._detection_times: list[float] = []
        self._listening = False

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

    def start_listening(self) -> None:
        self._detection_times = []
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

    def _score_one(self, scen: dict) -> dict:
        sid = scen["id"]
        label = scen["label"]

        # Find bag file
        bag_path = self._find_bag(sid)
        if bag_path is None:
            return {
                "id": sid, "label": label, "detected": False,
                "pass": False, "note": "BAG NOT FOUND",
            }

        # Load sidecar
        sidecar = self._load_sidecar(sid)
        window_s = sidecar.get("detection_window_s", DETECTION_WINDOW_DEFAULT_S)

        # Replay bag and collect detections
        self.start_listening()
        bag_duration = self._replay_bag(bag_path)
        detections = self.stop_listening()

        detected = len(detections) > 0

        if label == "positive":
            # TP: at least one centroid within detection_window_s of bag start
            bag_start = sidecar.get("bag_start_sim_t", None)
            if bag_start is not None and detections:
                in_window = any(t - bag_start <= window_s for t in detections)
            else:
                in_window = detected  # fallback: any detection counts
            passed = in_window
            note = f"window={window_s:.1f}s in_window={in_window}"
        else:  # negative
            # TN: no centroid at all
            passed = not detected
            note = f"fp_count={len(detections)}"

        return {
            "id": sid, "label": label,
            "detected": detected, "pass": passed,
            "detections": len(detections),
            "bag_duration_s": bag_duration,
            "note": note,
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

    def _replay_bag(self, bag_path: Path) -> float:
        """
        Replay a bag with ros2 bag play --clock and wait for it to finish.
        Returns approximate sim duration (seconds).  Non-blocking spin runs
        in background via thread.
        """
        cmd = [
            "ros2", "bag", "play",
            str(bag_path),
            "--clock",
            "--rate", "1.0",
        ]
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
        positives = [r for r in results if r["label"] == "positive"]
        negatives = [r for r in results if r["label"] == "negative"]

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
        lines += [
            "",
            "  ─" + "─" * 55,
            f"  TP={tp}  FN={fn}  TN={tn}  FP={fp}",
            f"  Recall (TP rate): {recall*100:.1f}%  "
            f"(floor: {self._floor*100:.0f}%)",
            f"  Precision:        {precision*100:.1f}%",
            f"  FP rate:          {fp_rate*100:.1f}%",
            "",
            f"  {'PASS ✓' if recall >= self._floor else 'FAIL ✗ — recall below floor'}",
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

        return 0 if recall >= self._floor else 1


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScorerNode()

    # Run scoring in a background thread so rclpy.spin can process callbacks
    exit_code = [0]

    def _run():
        exit_code[0] = node.run()
        # Signal done
        node._done = True

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()

    while rclpy.ok() and not getattr(node, "_done", False):
        rclpy.spin_once(node, timeout_sec=0.05)

    thread.join(timeout=5.0)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
