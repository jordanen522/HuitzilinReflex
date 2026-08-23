"""
truth_score_heldout.py — the ground-truth-attributed heldout scorer.

score_bags.py's `split=heldout` invocation uses
score_bags_logic.attribute_closing_ball -- a range-closure-RATE heuristic
built for bags with no ground truth at all. It answers a different question
("did something cross the window") than this scorer does ("were there >= K
detections matched to the BALL'S TRUE POSITION"), and on this bag library
every positive it scored came back `attributable=False` in the same run
(0/18 "attributable to a closing ball") -- a real gap between the two
questions, not a scoring bug in either.

truth_attribution.py (commit 8b27f75) implements the ground-truth matching
rule -- match_radius_m, interpolate_track, void_reason, score_scenario -- and
carries its own 297-line test suite. It was never wired to anything that
reads real bags; this file is that wiring. Nothing in truth_attribution.py's
matching rule, K, or window is touched here -- this module only gets
detections and ground truth INTO the shapes score_scenario already expects.

FRAME. detector_node.py publishes /threat/centroid in base_link (a MOVING
frame -- see its `_publish_centroid`, frame_id="base_link"), while
/gz/dynamic_poses is in the world frame (gz_pose_bridge's frame_id="world").
truth_attribution.score_scenario assumes detections and both truth tracks are
already in ONE common frame and does no conversion itself (its own docstring
says so, deliberately, to keep frame conversion in exactly one place per
CLAUDE.md). This module is that one place for THIS scorer: every detection is
rotated+translated into world frame using the drone's own nearest-in-time
pose (position AND orientation) from /gz/dynamic_poses, before
score_scenario ever sees it. Nearest-sample rather than interpolated
orientation: every heldout positive and every hover negative holds the
airframe stationary by construction (scenario_matrix.yaml's HOVER notes), so
orientation is not moving between samples; the two patrol negatives (HN01,
HN03) carry no ball track at all, so their frame accuracy cannot affect
"matched" -- interpolate_track(ball_track=[], t) is None for every t and
every detection in those two bags is unmatched regardless of transform error.

WINDOW. Every heldout positive/ball-bearing-negative is thrown with
compensate_gravity=True (a lofted arc, apex tens of metres up, still airborne
long after the scoring window -- see the yaml's own apex/max-elevation
notes). score_bags_logic.flat_throw_fall_time_s models a FLAT Week-3 throw
that hits the ground at sqrt(2h/g) =~ 0.639 s and returns that clamp for
every scenario with offset_vertical_m == 0 -- which describes NONE of these
scenarios' real physics (they all carry offset_vertical_m: 0.0 AND
compensate_gravity: true, a combination flat_throw_fall_time_s's guard
doesn't check for and was never written to see). Passing its clamp here
would clip the window's late edge at a ground-impact time that never
happens, for every single heldout scenario. This module always calls
strict_window_bounds(fall_time_s=None) for the heldout split instead --
exactly the escape hatch flat_throw_fall_time_s's own docstring names for "a
scenario whose geometry the flat-throw model does not describe" -- which
restores the nominal event (spawn + time_to_closest_s) as the window's
anchor, matching what time_to_closest_s actually measures for a lofted
throw. The window WIDTH (post_event_s, the floor, K) is untouched.

ENTITY NAMES. The drone is `drone_model` (default iris_ar0234, matching
dodge_battery.py's own parameter). The ball has no fixed name --
spawn_projectile.py names it f"projectile_{scenario_id}_{int(time.time())}",
a fresh unix-time suffix every throw -- so the ball track is every
/gz/dynamic_poses entry whose child_frame_id starts with
f"projectile_{scenario_id}_", scoped to THIS scenario's id so a stray
leftover projectile from an earlier scenario in the same bag cannot
contaminate it.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from huitzilin_perception.heldout_report import build_report
from huitzilin_perception.score_bags_logic import (
    POST_EVENT_TOLERANCE_S,
    REQUIRED_EXCLUDE_TOPICS,
    SPAWN_LEAD_S,
    build_bag_play_cmd,
    clock_msg_to_sim_t,
    compute_exclude_topics,
    strict_window_bounds,
)
from huitzilin_perception.truth_attribution import (
    DEFAULT_MIN_MATCHED,
    score_scenario,
    void_reason,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=50,
)

DETECTION_WINDOW_DEFAULT_S = 4.0
TOPIC_DISCOVERY_RETRIES = 10
TOPIC_DISCOVERY_RETRY_S = 0.5


def quat_rotate(q: tuple[float, float, float, float], v: np.ndarray) -> np.ndarray:
    """Rotate vector v (3,) by quaternion q=(x,y,z,w). Standard rotation matrix."""
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return R @ v


def nearest_pose(pose_track: list[tuple[float, tuple, tuple]], t: float):
    """(position, quaternion) of the nearest-in-time sample to t, or None."""
    if not pose_track:
        return None
    best = min(pose_track, key=lambda s: abs(s[0] - t))
    return best[1], best[2]


class TruthScorerNode(Node):
    def __init__(self) -> None:
        super().__init__("truth_score_heldout")

        self.declare_parameter("bag_dir", "/data/huitzilin_bags")
        self.declare_parameter("scenario_matrix", "config/scenario_matrix.yaml")
        self.declare_parameter("split", "heldout")
        self.declare_parameter("bag_prefix", "heldout_")
        self.declare_parameter("output_file", "/tmp/heldout_truth_scoring.txt")
        self.declare_parameter("detector_node_name", "detector")
        self.declare_parameter("drone_model", "iris_ar0234")
        self.declare_parameter("min_matched", DEFAULT_MIN_MATCHED)
        self.declare_parameter("debug", False)
        # Empty (default) = score every id in the split, in this one process,
        # as before. Set to a single id ("H12") to score only that scenario --
        # used by run_heldout_eval.sh to restart the detector process between
        # scenarios (see that script's header for why: the frozen detector's
        # persistent background map otherwise carries state across scenarios
        # that share corridor geometry, which is a harness isolation bug, not
        # a property of the frozen detector being evaluated).
        self.declare_parameter("scenario_id", "")
        # When set, append this scenario's raw result dict as one JSON line
        # instead of (single-scenario mode) or in addition to (whole-split
        # mode) writing the formatted report. finalize_heldout_report.py reads
        # this file back to produce the official artifact once every scenario
        # in the split has been scored, one detector process at a time.
        self.declare_parameter("results_jsonl", "")

        self._debug = bool(self.get_parameter("debug").value)
        self._bag_dir = Path(self.get_parameter("bag_dir").value)
        self._matrix_f = Path(self.get_parameter("scenario_matrix").value)
        self._split = self.get_parameter("split").value
        self._out_file = Path(self.get_parameter("output_file").value)
        self._detector_node_name = self.get_parameter("detector_node_name").value
        self._bag_prefix = self.get_parameter("bag_prefix").value
        self._drone_model = self.get_parameter("drone_model").value
        self._min_matched = int(self.get_parameter("min_matched").value)
        self._scenario_id = self.get_parameter("scenario_id").value or ""
        self._results_jsonl = self.get_parameter("results_jsonl").value or ""

        self._detections: list[tuple[float, tuple]] = []   # (t, (x,y,z)) base_link
        self._pose_tracks: dict[str, list[tuple]] = {}      # name -> [(t,(x,y,z),(qx,qy,qz,qw))]
        self._listening = False
        self._exclude_topics: list[str] = []

        self._sub_centroid = self.create_subscription(
            PointStamped, "/threat/centroid", self._centroid_cb, RELIABLE_QOS
        )
        self._sub_poses = self.create_subscription(
            TFMessage, "/gz/dynamic_poses", self._poses_cb, POSE_QOS
        )
        self.get_logger().info(
            f"truth_score_heldout ready | bag_dir={self._bag_dir} "
            f"split={self._split} min_matched={self._min_matched}"
        )

    def _centroid_cb(self, msg: PointStamped) -> None:
        if not self._listening:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._detections.append((t, (msg.point.x, msg.point.y, msg.point.z)))

    def _poses_cb(self, msg: TFMessage) -> None:
        if not self._listening:
            return
        for tr in msg.transforms:
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            pos = (tr.transform.translation.x, tr.transform.translation.y,
                   tr.transform.translation.z)
            quat = (tr.transform.rotation.x, tr.transform.rotation.y,
                    tr.transform.rotation.z, tr.transform.rotation.w)
            self._pose_tracks.setdefault(tr.child_frame_id, []).append(
                (t, pos, quat)
            )

    def start_listening(self) -> None:
        self._detections = []
        self._pose_tracks = {}
        self._listening = True

    def stop_listening(self) -> None:
        self._listening = False

    def run(self) -> int:
        if not self._matrix_f.exists():
            self.get_logger().error(f"Scenario matrix not found: {self._matrix_f}")
            return 1
        try:
            self._exclude_topics = self._discover_exclude_topics()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return 1
        self.get_logger().info(f"excluding from replay: {self._exclude_topics}")

        with open(self._matrix_f) as f:
            matrix = yaml.safe_load(f)
        splits = matrix.get("split", {})
        if self._split not in splits:
            raise ValueError(f"split {self._split!r} not in {self._matrix_f}")
        split_ids = splits[self._split]
        scenarios = {s["id"]: s for s in matrix["scenarios"]}

        if self._scenario_id:
            if self._scenario_id not in split_ids:
                self.get_logger().error(
                    f"scenario_id {self._scenario_id!r} is not in split "
                    f"{self._split!r} ({split_ids})"
                )
                return 1
            ids_to_score = [self._scenario_id]
        else:
            ids_to_score = list(split_ids)

        results: list[dict] = []
        for sid in ids_to_score:
            scen = scenarios[sid]
            result = self._score_one(scen)
            results.append(result)
            self.get_logger().info(
                f"  [{sid}] label={scen['label']} void={result.get('void')} "
                f"matched={result.get('matched')} unmatched={result.get('unmatched')} "
                f"recalled={result.get('recalled')} fired={result.get('fired')}"
            )
            if self._results_jsonl:
                self._append_jsonl(result)

        # Single-scenario mode is driven by run_heldout_eval.sh, which
        # restarts the detector between calls and produces the combined
        # report itself via finalize_heldout_report.py once every id in the
        # split has a line in results_jsonl -- writing a partial report here
        # (n=1 out of the split's real denominator) would be misleading.
        if self._scenario_id:
            return 0
        return self._report(results)

    def _append_jsonl(self, result: dict) -> None:
        path = Path(self._results_jsonl)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(result) + "\n")

    def _discover_exclude_topics(self) -> list[str]:
        last_seen: list[str] = []
        last_exc = None
        for attempt in range(TOPIC_DISCOVERY_RETRIES):
            try:
                pubs = self.get_publisher_names_and_types_by_node(
                    self._detector_node_name, "/"
                )
                last_seen = [name for name, _t in pubs]
                last_exc = None
                if REQUIRED_EXCLUDE_TOPICS.issubset(set(last_seen)):
                    break
            except Exception as e:
                last_exc = e
                last_seen = []
            time.sleep(TOPIC_DISCOVERY_RETRY_S)
        try:
            return compute_exclude_topics(last_seen)
        except ValueError as e:
            detail = f"; last discovery error: {last_exc!r}" if last_exc else ""
            raise RuntimeError(
                f"could not establish exclude-topics for "
                f"'{self._detector_node_name}'{detail}: {e}"
            ) from e

    def _find_bag(self, sid: str) -> Optional[Path]:
        prefix = f"{self._bag_prefix}{sid}"
        for f in sorted(self._bag_dir.glob(f"{prefix}*.mcap")):
            return f
        for d in sorted(self._bag_dir.glob(f"{prefix}*/")):
            return d
        return None

    def _load_sidecar(self, sid: str) -> dict:
        p = self._bag_dir / f"{self._bag_prefix}{sid}.label.yaml"
        if p.exists():
            with open(p) as f:
                return yaml.safe_load(f) or {}
        return {}

    def _read_bag_start_sim_t(self, bag_path: Path) -> Optional[float]:
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path))
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            self.get_logger().error(f"could not open {bag_path}: {e}")
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

    def _replay_bag(self, bag_path: Path) -> None:
        cmd = build_bag_play_cmd(str(bag_path), self._exclude_topics, rate=1.0)
        try:
            proc = subprocess.run(cmd, timeout=120, capture_output=True)
            if proc.returncode not in (0, -2):
                self.get_logger().warn(
                    f"bag play returned {proc.returncode}: "
                    f"{proc.stderr.decode()[:200]}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"bag replay timed out for {bag_path.name}")

    def _score_one(self, scen: dict) -> dict:
        sid = scen["id"]
        label = scen["label"]

        bag_path = self._find_bag(sid)
        if bag_path is None:
            return {"id": sid, "label": label, "error": "BAG MISSING"}

        sidecar = self._load_sidecar(sid)
        window_s = sidecar.get("detection_window_s", DETECTION_WINDOW_DEFAULT_S)
        ttc = sidecar.get("time_to_closest_s")

        bag_start = self._read_bag_start_sim_t(bag_path)
        if bag_start is None:
            return {"id": sid, "label": label, "error": "NO /clock IN BAG"}

        self.start_listening()
        self._replay_bag(bag_path)
        self.stop_listening()

        raw_detections = list(self._detections)
        raw_pose_tracks = dict(self._pose_tracks)

        drone_track_raw = raw_pose_tracks.get(self._drone_model, [])
        ball_prefix = f"projectile_{sid}_"
        ball_track_raw = [
            entry for name, track in raw_pose_tracks.items()
            if name.startswith(ball_prefix)
            for entry in track
        ]

        # void_reason wants [(t,(x,y,z)), ...] position-only tracks.
        drone_pos_track = [(t, pos) for (t, pos, _q) in drone_track_raw]
        ball_pos_track = [(t, pos) for (t, pos, _q) in ball_track_raw]

        reason = void_reason(label, ball_pos_track, drone_pos_track)
        if reason is not None:
            return {
                "id": sid, "label": label, "void": True,
                "void_reason": reason,
                "n_detections_raw": len(raw_detections),
                "n_drone_pose": len(drone_pos_track),
                "n_ball_pose": len(ball_pos_track),
            }

        # Transform every base_link detection into world frame using the
        # drone's nearest-in-time pose (position + orientation). See module
        # docstring FRAME section.
        detections_world: list[tuple[float, tuple]] = []
        for t, p_bl in raw_detections:
            np_result = nearest_pose(drone_track_raw, t)
            if np_result is None:
                continue
            (dpos, dquat) = np_result
            world_vec = np.array(dpos) + quat_rotate(dquat, np.array(p_bl))
            detections_world.append((t, tuple(float(c) for c in world_vec)))

        speed_mps = float(scen.get("speed_mps") or 0.0)

        # ALWAYS fall_time_s=None for this split -- see module docstring
        # WINDOW section: every heldout scenario is a lofted
        # compensate_gravity=True throw, never a flat Week-3 throw, so the
        # flat-throw ground-impact clamp does not describe its physics.
        fall_time_s = None
        window_ttc = ttc if ttc is not None else 0.0

        # SPAWN ANCHOR: use the ball's own first truth-track timestamp when
        # one exists, not the fixed SPAWN_LEAD_S=3.0 constant.
        #
        # SPAWN_LEAD_S is `sleep "$SPAWN_LEAD"` in capture_scenario.sh -- a
        # WALL-CLOCK sleep, not a sim-time delay -- baked in from the
        # original Week 3 capture pipeline (no depth rendering, RTF ~1).
        # This rendered/depth lane runs at a different, non-1:1 sim:wall
        # rate (CLAUDE.md), so 3 wall-seconds of sleep does not land the
        # spawn command at bag_start + 3.0 SIM seconds. Measured directly on
        # H03: the ball's first /gz/dynamic_poses sample is at bag_start +
        # 2.351 s, not +3.000 s, and its z-trajectory (loft to apex, fall
        # back through hover altitude) matches the labelled ballistics
        # almost exactly if spawn is taken at ~2.35 s -- confirming the
        # constant, not the data, was wrong. Every positive here HAS a real
        # ball track (that is what a positive scenario means), so we have
        # the actual answer and no longer need to guess it: spawn_t is the
        # ball's own first appearance in the truth stream (gz's
        # dynamic_pose/info carries only MOVING entities, and the
        # projectile's link ships gravity-off/motionless until the exact
        # instant of the throw impulse, so first-appearance is a tight proxy
        # for the real spawn instant). This is derived ENTIRELY from truth
        # timing, never from detection positions or counts, so it cannot be
        # "tuned toward a better match" -- it is computed identically
        # whether or not any detection ever matches.
        #
        # Ball-free negatives (HN01-03) have no ball track by definition and
        # keep the SPAWN_LEAD_S proxy -- their window only matters for
        # excluding the cold-start FP burst, not for anchoring a real event.
        if ball_pos_track:
            spawn_t = min(t for t, _p in ball_pos_track)
            spawn_lead_s = spawn_t - bag_start
        else:
            spawn_lead_s = SPAWN_LEAD_S

        try:
            lower, upper = strict_window_bounds(
                bag_start, window_ttc, window_s,
                spawn_lead_s=spawn_lead_s,
                post_event_s=POST_EVENT_TOLERANCE_S,
                fall_time_s=fall_time_s,
            )
        except ValueError as e:
            return {"id": sid, "label": label, "error": f"WINDOW REFUSED: {e}"}

        if self._debug:
            from huitzilin_perception.truth_attribution import (
                interpolate_track, match_radius_m,
            )
            self.get_logger().info(
                f"    {sid} DEBUG bag_start={bag_start:.3f} window="
                f"[{lower-bag_start:.3f},{upper-bag_start:.3f}] "
                f"n_raw_det={len(raw_detections)} n_world_det={len(detections_world)} "
                f"n_ball_pose={len(ball_pos_track)} n_drone_pose={len(drone_pos_track)}"
            )
            for t, wp in detections_world:
                in_win = lower <= t <= upper
                truth = interpolate_track(ball_pos_track, t)
                drone = interpolate_track(drone_pos_track, t)
                if truth is not None and drone is not None:
                    err = math.dist(wp, truth)
                    radius = match_radius_m(math.dist(wp, drone), speed_mps)
                    self.get_logger().info(
                        f"      t_rel={t-bag_start:.3f} in_win={in_win} "
                        f"world_det=({wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f}) "
                        f"truth=({truth[0]:.2f},{truth[1]:.2f},{truth[2]:.2f}) "
                        f"err={err:.2f} radius={radius:.2f} "
                        f"MATCH={'Y' if err<=radius else 'N'}"
                    )
                else:
                    self.get_logger().info(
                        f"      t_rel={t-bag_start:.3f} in_win={in_win} "
                        f"world_det=({wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f}) "
                        f"NO TRUTH AT THIS t (outside ball/drone track span)"
                    )

        result = score_scenario(
            detections_world, ball_pos_track, drone_pos_track,
            speed_mps, (lower, upper), min_matched=self._min_matched,
        )
        result.update({
            "id": sid, "label": label, "void": False,
            "window_rel": (lower - bag_start, upper - bag_start),
            "n_detections_raw": len(raw_detections),
            "n_drone_pose": len(drone_pos_track),
            "n_ball_pose": len(ball_pos_track),
        })
        return result

    def _report(self, results: list[dict]) -> int:
        report = build_report(results, self._split, str(self._bag_dir),
                               self._min_matched)
        try:
            self._out_file.parent.mkdir(parents=True, exist_ok=True)
            self._out_file.write_text(report, encoding="utf-8")
        except Exception as e:
            self.get_logger().warn(f"Could not write report: {e}")
        print(report)
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TruthScorerNode()
    exit_code = [0]

    def _run():
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
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
