"""
dodge_battery.py — HuitzilinReflex Week 4: closed-loop dodge battery + sweep.

Spawns gravity-compensated projectiles at the patrolling drone and scores
each run against simulator ground truth:

  dodged      : did /threat/evade_event fire inside the run window?
  latency_s   : centroid-stamp -> dodge-command latency (from the event JSON)
  min_dist_m  : true minimum drone<->ball range, from the bridged Gazebo
                dynamic-pose stream (/gz/dynamic_poses, sim-time stamped)

Success per run:
  expect_dodge true  -> dodged AND min_dist_m > hit_radius_m
  expect_dodge false -> did NOT dodge (false-dodge check)

No hard success-rate gate (2026-07-16 decision): measured rates are
reported; the exit code is non-zero only for harness errors (no odom, no
ground-truth stream, spawn failure, missing config).

Sweep mode (-p sweep_config:=<yaml>) grids evasion-node parameters through
the /evasion set_parameters service between battery passes.

PREREQS (Dell only — live Gazebo depth): the full Week 4 stack is up and
the drone is patrolling. See docs/week4_dodge_runbook.md.

All run windows are timed in SIM time (use_sim_time:=true required).
"""

from __future__ import annotations

import csv
import itertools
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from huitzilin_perception.ballistics import compute_spawn
from huitzilin_perception.spawn_projectile import MIN_SPAWN_Z, gz_remove, gz_spawn

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Best-effort matches a publisher of EITHER reliability; a RELIABLE
# subscriber would silently never match a best-effort bridge publisher
# and every run would die on the pose-stream pre-flight timeout.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

ODOM_WAIT_TIMEOUT_WALL_S = 60.0   # bring-up bound (wall clock; generous, exits early)
POSE_STREAM_TIMEOUT_WALL_S = 30.0


class DodgeBatteryNode(Node):
    """Battery orchestrator. run() executes in a worker thread while the
    main thread spins the executor (same pattern as score_bags.py)."""

    def __init__(self) -> None:
        super().__init__("dodge_battery")

        share = get_package_share_directory("huitzilin_perception")
        self.declare_parameter(
            "battery_config", str(Path(share) / "config" / "week4_battery.yaml"))
        self.declare_parameter("sweep_config", "")
        self.declare_parameter("output_file", "/tmp/week4_battery.txt")
        self.declare_parameter("csv_file", "/tmp/week4_battery.csv")
        self.declare_parameter("world_name", "huitzilin_runway")
        self.declare_parameter("drone_model", "iris_depth")
        self.declare_parameter("model_uri", "model://projectile")
        self.declare_parameter("hit_radius_m", 0.30)   # drone body + ball radius
        self.declare_parameter("run_window_s", 5.0)    # sim s to watch each throw
        self.declare_parameter("settle_s", 6.0)        # sim s between runs
        self.declare_parameter("latency_budget_s", 0.15)
        self.declare_parameter("evasion_node_name", "/evasion")

        self._battery_f = Path(self.get_parameter("battery_config").value)
        self._sweep_f = self.get_parameter("sweep_config").value
        self._out_f = Path(self.get_parameter("output_file").value)
        self._csv_f = Path(self.get_parameter("csv_file").value)
        self._world = self.get_parameter("world_name").value
        self._drone_model = self.get_parameter("drone_model").value
        self._model_uri = self.get_parameter("model_uri").value
        self._hit_radius = float(self.get_parameter("hit_radius_m").value)
        self._window_s = float(self.get_parameter("run_window_s").value)
        self._settle_s = float(self.get_parameter("settle_s").value)
        self._budget_s = float(self.get_parameter("latency_budget_s").value)
        self._evasion_name = self.get_parameter("evasion_node_name").value

        self._lock = threading.Lock()
        self._latest_odom = None
        self._pose_stream_seen = False
        self._active_ball = None
        self._min_dist = float("inf")
        self._events = []
        self._listening = False

        self.create_subscription(Odometry, "/huitzilin/odom",
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, SENSOR_QOS)
        self.create_subscription(String, "/threat/evade_event",
                                 self._event_cb, RELIABLE_QOS)

        # Created once and reused for every sweep combo + the baseline
        # snapshot/restore — avoids leaking a service client per combo.
        self._set_params_cli = self.create_client(
            SetParameters, f"{self._evasion_name}/set_parameters")
        self._get_params_cli = self.create_client(
            GetParameters, f"{self._evasion_name}/get_parameters")

        self.get_logger().info(
            f"dodge_battery ready | config={self._battery_f} "
            f"sweep={self._sweep_f or '-'} "
            f"world={self._world} drone_model={self._drone_model}"
        )

    # ── Subscriptions ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _pose_cb(self, msg: TFMessage) -> None:
        self._pose_stream_seen = True
        with self._lock:
            ball_name = self._active_ball
            if ball_name is None:
                return
            drone = ball = None
            for tr in msg.transforms:
                if tr.child_frame_id == self._drone_model:
                    t = tr.transform.translation
                    drone = np.array([t.x, t.y, t.z])
                elif tr.child_frame_id == ball_name:
                    t = tr.transform.translation
                    ball = np.array([t.x, t.y, t.z])
            if drone is not None and ball is not None:
                d = float(np.linalg.norm(ball - drone))
                if d < self._min_dist:
                    self._min_dist = d

    def _event_cb(self, msg: String) -> None:
        with self._lock:
            if not self._listening:
                return
            try:
                self._events.append(json.loads(msg.data))
            except json.JSONDecodeError:
                self.get_logger().warn("unparseable /threat/evade_event payload")

    # ── Time helpers (SIM time via node clock; use_sim_time:=true) ───────

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait_sim(self, duration_s: float) -> bool:
        """Wait for duration_s of SIM time. Returns True if it completed
        normally, False if the wall-clock bail-out fired (stalled /clock)."""
        max_wall_s = duration_s * 10.0 + 30.0
        t0 = self._sim_now()
        t0_wall = time.monotonic()
        while rclpy.ok() and self._sim_now() - t0 < duration_s:
            if time.monotonic() - t0_wall > max_wall_s:
                self.get_logger().warn(
                    f"_wait_sim bailed after {max_wall_s:.0f} s wall time "
                    f"waiting for {duration_s} s sim time — /clock stalled?")
                return False
            time.sleep(0.02)  # wall-sleep; the WINDOW is judged in sim time
        return True

    def _wait_wall_for(self, predicate, timeout_s: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    # ── Main flow ────────────────────────────────────────────────────────

    def run(self) -> int:
        if not self._battery_f.exists():
            self.get_logger().error(f"battery config not found: {self._battery_f}")
            return 1
        with open(self._battery_f) as f:
            cfg = yaml.safe_load(f)
        defaults = cfg.get("defaults", {})
        runs = {r["id"]: {**defaults, **r} for r in cfg["runs"]}

        combos = [{}]
        selected = list(runs)
        repeats_override = None
        sweep_keys = None
        if self._sweep_f:
            sweep_path = Path(self._sweep_f)
            if not sweep_path.exists():
                self.get_logger().error(f"sweep config not found: {sweep_path}")
                return 1
            with open(sweep_path) as f:
                sweep = yaml.safe_load(f)
            sweep_keys = sorted(sweep["parameters"])
            combos = [dict(zip(sweep_keys, vals)) for vals in
                      itertools.product(*(sweep["parameters"][k] for k in sweep_keys))]
            selected = sweep.get("runs", selected)
            repeats_override = sweep.get("repeats")

        # Pre-flight: odom + ground-truth stream must be alive.
        if not self._wait_wall_for(lambda: self._latest_odom is not None,
                                   ODOM_WAIT_TIMEOUT_WALL_S):
            self.get_logger().error("no /huitzilin/odom — is the flight stack up?")
            return 1
        if not self._wait_wall_for(lambda: self._pose_stream_seen,
                                   POSE_STREAM_TIMEOUT_WALL_S):
            self.get_logger().error(
                "no /gz/dynamic_poses — launch week4_evasion.launch.py "
                "(it bridges /world/<world>/dynamic_pose/info)")
            return 1

        rows = []
        baseline_snapshot = None
        if sweep_keys is not None:
            baseline_snapshot = self._snapshot_params(sweep_keys)
            if baseline_snapshot is None:
                self.get_logger().error(
                    "could not snapshot baseline params — sweep aborted")
                rows.append({"combo": "baseline", "id": "-", "rep": 0,
                             "error": True, "success": False,
                             "expect_dodge": False,
                             "note": "could not snapshot baseline params — "
                                     "sweep aborted"})
                return self._report(rows)

        try:
            for combo in combos:
                if combo and not self._apply_params(combo):
                    rows.append({"combo": self._combo_str(combo), "id": "-",
                                 "rep": 0, "error": True, "success": False,
                                 "expect_dodge": False,
                                 "note": "set_parameters failed"})
                    continue
                for rid in selected:
                    if rid not in runs:
                        self.get_logger().warn(f"unknown run id {rid}; skipping")
                        continue
                    scen = runs[rid]
                    n_rep = int(repeats_override if repeats_override is not None
                                else scen.get("repeats", 1))
                    for rep in range(n_rep):
                        try:
                            row = self._one_run(scen, rep, combo)
                        except Exception as e:  # noqa: BLE001 — a crashed run must
                            # still produce a scored report, not kill the battery
                            with self._lock:
                                self._listening = False
                                self._active_ball = None
                            self.get_logger().error(f"run {rid} r{rep} crashed: {e}")
                            row = {"combo": self._combo_str(combo), "id": rid,
                                   "rep": rep,
                                   "expect_dodge": bool(scen.get("expect_dodge", False)),
                                   "error": True, "success": False,
                                   "note": f"harness exception: {e}"}
                        rows.append(row)
                        mark = "✓" if row.get("success") else "✗"
                        self.get_logger().info(
                            f"  [{mark}] {row['combo']:<24} {row['id']:>4} r{rep} "
                            f"dodged={row.get('dodged')} "
                            f"min={row.get('min_dist_m', float('nan'))} m "
                            f"lat={row.get('latency_ms', '-')} ms | {row['note']}")
        finally:
            if baseline_snapshot is not None:
                if not self._apply_params(baseline_snapshot):
                    self.get_logger().error(
                        "could not restore baseline evasion params after sweep "
                        f"— manually restore: {baseline_snapshot}")

        return self._report(rows)

    @staticmethod
    def _combo_str(combo: dict) -> str:
        return ",".join(f"{k}={v}" for k, v in sorted(combo.items())) or "baseline"

    def _one_run(self, scen: dict, rep: int, combo: dict) -> dict:
        rid = scen["id"]
        base = {"combo": self._combo_str(combo), "id": rid, "rep": rep,
                "expect_dodge": bool(scen["expect_dodge"])}

        odom = self._latest_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        plan = compute_spawn(
            (p.x, p.y, p.z), yaw,
            speed_mps=float(scen["speed_mps"]),
            approach_angle_deg=float(scen.get("approach_angle_deg", 0.0)),
            miss_distance_m=float(scen.get("miss_distance_m", 0.0)),
            offset_forward_m=float(scen.get("offset_forward_m", 6.0)),
            offset_vertical_m=float(scen.get("offset_vertical_m", 0.0)),
            compensate_gravity=bool(scen.get("compensate_gravity", True)),
            aim_at_drone=bool(scen.get("aim_at_drone", False)),
        )
        if plan.position[2] < MIN_SPAWN_Z:
            return {**base, "error": True, "success": False,
                    "note": f"spawn z={plan.position[2]:.2f} < {MIN_SPAWN_Z}"}

        name = f"ball_{rid}_r{rep}_{int(time.time())}"
        with self._lock:
            self._active_ball = name
            self._min_dist = float("inf")
            self._events = []
            self._listening = True

        ok, msg = gz_spawn(self._world, self._model_uri, name,
                           plan.position, plan.velocity)
        if not ok:
            with self._lock:
                self._listening = False
                self._active_ball = None
            return {**base, "error": True, "success": False, "note": msg}

        window_ok = self._wait_sim(self._window_s)
        if not window_ok:
            with self._lock:
                self._listening = False
                self._active_ball = None
            gz_remove(self._world, name)
            return {**base, "error": True, "success": False,
                    "note": "sim clock stalled mid-window"}

        with self._lock:
            self._listening = False
            self._active_ball = None
            min_dist = self._min_dist
            events = list(self._events)

        gz_remove(self._world, name)
        self._wait_sim(self._settle_s)   # let patrol resume + bg model settle
        # (settle-wait result ignored — a stall here doesn't invalidate the
        # run that already completed above)

        dodged = len(events) > 0
        latency_ms = round(events[0]["latency_s"] * 1000.0, 1) if dodged else None
        if min_dist == float("inf"):
            return {**base, "error": True, "success": False, "dodged": dodged,
                    "note": f"ball '{name}' never seen on /gz/dynamic_poses "
                            f"(check drone_model:={self._drone_model})"}

        if base["expect_dodge"]:
            success = dodged and min_dist > self._hit_radius
            note = ("clean dodge" if success else
                    "no dodge fired" if not dodged else
                    f"dodged but min_dist {min_dist:.2f} <= hit_radius")
        else:
            success = not dodged
            note = "correctly ignored" if success else "FALSE DODGE"

        return {**base, "error": False, "dodged": dodged,
                "latency_ms": latency_ms, "min_dist_m": round(min_dist, 3),
                "success": success, "note": note}

    def _snapshot_params(self, names: list) -> dict | None:
        """Read current values of `names` from /evasion via get_parameters,
        for restoring the node's baseline config after a sweep."""
        if not self._get_params_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"{self._evasion_name}/get_parameters unavailable")
            return None
        req = GetParameters.Request()
        req.names = list(names)
        fut = self._get_params_cli.call_async(req)
        if not self._wait_wall_for(fut.done, 10.0):
            self.get_logger().error("get_parameters call timed out")
            return None
        result = fut.result()
        if result is None:
            self.get_logger().error("get_parameters returned no result")
            return None
        snapshot = {}
        for name, pv in zip(names, result.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                snapshot[name] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                snapshot[name] = pv.integer_value
            elif pv.type == ParameterType.PARAMETER_BOOL:
                snapshot[name] = pv.bool_value
            else:
                self.get_logger().error(
                    f"unexpected parameter type for {name}: {pv.type}")
                return None
        return snapshot

    def _apply_params(self, combo: dict) -> bool:
        cli = self._set_params_cli
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"{self._evasion_name}/set_parameters unavailable")
            return False
        req = SetParameters.Request()
        for k, v in combo.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = v
            elif isinstance(v, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = v
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(v)
            req.parameters.append(Parameter(name=k, value=pv))
        fut = cli.call_async(req)
        if not self._wait_wall_for(fut.done, 10.0):
            self.get_logger().error("set_parameters call timed out")
            return False
        ok = all(r.successful for r in fut.result().results)
        if not ok:
            self.get_logger().error(f"set_parameters rejected: {combo}")
        else:
            self.get_logger().info(f"sweep combo applied: {self._combo_str(combo)}")
        return ok

    # ── Reporting ────────────────────────────────────────────────────────

    def _report(self, rows: list) -> int:
        errors = [r for r in rows if r.get("error")]
        scored = [r for r in rows if not r.get("error")]

        lines = [
            "",
            "═" * 72,
            "  HuitzilinReflex Week 4 — Dodge Battery",
            f"  Battery: {self._battery_f.name}   "
            f"Sweep: {Path(self._sweep_f).name if self._sweep_f else '—'}   "
            f"hit_radius: {self._hit_radius} m   window: {self._window_s} s",
            "═" * 72,
            "",
            f"  {'Combo':<26} {'ID':>4} {'Rep':>3} {'Dodge':>5} "
            f"{'MinDist':>8} {'Lat ms':>7} {'OK':>3}  Note",
            "  " + "─" * 68,
        ]
        for r in rows:
            md = r.get("min_dist_m")
            lines.append(
                f"  {r['combo']:<26} {r['id']:>4} {r['rep']:>3} "
                f"{str(r.get('dodged', '-')):>5} "
                f"{md if md is not None else float('nan'):>8} "
                f"{str(r.get('latency_ms', '-')):>7} "
                f"{'✓' if r.get('success') else '✗':>3}  {r['note']}"
            )

        for combo in sorted({r["combo"] for r in scored}):
            sub = [r for r in scored if r["combo"] == combo]
            hits = [r for r in sub if r["expect_dodge"]]
            wides = [r for r in sub if not r["expect_dodge"]]
            n_dodge_ok = sum(1 for r in hits if r["success"])
            n_false = sum(1 for r in wides if not r["success"])
            lats = [r["latency_ms"] for r in sub if r.get("latency_ms") is not None]
            lines += [
                "",
                f"  [{combo}]",
                f"    dodge success: {n_dodge_ok}/{len(hits)}"
                + (f"  ({100.0 * n_dodge_ok / len(hits):.0f}%)" if hits else ""),
                f"    false dodges:  {n_false}/{len(wides)}",
            ]
            if lats:
                lines.append(
                    f"    latency: mean {np.mean(lats):.0f} ms, "
                    f"max {np.max(lats):.0f} ms "
                    f"(budget {self._budget_s * 1000:.0f} ms)")

        verdict = ("HARNESS ERRORS — see rows above" if errors
                   else "COMPLETE (no hard gate; judge rates above)")
        lines += ["", f"  {verdict}", "═" * 72, ""]

        report = "\n".join(lines)
        print(report)
        try:
            self._out_f.parent.mkdir(parents=True, exist_ok=True)
            self._out_f.write_text(report)
            with open(self._csv_f, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=[
                    "combo", "id", "rep", "expect_dodge", "dodged",
                    "latency_ms", "min_dist_m", "success", "error", "note"])
                w.writeheader()
                for r in rows:
                    w.writerow({k: r.get(k) for k in w.fieldnames})
            self.get_logger().info(
                f"Report → {self._out_f} | CSV → {self._csv_f}")
        except Exception as e:  # noqa: BLE001 — report I/O must not mask results
            self.get_logger().warn(f"could not write report: {e}")

        return 1 if errors else 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DodgeBatteryNode()
    exit_code = [1]

    def _run() -> None:
        try:
            exit_code[0] = node.run()
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
