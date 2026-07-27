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

import collections
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
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from huitzilin_perception.ballistics import compute_spawn
from huitzilin_perception.throw_window import (
    straight_leg_time_s,
    throw_window_ok,
)
from huitzilin_perception.spawn_projectile import (
    MIN_SPAWN_Z,
    WrenchThrower,
    gz_remove,
    gz_spawn,
)

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
        # Lead the throw at the drone's PREDICTED position. Without this the aim
        # is a stale snapshot and a patrolling drone (measured 2.5-3.2 m/s on a
        # single clean stack) walks clear on its own: 8 m/s "direct hit" runs
        # measured 1.3-2.6 m closest approach, ~= |v| * t_flight, so the trigger
        # correctly ignored them and they scored as dodge failures. Leading the
        # same throw measures ~0.33 m. Set false only to reproduce old reports.
        self.declare_parameter("lead_target", True)
        # Dead time between sampling odom and the ball actually launching.
        #
        # MEASURED directly 2026-07-27 (spawn_dead_s: odom-sample sim time vs.
        # the ball's first appearance on /gz/dynamic_poses): 0.216 s mean,
        # 0.185-0.247 s over 18 throws. So the dead time is real and it is NOT
        # the ~0.0 s the 2026-07-26 sweep inferred — that sweep was n=2-3 per
        # point and predates the throw-window gate, so it was reading corner
        # noise, not latency.
        #
        # Set to the measured value: declaring it takes the reported undeclared
        # remainder to -0.006 s. But note what it does NOT buy — aim error under
        # patrol only improved 1.18 -> 0.96 m (B04 0.92-0.97 -> 0.82-0.86, B05
        # 1.61-1.66 -> 1.45-1.47), so ~0.8-1.5 m of the patrol aim error is
        # something else. Do not read this parameter as the fix for that; see
        # docs/JOURNAL.md 2026-07-27 for the miss-vector decomposition that is
        # needed next. Re-measure if the throw path ever falls back to the gz
        # CLI (which restores gravity late).
        self.declare_parameter("spawn_latency_s", 0.216)
        # The lead extrapolates the drone at CONSTANT velocity across the
        # ball's flight (0.43 s at 14 m/s, 1.5 s at 4 m/s). That is false while
        # patrol is turning a waypoint, which is what still spoiled battery v7:
        # slow B01 runs missed by 1.0-4.2 m and B07's nominal 1.5 m wide miss
        # arrived at 0.30 m. Waiting for the velocity to stop changing makes the
        # assumption true before spending a throw on it.
        self.declare_parameter("steady_vel_gate_mps", 0.35)
        self.declare_parameter("steady_vel_timeout_s", 5.0)  # wall
        # The steady-velocity gate above is necessary but NOT sufficient: it
        # samples |dv| over 0.15 s, and ArduPilot decelerating into a waypoint
        # looks smooth by that measure while the path is still about to bend.
        # Battery v9 shipped with only that gate and still put 11/17 throws
        # off-target (aim error mean 1.50 m, max 3.79 m). The geometric gate
        # below refuses any throw whose flight would span a patrol corner —
        # see throw_window.py. Requires /huitzilin/patrol_state (patrol_node
        # >= 2026-07-26); with require_throw_window true and no such topic,
        # every run is skipped rather than thrown blind.
        self.declare_parameter("require_throw_window", True)
        self.declare_parameter("throw_window_margin_s", 0.30)
        self.declare_parameter("throw_window_timeout_s", 40.0)  # wall
        # Cruise is ESTIMATED from odom (rolling max over this window) rather
        # than read from patrol's cruise_speed_ms: patrol flies position mode,
        # so ArduPilot's WPNAV_SPEED governs, not patrol's parameter. Measured
        # 3.49 m/s max against a patrol.yaml cruise_speed_ms of 1.5.
        self.declare_parameter("cruise_estimate_window_s", 20.0)  # wall
        # 0.95, derived from v12's residual bias — see throw_window.py.
        self.declare_parameter("min_cruise_frac", 0.95)
        # ── Hover control mode (2026-07-27) ──────────────────────────────────
        # Aim error under patrol has two possible sources — the constant-velocity
        # lead being wrong, and the spawn geometry itself being wrong — and the
        # patrol battery cannot separate them. Against a stationary target the
        # lead is exact by construction, so any residual error is geometry.
        #
        # This needs its own mode because a hover battery is otherwise
        # impossible to run: evasion_node resumes patrol when a dodge completes
        # ("dodge complete -> TRACKING (patrol resumed)"), so simply calling
        # /huitzilin/start_patrol false before the battery only holds for the
        # first successful dodge. Measured 2026-07-27: run 1 hovered, all 17
        # after it were back at cruise. Patrol is therefore re-stopped before
        # every throw, not once at the start.
        self.declare_parameter("hover_mode", False)
        self.declare_parameter("hover_speed_gate_mps", 0.15)
        self.declare_parameter("hover_settle_timeout_s", 25.0)  # wall
        # A throw whose measured closest approach is this far from the
        # scenario's miss_distance_m did not test what the scenario claims.
        # Reported separately so aiming error is never read as trigger error.
        self.declare_parameter("off_target_tol_m", 0.5)

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
        self._lead_target = bool(self.get_parameter("lead_target").value)
        self._spawn_latency_s = float(
            self.get_parameter("spawn_latency_s").value)
        self._steady_gate = float(
            self.get_parameter("steady_vel_gate_mps").value)
        self._steady_timeout = float(
            self.get_parameter("steady_vel_timeout_s").value)
        self._off_target_tol = float(
            self.get_parameter("off_target_tol_m").value)
        self._require_window = bool(
            self.get_parameter("require_throw_window").value)
        self._window_margin_s = float(
            self.get_parameter("throw_window_margin_s").value)
        self._window_timeout_s = float(
            self.get_parameter("throw_window_timeout_s").value)
        self._cruise_win_s = float(
            self.get_parameter("cruise_estimate_window_s").value)
        self._min_cruise_frac = float(
            self.get_parameter("min_cruise_frac").value)
        self._evasion_name = self.get_parameter("evasion_node_name").value
        self._hover_mode = bool(self.get_parameter("hover_mode").value)
        self._hover_gate = float(self.get_parameter("hover_speed_gate_mps").value)
        self._hover_timeout_s = float(
            self.get_parameter("hover_settle_timeout_s").value)

        # Warm wrench publishers, built once: every throw needs its impulse and
        # its gravity restore on the same physics step, which the gz CLI cannot
        # do. run() blocks on wait_for_bridge before the first scenario.
        self._thrower = WrenchThrower(self, self._world)

        self._lock = threading.Lock()
        self._latest_odom = None
        self._latest_patrol = None    # None until /huitzilin/patrol_state arrives
        self._speed_hist = collections.deque()   # (wall_t, |v|) for cruise est
        # _odom_cb mutates _speed_hist on the executor thread while
        # _cruise_est reads it on the run() worker thread; without this the
        # read raises "deque mutated during iteration" and kills a run
        # (observed once in battery v13, B01 r2).
        self._speed_lock = threading.Lock()
        self._pose_stream_seen = False
        self._active_ball = None
        # Sim time at which the spawned ball first appeared on
        # /gz/dynamic_poses. Paired with the sim time the odom sample was taken,
        # this measures the spawn dead time the lead has to compensate for --
        # compute_spawn's spawn_latency_s. It is currently parameterised at 0.0
        # while the docstring estimates ~0.5 s of sim time for the gz create
        # call, and at a 5.26 m/s cruise every 0.1 s of it is 0.5 m of aim
        # error. The hover control (2026-07-27) showed the spawn geometry is
        # exact (aim error 0.10 m mean, 0/19 off-target), so this dead time is
        # the leading candidate for the whole patrol aim error -- measure it
        # rather than assume the 0.5 s.
        self._ball_seen_sim = None
        self._min_dist = float("inf")
        self._events = []
        self._listening = False

        self.create_subscription(Odometry, "/huitzilin/odom",
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, SENSOR_QOS)
        self.create_subscription(String, "/threat/evade_event",
                                 self._event_cb, RELIABLE_QOS)
        self.create_subscription(String, "/huitzilin/patrol_state",
                                 self._patrol_cb, RELIABLE_QOS)

        # Created once and reused for every sweep combo + the baseline
        # snapshot/restore — avoids leaking a service client per combo.
        self._patrol_cli = self.create_client(SetBool,
                                              "/huitzilin/start_patrol")
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
        tw = msg.twist.twist.linear
        speed = math.sqrt(tw.x * tw.x + tw.y * tw.y + tw.z * tw.z)
        now = time.monotonic()
        cutoff = now - self._cruise_win_s
        with self._speed_lock:
            self._speed_hist.append((now, speed))
            while self._speed_hist and self._speed_hist[0][0] < cutoff:
                self._speed_hist.popleft()

    def _cruise_est(self) -> float:
        """Rolling max odom speed — the drone's actual cruise.

        Max rather than mean because the mean is dragged down by the corner
        transitions the gate exists to avoid (measured median 2.09 m/s vs max
        3.49 m/s over the same 45 s). Returns 0.0 before any odom, which makes
        straight_leg_time_s report inf; that is safe because run() will not
        start a run until odom exists.
        """
        with self._speed_lock:
            if not self._speed_hist:
                return 0.0
            return max(s for _, s in self._speed_hist)

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
            if ball is not None and self._ball_seen_sim is None:
                # First sighting only — this is a launch timestamp, not a
                # tracking one, and /gz/dynamic_poses keeps reporting the ball
                # for the rest of the window.
                self._ball_seen_sim = self._sim_now()
            if drone is not None and ball is not None:
                d = float(np.linalg.norm(ball - drone))
                if d < self._min_dist:
                    self._min_dist = d

    def _patrol_cb(self, msg: String) -> None:
        try:
            self._latest_patrol = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("unparseable /huitzilin/patrol_state payload")

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

    def _odom_vel(self) -> np.ndarray:
        tw = self._latest_odom.twist.twist.linear
        return np.array([tw.x, tw.y, tw.z])

    def _wait_steady_velocity(self) -> tuple[bool, np.ndarray]:
        """Block until the drone's velocity stops changing.

        compute_spawn's lead assumes constant velocity over the whole flight,
        so throwing mid-turn aims at a point the drone never reaches. Returns
        (steady, velocity); on timeout returns the latest velocity anyway so a
        stuck-in-a-turn patrol cannot deadlock the battery — the run is then
        tagged off-target rather than silently trusted.
        """
        t0 = time.monotonic()
        prev = self._odom_vel()
        while time.monotonic() - t0 < self._steady_timeout:
            time.sleep(0.15)
            cur = self._odom_vel()
            if float(np.linalg.norm(cur - prev)) <= self._steady_gate:
                return True, cur
            prev = cur
        return False, self._odom_vel()

    def _wait_throw_window(self, t_flight_s: float):
        """Block until enough straight patrol leg remains to aim this throw.

        Returns (ok, reason, best_leg_s, needed_s). best_leg_s is the longest
        window actually observed while waiting, which is the evidence for
        whether a scenario is measurable on this patrol loop at all: if the
        best window over 25 s is still short of needed_s, the loop's legs are
        too short for that flight time and no amount of retrying will help.
        """
        needed_s = float(t_flight_s) + self._window_margin_s
        if not self._require_window:
            return True, "throw window not enforced", None, needed_s

        t0 = time.monotonic()
        best_leg = 0.0
        reason = "no patrol state on /huitzilin/patrol_state"
        while time.monotonic() - t0 < self._window_timeout_s:
            st = self._latest_patrol
            speed = float(np.linalg.norm(self._odom_vel()))
            cruise = self._cruise_est()
            if st is None:
                ok, reason = throw_window_ok(
                    dist_to_wp_m=None, accept_radius_m=0.6,
                    speed_mps=speed, cruise_mps=cruise,
                    t_flight_s=t_flight_s,
                    margin_s=self._window_margin_s,
                    min_cruise_frac=self._min_cruise_frac)
            else:
                accept = float(st.get("accept_radius_m", 0.6))
                dist = st.get("dist_m")
                running = bool(st.get("running", True))
                ok, reason = throw_window_ok(
                    dist_to_wp_m=dist, accept_radius_m=accept,
                    speed_mps=speed, cruise_mps=cruise,
                    t_flight_s=t_flight_s,
                    margin_s=self._window_margin_s,
                    min_cruise_frac=self._min_cruise_frac,
                    patrol_running=running)
                if dist is not None:
                    # Evaluated at cruise, so best_leg is the honest ceiling
                    # this patrol loop can offer — the evidence for whether a
                    # scenario is measurable at all.
                    leg = straight_leg_time_s(dist, accept, cruise)
                    if leg != math.inf:
                        best_leg = max(best_leg, leg)
            if ok:
                return True, reason, best_leg, needed_s
            time.sleep(0.05)
        return False, reason, best_leg, needed_s

    # ── Main flow ────────────────────────────────────────────────────────

    def run(self) -> int:
        if not self._thrower.wait_for_bridge():
            self.get_logger().error(
                "wrench bridge never connected — every throw would leave the "
                "ball hanging motionless. Start the stack with "
                "week4_evasion.launch.py (it launches wrench_bridge).")
            return 1
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

        speed = float(scen["speed_mps"])
        spec_miss = float(scen.get("miss_distance_m", 0.0))
        offset_forward = float(scen.get("offset_forward_m", 6.0))
        # Ball flight time: the horizon the constant-velocity lead has to
        # extrapolate over, and therefore the length of straight patrol leg
        # this throw needs in order to test what the scenario claims.
        t_flight = offset_forward / speed if speed > 0.0 else 0.0

        if self._hover_mode:
            # Before the steady-velocity wait, not after: the point is to make
            # the velocity zero, and _wait_steady_velocity would otherwise pass
            # trivially on a drone flying a smooth straight leg.
            hover_ok, hover_reason = self._enter_hover()
            if not hover_ok:
                return {**base, "error": True, "success": False,
                        "note": f"hover_mode: {hover_reason}"}

        steady, vel = self._wait_steady_velocity()
        if not steady:
            self.get_logger().warn(
                f"{rid} r{rep}: velocity still changing after "
                f"{self._steady_timeout:.0f} s (|dv| > {self._steady_gate} m/s) "
                "— the constant-velocity lead may be off")
        # Geometric gate LAST, so the window is fresh when the ball launches.
        win_ok, win_reason, win_leg, win_needed = self._wait_throw_window(t_flight)
        if not win_ok:
            self.get_logger().warn(f"{rid} r{rep}: no aimable window — {win_reason}")
            return {**base, "error": False, "skipped": True, "success": False,
                    "steady": steady,
                    "window_s": round(win_leg, 3) if win_leg is not None else None,
                    "needed_window_s": round(win_needed, 3),
                    "note": f"SKIPPED, not thrown (no aimable window): "
                            f"{win_reason} | best leg seen "
                            f"{win_leg:.2f} s over {self._window_timeout_s:.0f} s"}

        # Re-read odom AFTER the waits so position matches that velocity.
        odom = self._latest_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # twist.linear is world ENU here (mav_bridge_node publishes
        # ned_to_enu(vn, ve, vd)), NOT body FLU as REP-103 would imply for
        # child_frame_id=base_link. compute_spawn's lead assumes ENU, so this
        # pairing is only correct as long as the bridge keeps doing that.
        # A speed-0 scenario (the Week 3 matrix has some) has no flight time to
        # lead over, and compute_spawn rejects the combination; degrade instead
        # of failing the run.
        lead = tuple(vel) if (self._lead_target and speed > 0.0) else None
        plan = compute_spawn(
            (p.x, p.y, p.z), yaw,
            speed_mps=speed,
            approach_angle_deg=float(scen.get("approach_angle_deg", 0.0)),
            miss_distance_m=spec_miss,
            offset_forward_m=offset_forward,
            offset_vertical_m=float(scen.get("offset_vertical_m", 0.0)),
            compensate_gravity=bool(scen.get("compensate_gravity", True)),
            aim_at_drone=bool(scen.get("aim_at_drone", False)),
            target_vel_enu=lead,
            spawn_latency_s=self._spawn_latency_s,
        )
        if plan.position[2] < MIN_SPAWN_Z:
            return {**base, "error": True, "success": False,
                    "note": f"spawn z={plan.position[2]:.2f} < {MIN_SPAWN_Z}"}

        # Sim time of the state the plan was computed from. Taken here, after
        # compute_spawn and immediately before the create call, so the measured
        # dead time covers exactly what the lead fails to account for.
        sample_sim = self._sim_now()
        name = f"ball_{rid}_r{rep}_{int(time.time())}"
        with self._lock:
            self._active_ball = name
            self._ball_seen_sim = None
            self._min_dist = float("inf")
            self._events = []
            self._listening = True

        ok, msg = gz_spawn(self._world, self._model_uri, name,
                           plan.position, plan.velocity,
                           thrower=self._thrower)
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
            seen_sim = self._ball_seen_sim
        dead_s = (None if seen_sim is None
                  else round(seen_sim - sample_sim, 3))

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

        # Did this throw actually test the specified geometry? A dodge changes
        # min_dist by design, so aim error is only meaningful when no dodge
        # fired; a successful dodge is credited as on-target.
        aim_err = abs(min_dist - spec_miss)
        off_target = (not dodged) and aim_err > self._off_target_tol
        if off_target:
            note += (f" | OFF-TARGET: spec miss {spec_miss:.2f} m, "
                     f"measured {min_dist:.2f} m — aiming, not trigger")
        if not steady:
            note += " | unsteady aim"

        return {**base, "error": False, "skipped": False, "dodged": dodged,
                "latency_ms": latency_ms, "min_dist_m": round(min_dist, 3),
                "spec_miss_m": spec_miss, "aim_err_m": round(aim_err, 3),
                "off_target": off_target, "steady": steady,
                "window_s": round(win_leg, 3) if win_leg is not None else None,
                "needed_window_s": round(win_needed, 3),
                "spawn_dead_s": dead_s,
                "success": success, "note": note}

    def _enter_hover(self) -> tuple[bool, str]:
        """Stop patrol and wait for the drone to actually be stationary.

        Two separate things, and the second is the one that matters: stopping
        patrol only stops new setpoints being sent. ArduPilot keeps flying to
        the last position target it was given, which on a 12 m loop can be most
        of a leg away, so the drone is still at cruise for seconds afterwards.
        Returning as soon as the service replies would silently produce a
        moving "hover" control — which is exactly what the first attempt at
        this measurement did.

        Returns (ok, reason); a timeout is reported, never assumed away.
        """
        if not self._patrol_cli.wait_for_service(timeout_sec=10.0):
            return False, "/huitzilin/start_patrol unavailable"
        fut = self._patrol_cli.call_async(SetBool.Request(data=False))
        if not self._wait_wall_for(fut.done, 10.0):
            return False, "start_patrol(false) call timed out"

        def stopped() -> bool:
            with self._speed_lock:
                if not self._speed_hist:
                    return False
                return self._speed_hist[-1][1] < self._hover_gate

        if not self._wait_wall_for(stopped, self._hover_timeout_s):
            with self._speed_lock:
                last = self._speed_hist[-1][1] if self._speed_hist else float("nan")
            return False, (f"still moving at {last:.2f} m/s after "
                           f"{self._hover_timeout_s:.0f} s (gate "
                           f"{self._hover_gate} m/s)")
        return True, "hovering"

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
        # A skipped run is neither an error nor a measurement: no ball was
        # thrown, so counting it as a dodge failure would repeat exactly the
        # v9 mistake of reading aiming problems as trigger problems.
        skipped = [r for r in rows if r.get("skipped") and not r.get("error")]
        scored = [r for r in rows
                  if not r.get("error") and not r.get("skipped")]

        lines = [
            "",
            "═" * 72,
            "  HuitzilinReflex Week 4 — Dodge Battery",
            f"  Battery: {self._battery_f.name}   "
            f"Sweep: {Path(self._sweep_f).name if self._sweep_f else '—'}   "
            f"hit_radius: {self._hit_radius} m   window: {self._window_s} s",
            # State the target's motion first: every aim number below means
            # something different depending on it, and a hover control read as
            # a patrol result would be badly misleading.
            ("  target: HOVER CONTROL (patrol stopped before every throw — "
             "the lead is exact, so residual aim error is spawn GEOMETRY)"
             if self._hover_mode else
             "  target: patrolling (aim error mixes lead error with geometry)"),
            f"  lead_target: {self._lead_target}   "
            f"spawn_latency_s: {self._spawn_latency_s} s"
            + ("" if self._lead_target else
               "   << UNLED: throws aim at a stale point, so a patrolling "
               "drone walks clear on its own"),
            f"  throw window: {'enforced' if self._require_window else 'OFF'}"
            + (f"   margin: {self._window_margin_s} s   "
               f"wait: {self._window_timeout_s} s   "
               f"cruise est: {self._cruise_est():.2f} m/s "
               f"(floor {self._min_cruise_frac:.0%})" if self._require_window else
               "   << throws may span a patrol corner; aim error will be large"),
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
            # On-target subsets: a throw that missed its specified geometry
            # tests the throw harness, not the dodge. Battery v7 (2026-07-26)
            # is why this is printed: 3 of 3 B01 runs missed by 1.0-4.2 m and
            # B07's nominal 1.5 m wide miss arrived at 0.30 m, so its
            # "0/2 false dodges" was crediting the trigger for ignoring a
            # near-hit.
            on_hits = [r for r in hits if not r.get("off_target")]
            on_wides = [r for r in wides if not r.get("off_target")]
            n_on_ok = sum(1 for r in on_hits if r["success"])
            n_on_false = sum(1 for r in on_wides if not r["success"])
            n_off = sum(1 for r in sub if r.get("off_target"))
            n_unsteady = sum(1 for r in sub if not r.get("steady", True))
            aim_errs = [r["aim_err_m"] for r in sub
                        if r.get("aim_err_m") is not None
                        and not r.get("dodged")]
            lines += [
                "",
                f"  [{combo}]",
                f"    dodge success: {n_dodge_ok}/{len(hits)}"
                + (f"  ({100.0 * n_dodge_ok / len(hits):.0f}%)" if hits else ""),
                f"    false dodges:  {n_false}/{len(wides)}",
                f"    -- on-target runs only (valid dodge measurement) --",
                f"    dodge success: {n_on_ok}/{len(on_hits)}"
                + (f"  ({100.0 * n_on_ok / len(on_hits):.0f}%)" if on_hits else ""),
                f"    false dodges:  {n_on_false}/{len(on_wides)}",
                f"    off-target: {n_off}/{len(sub)} throws "
                f"(> {self._off_target_tol} m from spec)"
                + (f"   unsteady aim: {n_unsteady}" if n_unsteady else ""),
            ]
            if aim_errs:
                lines.append(
                    f"    aim error (no-dodge runs): mean "
                    f"{np.mean(aim_errs):.2f} m, max {np.max(aim_errs):.2f} m")
            # Spawn dead time: sim seconds between the odom sample the plan was
            # built from and the ball actually existing. The lead only
            # compensates for the part declared in spawn_latency_s, so the
            # undeclared remainder becomes aim error at the target's speed.
            deads = [r["spawn_dead_s"] for r in sub
                     if r.get("spawn_dead_s") is not None]
            if deads:
                undeclared = float(np.mean(deads)) - self._spawn_latency_s
                cruise = self._cruise_est()
                lines.append(
                    f"    spawn dead time: mean {np.mean(deads):.3f} s, "
                    f"max {np.max(deads):.3f} s (sim) — declared "
                    f"spawn_latency_s {self._spawn_latency_s:.3f} s, "
                    f"undeclared {undeclared:+.3f} s "
                    f"= {abs(undeclared) * cruise:.2f} m of aim error at "
                    f"{cruise:.2f} m/s")
            if lats:
                lines.append(
                    f"    latency: mean {np.mean(lats):.0f} ms, "
                    f"max {np.max(lats):.0f} ms "
                    f"(budget {self._budget_s * 1000:.0f} ms)")

        if skipped:
            # Per-scenario, because the constraint is per flight time: a 4 m/s
            # throw needs 1.5 s of straight leg, a 14 m/s throw only 0.43 s.
            lines += ["", f"  [skipped: {len(skipped)}/{len(rows)} runs never "
                          f"thrown — no aimable window]"]
            for rid in sorted({r["id"] for r in skipped}):
                sub = [r for r in skipped if r["id"] == rid]
                legs = [r["window_s"] for r in sub if r.get("window_s") is not None]
                need = next((r["needed_window_s"] for r in sub
                             if r.get("needed_window_s") is not None), None)
                best = f"{max(legs):.2f}" if legs else "?"
                lines.append(
                    f"    {rid}: {len(sub)} skipped | best straight leg seen "
                    f"{best} s, needed {need} s")
            lines += [
                "    A scenario skipped every time is UNMEASURABLE on this",
                "    patrol loop: its flight time exceeds the longest leg.",
                "    Lengthen the loop (patrol.yaml waypoints_ned) or shorten",
                "    the flight (offset_forward_m) — do not lower the margin.",
            ]

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
                    "latency_ms", "min_dist_m", "spec_miss_m", "aim_err_m",
                    "off_target", "steady", "success", "error", "note",
                    "skipped", "window_s", "needed_window_s", "spawn_dead_s"])
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
