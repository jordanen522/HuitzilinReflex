"""
evasion_node.py — HuitzilinReflex Week 4: Kalman filter + dodge trigger.

Closes the sense->dodge loop:
  /threat/centroid (base_link) --re-express in odom--> ballistic KF track
  -> predicted closest approach vs the drone -> trigger policy
  -> pause patrol + stream /cmd/evade (body FLU) + /payload/alarm (mock)
  -> recover -> resume patrol.

State machine (docs/state_machine.md PATROL->EVADE->PATROL):
  TRACKING -> EVADING -> RECOVERING -> TRACKING

Patrol preemption: patrol_node (position mode) streams position setpoints on
its OWN MAVLink connection, so a dodge velocity spike would fight it. The
dodge therefore calls /huitzilin/start_patrol false first; mav_bridge gives
/cmd/evade priority over /huitzilin/cmd_vel for velocity-mode patrol too.

All timing is SIM time (launch sets use_sim_time; latency is measured
against message stamps) — never wall-clock (CLAUDE.md sharp edge).

Frames: centroids arrive in base_link; the filter runs in fixed odom ENU
(filtering in the moving body frame would alias drone motion into the
projectile velocity — Week 3's egomotion lesson, one layer up). Dodge
commands go out in body FLU; mav_bridge owns the only NED conversion.
"""

from __future__ import annotations

import sys
import json
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Accel, PointStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

from huitzilin_perception.cloud_geometry import (
    apply_transform,
    is_valid_quat,
    make_transform,
    quat_to_rot,
    rotate_covariance,
)
from huitzilin_perception.kalman import (
    GRAVITY_ENU,
    MultiHypothesisTracker,
    dodge_velocity_command,
    effective_min_updates,
    plan_dodge,
    should_dodge,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

# Declared params that are consumed exactly once at node start (tracker
# construction, topic wiring, timer creation). Rejecting writes keeps the
# ROS parameter surface honest: what `ros2 param get` shows is what runs.
FIXED_AT_START = {
    "centroid_topic", "odom_topic", "evade_topic", "alarm_topic",
    "intercept_topic", "event_topic", "marker_topic", "patrol_service",
    "process_accel_std", "meas_std_m", "meas_std_xyz_m", "init_vel_std",
    "track_timeout_s", "max_tracks", "max_assoc_sigma_m", "evade_cmd_rate_hz",
    "evade_accel_topic", "cue_topic",
}


def _resolve_meas_std_xyz(values) -> Optional[np.ndarray]:
    """meas_std_xyz_m -> the per-axis (forward, left, up) body-frame std used
    to build a per-measurement R, or None if the anisotropic path is
    disabled.

    All-zero is the disable sentinel, not empty -- the identical convention
    oracle_detector.yaml's noise_std_xyz_m already uses. Length is checked
    eagerly because this value IS the enable/disable decision: a mis-sized
    override must fail loudly at startup, not silently broadcast into the
    wrong shape on the first centroid.
    """
    xyz = [float(v) for v in values]
    if len(xyz) != 3:
        raise ValueError(
            "meas_std_xyz_m must be exactly 3 values, got %d" % len(xyz))
    return np.asarray(xyz, dtype=np.float64) if any(v > 0.0 for v in xyz) else None


class EvadeState(Enum):
    TRACKING = "TRACKING"
    EVADING = "EVADING"
    RECOVERING = "RECOVERING"
    HANDOFF = "HANDOFF"


class EvasionNode(Node):
    """Kalman tracker + dodge trigger. See module docstring."""

    def __init__(self) -> None:
        super().__init__("evasion")

        # ── Params (values from params/evasion.yaml) ─────────────────────
        self.declare_parameter("centroid_topic", "/threat/centroid")
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("evade_topic", "/cmd/evade")
        self.declare_parameter("alarm_topic", "/payload/alarm")
        self.declare_parameter("intercept_topic", "/threat/intercept")
        self.declare_parameter("event_topic", "/threat/evade_event")
        self.declare_parameter("marker_topic", "/threat/intercept_marker")
        self.declare_parameter("patrol_service", "/huitzilin/start_patrol")
        self.declare_parameter("process_accel_std", 3.0)
        self.declare_parameter("meas_std_m", 0.15)
        # (forward, left, up) sigma in base_link. All-zero = disabled, use
        # the isotropic meas_std_m above -- mirrors oracle_detector.yaml's
        # identical noise_std_xyz_m sentinel convention.
        self.declare_parameter("meas_std_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("init_vel_std", 15.0)
        self.declare_parameter("track_timeout_s", 0.5)
        self.declare_parameter("max_tracks", 6)
        self.declare_parameter("max_assoc_sigma_m", 0.75)
        self.declare_parameter("min_track_updates", 3)
        self.declare_parameter("threat_radius_m", 0.75)
        self.declare_parameter("trigger_horizon_s", 1.5)
        self.declare_parameter("prediction_horizon_s", 3.0)
        self.declare_parameter("latency_budget_s", 0.15)
        self.declare_parameter("dodge_speed_mps", 1.5)
        self.declare_parameter("dodge_duration_s", 1.0)
        self.declare_parameter("dodge_floor_m", 1.0)
        self.declare_parameter("dodge_max_speed_mps", 6.0)
        self.declare_parameter("recover_hold_s", 0.5)
        self.declare_parameter("patrol_handoff_s", 0.8)
        self.declare_parameter("evade_cmd_rate_hz", 20.0)
        self.declare_parameter("auto_resume_patrol", True)
        # ── Phase 5: acceleration feedforward (default off = old behaviour) ──
        self.declare_parameter("evade_accel_topic", "/cmd/evade_accel")
        self.declare_parameter("evade_accel_ff_mps2", 0.0)
        # ── Phase 6: cue-gated confirmation (inert with no cue publisher) ───
        self.declare_parameter("cue_topic", "/threat/cue")
        self.declare_parameter("alert_min_track_updates", 2)
        self.declare_parameter("cue_timeout_s", 2.0)
        # ── Phase 7: vertical escape (default off = old behaviour) ──────────
        self.declare_parameter("allow_upward_escape", False)
        self.declare_parameter("escape_ceiling_m", 5.0)

        self._p = {
            name: self.get_parameter(name).value
            for name in (
                "centroid_topic", "odom_topic", "evade_topic", "alarm_topic",
                "intercept_topic", "event_topic", "marker_topic",
                "patrol_service", "process_accel_std", "meas_std_m",
                "meas_std_xyz_m",
                "init_vel_std", "track_timeout_s", "max_tracks",
                "max_assoc_sigma_m", "min_track_updates",
                "threat_radius_m", "trigger_horizon_s", "prediction_horizon_s",
                "latency_budget_s", "dodge_speed_mps", "dodge_duration_s",
                "dodge_floor_m", "dodge_max_speed_mps",
                "recover_hold_s", "patrol_handoff_s", "evade_cmd_rate_hz",
                "auto_resume_patrol",
                "evade_accel_topic", "evade_accel_ff_mps2",
                "cue_topic", "alert_min_track_updates", "cue_timeout_s",
                "allow_upward_escape", "escape_ceiling_m",
            )
        }
        self.add_on_set_parameters_callback(self._on_param_set)
        self._meas_std_xyz = _resolve_meas_std_xyz(self._p["meas_std_xyz_m"])

        # ── Tracker + state machine ──────────────────────────────────────
        self._tracker = MultiHypothesisTracker(
            max_tracks=int(self._p["max_tracks"]),
            max_assoc_sigma_m=float(self._p["max_assoc_sigma_m"]),
            process_accel_std=self._p["process_accel_std"],
            meas_std_m=self._p["meas_std_m"],
            init_vel_std=self._p["init_vel_std"],
            track_timeout_s=self._p["track_timeout_s"],
        )
        self._state = EvadeState.TRACKING
        self._phase_end = None            # rclpy Time when current phase ends
        self._dodge_cmd_body = np.zeros(3)
        self._dodge_accel_body = np.zeros(3)
        self._last_cue_s = None           # sim seconds of the last ALERT cue
        self._last_odom = None
        self._warned_no_odom = False
        self._warned_bad_quat = False

        # ── ROS interfaces ───────────────────────────────────────────────
        self.create_subscription(PointStamped, self._p["centroid_topic"],
                                 self._centroid_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, self._p["odom_topic"],
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(Bool, self._p["cue_topic"],
                                 self._cue_cb, RELIABLE_QOS)
        self._evade_pub = self.create_publisher(
            Twist, self._p["evade_topic"], RELIABLE_QOS)
        self._evade_accel_pub = self.create_publisher(
            Accel, self._p["evade_accel_topic"], RELIABLE_QOS)
        self._alarm_pub = self.create_publisher(
            Bool, self._p["alarm_topic"], RELIABLE_QOS)
        self._intercept_pub = self.create_publisher(
            PointStamped, self._p["intercept_topic"], RELIABLE_QOS)
        self._event_pub = self.create_publisher(
            String, self._p["event_topic"], RELIABLE_QOS)
        self._marker_pub = self.create_publisher(
            Marker, self._p["marker_topic"], RELIABLE_QOS)
        self._patrol_cli = self.create_client(SetBool, self._p["patrol_service"])

        self.create_timer(1.0 / float(self._p["evade_cmd_rate_hz"]),
                          self._evade_tick)

        self.get_logger().info(
            "evasion_node ready — "
            f"threat_radius {self._p['threat_radius_m']} m, "
            f"trigger_horizon {self._p['trigger_horizon_s']} s, "
            f"dodge {self._p['dodge_speed_mps']} m/s "
            f"for {self._p['dodge_duration_s']} s"
        )

    # ── Param updates (sweep support) ────────────────────────────────────

    def _on_param_set(self, params) -> SetParametersResult:
        """Validate every parameter in the set before applying any of them.

        `ros2 param set` is atomic from ROS's side: one rejected parameter
        rolls back the whole request, so none of the declared values change.
        A single loop that wrote self._p as it went could not honour that --
        an earlier key was already live in the shadow dict by the time a later
        key was refused, leaving self._p and `ros2 param get` disagreeing with
        no way to tell from either side. The dodge battery's sweep sets several
        keys in one call, so that divergence would silently mislabel a row.
        """
        for prm in params:
            if prm.name in FIXED_AT_START:
                return SetParametersResult(
                    successful=False,
                    reason=f"{prm.name} is fixed at node start — "
                           "edit params/evasion.yaml and restart")

        for prm in params:
            # Every live-read tunable, i.e. everything in the shadow dict that
            # FIXED_AT_START did not already refuse above. There is deliberately
            # no second allow-list: the sweep surface the battery actually grids
            # is owned by config/week4_sweep.yaml, and a hand-maintained copy of
            # it here would drift without anything failing.
            if prm.name in self._p:
                self._p[prm.name] = prm.value
                self.get_logger().info(f"param {prm.name} -> {prm.value}")
        return SetParametersResult(successful=True)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _cue_cb(self, msg: Bool) -> None:
        """External ALERT cue: something fast is probably incoming.

        Only a True latches. A False is not "stand down" -- it would let a
        stale or mistaken publisher cancel an alert the moment before it
        mattered -- so the alert always expires by its own timeout instead.
        """
        if msg.data:
            self._last_cue_s = self.get_clock().now().nanoseconds * 1e-9

    def _publish_evade(self, vel_body, accel_body) -> None:
        """One evade command. The acceleration goes out FIRST, and always.

        First, because the bridge only attaches a fresh acceleration to a
        velocity setpoint -- publishing it after would leave the current
        command carrying the previous tick's feedforward.

        Always, including the zeros during RECOVERING: the settle zeros are
        still fresh /cmd/evade messages, so an accel left un-updated would be
        attached to them and shove the aircraft while it was meant to be
        settling.
        """
        acc = Accel()
        acc.linear.x = float(accel_body[0])
        acc.linear.y = float(accel_body[1])
        acc.linear.z = float(accel_body[2])
        self._evade_accel_pub.publish(acc)

        cmd = Twist()
        cmd.linear.x = float(vel_body[0])
        cmd.linear.y = float(vel_body[1])
        cmd.linear.z = float(vel_body[2])
        self._evade_pub.publish(cmd)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _update_tracker(self, t: float, z_odom: np.ndarray,
                        Rot: np.ndarray) -> None:
        """One measurement into the tracker. With meas_std_xyz_m disabled
        (the default) this is EXACTLY the old two-argument call -- never
        R=None, never a computed-but-equal R -- reached by an early branch.

        `Rot` must be the SAME base_link->odom rotation already used to lift
        the point itself (T_odom_bl's rotation block), so the covariance and
        the point it belongs to are re-expressed the same way.
        """
        if self._meas_std_xyz is None:
            self._tracker.process(t, z_odom)
            return
        R_body = np.diag(np.square(self._meas_std_xyz))
        self._tracker.process(t, z_odom, rotate_covariance(R_body, Rot))

    def _centroid_cb(self, msg: PointStamped) -> None:
        if self._state is not EvadeState.TRACKING:
            return  # mid-dodge/recovery: the tracker restarts fresh afterwards

        odom = self._last_odom
        if odom is None:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn("centroid before first odom — waiting")
            return
        q = odom.pose.pose.orientation
        if not is_valid_quat(q.x, q.y, q.z, q.w):
            if not self._warned_bad_quat:
                self._warned_bad_quat = True
                self.get_logger().warn(
                    "odom carries no valid orientation — cannot lift centroids "
                    "into odom; evasion inactive (pre-b0eedd5-style stream?)")
            return

        # base_link -> odom (odom pose IS T_odom_baselink)
        p = odom.pose.pose.position
        T_odom_bl = make_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        z_bl = np.array([msg.point.x, msg.point.y, msg.point.z])
        z_odom = apply_transform(T_odom_bl, z_bl[None, :])[0].astype(np.float64)

        self._update_tracker(self._stamp_to_sec(msg.header.stamp), z_odom,
                             T_odom_bl[:3, :3])

        min_updates = effective_min_updates(
            self.get_clock().now().nanoseconds * 1e-9, self._last_cue_s,
            cue_timeout_s=float(self._p["cue_timeout_s"]),
            patrol_updates=int(self._p["min_track_updates"]),
            alert_updates=int(self._p["alert_min_track_updates"]))
        confirmed = self._tracker.confirmed(min_updates=min_updates)
        if not confirmed:
            return

        p_drone = np.array([p.x, p.y, p.z])
        tw = odom.twist.twist.linear
        v_drone = np.array([tw.x, tw.y, tw.z])  # world ENU (bridge converts NED)

        # Plan against EVERY confirmed hypothesis and act on the most
        # threatening. With one filter the question did not arise — whatever
        # the last centroid became was the threat — and that is precisely how a
        # false positive got to speak for the ball (see MultiHypothesisTracker).
        # Ranking: a track that clears the trigger policy always outranks one
        # that does not, then soonest closest approach, then smallest miss.
        best = None
        for track in confirmed:
            pos_proj, vel_proj = track.state()
            # p.z is height above the odom origin, which the bridge places at
            # the EKF origin = the takeoff point on the ground, so it IS
            # altitude AGL on flat ground. On sloped terrain this over-reads
            # and the clamp gets optimistic; a Week 6 field-test item.
            plan = plan_dodge(
                pos_proj, vel_proj, p_drone, v_drone,
                horizon_s=float(self._p["prediction_horizon_s"]),
                altitude_m=float(p.z),
                floor_m=float(self._p["dodge_floor_m"]),
                descent_len_m=(float(self._p["dodge_speed_mps"])
                               * float(self._p["dodge_duration_s"])),
                allow_upward=bool(self._p["allow_upward_escape"]),
                # Always passed, even with upward escape off: an escape that
                # already pointed up was previously unclamped, which only ever
                # stayed inside the fence because the geometric escape could
                # not reach it from 2 m AGL.
                ceiling_m=float(self._p["escape_ceiling_m"]),
            )
            fires = should_dodge(
                track.n_updates, plan.miss_m, plan.tca_s,
                min_updates=min_updates,
                threat_radius_m=float(self._p["threat_radius_m"]),
                trigger_horizon_s=float(self._p["trigger_horizon_s"]),
            )
            rank = (not fires, plan.tca_s, plan.miss_m)
            if best is None or rank < best[0]:
                best = (rank, fires, plan, track, pos_proj, vel_proj)

        _, fires, plan, track, pos_proj, vel_proj = best
        self._publish_intercept(plan, pos_proj, vel_proj, T_odom_bl,
                                msg.header.stamp)
        if fires:
            self._start_dodge(plan, track, q, msg.header.stamp, v_drone,
                              min_updates)

    # ── Dodge lifecycle ──────────────────────────────────────────────────

    def _start_dodge(self, plan, track, q, stamp, v_drone,
                     min_updates_used) -> None:
        R = quat_to_rot(q.x, q.y, q.z, q.w)          # base_link -> odom
        dir_body = R.T @ plan.direction               # odom -> body FLU
        # Escape is measured against the drone's OWN extrapolated track,
        # because that is where a thrown ball is aimed. Commanding an absolute
        # dodge_speed here instead of adding it to the cruise is what made a
        # dodge along the direction of travel do nothing -- see
        # dodge_velocity_command for the arithmetic and the measurement.
        cmd_enu = dodge_velocity_command(
            plan.direction, v_drone,
            dodge_speed_mps=float(self._p["dodge_speed_mps"]),
            max_speed_mps=float(self._p["dodge_max_speed_mps"]))
        self._dodge_cmd_body = R.T @ cmd_enu
        # Feedforward along the escape direction only -- never along the cruise
        # term, which the vehicle is already flying and does not need to be
        # pushed into. Zero ff reproduces the velocity-only command exactly.
        accel_ff = float(self._p["evade_accel_ff_mps2"])
        self._dodge_accel_body = dir_body * accel_ff

        now = self.get_clock().now()
        latency = now.nanoseconds * 1e-9 - self._stamp_to_sec(stamp)
        over = latency > float(self._p["latency_budget_s"])
        self.get_logger().warn(
            f"DODGE: miss={plan.miss_m:.2f} m tca={plan.tca_s:.2f} s "
            f"dir_body=({dir_body[0]:+.2f},{dir_body[1]:+.2f},{dir_body[2]:+.2f}) "
            f"latency={latency * 1000:.0f} ms "
            f"track={track.n_updates}u/{track.track_age_s:.3f}s "
            f"of {self._tracker.n_tracks} live"
            + ("  ** OVER BUDGET **" if over else "")
        )
        event = String()
        event.data = json.dumps({
            "t_trigger_s": now.nanoseconds * 1e-9,
            "latency_s": latency,
            "tca_s": plan.tca_s,
            "miss_m": plan.miss_m,
            # Maturity of the hypothesis that FIRED — not of the tracker, which
            # now holds several. tca is bounded by how long this filter took to
            # believe the ball was inbound, so these say whether a late dodge
            # was a late detection or a late-starting track.
            "track_updates": track.n_updates,
            "track_age_s": track.track_age_s,
            # Live hypotheses at commit, and the lifetime total started. The
            # spawn count is a direct read on the detector's false-positive
            # rate; before the multi-hypothesis tracker those false positives
            # were reseeding the ball's own filter mid-flight.
            "n_hypotheses": self._tracker.n_tracks,
            "hypotheses_spawned": self._tracker.n_spawned,
            "dodge_body": [float(v) for v in self._dodge_cmd_body],
            "dodge_enu": [float(v) for v in plan.direction],
            # The cruise the escape is added to, and the absolute velocity that
            # results. Escape is deviation from the extrapolated cruise, so a
            # dodge can only be judged against the speed it started from.
            "cruise_speed_mps": float(np.linalg.norm(v_drone)),
            "cmd_enu": [float(v) for v in cmd_enu],
            "over_budget": over,
            # So a battery row can be attributed to the command FORM it flew
            # under. 0.0 is a velocity-only setpoint, i.e. every result
            # recorded before this existed.
            "accel_ff_mps2": accel_ff,
            "accel_body": [float(v) for v in self._dodge_accel_body],
            # Which confirmation threshold this dodge actually fired on --
            # the patrol value, or the relaxed one an ALERT cue bought.
            "min_track_updates_used": int(min_updates_used),
        })
        self._event_pub.publish(event)
        self._alarm_pub.publish(Bool(data=True))
        self._set_patrol(False)
        self._state = EvadeState.EVADING
        self._phase_end = now + Duration(
            seconds=float(self._p["dodge_duration_s"]))

        # Publish the first evade command immediately so the command path
        # doesn't wait up to one 20 Hz tick for _evade_tick to fire it.
        self._publish_evade(self._dodge_cmd_body, self._dodge_accel_body)

    def _evade_tick(self) -> None:
        if self._state is EvadeState.TRACKING:
            return
        now = self.get_clock().now()

        if self._state is EvadeState.EVADING:
            if now < self._phase_end:
                self._publish_evade(self._dodge_cmd_body,
                                    self._dodge_accel_body)
            else:
                self._alarm_pub.publish(Bool(data=False))
                # Zero BOTH: begin settle. The acceleration has to be cleared
                # explicitly or the bridge would keep attaching the dodge's
                # feedforward to these zero-velocity setpoints.
                self._publish_evade(np.zeros(3), np.zeros(3))
                self._state = EvadeState.RECOVERING
                self._phase_end = now + Duration(
                    seconds=float(self._p["recover_hold_s"]))
        elif self._state is EvadeState.RECOVERING:
            self._publish_evade(np.zeros(3), np.zeros(3))   # hold zero
            if now >= self._phase_end:
                # Don't resume patrol yet: the bridge treats /cmd/evade as
                # fresh for cmd_timeout_s after our last zero publish and
                # would keep streaming zero-velocity setpoints that fight
                # patrol's position setpoints. Go silent on /cmd/evade for
                # patrol_handoff_s and let the bridge's own stale-evade
                # handback (a single zero) cover the gap instead.
                self._state = EvadeState.HANDOFF
                self._phase_end = now + Duration(
                    seconds=float(self._p["patrol_handoff_s"]))
        elif self._state is EvadeState.HANDOFF:
            # Publish nothing here: this is the silent gap.
            if now >= self._phase_end:
                if bool(self._p["auto_resume_patrol"]):
                    self._set_patrol(True)
                self._tracker.reset()
                self._state = EvadeState.TRACKING
                self.get_logger().info("dodge complete -> TRACKING (patrol resumed)")

    def _set_patrol(self, run: bool) -> None:
        if not self._patrol_cli.service_is_ready():
            self.get_logger().warn(
                f"patrol service unavailable — continuing without "
                f"{'resuming' if run else 'pausing'} patrol",
                throttle_duration_sec=5.0)
            return
        req = SetBool.Request()
        req.data = run
        self._patrol_cli.call_async(req)

    # ── Intercept output ─────────────────────────────────────────────────

    def _publish_intercept(self, plan, pos_proj, vel_proj, T_odom_bl,
                           stamp) -> None:
        tca = plan.tca_s
        intercept_odom = (pos_proj + tca * vel_proj
                          + 0.5 * tca * tca * GRAVITY_ENU)
        T_bl_odom = np.linalg.inv(T_odom_bl)
        xyz = apply_transform(T_bl_odom, intercept_odom[None, :])[0]

        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        out.point.x = float(xyz[0])
        out.point.y = float(xyz[1])
        out.point.z = float(xyz[2])
        self._intercept_pub.publish(out)

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = "base_link"
        m.ns = "intercept"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(xyz[0])
        m.pose.position.y = float(xyz[1])
        m.pose.position.z = float(xyz[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.20
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.0
        m.color.a = 0.9
        m.lifetime.nanosec = int(0.5e9)
        self._marker_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvasionNode()
    install_clock_guard(node)
    clock_failed = False
    try:
        rclpy.spin(node)
    except ClockGuardError:
        # Already logged fatal by the guard; exit non-zero so a launch
        # file or shell script cannot mistake this for a clean start.
        clock_failed = True
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if clock_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
