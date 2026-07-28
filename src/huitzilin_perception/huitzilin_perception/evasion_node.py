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

import json
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
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
)
from huitzilin_perception.kalman import (
    GRAVITY_ENU,
    MultiHypothesisTracker,
    dodge_velocity_command,
    plan_dodge,
    should_dodge,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Documented sweep surface used by the dodge battery's sweep config (ros2
# param set /evasion ...). Any live-read key in self._p outside
# FIXED_AT_START also accepts writes -- this set is the subset the battery
# actually grids, not an exhaustive allow-list.
SWEEPABLE = {
    "dodge_speed_mps", "threat_radius_m", "trigger_horizon_s",
    "dodge_duration_s", "min_track_updates", "dodge_max_speed_mps",
}

# Declared params that are consumed exactly once at node start (tracker
# construction, topic wiring, timer creation). Rejecting writes keeps the
# ROS parameter surface honest: what `ros2 param get` shows is what runs.
FIXED_AT_START = {
    "centroid_topic", "odom_topic", "evade_topic", "alarm_topic",
    "intercept_topic", "event_topic", "marker_topic", "patrol_service",
    "process_accel_std", "meas_std_m", "init_vel_std", "track_timeout_s",
    "max_tracks", "max_assoc_sigma_m", "evade_cmd_rate_hz",
}


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

        self._p = {
            name: self.get_parameter(name).value
            for name in (
                "centroid_topic", "odom_topic", "evade_topic", "alarm_topic",
                "intercept_topic", "event_topic", "marker_topic",
                "patrol_service", "process_accel_std", "meas_std_m",
                "init_vel_std", "track_timeout_s", "max_tracks",
                "max_assoc_sigma_m", "min_track_updates",
                "threat_radius_m", "trigger_horizon_s", "prediction_horizon_s",
                "latency_budget_s", "dodge_speed_mps", "dodge_duration_s",
                "dodge_floor_m", "dodge_max_speed_mps",
                "recover_hold_s", "patrol_handoff_s", "evade_cmd_rate_hz",
                "auto_resume_patrol",
            )
        }
        self.add_on_set_parameters_callback(self._on_param_set)

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
        self._last_odom = None
        self._warned_no_odom = False
        self._warned_bad_quat = False

        # ── ROS interfaces ───────────────────────────────────────────────
        self.create_subscription(PointStamped, self._p["centroid_topic"],
                                 self._centroid_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, self._p["odom_topic"],
                                 self._odom_cb, RELIABLE_QOS)
        self._evade_pub = self.create_publisher(
            Twist, self._p["evade_topic"], RELIABLE_QOS)
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
        for prm in params:
            if prm.name in FIXED_AT_START:
                return SetParametersResult(
                    successful=False,
                    reason=f"{prm.name} is fixed at node start — "
                           "edit params/evasion.yaml and restart")
            if prm.name in self._p:  # SWEEPABLE + live-read tunables
                self._p[prm.name] = prm.value
                self.get_logger().info(f"param {prm.name} -> {prm.value}")
        return SetParametersResult(successful=True)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

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

        self._tracker.process(self._stamp_to_sec(msg.header.stamp), z_odom)

        min_updates = int(self._p["min_track_updates"])
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
            self._start_dodge(plan, track, q, msg.header.stamp, v_drone)

    # ── Dodge lifecycle ──────────────────────────────────────────────────

    def _start_dodge(self, plan, track, q, stamp, v_drone) -> None:
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
        })
        self._event_pub.publish(event)
        self._alarm_pub.publish(Bool(data=True))
        self._set_patrol(False)
        self._state = EvadeState.EVADING
        self._phase_end = now + Duration(
            seconds=float(self._p["dodge_duration_s"]))

        # Publish the first evade command immediately so the command path
        # doesn't wait up to one 20 Hz tick for _evade_tick to fire it.
        cmd = Twist()
        cmd.linear.x = float(self._dodge_cmd_body[0])
        cmd.linear.y = float(self._dodge_cmd_body[1])
        cmd.linear.z = float(self._dodge_cmd_body[2])
        self._evade_pub.publish(cmd)

    def _evade_tick(self) -> None:
        if self._state is EvadeState.TRACKING:
            return
        now = self.get_clock().now()

        if self._state is EvadeState.EVADING:
            if now < self._phase_end:
                cmd = Twist()
                cmd.linear.x = float(self._dodge_cmd_body[0])
                cmd.linear.y = float(self._dodge_cmd_body[1])
                cmd.linear.z = float(self._dodge_cmd_body[2])
                self._evade_pub.publish(cmd)
            else:
                self._alarm_pub.publish(Bool(data=False))
                self._evade_pub.publish(Twist())  # zero: begin settle
                self._state = EvadeState.RECOVERING
                self._phase_end = now + Duration(
                    seconds=float(self._p["recover_hold_s"]))
        elif self._state is EvadeState.RECOVERING:
            self._evade_pub.publish(Twist())      # hold zero while settling
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
