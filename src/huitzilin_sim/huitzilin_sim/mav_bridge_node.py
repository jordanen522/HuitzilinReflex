#!/usr/bin/env python3
"""ROS 2 Jazzy wrapper around MavBridge: cmd_vel/evade in, odom/state out, services."""
import sys
import json
import threading
from dataclasses import replace

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Accel, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from huitzilin_sim.mav_bridge import MavBridge
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard
from huitzilin_sim.cmd_router import Action, RouterState, route


class MavBridgeNode(Node):
    """ROS 2 face of the MAVLink link: the only place frames are converted.

    ENU/FLU on the ROS side, NED on the MAVLink side, via
    MavBridge.ned_to_enu / enu_to_ned. No other node converts -- mirrored
    RViz markers mean a bug here, not in the marker code. See
    docs/frames.md.
    """

    def __init__(self):
        super().__init__("mav_bridge")

        # parameters (override via bridge.yaml)
        self.declare_parameter("connection", "udp:127.0.0.1:14550")
        self.declare_parameter("cmd_rate_hz", 10.0)
        self.declare_parameter("evade_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 0.7)   # if patrol goes quiet -> hold
        self.declare_parameter("takeoff_alt_m", 2.0)
        self.declare_parameter("stream_rate_hz", 10.0)
        # No autopilot message for this long = link down: odom stops, so the
        # supervisor's odom watch sees it. Judged on the node clock (sim time
        # in SITL, where the autopilot also runs at sim rate).
        self.declare_parameter("fc_timeout_s", 1.0)

        conn = self.get_parameter("connection").value
        self.cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self.evade_rate = float(self.get_parameter("evade_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.takeoff_alt = float(self.get_parameter("takeoff_alt_m").value)
        self.fc_timeout = float(self.get_parameter("fc_timeout_s").value)
        self._fc_msgs_seen = 0
        self._fc_last_s = None

        self.bridge = MavBridge(conn, log=self.get_logger().info)
        self.bridge.connect()
        self.bridge.request_streams(int(self.get_parameter("stream_rate_hz").value))

        # Latched commands, arbitrated by cmd_router.route(). /cmd/evade
        # preempts /huitzilin/cmd_vel while fresh (Week 4); they are kept
        # separate so a finished dodge hands control back cleanly instead of
        # leaving a zero-velocity stream fighting patrol's position setpoints.
        self._lock = threading.Lock()
        self._router = RouterState()

        # ROS interfaces. Contracts: docs/architecture.md.
        self.create_subscription(Twist, "/huitzilin/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Twist, "/cmd/evade", self._on_evade, 10)
        # A parallel topic rather than a change to /cmd/evade: Twist has no
        # acceleration field, and widening the evade contract would drag the
        # patrol-handoff behaviour along with it. An
        # absent publisher simply means velocity-only setpoints, which is the
        # pre-existing behaviour.
        self.create_subscription(Accel, "/cmd/evade_accel", self._on_evade_accel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/huitzilin/odom", 10)
        self.state_pub = self.create_publisher(String, "/huitzilin/state", 10)

        # The services block (arm/takeoff/set_mode wait for the autopilot to
        # confirm), so they get their own callback group and main() runs a
        # two-thread executor. Telemetry, setpoints and the heartbeat keep
        # flowing while a takeoff climbs; before this, a blocked service froze
        # odom and the supervisor read every takeoff as a link loss.
        services = MutuallyExclusiveCallbackGroup()
        self.create_service(SetBool, "/huitzilin/arm", self._srv_arm,
                            callback_group=services)
        self.create_service(Trigger, "/huitzilin/takeoff", self._srv_takeoff,
                            callback_group=services)
        # NOTE: /huitzilin/start_patrol is owned by patrol_node (see patrol_node.py)
        self.declare_parameter("mode", "GUIDED")
        self.create_service(Trigger, "/huitzilin/set_mode", self._srv_set_mode,
                            callback_group=services)
        self.create_timer(1.0, self.bridge.send_heartbeat)

        self.create_timer(1.0 / self.cmd_rate, self._tick_setpoint)   # watchdog/stream
        # Dodges are retransmitted far faster than patrol. The watchdog rate is
        # sized to keep ArduPilot's ~3 s setpoint timeout happy, which is the
        # wrong scale entirely for a manoeuvre whose whole window is ~0.2 s.
        if self.evade_rate > 0.0:
            self.create_timer(1.0 / self.evade_rate, self._tick_evade)
        # Telemetry tick follows stream_rate_hz: odom must be >= the 15 Hz
        # depth-cloud rate or the detector's latest-TF fallback goes stale.
        self.create_timer(1.0 / float(self.get_parameter("stream_rate_hz").value),
                          self._tick_telemetry)
        self.get_logger().info("mav_bridge up: cmd_vel in, odom/state out")

    @staticmethod
    def _flu_to_ned(msg: Twist):
        """ROS body FLU (x fwd, y left, z up) -> AP body NED (x fwd, y right, z down)."""
        return (msg.linear.x,
                -msg.linear.y,             # FLU y(left) -> NED y(right)
                -msg.linear.z,             # up -> down
                -msg.angular.z)            # ENU yaw(ccw+) -> NED yaw(cw+)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._router = replace(self._router,
                                   last_cmd=self._flu_to_ned(msg),
                                   last_cmd_t=self._now_s())

    def _on_evade(self, msg: Twist):
        """Latch the dodge and send it immediately.

        Waiting for the next watchdog tick cost up to a full cmd_rate period
        (100 ms at 10 Hz) on a command whose entire useful window is the
        0.18-0.29 s of tca measured at commit. rclpy's default executor is
        single-threaded, so this cannot race a timer callback.
        """
        with self._lock:
            self._router = replace(self._router,
                                   last_evade=self._flu_to_ned(msg),
                                   last_evade_t=self._now_s())
        self._route_and_send(evade_only=False)

    def _on_evade_accel(self, msg: Accel):
        """Latch only. The velocity message is what decides that a dodge is
        live, so an accel arriving on its own must not transmit anything."""
        with self._lock:
            self._router = replace(
                self._router,
                # Same FLU->NED mapping as the velocity path.
                last_accel=(msg.linear.x, -msg.linear.y, -msg.linear.z),
                last_accel_t=self._now_s())

    def _tick_evade(self):
        """Fast retransmit while a dodge is live; otherwise a no-op.

        Deliberately refuses to act on anything but EVADE. If it handled the
        handback edge it would also have to own cmd_vel, which would stream
        patrol at the evade rate -- and the handback would land here rather
        than on the watchdog tick without any benefit.
        """
        self._route_and_send(evade_only=True)

    def _tick_setpoint(self):
        """Stream the freshest command at the watchdog rate.

        Priority: fresh /cmd/evade > /huitzilin/cmd_vel. When an evade goes
        stale, send ONE zero setpoint (so ArduPilot doesn't coast on the last
        dodge velocity), then fall back — to cmd_vel zero-hold if patrol ever
        commanded velocity, or to full silence (patrol position mode owns the
        vehicle again). The rules live in cmd_router.route().
        """
        self._route_and_send(evade_only=False)

    def _route_and_send(self, evade_only: bool):
        """Decide under the lock, transmit outside it."""
        with self._lock:
            r = route(self._router, self._now_s(), self.cmd_timeout)
            if evade_only and r.action is not Action.EVADE:
                return
            self._router = replace(self._router, evade_active=r.evade_active)
        if not r.sends:
            return
        if r.accel is None:
            self.bridge.send_velocity_body(*r.velocity)
        else:
            vx, vy, vz, yaw_rate = r.velocity
            self.bridge.send_velocity_accel_body(vx, vy, vz, *r.accel,
                                                 yaw_rate=yaw_rate)

    def _tick_telemetry(self):
        """Publish /huitzilin/state every tick, odom only while the link is live.

        State goes out with or without a position fix: armed, mode and battery
        come from HEARTBEAT and SYS_STATUS, which a flight controller on the
        bench sends long before its EKF has an origin. Gating them on
        LOCAL_POSITION_NED left a GPS-less bench with no telemetry at all.

        Odom stops when the autopilot goes quiet for fc_timeout_s. The cached
        pose would otherwise be republished with fresh stamps forever, and the
        supervisor's odom-staleness watch -- its only link-loss detector --
        could never fire.
        """
        s = self.bridge.get_state()
        now = self._now_s()
        if s.get("fc_msgs", 0) != self._fc_msgs_seen:
            self._fc_msgs_seen = s.get("fc_msgs", 0)
            self._fc_last_s = now
        fc_age = None if self._fc_last_s is None else now - self._fc_last_s
        link_up = fc_age is not None and fc_age <= self.fc_timeout

        if link_up and {"n", "e", "d"} <= s.keys():
            self.odom_pub.publish(self._odom_from(s))

        # armed/mode/batt_v/fc_failsafe cover four of the six detection
        # columns in SAFETY_CASE.md section 1. Absent keys stay absent rather
        # than defaulting, so a consumer can tell "not reported yet" from
        # "reported as false".
        st = String()
        st.data = json.dumps({
            "n": s.get("n"), "e": s.get("e"),
            "alt": None if "d" not in s else -s["d"],
            "yaw": s.get("yaw"),
            "armed": s.get("armed"),
            "mode": s.get("mode"),
            "batt_v": s.get("batt_v"),
            "batt_pct": s.get("batt_pct"),
            "fc_failsafe": s.get("fc_failsafe"),
            "fc_age_s": None if fc_age is None else round(fc_age, 3),
        })
        self.state_pub.publish(st)

    def _odom_from(self, s):
        x, y, z = MavBridge.ned_to_enu(s["n"], s["e"], s["d"])
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.position.z = z
        # Orientation is REQUIRED by the detector's egomotion compensation:
        # without it, bags carry the all-zero default quaternion and background
        # differencing falls back to the flood-prone camera-frame mode (the
        # 60%-recall root cause).
        if {"roll", "pitch", "yaw"} <= s.keys():
            qx, qy, qz, qw = MavBridge.ned_rpy_to_enu_quat(
                s["roll"], s["pitch"], s["yaw"])
            od.pose.pose.orientation.x = qx
            od.pose.pose.orientation.y = qy
            od.pose.pose.orientation.z = qz
            od.pose.pose.orientation.w = qw
        if {"vn", "ve", "vd"} <= s.keys():
            vx, vy, vz = MavBridge.ned_to_enu(s["vn"], s["ve"], s["vd"])
            od.twist.twist.linear.x = vx
            od.twist.twist.linear.y = vy
            od.twist.twist.linear.z = vz
        return od

    def _srv_arm(self, req, resp):
        try:
            self.bridge.arm(req.data)
            resp.success, resp.message = True, ("armed" if req.data else "disarmed")
        except Exception as e:
            resp.success, resp.message = False, str(e)
        return resp

    def _srv_takeoff(self, req, resp):
        try:
            self.bridge.set_mode("GUIDED")
            self.bridge.takeoff(self.takeoff_alt)
            resp.success, resp.message = True, f"takeoff {self.takeoff_alt} m"
        except Exception as e:
            resp.success, resp.message = False, str(e)
        return resp

    def _srv_set_mode(self, req, resp):
        mode = self.get_parameter("mode").value
        try:
            self.bridge.set_mode(mode)
            resp.success, resp.message = True, f"mode {mode}"
        except Exception as e:
            resp.success, resp.message = False, str(e)
        return resp


def main():
    rclpy.init()
    node = MavBridgeNode()
    install_clock_guard(node)
    clock_failed = False
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except ClockGuardError:
        # Already logged fatal by the guard; exit non-zero so a launch
        # file or shell script cannot mistake this for a clean start.
        clock_failed = True
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM/SIGINT already shut the context down; a second shutdown
        # raises RCLError (seen as a traceback after every launch teardown).
        if rclpy.ok():
            rclpy.shutdown()

    if clock_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
