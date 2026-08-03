#!/usr/bin/env python3
"""ROS 2 Jazzy wrapper around MavBridge: cmd_vel/evade in, odom/state out, services."""
import sys
import json
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from huitzilin_sim.mav_bridge import MavBridge
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard


class MavBridgeNode(Node):
    def __init__(self):
        super().__init__("mav_bridge")

        # --- parameters (override via bridge.yaml) ---
        self.declare_parameter("connection", "udp:127.0.0.1:14550")
        self.declare_parameter("cmd_rate_hz", 10.0)
        self.declare_parameter("cmd_timeout_s", 0.7)   # if patrol goes quiet -> hold
        self.declare_parameter("takeoff_alt_m", 2.0)
        self.declare_parameter("stream_rate_hz", 10.0)

        conn = self.get_parameter("connection").value
        self.cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.takeoff_alt = float(self.get_parameter("takeoff_alt_m").value)

        # --- MAVLink bridge ---
        self.bridge = MavBridge(conn)
        self.bridge.connect()
        self.bridge.request_streams(int(self.get_parameter("stream_rate_hz").value))

        # last commanded body velocity + when it arrived (watchdog state)
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)
        self._last_cmd_t = self.get_clock().now()
        self._lock = threading.Lock()
        self._cmd_ever_received = False   # don't send setpoints until patrol starts

        # evade override state: /cmd/evade preempts /huitzilin/cmd_vel while
        # fresh (Week 4). Separate from cmd_vel so a finished dodge hands
        # control back cleanly instead of leaving a zero-velocity stream
        # fighting patrol's position setpoints.
        self._last_evade = (0.0, 0.0, 0.0, 0.0)
        self._last_evade_t = self.get_clock().now()
        self._evade_ever_received = False
        self._evade_active = False

        # --- ROS interfaces (contracts: see playbook §3) ---
        self.create_subscription(Twist, "/huitzilin/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Twist, "/cmd/evade", self._on_evade, 10)
        self.odom_pub = self.create_publisher(Odometry, "/huitzilin/odom", 10)
        self.state_pub = self.create_publisher(String, "/huitzilin/state", 10)

        self.create_service(SetBool, "/huitzilin/arm", self._srv_arm)
        self.create_service(Trigger, "/huitzilin/takeoff", self._srv_takeoff)
        # NOTE: /huitzilin/start_patrol is owned by patrol_node (see patrol_node.py)
        self.declare_parameter("mode", "GUIDED")
        self.create_service(Trigger, "/huitzilin/set_mode", self._srv_set_mode)

        # --- timers ---
        self.create_timer(1.0 / self.cmd_rate, self._tick_setpoint)   # watchdog/stream
        # Telemetry tick follows stream_rate_hz: odom must be >= the 15 Hz
        # depth-cloud rate or the detector's latest-TF fallback goes stale.
        self.create_timer(1.0 / float(self.get_parameter("stream_rate_hz").value),
                          self._tick_telemetry)
        self.get_logger().info("mav_bridge up: cmd_vel in, odom/state out")

    # -- cmd_vel: ROS body FLU (x fwd, y left, z up) -> AP body NED (x fwd, y right, z down)
    def _on_cmd_vel(self, msg: Twist):
        vx = msg.linear.x
        vy = -msg.linear.y                 # FLU y(left) -> NED y(right)
        vz = -msg.linear.z                 # up -> down
        yaw_rate = -msg.angular.z          # ENU yaw(ccw+) -> NED yaw(cw+)
        with self._lock:
            self._last_cmd = (vx, vy, vz, yaw_rate)
            self._last_cmd_t = self.get_clock().now()
            self._cmd_ever_received = True

    # -- /cmd/evade: same FLU->NED mapping as cmd_vel, but takes priority
    def _on_evade(self, msg: Twist):
        vx = msg.linear.x
        vy = -msg.linear.y                 # FLU y(left) -> NED y(right)
        vz = -msg.linear.z                 # up -> down
        yaw_rate = -msg.angular.z          # ENU yaw(ccw+) -> NED yaw(cw+)
        with self._lock:
            self._last_evade = (vx, vy, vz, yaw_rate)
            self._last_evade_t = self.get_clock().now()
            self._evade_ever_received = True

    def _tick_setpoint(self):
        """Stream the freshest command at a fixed rate.

        Priority: fresh /cmd/evade > /huitzilin/cmd_vel. When an evade goes
        stale, send ONE zero setpoint (so ArduPilot doesn't coast on the last
        dodge velocity), then fall back — to cmd_vel zero-hold if patrol ever
        commanded velocity, or to full silence (patrol position mode owns the
        vehicle again)."""
        now = self.get_clock().now()
        with self._lock:
            evade_fresh = False
            if self._evade_ever_received:
                e_age = (now - self._last_evade_t).nanoseconds * 1e-9
                evade_fresh = e_age <= self.cmd_timeout
            evx, evy, evz, eyr = self._last_evade
            cmd_age = (now - self._last_cmd_t).nanoseconds * 1e-9
            vx, vy, vz, yr = self._last_cmd
            handback = (not evade_fresh) and self._evade_active
            self._evade_active = evade_fresh

        if evade_fresh:
            self.bridge.send_velocity_body(evx, evy, evz, eyr)
            return
        if handback:
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)  # handback zero
            return

        if not self._cmd_ever_received:
            return
        if cmd_age > self.cmd_timeout:
            vx = vy = vz = yr = 0.0        # dropout -> calm hold, never a coast/lunge
        self.bridge.send_velocity_body(vx, vy, vz, yr)

    def _tick_telemetry(self):
        s = self.bridge.get_state()
        if {"n", "e", "d"} <= s.keys():
            x, y, z = MavBridge.ned_to_enu(s["n"], s["e"], s["d"])
            od = Odometry()
            od.header.stamp = self.get_clock().now().to_msg()
            od.header.frame_id = "odom"
            od.child_frame_id = "base_link"
            od.pose.pose.position.x = x
            od.pose.pose.position.y = y
            od.pose.pose.position.z = z
            # Orientation is REQUIRED by the detector's egomotion compensation
            # (W3-13): without it, bags carry the all-zero default quaternion
            # and background differencing falls back to the flood-prone
            # camera-frame mode (the 2026-07-06 60%-recall root cause).
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
            self.odom_pub.publish(od)
            st = String()
            # armed/mode/batt_v/fc_failsafe cover four of the six detection
            # columns in SAFETY_CASE.md section 1. They were already on the
            # wire -- request_streams asks for SYS_STATUS and HEARTBEAT is
            # unsolicited -- and were being drained and dropped. Absent keys
            # stay absent rather than defaulting, so a consumer can tell
            # "not reported yet" from "reported as false".
            st.data = json.dumps({
                "n": s.get("n"), "e": s.get("e"),
                "alt": -s.get("d", 0.0),
                "yaw": s.get("yaw"),
                "armed": s.get("armed"),
                "mode": s.get("mode"),
                "batt_v": s.get("batt_v"),
                "batt_pct": s.get("batt_pct"),
                "fc_failsafe": s.get("fc_failsafe"),
            })
            self.state_pub.publish(st)

    # -- services --
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
        # SIGTERM/SIGINT already shut the context down; a second shutdown
        # raises RCLError (seen as a traceback after every launch teardown).
        if rclpy.ok():
            rclpy.shutdown()

    if clock_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
