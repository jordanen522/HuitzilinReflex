#!/usr/bin/env python3
"""ROS 2 Jazzy wrapper around MavBridge: cmd_vel in, odom/state out, services."""
import json
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from huitzilin_sim.mav_bridge import MavBridge


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

        # --- ROS interfaces (contracts: see playbook §3) ---
        self.create_subscription(Twist, "/huitzilin/cmd_vel", self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "/huitzilin/odom", 10)
        self.state_pub = self.create_publisher(String, "/huitzilin/state", 10)

        self.create_service(SetBool, "/huitzilin/arm", self._srv_arm)
        self.create_service(Trigger, "/huitzilin/takeoff", self._srv_takeoff)
        # NOTE: /huitzilin/start_patrol is owned by patrol_node (see patrol_node.py)
        self.declare_parameter("mode", "GUIDED")
        self.create_service(Trigger, "/huitzilin/set_mode", self._srv_set_mode)

        # --- timers ---
        self.create_timer(1.0 / self.cmd_rate, self._tick_setpoint)   # watchdog/stream
        self.create_timer(1.0 / 10.0, self._tick_telemetry)
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

    def _tick_setpoint(self):
        """Stream the last command at a fixed rate; zero-hold if stale (fail-safe).
        Does nothing until the first cmd_vel arrives so takeoff/hover aren't disrupted."""
        if not self._cmd_ever_received:
            return
        now = self.get_clock().now()
        with self._lock:
            age = (now - self._last_cmd_t).nanoseconds * 1e-9
            vx, vy, vz, yr = self._last_cmd
        if age > self.cmd_timeout:
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
            if {"vn", "ve", "vd"} <= s.keys():
                vx, vy, vz = MavBridge.ned_to_enu(s["vn"], s["ve"], s["vd"])
                od.twist.twist.linear.x = vx
                od.twist.twist.linear.y = vy
                od.twist.twist.linear.z = vz
            self.odom_pub.publish(od)
            st = String()
            st.data = json.dumps({
                "n": s.get("n"), "e": s.get("e"),
                "alt": -s.get("d", 0.0),
                "yaw": s.get("yaw"),
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
