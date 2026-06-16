#!/usr/bin/env python3
"""Patrol path-follower: walk a closed loop of NED waypoints, advance on arrival."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

from huitzilin_sim.mav_bridge import MavBridge, MASK_POS_ONLY  # reuse frame helpers


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol")
        # waypoints as flat [n0,e0,d0, n1,e1,d1, ...] (NED metres, d negative = up)
        self.declare_parameter("waypoints_ned",
                               [5.0, 0.0, -2.0,  5.0, 5.0, -2.0,
                                0.0, 5.0, -2.0,  0.0, 0.0, -2.0])
        self.declare_parameter("accept_radius_m", 0.6)
        self.declare_parameter("cruise_speed_ms", 1.5)
        self.declare_parameter("mode", "position")     # "position" | "velocity"
        self.declare_parameter("loop", True)
        self.declare_parameter("autostart", True)      # True = patrol on launch (demo-friendly)

        flat = list(self.get_parameter("waypoints_ned").value)
        self.wps = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.accept = float(self.get_parameter("accept_radius_m").value)
        self.cruise = float(self.get_parameter("cruise_speed_ms").value)
        self.mode = self.get_parameter("mode").value
        self.loop = bool(self.get_parameter("loop").value)
        self.idx = 0
        self.cur_enu = None
        self.running = bool(self.get_parameter("autostart").value)

        # position mode holds its own MAVLink connection for position setpoints;
        # velocity mode publishes through the bridge's /huitzilin/cmd_vel topic.
        self.declare_parameter("connection", "udp:127.0.0.1:14550")
        if self.mode == "position":
            self.mav = MavBridge(self.get_parameter("connection").value)
            self.mav.connect()
        else:
            self.cmd_pub = self.create_publisher(Twist, "/huitzilin/cmd_vel", 10)

        self.create_subscription(Odometry, "/huitzilin/odom", self._on_odom, 10)
        self.create_service(SetBool, "/huitzilin/start_patrol", self._srv_start)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/huitzilin/mission_marker", 1)
        self.create_timer(0.1, self._tick)          # 10 Hz control
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f"patrol up: {len(self.wps)} WPs, mode={self.mode}, running={self.running}")

    def _srv_start(self, req, resp):
        self.running = req.data
        resp.success = True
        resp.message = "patrol started" if req.data else "patrol stopped"
        return resp

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.cur_enu = (p.x, p.y, p.z)

    def _tick(self):
        if not self.running or self.cur_enu is None:
            return
        # convert current ENU pose back to NED to compare with NED waypoints
        n, e, d = MavBridge.enu_to_ned(*self.cur_enu)
        tn, te, td = self.wps[self.idx]
        dist = math.sqrt((tn - n) ** 2 + (te - e) ** 2 + (td - d) ** 2)

        if dist < self.accept:
            self.get_logger().info(f"reached WP {self.idx} {self.wps[self.idx]}")
            self.idx += 1
            if self.idx >= len(self.wps):
                if self.loop:
                    self.idx = 0
                else:
                    self.running = False
                    self.get_logger().info("patrol complete")
                    return

        tn, te, td = self.wps[self.idx]
        if self.mode == "position":
            # let ArduPilot fly to the absolute NED setpoint
            self.mav.send_position_ned(tn, te, td)
        else:
            # velocity pursuit: unit vector toward WP, scaled to cruise speed.
            # Uses small-yaw world-aligned approximation (fine for a square loop).
            # For true body-frame pursuit rotate by current yaw before publishing.
            dn, de, dd = tn - n, te - e, td - d
            norm = max(1e-3, math.sqrt(dn * dn + de * de + dd * dd))
            scale = min(self.cruise, norm) / norm
            t = Twist()
            t.linear.x = dn * scale          # NED north ~ body fwd (small-yaw approx)
            t.linear.y = -de * scale         # NED east -> FLU left is -east
            t.linear.z = -dd * scale         # NED down -> FLU up is -down
            self.cmd_pub.publish(t)

    def _publish_markers(self):
        arr = MarkerArray()
        for i, (n, e, d) in enumerate(self.wps):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "patrol"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            x, y, z = MavBridge.ned_to_enu(n, e, d)
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.scale.x = m.scale.y = m.scale.z = 0.4
            m.color.a = 1.0
            m.color.g = 1.0
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
