#!/usr/bin/env python3
"""Log odom + cmd_vel to a timestamped CSV for quick plotting (proof artifact)."""
import csv
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__("telemetry_logger")
        self.declare_parameter("csv_path", f"week2_telemetry_{int(time.time())}.csv")
        path = self.get_parameter("csv_path").value
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "z", "vx", "vy", "vz",
                         "cmd_vx", "cmd_vy", "cmd_vz"])
        self.cmd = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, "/huitzilin/odom", self._on_odom, 50)
        self.create_subscription(Twist, "/huitzilin/cmd_vel", self._on_cmd, 50)
        self.get_logger().info(f"logging telemetry -> {path}")

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.linear.z)

    def _on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        self.w.writerow([f"{time.time():.3f}",
                         p.x, p.y, p.z,
                         v.x, v.y, v.z,
                         *self.cmd])
        self.f.flush()

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
