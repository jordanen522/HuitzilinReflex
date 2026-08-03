#!/usr/bin/env python3
"""Log odom + cmd_vel to a timestamped CSV for quick plotting (proof artifact)."""
import csv
import time

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard


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
        # Flush on a timer, not per row. Odom arrives at ~30 Hz and a flush is a
        # write(2) each time, inside the subscription callback on the same
        # single-threaded executor everything else here runs on. Once a second
        # bounds the loss on a hard kill to ~30 rows, which is what tailing the
        # file live actually needs. destroy_node still closes (and so flushes).
        self.create_timer(1.0, self._flush)
        self.get_logger().info(f"logging telemetry -> {path}")

    def _flush(self):
        self.f.flush()

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.linear.z)

    def _on_odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        # ROS time, not time.time(). Under use_sim_time the two diverge by the
        # real-time factor -- ~0.33 on the Dell under depth rendering -- so a
        # wall-clock stamp stretched plot_telemetry.py's x-axis by 3x and made
        # every duration read off it wrong. CLAUDE.md: judge timing in sim time.
        t = self.get_clock().now().nanoseconds * 1e-9
        self.w.writerow([f"{t:.3f}",
                         p.x, p.y, p.z,
                         v.x, v.y, v.z,
                         *self.cmd])

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TelemetryLogger()
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
