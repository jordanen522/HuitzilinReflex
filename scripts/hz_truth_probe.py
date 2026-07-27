#!/usr/bin/env python3
"""
hz_truth_probe.py — record ground truth alongside a detector frame dump.

Companion to the detector's `debug_dump_dir`. The dump says what the pipeline
saw and where it threw it away; this says where the ball actually was. Joined
on the SIM clock, the two attribute a detection miss to a specific gate.

Records, one CSV row each, every sample of:
  pose    — /gz/dynamic_poses, every model in the world (ball + drone), in the
            Gazebo WORLD frame
  odom    — /huitzilin/odom, the drone in the ODOM frame
  cent    — /threat/centroid, every published detection, in base_link

Stamps come from the message header (gz sim time), never from arrival time.
Deriving a time from arrival order on this stream is a measurement trap that
has already produced one wrong answer in this project — see docs/JOURNAL.md,
"never derive a velocity off /gz/dynamic_poses arrival times".

Run it alongside the live stack, then throw:

    source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
    python3 ~/huitzilin_ws/scripts/hz_truth_probe.py --out /tmp/truth.csv

Ctrl-C to stop; rows are flushed continuously, so a kill loses nothing.
"""

from __future__ import annotations

import argparse
import csv
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_msgs.msg import TFMessage

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def _stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class TruthProbe(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("hz_truth_probe")
        # The stack runs on sim time; a probe on wall time would stamp its own
        # rows with a clock the dump cannot be joined to.
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self._fh = open(out_path, "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(["kind", "stamp_s", "name", "x", "y", "z"])
        self._n = 0

        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, SENSOR_QOS)
        self.create_subscription(Odometry, "/huitzilin/odom",
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(PointStamped, "/threat/centroid",
                                 self._cent_cb, RELIABLE_QOS)
        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(f"recording truth -> {out_path}")

    def _row(self, kind: str, stamp: float, name: str, x, y, z) -> None:
        self._csv.writerow([kind, f"{stamp:.6f}", name,
                            f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
        self._n += 1
        # Flush every row: the interesting window is the ~0.5 s before a kill,
        # and a buffered writer is exactly where that window would be lost.
        self._fh.flush()

    def _pose_cb(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            t = tr.transform.translation
            self._row("pose", _stamp_s(tr.header), tr.child_frame_id,
                      t.x, t.y, t.z)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._row("odom", _stamp_s(msg.header), "drone", p.x, p.y, p.z)

    def _cent_cb(self, msg: PointStamped) -> None:
        self._row("cent", _stamp_s(msg.header), "centroid",
                  msg.point.x, msg.point.y, msg.point.z)

    def _heartbeat(self) -> None:
        self.get_logger().info(f"{self._n} rows", throttle_duration_sec=10.0)

    def close(self) -> None:
        self._fh.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/truth.csv")
    args = ap.parse_args()

    rclpy.init()
    node = TruthProbe(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
