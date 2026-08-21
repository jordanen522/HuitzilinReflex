#!/usr/bin/env python3
"""Optics probe: how many depth returns does the ball produce, and at what cost.

The probe world has no ground plane, so every finite point in the cloud is a
ball return. That makes the count exact without any spatial gate, which is the
whole reason the world is built that way -- a spatial gate would beg the
question the probe exists to answer.

Counting is vectorised on purpose. A per-point Python loop over a 640x480 cloud
takes ~0.4 s, which throttles the subscriber below the sensor rate and makes the
measured delivered_hz a property of this script rather than of the camera. That
mistake was made once here and the loop is what replaced it.

Reports, over a fixed number of frames:
  points/frame   -- min/median/max finite returns on the ball
  delivered_hz   -- clouds arriving, from message stamps (sim time)
  rtf            -- sim seconds per wall second, the rendering cost

Exits non-zero if no cloud ever arrives, so a broken bridge cannot read as a
zero-return optical result.
"""
import argparse
import statistics
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Probe(Node):
    def __init__(self, topic: str, frames: int) -> None:
        super().__init__("optics_probe")
        self._target_frames = frames
        self._counts: list[int] = []
        self._stamps: list[float] = []
        self._wall0: float | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._wall0 is None:
            self._wall0 = time.monotonic()
        arr = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=False
        )
        # skip_nans=False on purpose: this camera reports "no return" as +/-inf
        # as well as NaN, and isfinite is the only test that catches both.
        finite = (
            np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
        )
        self._counts.append(int(np.count_nonzero(finite)))
        self._stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    @property
    def done(self) -> bool:
        return len(self._counts) >= self._target_frames

    def report(self) -> int:
        if not self._counts:
            print("PROBE FAIL: no cloud received on the subscribed topic")
            return 2
        wall = time.monotonic() - (self._wall0 or time.monotonic())
        span = (self._stamps[-1] - self._stamps[0]) if len(self._stamps) > 1 else 0.0
        hz = (len(self._stamps) - 1) / span if span > 0 else float("nan")
        rtf = span / wall if wall > 0 else float("nan")
        c = sorted(self._counts)
        print(f"frames         {len(c)}")
        print(f"points/frame   min={c[0]} median={int(statistics.median(c))} max={c[-1]}")
        print(f"delivered_hz   {hz:.2f}")
        print(f"rtf            {rtf:.3f}")
        print(f"raw_counts     {self._counts}")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/probe/points")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.topic, args.frames)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rc = node.report()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
