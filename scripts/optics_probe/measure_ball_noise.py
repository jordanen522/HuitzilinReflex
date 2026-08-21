#!/usr/bin/env python3
"""measure_ball_noise.py — does the noise stage actually do what it claims, live?

`test_depth_noise.py` pins the model against synthetic arrays. This checks the
whole chain instead:

    gz depth camera -> ros_gz_bridge -> depth_noise_node -> HERE

It subscribes to a cloud topic, takes the centroid of every finite return in
each frame, and reports the spread of that centroid's RANGE over many frames.
The probe world has no ground plane, so every finite point is a ball return and
no spatial gate is needed -- the same reason `count_ball_points.py` can count
without one.

WHAT THE NUMBERS SHOULD BE. With the ball parked at a true 26 m and
`sigma_ref_m: 0.30`, the centroid range should scatter with std ~0.30 m about a
mean of ~26 m: the ball spans ~5 px, one correlation cell, so it gets a single
disparity solution and the error lands on the centroid rather than smearing it.
Run again with `sigma_ref_m:=0.0` and the std must collapse to ~0 -- that arm is
the control, and `apply_image_depth_noise` consumes no randomness there, so it
is a genuinely noiseless comparison rather than a differently-seeded one.

A std near zero on the NOISED arm means the node is passing the cloud through
untouched, which would silently restore the too-good sensor this whole stage
exists to remove.

Exit code 2 if no cloud ever arrives, so a broken bridge cannot be read as a
clean zero-noise result -- the failure mode `count_ball_points.py` also guards.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class BallNoiseProbe(Node):
    def __init__(self, topic: str, frames: int) -> None:
        super().__init__("measure_ball_noise")
        self._target = frames
        self._ranges: list[float] = []
        self._counts: list[int] = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if len(self._ranges) >= self._target:
            return
        arr = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=False)
        # isfinite, not isnan: this camera reports "no return" as +/-inf as
        # well as NaN, and only isfinite catches both.
        finite = arr[np.isfinite(arr).all(axis=1)]
        if finite.shape[0] == 0:
            return
        centroid = finite.mean(axis=0)
        self._ranges.append(float(np.linalg.norm(centroid)))
        self._counts.append(int(finite.shape[0]))

    @property
    def done(self) -> bool:
        return len(self._ranges) >= self._target

    def report(self) -> int:
        if not self._ranges:
            print("PROBE FAIL: no cloud with any finite return arrived")
            return 2
        r = np.array(self._ranges)
        c = np.array(self._counts)
        print(f"frames              {len(r)}")
        print(f"points/frame        min {c.min()}  median {int(np.median(c))}"
              f"  max {c.max()}")
        print(f"centroid range mean {r.mean():.4f} m")
        print(f"centroid range std  {r.std(ddof=1):.4f} m")
        print(f"centroid range p2p  {r.max() - r.min():.4f} m")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/oak/points")
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()

    rclpy.init()
    node = BallNoiseProbe(args.topic, args.frames)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    rc = node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
