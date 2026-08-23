"""
mono_flash_detector_node.py — high-rate ball detection from the OAK-D mono pair.

HARDWARE LANE, WEEK 6. Nothing in sim publishes mono image streams, so this
node cannot be exercised end-to-end until the camera is on the bench. What CAN
be checked today is the arithmetic, and it is: mono_flash.py is pure and
test_mono_flash.py covers differencing, sub-pixel centroiding, epipolar
matching, the triangulation round-trip and the covariance growth laws. This
file is the ROS shell around that, written now so bring-up is a tuning
exercise rather than a design exercise.

WHY IT EXISTS. The depth pipeline's limit is neither compute nor the tracker:
it first reports the 80 mm ball at ~3.2-3.6 m, which at 20 m/s is 0.17 s of
warning, less than confirmation plus pipeline. This node attacks both halves
of that sentence at once -- range, because a sub-pixel centroid needs a few
pixels rather than a resolvable disk, and confirmation time, because three
frames at 90 fps cost 0.033 s instead of 0.21 s.

IT DOES NOT REPLACE THE DEPTH DETECTOR. The failure modes are different:
differencing finds motion and is blind to a stationary threat, while the depth
cloud sees shape and needs no motion. The intended Week 6 configuration runs
both, this one feeding long-range measurements (loose depth, tight bearing)
and the depth detector confirming near-field. That is what the tracker's
per-measurement covariance hook is for -- and until it is measured, running
both is a hypothesis, not a plan.

THREE OUTPUTS, deliberately separate:
  /threat/centroid      the existing contract, so the tracker needs no change
  /threat/centroid_cov  the same point WITH its 3x3 covariance (anisotropic:
                        depth error grows as z^2, bearing as z)
  /threat/cue           "something fast is inbound" -- feeds evasion_node's
                        ALERT state, which relaxes confirmation for a bounded
                        window. Published on a matched, size-gated, in-range
                        detection, i.e. exactly the evidence a centroid needs.

Frames: triangulation yields a point in camera_optical_frame; it is
transformed to base_link through the existing static TF chain, which is what
/threat/centroid consumers expect.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from huitzilin_perception.mono_flash import (
    DEFAULT_BASELINE_M,
    DEFAULT_FX_PX,
    extract_blobs,
    match_stereo,
    temporal_difference,
    triangulate,
    triangulation_covariance,
)

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class MonoFlashDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("mono_flash_detector")

        self.declare_parameter("left_topic", "/oak/left/image_raw")
        self.declare_parameter("right_topic", "/oak/right/image_raw")
        self.declare_parameter("centroid_topic", "/threat/centroid")
        self.declare_parameter("cov_topic", "/threat/centroid_cov")
        self.declare_parameter("cue_topic", "/threat/cue")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "base_link")

        # Pairing tolerance. The two sensors are hardware-synchronised on the
        # Myriad X, so this is a sanity bound rather than a real alignment
        # mechanism: at 90 fps a whole frame is 11 ms, and a pair further
        # apart than half a frame is a dropped frame, not a stereo pair.
        self.declare_parameter("pair_tolerance_s", 0.005)

        self.declare_parameter("diff_threshold", 25.0)
        self.declare_parameter("min_area_px", 3)
        self.declare_parameter("max_area_px", 400)
        self.declare_parameter("max_dy_px", 2.0)
        self.declare_parameter("min_disparity_px", 0.5)
        self.declare_parameter("max_disparity_px", 300.0)

        # NOMINAL until the device's own calibration is read at bring-up.
        # A few percent of focal error is a few percent of every range.
        self.declare_parameter("fx_px", DEFAULT_FX_PX)
        self.declare_parameter("cx_px", 320.0)
        self.declare_parameter("cy_px", 240.0)
        self.declare_parameter("baseline_m", DEFAULT_BASELINE_M)
        self.declare_parameter("centroid_std_px", 0.3)

        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("max_range_m", 15.0)
        self.declare_parameter("publish_cue", True)

        self._p = {n: self.get_parameter(n).value for n in (
            "left_topic", "right_topic", "centroid_topic", "cov_topic",
            "cue_topic", "optical_frame", "target_frame", "pair_tolerance_s",
            "diff_threshold", "min_area_px", "max_area_px", "max_dy_px",
            "min_disparity_px", "max_disparity_px", "fx_px", "cx_px", "cy_px",
            "baseline_m", "centroid_std_px", "min_range_m", "max_range_m",
            "publish_cue")}

        self._prev_left = None
        self._prev_right = None
        self._pending_right: list = []      # (stamp_s, frame)
        self._n_published = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._centroid_pub = self.create_publisher(
            PointStamped, self._p["centroid_topic"], RELIABLE_QOS)
        self._cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, self._p["cov_topic"], RELIABLE_QOS)
        self._cue_pub = self.create_publisher(
            Bool, self._p["cue_topic"], RELIABLE_QOS)

        self.create_subscription(Image, self._p["left_topic"],
                                 self._left_cb, SENSOR_QOS)
        self.create_subscription(Image, self._p["right_topic"],
                                 self._right_cb, SENSOR_QOS)

        self.get_logger().info(
            "mono_flash_detector up — NOMINAL calibration "
            f"(fx {self._p['fx_px']} px, baseline {self._p['baseline_m']} m). "
            "Replace with the device's own before trusting a range.")

    # frame plumbing

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def _as_gray(msg: Image) -> np.ndarray:
        """mono8 only. Refusing anything else is deliberate: silently reading
        the first channel of an rgb8 frame would work well enough to look
        correct while halving the effective contrast."""
        if msg.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8, got %r" % msg.encoding)
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width)

    def _right_cb(self, msg: Image) -> None:
        """Buffer right frames; the left frame drives the pipeline.

        Bounded to a handful: if the left stream dies this must not grow
        without limit, and a right frame older than a few frames can no longer
        pair with anything.
        """
        try:
            frame = self._as_gray(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self._pending_right.append((self._stamp_to_sec(msg.header.stamp), frame))
        del self._pending_right[:-8]

    def _left_cb(self, msg: Image) -> None:
        try:
            left = self._as_gray(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        t_left = self._stamp_to_sec(msg.header.stamp)
        right = self._take_right(t_left)
        prev_left, prev_right = self._prev_left, self._prev_right
        self._prev_left = left
        if right is None:
            return
        self._prev_right = right
        if prev_left is None or prev_right is None:
            return          # first pair: nothing to difference against yet

        point, cov = self._detect(prev_left, left, prev_right, right)
        if point is None:
            return
        self._publish(point, cov, msg.header.stamp)

    def _take_right(self, t_left: float) -> Optional[np.ndarray]:
        """The buffered right frame closest in time, if close enough."""
        if not self._pending_right:
            return None
        best_i, best_dt = None, float(self._p["pair_tolerance_s"])
        for i, (t, _f) in enumerate(self._pending_right):
            dt = abs(t - t_left)
            if dt <= best_dt:
                best_i, best_dt = i, dt
        if best_i is None:
            return None
        frame = self._pending_right[best_i][1]
        # Drop everything up to and including the match: earlier right frames
        # can only pair with left frames already gone.
        del self._pending_right[:best_i + 1]
        return frame

    # detection

    def _detect(self, prev_l, cur_l, prev_r, cur_r):
        diff_l = np.abs(cur_l.astype(np.int16) - prev_l.astype(np.int16))
        diff_r = np.abs(cur_r.astype(np.int16) - prev_r.astype(np.int16))
        thr = float(self._p["diff_threshold"])
        mask_l = temporal_difference(prev_l, cur_l, threshold=thr)
        mask_r = temporal_difference(prev_r, cur_r, threshold=thr)

        kw = dict(min_area=int(self._p["min_area_px"]),
                  max_area=int(self._p["max_area_px"]))
        blobs_l = extract_blobs(mask_l, intensity=diff_l, **kw)
        blobs_r = extract_blobs(mask_r, intensity=diff_r, **kw)
        if not blobs_l or not blobs_r:
            return None, None

        matches = match_stereo(
            blobs_l, blobs_r,
            max_dy_px=float(self._p["max_dy_px"]),
            min_disparity_px=float(self._p["min_disparity_px"]),
            max_disparity_px=float(self._p["max_disparity_px"]))
        if not matches:
            return None, None

        # Blobs come back brightest-first and match_stereo preserves that
        # order, so the first match is the most confident candidate.
        point = triangulate(matches[0],
                            fx=float(self._p["fx_px"]),
                            cx=float(self._p["cx_px"]),
                            cy=float(self._p["cy_px"]),
                            baseline_m=float(self._p["baseline_m"]))
        if point is None:
            return None, None
        rng = float(np.linalg.norm(point))
        if not (float(self._p["min_range_m"]) <= rng
                <= float(self._p["max_range_m"])):
            return None, None

        cov = triangulation_covariance(
            point,
            fx=float(self._p["fx_px"]),
            baseline_m=float(self._p["baseline_m"]),
            centroid_std_px=float(self._p["centroid_std_px"]))
        return point, cov

    # output

    def _publish(self, point, cov, stamp) -> None:
        src = PointStamped()
        src.header.stamp = stamp
        src.header.frame_id = str(self._p["optical_frame"])
        src.point.x, src.point.y, src.point.z = (float(v) for v in point)

        try:
            out = self._tf_buffer.transform(src, str(self._p["target_frame"]))
        except Exception as exc:   # noqa: BLE001 — a TF gap must not kill the node
            self.get_logger().warn(
                "no %s -> %s transform yet (%s)" % (
                    self._p["optical_frame"], self._p["target_frame"], exc),
                throttle_duration_sec=5.0)
            return

        self._centroid_pub.publish(out)

        # The covariance is published in the OPTICAL frame alongside the
        # optical-frame point, not rotated: a consumer that wants it in
        # base_link must rotate both together. Rotating the point here without
        # the covariance would silently pair a base_link point with an
        # optical-frame R, which is the kind of error that tunes away.
        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header = src.header
        cov_msg.pose.pose.position.x = float(point[0])
        cov_msg.pose.pose.position.y = float(point[1])
        cov_msg.pose.pose.position.z = float(point[2])
        cov_msg.pose.pose.orientation.w = 1.0
        full = np.zeros((6, 6))
        full[:3, :3] = cov
        cov_msg.pose.covariance = [float(v) for v in full.flatten()]
        self._cov_pub.publish(cov_msg)

        if bool(self._p["publish_cue"]):
            self._cue_pub.publish(Bool(data=True))

        self._n_published += 1
        self.get_logger().info(
            "mono detection #%d at %.2f m (sigma_depth %.2f m)" % (
                self._n_published, float(np.linalg.norm(point)),
                float(np.sqrt(cov[2, 2]))),
            throttle_duration_sec=1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonoFlashDetectorNode()
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


# NOTE: no install_clock_guard here, unlike every other node in this package.
# This one is for HARDWARE, where use_sim_time is false and there is no
# /clock, so the guard's WARN branch (the Week 7 HITL shape) would fire on
# every start. Add it if this ever grows a sim path.
