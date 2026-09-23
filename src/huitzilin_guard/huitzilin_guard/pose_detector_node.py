"""Runs a pose model on a camera image and publishes body joints.

In:  image_topic (sensor_msgs/Image, BEST_EFFORT to match a camera driver)
     depth_topic (sensor_msgs/Image, optional, only in range_mode 'depth')
Out: /guard/detections (std_msgs/String, JSON)

This is the real detection stage. The model emits 17 COCO keypoints and five
of them are the face; pose_detector.decode drops those before returning, so no
face coordinate exists anywhere in this process beyond the raw output tensor,
which is discarded with the frame. No image, crop or embedding is published,
stored or logged, and there is no recording path in this node at all.

onnxruntime is imported lazily, inside __init__. It is not a ROS package and
is not installed system-wide, so importing it at module scope would break test
collection for the whole package on any machine without it.

cv_bridge is deliberately not used: it is not installed on the lab machine, and
an Image of a known encoding is a numpy reshape. One less dependency on the
only path that touches the camera.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from huitzilin_guard import pose_detector as pdet
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

# Camera drivers publish best-effort. A RELIABLE subscription would silently
# never match, and the node would sit with a healthy camera seeing nothing.
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

_BGR = ("bgr8", "8UC3")
_RGB = ("rgb8",)


def image_to_bgr(msg) -> np.ndarray:
    """Decode a colour Image message into an HxWx3 BGR array."""
    if msg.encoding in _BGR:
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return arr
    if msg.encoding in _RGB:
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1]
    raise ValueError(
        "unsupported image encoding %r; this node handles bgr8 and rgb8. A "
        "mono or bayer stream needs converting upstream." % msg.encoding)


def depth_to_metres(msg) -> np.ndarray:
    """Decode a depth Image into metres.

    16UC1 is millimetres by ROS convention and is what a stereo camera emits;
    silently treating it as metres would put every subject 1000x too far away
    and reject every range as out of bounds.
    """
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    if msg.encoding == "16UC1":
        raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float32) * 0.001
    raise ValueError("unsupported depth encoding %r" % msg.encoding)


class PoseDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__("pose_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth")
        self.declare_parameter("keypoints_topic", "/guard/detections")
        self.declare_parameter("model_path", "")
        self.declare_parameter("person_conf", 0.50)
        self.declare_parameter("joint_conf", 0.50)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("subject_height_m", 1.70)
        self.declare_parameter("fov_h_deg", 69.0)
        self.declare_parameter("range_mode", "monocular")
        self.declare_parameter("min_range_m", 0.8)
        self.declare_parameter("max_range_m", 25.0)
        # Inference is the expensive stage and the camera is faster than it.
        # Processing every frame just queues latency; this drops instead.
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("record_video", False)

        def p(name):
            return self.get_parameter(name).value

        if bool(p("record_video")):
            raise ValueError(
                "record_video is true. This node has no recording path and "
                "must not acquire one by configuration.")

        mode = str(p("range_mode"))
        if mode not in ("monocular", "depth"):
            raise ValueError("range_mode %r must be 'monocular' or 'depth'"
                             % mode)

        model = os.path.expanduser(str(p("model_path")))
        if not model or not os.path.isfile(model):
            raise ValueError(
                "model_path %r does not exist. The pose model is not tracked "
                "in this repository; fetch it with "
                "scripts/fetch_pose_model.sh and point model_path at it. "
                "Refusing to start is deliberate: a detector that came up "
                "without a model would publish nothing, and silence here is "
                "indistinguishable from a scene with nobody in it."
                % str(p("model_path")))

        # Lazy on purpose. See the module docstring.
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise ValueError(
                "onnxruntime is not importable (%s). It is not a ROS package; "
                "install it into a directory on PYTHONPATH rather than into "
                "the system interpreter, which is externally managed." % exc)

        self._session = ort.InferenceSession(
            model, providers=["CPUExecutionProvider"])
        self._input_name = self._session.get_inputs()[0].name

        self._cfg = pdet.DetectorConfig(
            person_conf=float(p("person_conf")),
            joint_conf=float(p("joint_conf")),
            input_size=int(p("input_size")),
            subject_height_m=float(p("subject_height_m")),
            fov_h_deg=float(p("fov_h_deg")),
            range_mode=mode,
            min_range_m=float(p("min_range_m")),
            max_range_m=float(p("max_range_m")))

        self._min_period_s = 1.0 / float(p("max_rate_hz"))
        self._last_infer_s = None
        self._depth = None
        self._frames_in = 0
        self._frames_published = 0

        self._pub = self.create_publisher(
            String, str(p("keypoints_topic")), RELIABLE_QOS)
        self.create_subscription(Image, str(p("image_topic")),
                                 self._image_cb, SENSOR_QOS)
        if mode == "depth":
            self.create_subscription(Image, str(p("depth_topic")),
                                     self._depth_cb, SENSOR_QOS)

        self.create_timer(5.0, self._log_throughput)

        self.get_logger().info(
            "pose_detector ready -- model %s, range_mode %s, %d body joints "
            "(COCO-17 minus the 5 face keypoints, which are dropped before "
            "publication). No image is stored or republished."
            % (os.path.basename(model), mode, len(pdet.BODY_KEYPOINTS)))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _depth_cb(self, msg) -> None:
        try:
            self._depth = depth_to_metres(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)

    def _image_cb(self, msg) -> None:
        now_s = self._now_s()
        if (self._last_infer_s is not None
                and now_s - self._last_infer_s < self._min_period_s):
            return
        self._last_infer_s = now_s
        self._frames_in += 1

        try:
            bgr = image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
            return

        import cv2
        height, width = bgr.shape[:2]
        size = self._cfg.input_size
        blob = cv2.resize(bgr, (size, size))[:, :, ::-1].transpose(2, 0, 1)
        blob = (np.ascontiguousarray(blob, dtype=np.float32) / 255.0)[None]

        raw = self._session.run(None, {self._input_name: blob})[0]
        joints_px, bbox = pdet.decode(raw, (width, height), self._cfg)
        if not joints_px:
            return

        metric = pdet.to_metric(joints_px, bbox, (width, height), self._cfg,
                               depth_m=self._depth)
        if not metric:
            # No believable range. Publishing joints with a fabricated Z would
            # place a person in or out of the box on a range nobody measured.
            self.get_logger().warn(
                "no usable range for this frame; dropping it",
                throttle_duration_sec=10.0)
            return

        # The source message stamp, never arrival time: the guard's
        # staleness and confirmation windows are denominated in it.
        payload = pdet.frame_payload(
            metric, self._stamp_to_sec(msg.header.stamp),
            msg.header.frame_id or "camera_optical_frame")
        self._pub.publish(String(data=json.dumps(payload)))
        self._frames_published += 1

    def _log_throughput(self) -> None:
        self.get_logger().info(
            "pose_detector: %d frames in, %d published"
            % (self._frames_in, self._frames_published))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseDetectorNode()
    install_clock_guard(node)
    clock_failed = False
    try:
        rclpy.spin(node)
    except ClockGuardError:
        clock_failed = True
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if clock_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
