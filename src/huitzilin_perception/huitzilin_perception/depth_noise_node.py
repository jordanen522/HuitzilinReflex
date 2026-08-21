"""depth_noise_node.py — the rendered lane's noise stage, as a ROS 2 node.

Sits between `ros_gz_bridge` and the REAL, unmodified `detector_node`:

    Gazebo depth camera -> ros_gz_bridge -> THIS -> detector -> /threat/centroid
      (real geometry)                    (real error)  (real pipeline)

WHY IT EXISTS. A Gazebo depth camera reports very nearly exact geometry:
`iris_depth` declares a 1 cm per-pixel gaussian, and the optics probe world
declared none at all. Either way the error is ~30x too small at 26 m -- where
the real figure is 0.30 m -- and independent per pixel where the real one is
correlated. Handing that to the detector would make the simulated sensor
strictly better than the $89 AR0234 part it stands for, and every save rate
measured through it would be an overstatement. The error model lives in
`depth_noise.py`; this file is only the plumbing, so the model stays testable
without ROS.

WHY NOT GAZEBO'S OWN `<noise>`. A depth camera's `<noise>` tag is independent
per pixel. That is the white-noise limit, which `test_depth_noise.py` shows is
wrong in BOTH directions at once: it smears the ball's cluster while averaging
its centroid error down by ~sqrt(N), i.e. it makes the cluster harder to find
and the position it reports more accurate than the real sensor's. The
correlated field this node applies is the point of the exercise.

THE OUTPUT CLOUD keeps the input's header (stamp and frame_id) and its
organized height/width, and carries x, y, z as float32 and nothing else. Any
further fields on the input -- Gazebo's PointCloudPacked usually carries rgb --
are dropped, because the detector reads only xyz. If a downstream consumer ever
needs them, that is a real change, not a detail.

CONFIGURATION IS LOGGED AT STARTUP, deliberately. CLAUDE.md's standing rule is
that a sensor is reach AND sector AND rate, and that quoting one axis describes
a different instrument; the same trap applies to the noise, so `sigma_ref_m`,
`ref_range_m` and `correlation_px` are printed beside the cloud's shape where a
run log will capture them.

`sigma_ref_m: 0.0` disables the noise exactly -- `apply_image_depth_noise`
consumes no randomness in that case -- so the noiseless arm of an A/B is a true
control rather than a differently-seeded one.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from huitzilin_perception.depth_noise import (
    DEFAULT_CORRELATION_PX,
    apply_image_depth_noise,
    required_cluster_max_extent_m,
)
from huitzilin_perception.synthetic_depth import (
    DEFAULT_DEPTH_SIGMA_M,
    DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
)

_XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 12

# Depth clouds are a sensor stream: a late frame is worthless, so drop rather
# than queue. Matches how the detector subscribes to /oak/points.
_SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class DepthNoiseNode(Node):
    """Apply correlated stereo depth error to an organized rendered cloud."""

    def __init__(self) -> None:
        super().__init__("depth_noise")
        self.declare_parameter("cloud_in_topic", "/oak/points_rendered")
        self.declare_parameter("cloud_out_topic", "/oak/points")
        self.declare_parameter("sigma_ref_m", DEFAULT_DEPTH_SIGMA_M)
        self.declare_parameter("ref_range_m", DEFAULT_DEPTH_SIGMA_REF_RANGE_M)
        self.declare_parameter("correlation_px", DEFAULT_CORRELATION_PX)
        self.declare_parameter("seed", 0)

        self._sigma_ref_m = float(self._p("sigma_ref_m"))
        self._ref_range_m = float(self._p("ref_range_m"))
        self._correlation_px = float(self._p("correlation_px"))
        self._rng = np.random.default_rng(int(self._p("seed")))

        # Warn-once latches. A per-frame warning at 23 Hz would bury the log
        # and hide whatever else went wrong in the same run.
        self._warned_unorganized = False
        self._logged_shape = False
        self._frames = 0

        out_topic = str(self._p("cloud_out_topic"))
        in_topic = str(self._p("cloud_in_topic"))
        self._pub = self.create_publisher(PointCloud2, out_topic, _SENSOR_QOS)
        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._on_cloud, _SENSOR_QOS)

        self.get_logger().info(
            f"depth_noise: {in_topic} -> {out_topic} | "
            f"sigma_ref={self._sigma_ref_m:.3f} m at {self._ref_range_m:.1f} m, "
            f"correlation={self._correlation_px:.1f} px, "
            f"seed={int(self._p('seed'))}"
            + ("  [DISABLED: sigma_ref_m=0, cloud passes through exactly]"
               if self._sigma_ref_m <= 0.0 else ""))
        if self._sigma_ref_m > 0.0:
            self.get_logger().info(
                "depth_noise: this reach needs cluster_max_extent_m >= "
                f"{required_cluster_max_extent_m(self._ref_range_m):.2f} m at "
                f"{self._ref_range_m:.1f} m; the shipped 0.35 m was sized for "
                "a ball at <= 5 m and will reject ~20% of ball frames.")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_cloud(self, msg: PointCloud2) -> None:
        height, width = int(msg.height), int(msg.width)
        if height <= 1:
            if not self._warned_unorganized:
                self._warned_unorganized = True
                self.get_logger().error(
                    f"depth_noise: refusing an UNORGANIZED cloud (height="
                    f"{height}). The disparity correlation lives in image "
                    "space; without rows there is no neighbourhood to "
                    "correlate over, and white noise would smear the ball "
                    "while still reporting a plausible sigma. Nothing is "
                    "being republished -- fix the bridge, do not score this.")
            return

        xyz = pc2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=False)
        organized = xyz.reshape(height, width, 3)
        noised = apply_image_depth_noise(
            organized, self._rng,
            sigma_ref_m=self._sigma_ref_m,
            ref_range_m=self._ref_range_m,
            correlation_px=self._correlation_px)

        if not self._logged_shape:
            self._logged_shape = True
            finite = int(np.count_nonzero(np.isfinite(organized).all(axis=2)))
            self.get_logger().info(
                f"depth_noise: first cloud {width}x{height}, "
                f"{finite} finite returns of {height * width}")

        self._frames += 1
        self._pub.publish(self._pack(msg, noised))

    @staticmethod
    def _pack(src: PointCloud2, points_hw3: np.ndarray) -> PointCloud2:
        """Rebuild an organized xyz cloud, preserving the source header."""
        height, width, _ = points_hw3.shape
        out = PointCloud2()
        out.header = src.header
        out.height = height
        out.width = width
        out.fields = _XYZ_FIELDS
        out.is_bigendian = False
        out.point_step = _POINT_STEP
        out.row_step = _POINT_STEP * width
        # Not dense: no-return pixels stay NaN/inf, which is what they mean.
        out.is_dense = False
        out.data = np.ascontiguousarray(
            points_hw3, dtype=np.float32).tobytes()
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
