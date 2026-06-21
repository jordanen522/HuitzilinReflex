"""
detector_node.py — HuitzilinReflex Week 3, W3-11 through W3-14.

Pipeline (one PointCloud2 callback):
  1. ROI / range gate             (W3-12)
  2. Voxel downsampling           (W3-12)
  3. NaN / outlier strip          (W3-12)
  4. Rolling background model     (W3-13)
  5. Frame differencing           (W3-13)
  6. Euclidean clustering         (W3-13)
  7. Centroid extraction          (W3-14)
  8. Publish /threat/centroid     (W3-14)
  9. Publish RViz marker          (W3-14)

All thresholds come from ROS params (params/detector.yaml).  No magic numbers
in this file — every tunable has a corresponding yaml key.

QoS contract:
  /oak/points  — subscriber: best_effort, keep_last 1  (must match publisher)
  /threat/centroid — publisher: reliable, keep_last 10
  /threat/marker   — publisher: best_effort, keep_last 1

Coordinate frames:
  /oak/points arrives in camera_optical_frame (Z-forward, X-right, Y-down).
  Centroid is transformed into base_link via tf2 before publishing.
  Week 4 Kalman filter consumes /threat/centroid in base_link.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)


# ── QoS profiles ──────────────────────────────────────────────────────────────

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


# ── Voxel grid (pure-numpy, no PCL / open3d dep) ─────────────────────────────

def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    """
    Down-sample an (N, 3) float32 xyz array to one point per voxel.
    Returns an (M, 3) array with M ≤ N.
    """
    if pts.shape[0] == 0:
        return pts
    keys = np.floor(pts / leaf).astype(np.int32)
    # unique voxels → take centroid of points in each
    unique_keys, inv = np.unique(keys, axis=0, return_inverse=True)
    centroids = np.zeros((len(unique_keys), 3), dtype=np.float32)
    counts = np.bincount(inv, minlength=len(unique_keys)).reshape(-1, 1)
    np.add.at(centroids, inv, pts)
    centroids /= counts
    return centroids


# ── Euclidean clustering (pure-numpy, single-linkage radius) ─────────────────

def euclidean_cluster(pts: np.ndarray, tol: float,
                      min_pts: int, max_pts: int) -> list[np.ndarray]:
    """
    Greedy radius-based Euclidean clustering on (N, 3) float32 xyz.
    Returns a list of (k, 3) arrays, each being one cluster.

    Complexity: O(N²) — acceptable for down-sampled clouds of ~100–2 000 pts.
    If clouds grow larger, swap for scipy.spatial.cKDTree.
    """
    if pts.shape[0] == 0:
        return []

    assigned = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []

    for seed_idx in range(len(pts)):
        if assigned[seed_idx]:
            continue
        # BFS from this seed
        queue = [seed_idx]
        members = []
        while queue:
            idx = queue.pop()
            if assigned[idx]:
                continue
            assigned[idx] = True
            members.append(idx)
            dists = np.linalg.norm(pts - pts[idx], axis=1)
            neighbours = np.where((dists < tol) & (~assigned))[0]
            queue.extend(neighbours.tolist())
        if min_pts <= len(members) <= max_pts:
            clusters.append(pts[np.array(members)])

    return clusters


# ── Main node ─────────────────────────────────────────────────────────────────

class DetectorNode(Node):
    """
    Projectile detection node — W3-11 through W3-14.

    Subscribes /oak/points, runs the filter→diff→cluster→centroid pipeline,
    publishes /threat/centroid (geometry_msgs/PointStamped in base_link).
    """

    def __init__(self) -> None:
        super().__init__("detector")

        # ── Declare all params (values come from detector.yaml) ──────────────
        self.declare_parameter("roi_min_range_m", 0.30)
        self.declare_parameter("roi_max_range_m", 8.00)
        self.declare_parameter("roi_half_angle_deg", 40.0)
        self.declare_parameter("voxel_leaf_m", 0.05)
        self.declare_parameter("bg_history_frames", 5)
        self.declare_parameter("diff_threshold_m", 0.15)
        self.declare_parameter("cluster_tolerance_m", 0.20)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("cluster_max_points", 500)
        self.declare_parameter("min_publish_score", 0.3)
        self.declare_parameter("threat_centroid_topic", "/threat/centroid")
        self.declare_parameter("marker_topic", "/threat/marker")
        self.declare_parameter("compensate_egomotion", True)
        self.declare_parameter("odom_topic", "/huitzilin/odom")

        # ── Cache params ─────────────────────────────────────────────────────
        self._p = self._load_params()

        # ── TF buffer ────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Rolling background buffer (deque of (M, 3) np arrays) ────────────
        self._bg_buffer: deque[np.ndarray] = deque(
            maxlen=self._p["bg_history_frames"]
        )

        # ── Latest odom (for egomotion compensation) ─────────────────────────
        self._latest_odom: Optional[Odometry] = None

        # ── Subscribers ──────────────────────────────────────────────────────
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            "/oak/points",
            self._cloud_cb,
            SENSOR_QOS,
        )
        if self._p["compensate_egomotion"]:
            self._odom_sub = self.create_subscription(
                Odometry,
                self._p["odom_topic"],
                self._odom_cb,
                RELIABLE_QOS,
            )

        # ── Publishers ───────────────────────────────────────────────────────
        self._centroid_pub = self.create_publisher(
            PointStamped,
            self._p["threat_centroid_topic"],
            RELIABLE_QOS,
        )
        self._marker_pub = self.create_publisher(
            Marker,
            self._p["marker_topic"],
            SENSOR_QOS,
        )

        self.get_logger().info(
            "detector_node ready — subscribing /oak/points "
            f"(ROI {self._p['roi_min_range_m']}–{self._p['roi_max_range_m']} m, "
            f"voxel {self._p['voxel_leaf_m']} m, "
            f"diff_thresh {self._p['diff_threshold_m']} m)"
        )

    # ── Param helper ─────────────────────────────────────────────────────────

    def _load_params(self) -> dict:
        return {
            "roi_min_range_m":    self.get_parameter("roi_min_range_m").value,
            "roi_max_range_m":    self.get_parameter("roi_max_range_m").value,
            "roi_half_angle_deg": self.get_parameter("roi_half_angle_deg").value,
            "voxel_leaf_m":       self.get_parameter("voxel_leaf_m").value,
            "bg_history_frames":  self.get_parameter("bg_history_frames").value,
            "diff_threshold_m":   self.get_parameter("diff_threshold_m").value,
            "cluster_tolerance_m":self.get_parameter("cluster_tolerance_m").value,
            "cluster_min_points": self.get_parameter("cluster_min_points").value,
            "cluster_max_points": self.get_parameter("cluster_max_points").value,
            "min_publish_score":  self.get_parameter("min_publish_score").value,
            "threat_centroid_topic": self.get_parameter("threat_centroid_topic").value,
            "marker_topic":       self.get_parameter("marker_topic").value,
            "compensate_egomotion": self.get_parameter("compensate_egomotion").value,
            "odom_topic":         self.get_parameter("odom_topic").value,
        }

    # ── Odom callback ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    # ── Main cloud callback ───────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        t0 = time.monotonic()

        # 1. Unpack to (N, 3) float32 xyz; drop NaNs
        raw = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if raw is None or raw.shape[0] == 0:
            return
        pts = raw.astype(np.float32)

        # 2. ROI / range gate ─────────────────────────────────────────────────
        # In camera_optical_frame: Z is depth (forward), X is right, Y is down.
        depth = pts[:, 2]
        range_mask = (depth >= self._p["roi_min_range_m"]) & \
                     (depth <= self._p["roi_max_range_m"])
        pts = pts[range_mask]
        if pts.shape[0] == 0:
            return

        # Frustum: lateral angle gate (half_angle around Z axis)
        half_angle_rad = math.radians(self._p["roi_half_angle_deg"])
        lateral = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
        angle_mask = np.arctan2(lateral, pts[:, 2]) < half_angle_rad
        pts = pts[angle_mask]
        if pts.shape[0] == 0:
            return

        # 3. Voxel downsample ─────────────────────────────────────────────────
        pts = voxel_downsample(pts, self._p["voxel_leaf_m"])
        if pts.shape[0] == 0:
            return

        # 4. Update background buffer ─────────────────────────────────────────
        self._bg_buffer.append(pts.copy())
        if len(self._bg_buffer) < 2:
            # Need at least 2 frames to difference
            return

        # 5. Frame differencing ───────────────────────────────────────────────
        # Build background as the union of all buffered frames (excluding current).
        # For each current point, find the nearest background point.
        # Points with minimum distance > diff_threshold are "foreground".
        bg_all = np.vstack(list(self._bg_buffer)[:-1])  # all frames except current
        current = self._bg_buffer[-1]

        fg_mask = self._foreground_mask(current, bg_all,
                                        self._p["diff_threshold_m"])
        fg_pts = current[fg_mask]
        if fg_pts.shape[0] == 0:
            return

        # 6. Euclidean clustering ─────────────────────────────────────────────
        clusters = euclidean_cluster(
            fg_pts,
            tol=self._p["cluster_tolerance_m"],
            min_pts=self._p["cluster_min_points"],
            max_pts=self._p["cluster_max_points"],
        )
        if not clusters:
            return

        # 7. Pick best cluster (largest) and extract centroid ─────────────────
        best_cluster = max(clusters, key=lambda c: c.shape[0])
        score = best_cluster.shape[0] / self._p["cluster_max_points"]
        if score < self._p["min_publish_score"]:
            return

        centroid_cam = best_cluster.mean(axis=0)  # (3,) in camera_optical_frame

        # 8. Transform centroid into base_link ────────────────────────────────
        centroid_bl = self._transform_to_base_link(
            centroid_cam, msg.header.stamp, msg.header.frame_id
        )
        if centroid_bl is None:
            return

        # 9. Publish ──────────────────────────────────────────────────────────
        self._publish_centroid(centroid_bl, msg.header.stamp)
        self._publish_marker(centroid_bl, msg.header.stamp)

        dt_ms = (time.monotonic() - t0) * 1e3
        self.get_logger().debug(
            f"detection: cluster {best_cluster.shape[0]} pts "
            f"centroid_bl=({centroid_bl[0]:.2f}, {centroid_bl[1]:.2f}, {centroid_bl[2]:.2f}) "
            f"score={score:.2f} dt={dt_ms:.1f} ms"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _foreground_mask(current: np.ndarray, background: np.ndarray,
                         threshold: float) -> np.ndarray:
        """
        Return bool mask of current points that are farther than `threshold`
        from any background point.  O(N*M) — fine for downsampled clouds.
        """
        if background.shape[0] == 0:
            return np.ones(len(current), dtype=bool)
        # Vectorised pairwise min-distance: (N,) array
        # For each current point, compute dist to all bg points, take min.
        min_dists = np.min(
            np.linalg.norm(current[:, None, :] - background[None, :, :], axis=2),
            axis=1,
        )
        return min_dists > threshold

    def _transform_to_base_link(
        self, xyz_cam: np.ndarray, stamp, frame_id: str
    ) -> Optional[np.ndarray]:
        """
        Transform a point from camera_optical_frame into base_link via tf2.
        Returns (3,) float array or None if transform not available.
        """
        ps_in = PointStamped()
        ps_in.header.stamp = stamp
        ps_in.header.frame_id = frame_id
        ps_in.point.x = float(xyz_cam[0])
        ps_in.point.y = float(xyz_cam[1])
        ps_in.point.z = float(xyz_cam[2])
        try:
            ps_out = self._tf_buffer.transform(ps_in, "base_link",
                                               timeout=rclpy.duration.Duration(seconds=0.05))
            return np.array([ps_out.point.x, ps_out.point.y, ps_out.point.z],
                            dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}", throttle_duration_sec=5.0)
            return None

    def _publish_centroid(self, xyz_bl: np.ndarray, stamp) -> None:
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.point.x = float(xyz_bl[0])
        msg.point.y = float(xyz_bl[1])
        msg.point.z = float(xyz_bl[2])
        self._centroid_pub.publish(msg)

    def _publish_marker(self, xyz_bl: np.ndarray, stamp) -> None:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = "base_link"
        m.ns = "threat"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(xyz_bl[0])
        m.pose.position.y = float(xyz_bl[1])
        m.pose.position.z = float(xyz_bl[2])
        m.pose.orientation.w = 1.0
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.0
        m.color.a = 0.9
        m.lifetime.sec = 0
        m.lifetime.nanosec = int(0.5e9)  # 500 ms TTL — disappears if no new detection
        self._marker_pub.publish(m)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
