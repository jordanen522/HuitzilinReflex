"""
detector_node.py — projectile detection from the depth cloud.

Pipeline (one PointCloud2 callback):
  ROI/range gate -> voxel downsample -> NaN/outlier strip -> persistent-voxel
  background model -> frame differencing -> Euclidean clustering + extent split
  -> centroid extraction -> publish /threat/centroid + RViz marker

INVARIANT: no magic numbers in this file. Every threshold is a ROS param with a
corresponding key in params/detector.yaml, which owns the reason for its value.

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
import os
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from huitzilin_perception.background_map import VoxelBackgroundMap
from huitzilin_perception.cloud_geometry import (
    apply_transform,
    cluster_and_split,
    cluster_extent,
    foreground_mask,
    is_valid_quat,
    make_transform,
    voxel_downsample,
)
from huitzilin_perception.stage_profiler import StageProfiler


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
        self.declare_parameter("debug_funnel", False)  # W3-15 diagnostic: log per-stage counts
        # Funnel-log throttle. 1.0 keeps a 15 Hz stream readable, but it also
        # hides most of a projectile pass: a ball is inside the range gate for
        # only ~0.6 s, so a whole throw collapses to one line. Drop this to 0.0
        # to get every frame while diagnosing a specific throw.
        self.declare_parameter("debug_funnel_throttle_s", 1.0)
        self.declare_parameter("cloud_convention", "gz_flu")  # gz_flu | optical (see detector.yaml)
        self.declare_parameter("fg_max_points", 5000)  # skip frame if foreground explodes (egomotion flood)
        self.declare_parameter("cluster_max_extent_m", 0.35)  # reject clusters physically larger than the projectile
        self.declare_parameter("fixed_frame", "odom")  # frame for egomotion-compensated differencing
        # ── Persistent background map (W4, 2026-07-26) ────────────────────────
        # The bg_history_frames rolling deque only remembers 0.267 s, so any
        # region the camera newly sees under patrol reads as novel and the
        # foreground floods to ~48k points. See background_map.py. Set false to
        # fall back to the Week-3 rolling deque (kept for A/B against the bag
        # library, and it is what the recorded W3 recall numbers were scored on).
        self.declare_parameter("use_persistent_bg", True)
        self.declare_parameter("bg_map_leaf_m", 0.10)      # match diff_threshold_m
        self.declare_parameter("bg_map_ttl_s", 20.0)
        self.declare_parameter("bg_map_max_voxels", 400000)
        # Tighter linkage radius used to re-cluster an oversized blob once.
        # Must sit above voxel_leaf_m (below it, a surface shatters into
        # ball-sized fragments — measured) and below the ball's standoff.
        self.declare_parameter("cluster_split_tol_m", 0.10)
        self.declare_parameter("cluster_split_max_points", 5000)
        # Cloud subscription reliability. TRUE is not the usual sensor-data
        # choice, and it is deliberate: /oak/points is a 7.37 MB sample
        # (640x480 x 24 B) fragmented across thousands of UDP datagrams against
        # a 208 KB net.core.rmem_max, so under BEST_EFFORT a single lost
        # fragment discards the entire sample with no retransmit. Measured live
        # 2026-07-26 on this topic with one variable changed:
        #     best_effort depth=1   ->  3.99 Hz
        #     reliable    depth=10  -> 14.43 Hz  (the camera's full 15 Hz)
        # The detector itself was getting only 1.25 Hz. A ball crosses the
        # frustum in ~0.75 s of sim time, so at that rate it is seen once or
        # twice, the Kalman track never reaches min_track_updates, and no dodge
        # can fire -- that was the Week 4 "no DODGE" root cause, not the
        # detection thresholds. Set false for a real OAK-D, where DepthAI
        # publishes best-effort and dropping a frame beats queueing it.
        self.declare_parameter("cloud_reliable", True)
        self.declare_parameter("cloud_queue_depth", 5)
        # ── Per-stage latency profiling (W4, 2026-07-27) ──────────────────────
        # Independent of debug_funnel on purpose: the funnel's own min/max
        # logging runs six reductions over the full ~200k-point cloud, so
        # profiling with it on measures the instrument as much as the pipeline.
        # Turn this on with debug_funnel OFF to get a clean stage breakdown.
        self.declare_parameter("profile_stages", False)
        self.declare_parameter("profile_window_frames", 30)
        # ── Per-frame dump (W4, 2026-07-27) ───────────────────────────────────
        # The funnel log answers "how many survived each stage", which cannot
        # attribute a MISS: a frame with 40 foreground points and no ball-sized
        # cluster looks identical whether the ball was absorbed into the
        # background map or merely fell below cluster_min_points. Dumping the
        # post-voxel cloud plus the foreground mask makes the attribution exact
        # offline, because the ball's true position is known from
        # /gz/dynamic_poses and can be intersected with the dumped points.
        # Empty string = off. ~250 KB/frame at 15 Hz — a diagnostic, never a
        # setting to leave on.
        self.declare_parameter("debug_dump_dir", "")

        # ── Cache params ─────────────────────────────────────────────────────
        self._p = self._load_params()
        self._funnel_throttle_s = float(
            self.get_parameter("debug_funnel_throttle_s").value)

        # ── TF buffer ────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Background model ─────────────────────────────────────────────────
        # Two models, and which one runs depends on the differencing frame, not
        # only on the param: a persistent map is only meaningful in a FIXED
        # frame. In camera mode (no odom attitude — pre-b0eedd5 bags) the map
        # would accumulate a smear of every past viewpoint, so those runs stay
        # on the Week-3 rolling deque regardless of use_persistent_bg.
        self._bg_buffer: deque[np.ndarray] = deque(
            maxlen=self._p["bg_history_frames"]
        )
        self._bg_map = VoxelBackgroundMap(
            leaf_m=self._p["bg_map_leaf_m"],
            ttl_s=self._p["bg_map_ttl_s"],
            max_voxels=self._p["bg_map_max_voxels"],
        )

        # ── Egomotion compensation state (W3-13) ─────────────────────────────
        # Odom is folded into the tf buffer (odom -> base_link) so clouds can
        # be re-expressed in the fixed odom frame before differencing.
        # _bg_mode tracks which frame the bg buffer currently holds; the two
        # frames must never be mixed in one buffer.
        self._bg_mode: str = "camera"
        self._warned_no_attitude = False

        # ── Latency profiling ────────────────────────────────────────────────
        # Always instrumented, only reported when profile_stages is set: the
        # context managers cost ~1 us each against ms-scale stages, and an
        # always-live instrument cannot drift out of step with the pipeline the
        # way a separately-maintained profiling script would.
        self._prof = StageProfiler(
            int(self.get_parameter("profile_window_frames").value))
        self._profile_on = bool(self.get_parameter("profile_stages").value)

        # ── Per-frame dump ────────────────────────────────────────────────────
        self._dump_dir = str(self.get_parameter("debug_dump_dir").value).strip()
        self._dump: Optional[dict] = None
        if self._dump_dir:
            os.makedirs(self._dump_dir, exist_ok=True)
            self.get_logger().warn(
                f"debug_dump_dir={self._dump_dir} — writing one .npz per cloud "
                "frame. Diagnostic only; this costs latency and disk.")

        # ── Subscribers ──────────────────────────────────────────────────────
        # See the cloud_reliable comment above for why this is not BEST_EFFORT.
        cloud_qos = QoSProfile(
            reliability=(QoSReliabilityPolicy.RELIABLE
                         if self.get_parameter("cloud_reliable").value
                         else QoSReliabilityPolicy.BEST_EFFORT),
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter("cloud_queue_depth").value),
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            "/oak/points",
            self._cloud_cb,
            cloud_qos,
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
            f"({cloud_qos.reliability.name} depth={cloud_qos.depth}, "
            f"ROI {self._p['roi_min_range_m']}–{self._p['roi_max_range_m']} m, "
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
            "debug_funnel":       self.get_parameter("debug_funnel").value,
            "cloud_convention":   self.get_parameter("cloud_convention").value,
            "fg_max_points":      self.get_parameter("fg_max_points").value,
            "cluster_max_extent_m": self.get_parameter("cluster_max_extent_m").value,
            "fixed_frame":        self.get_parameter("fixed_frame").value,
            "use_persistent_bg":  self.get_parameter("use_persistent_bg").value,
            "bg_map_leaf_m":      self.get_parameter("bg_map_leaf_m").value,
            "bg_map_ttl_s":       self.get_parameter("bg_map_ttl_s").value,
            "bg_map_max_voxels":  self.get_parameter("bg_map_max_voxels").value,
            "cluster_split_tol_m": self.get_parameter("cluster_split_tol_m").value,
            "cluster_split_max_points": self.get_parameter("cluster_split_max_points").value,
        }

    # ── Odom callback ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        if not is_valid_quat(q.x, q.y, q.z, q.w):
            # Bags recorded before mav_bridge published attitude carry the
            # all-zero message default; without orientation the cloud cannot
            # be re-expressed in odom, so compensation stays off for them.
            if not self._warned_no_attitude:
                self._warned_no_attitude = True
                self.get_logger().warn(
                    "odom carries no valid orientation — egomotion compensation "
                    "OFF (re-capture bags with the attitude-publishing mav_bridge)")
            return
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or "odom"
        t.child_frame_id = msg.child_frame_id or "base_link"
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self._tf_buffer.set_transform(t, "mav_bridge_odom")

    # ── Main cloud callback ───────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """Count and time every arriving cloud, then run the pipeline.

        Every callback counts as a frame, including the eight early-out paths.
        The executor is single-threaded, so whatever this callback spends —
        publishing a centroid or bailing at the range gate — is what delays the
        next cloud. The mean over ALL callbacks is therefore the figure that has
        to fit inside the ~77 ms wall arrival period (15 Hz at RTF 0.864); a
        mean taken over publishing frames only would flatter the pipeline.

        try/finally so a raised exception still records the frame: a pipeline
        that throws on the interesting frames must not go silent in the report.
        """
        if self._dump_dir:
            self._dump = {}
        try:
            self._cloud_cb_impl(msg)
        finally:
            self._prof.frame_done()
            if self._profile_on and self._prof.due():
                self.get_logger().info(
                    self._prof.report() + " [budget 77 ms/frame wall]")
            # In the finally so the eight early-out paths are dumped too — a
            # frame the pipeline discarded is exactly the frame worth reading.
            if self._dump_dir and self._dump:
                self._write_dump(msg)

    def _cloud_cb_impl(self, msg: PointCloud2) -> None:
        t0 = time.monotonic()
        # Sim clock at callback entry, so the reported dodge latency can be
        # split into transport (sensor stamp -> here) and compute (here ->
        # centroid published).
        #
        # Compute is reported in WALL time only, deliberately. Measuring it in
        # sim time reads exactly 0 ms: this callback blocks the single-threaded
        # executor, so no /clock message is processed while it runs and the sim
        # clock cannot advance mid-callback. Sim-time compute is structurally
        # unmeasurable from in here — don't re-add it.
        #
        # To convert, measure RTF separately (0.864 on the Dell 2026-07-27 —
        # NOT the ~0.33 that older notes assume, so 15 Hz clouds arrive every
        # ~77 ms of wall time, and that is the real budget per frame).
        sim0 = self.get_clock().now().nanoseconds * 1e-9
        dbg = self._p.get("debug_funnel", False)
        d = self._dump
        if d is not None:
            d["stamp_s"] = (msg.header.stamp.sec
                            + msg.header.stamp.nanosec * 1e-9)
            d["sim_entry_s"] = sim0

        prof = self._prof

        with prof.stage("deserialize"):
            raw = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                        skip_nans=True)
            empty = raw is None or raw.shape[0] == 0
            pts = None if empty else raw.astype(np.float32)
        if pts is None:
            if dbg:
                self.get_logger().info("funnel: raw=0 (empty cloud)", throttle_duration_sec=self._funnel_throttle_s)
            return

        # W3-15 root cause of 0% recall: gz-sim fills PointCloudPacked in the
        # gz sensor BODY frame (X fwd, Y left, Z up). <optical_frame_id> in the
        # SDF only renames the header frame — it does NOT rotate the data.
        # Remap to REP-103 optical (X right, Y down, Z fwd) so the depth gate,
        # frustum angle, and TF into base_link all see what they expect.
        # cloud_convention: "optical" = passthrough (real OAK-D via DepthAI).
        if self._p["cloud_convention"] == "gz_flu":
            with prof.stage("convention"):
                pts = np.stack((-pts[:, 1], -pts[:, 2], pts[:, 0]), axis=1)

        # Drop non-finite returns: sky pixels are +inf, which skip_nans keeps.
        with prof.stage("finite"):
            pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] == 0:
            if dbg:
                self.get_logger().info("funnel: raw>0 but finite=0 (all inf)",
                                       throttle_duration_sec=self._funnel_throttle_s)
            return

        n_raw = pts.shape[0]
        if d is not None:
            d["n_raw"] = n_raw
        if dbg:
            # Six full-cloud reductions on ~200k points, and they run BEFORE the
            # range gate throws most of it away. Measured as its own stage so a
            # profiling run can tell the instrument from the pipeline.
            with prof.stage("funnel_log_minmax"):
                self.get_logger().info(
                    f"funnel: raw={n_raw} "
                    f"x[{pts[:,0].min():.2f},{pts[:,0].max():.2f}] "
                    f"y[{pts[:,1].min():.2f},{pts[:,1].max():.2f}] "
                    f"z[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]",
                    throttle_duration_sec=self._funnel_throttle_s)

        with prof.stage("range_gate"):
            depth = pts[:, 2]
            range_mask = (depth >= self._p["roi_min_range_m"]) & \
                         (depth <= self._p["roi_max_range_m"])
            pts = pts[range_mask]
        n_range = pts.shape[0]
        if d is not None:
            d["n_range"] = n_range
        if pts.shape[0] == 0:
            if dbg:
                self.get_logger().info(
                    f"funnel: raw={n_raw} range=0 "
                    f"(nothing on Z in {self._p['roi_min_range_m']}-{self._p['roi_max_range_m']} m)",
                    throttle_duration_sec=self._funnel_throttle_s)
            return

        with prof.stage("angle_gate"):
            half_angle_rad = math.radians(self._p["roi_half_angle_deg"])
            lateral = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
            angle_mask = np.arctan2(lateral, pts[:, 2]) < half_angle_rad
            pts = pts[angle_mask]
        n_angle = pts.shape[0]
        if d is not None:
            d["n_angle"] = n_angle
        if pts.shape[0] == 0:
            if dbg:
                self.get_logger().info(f"funnel: raw={n_raw} range={n_range} angle=0",
                                       throttle_duration_sec=self._funnel_throttle_s)
            return

        with prof.stage("voxel"):
            pts = voxel_downsample(pts, self._p["voxel_leaf_m"])
        n_voxel = pts.shape[0]
        if d is not None:
            d["n_voxel"] = n_voxel
        if pts.shape[0] == 0:
            return

        # ── Egomotion compensation (W3-13; 2026-07-06 recall root cause) ─────
        # Difference in the fixed odom frame, not the moving camera frame: at
        # patrol speed the whole scene shifts ~diff_threshold per frame in
        # camera coords, so uncompensated differencing floods and the
        # fg_max_points guard discards the very frames containing the ball.
        # Needs odom with a valid quaternion; old bags fall back to camera mode.
        T_fixed_cam = None
        if self._p["compensate_egomotion"]:
            with prof.stage("tf_lookup"):
                T_fixed_cam = self._lookup_matrix(
                    self._p["fixed_frame"], msg.header.frame_id,
                    msg.header.stamp)
        mode = "fixed" if T_fixed_cam is not None else "camera"
        if mode != self._bg_mode:
            # Never mix frames inside one background model.
            self._bg_buffer.clear()
            self._bg_map.clear()
            self.get_logger().info(
                f"differencing frame -> {mode} "
                f"({'odom TF available' if mode == 'fixed' else 'no odom TF — uncompensated'})")
            self._bg_mode = mode
        if mode == "fixed":
            with prof.stage("ego_transform"):
                pts = apply_transform(T_fixed_cam, pts)

        # Persistent voxel map only in a fixed frame — see __init__.
        use_map = bool(self._p["use_persistent_bg"]) and mode == "fixed"
        if use_map:
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with prof.stage("bg_query"):
                fg_mask = self._bg_map.foreground(pts)
            n_bg = len(self._bg_map)
            # Insert AFTER querying, or the frame is its own background.
            with prof.stage("bg_insert"):
                self._bg_map.insert(pts, stamp_s)
            with prof.stage("bg_prune"):
                self._bg_map.prune(stamp_s)
            fg_pts = pts[fg_mask]
        else:
            with prof.stage("bg_deque_diff"):
                self._bg_buffer.append(pts.copy())
                short = len(self._bg_buffer) < 2
                if not short:
                    bg_all = np.vstack(list(self._bg_buffer)[:-1])
                    current = self._bg_buffer[-1]
                    n_bg = bg_all.shape[0]
                    fg_mask = foreground_mask(current, bg_all,
                                              self._p["diff_threshold_m"])
                    fg_pts = current[fg_mask]
            if short:
                return
        n_fg = fg_pts.shape[0]
        if d is not None:
            # `pts` is post-voxel, post-egomotion — i.e. the fixed(odom)-frame
            # cloud the background map actually saw. fg_mask indexes it, so the
            # two together separate "the ball was never in the cloud" from "the
            # ball was in the cloud and the background map swallowed it".
            d["pts"] = pts.astype(np.float32)
            d["fg"] = np.asarray(fg_mask, dtype=bool)
            d["n_bg"] = int(n_bg)
            d["n_fg"] = int(n_fg)
            d["mode"] = mode
        if fg_pts.shape[0] == 0:
            if dbg:
                self.get_logger().info(
                    f"funnel: raw={n_raw} range={n_range} angle={n_angle} "
                    f"voxel={n_voxel} bg={n_bg} fg=0",
                    throttle_duration_sec=self._funnel_throttle_s)
            return

        if d is not None:
            d["fg_flood"] = bool(n_fg > self._p["fg_max_points"])
        if fg_pts.shape[0] > self._p["fg_max_points"]:
            # Whole-scene depth change (patrol turn / aggressive egomotion) —
            # not a projectile signature. Skip rather than cluster 10k+ points.
            if dbg:
                self.get_logger().info(
                    f"funnel: fg={n_fg} > fg_max_points={self._p['fg_max_points']} "
                    "-> egomotion flood, skip frame",
                    throttle_duration_sec=self._funnel_throttle_s)
            return

        # Physical-size gate: an 80 mm ball voxelized at 0.02 m can never span
        # more than ~0.15 m, while the dominant FP mode ("scene-entry" blobs —
        # a near wall/ground patch enters the ROI where the background had
        # nothing, so fg==voxel and the whole patch clusters) spans metres.
        # Point-count caps can't separate them (ball 3-50 pts overlaps patch
        # 42-500 pts); metric extent separates them cleanly.
        #
        # Oversized clusters are re-clustered once at a tighter tolerance rather
        # than discarded: measured 2026-07-26, the ball merges into a metre-scale
        # blob at tol=0.20 and was being thrown away with it. One pass only —
        # recursive shrinking shatters surfaces into ball-sized false positives.
        # See cluster_and_split().
        with prof.stage("cluster"):
            split = cluster_and_split(
                fg_pts,
                tol=self._p["cluster_tolerance_m"],
                min_pts=self._p["cluster_min_points"],
                max_extent=self._p["cluster_max_extent_m"],
                split_tol=self._p["cluster_split_tol_m"],
                max_split_points=self._p["cluster_split_max_points"],
            )
        # cluster_max_points still applies, but only AFTER splitting: a dense
        # sub-0.35 m blob is not a ball, and it would otherwise out-vote the ball
        # in the largest-cluster pick below. Pre-split it discarded the ball.
        ball_sized = [c for c in split
                      if c.shape[0] <= self._p["cluster_max_points"]]
        if d is not None:
            # Every cluster the splitter produced, BEFORE cluster_max_points, so
            # the dump can tell "no cluster near the ball" (lost upstream, at the
            # diff or at cluster_min_points) from "a cluster sat on the ball and
            # a later gate discarded it".
            d["cl_size"] = np.array([c.shape[0] for c in split], dtype=np.int32)
            d["cl_extent"] = np.array([cluster_extent(c) for c in split],
                                      dtype=np.float32)
            d["cl_centroid"] = (np.array([c.mean(axis=0) for c in split],
                                         dtype=np.float32)
                                if split else np.zeros((0, 3), np.float32))
            d["n_ball_sized"] = len(ball_sized)
        if dbg:
            desc = sorted(((c.shape[0], round(cluster_extent(c), 2))
                           for c in ball_sized), reverse=True)[:5]
            self.get_logger().info(
                f"funnel: raw={n_raw} range={n_range} angle={n_angle} voxel={n_voxel} "
                f"bg={n_bg} fg={n_fg} split={len(split)} ball_sized={len(ball_sized)} "
                f"(size,extent)={desc}",
                throttle_duration_sec=self._funnel_throttle_s)
        if not ball_sized:
            return

        best_cluster = max(ball_sized, key=lambda c: c.shape[0])
        score = best_cluster.shape[0] / self._p["cluster_max_points"]
        if d is not None:
            d["best_size"] = int(best_cluster.shape[0])
            d["best_centroid"] = best_cluster.mean(axis=0).astype(np.float32)
            d["score"] = float(score)
        if score < self._p["min_publish_score"]:
            if dbg:
                self.get_logger().info(
                    f"funnel: best={best_cluster.shape[0]} score={score:.3f} "
                    f"< min_publish={self._p['min_publish_score']} -> reject",
                    throttle_duration_sec=self._funnel_throttle_s)
            return

        with prof.stage("centroid_tf"):
            centroid = best_cluster.mean(axis=0)
            if self._bg_mode == "fixed":
                # cluster lives in the fixed odom frame
                T_bl_fixed = self._lookup_matrix(
                    "base_link", self._p["fixed_frame"], msg.header.stamp)
                centroid_bl = (apply_transform(T_bl_fixed, centroid[None, :])[0]
                               if T_bl_fixed is not None else None)
            else:
                centroid_bl = self._transform_to_base_link(
                    centroid, msg.header.stamp, msg.header.frame_id
                )
        if centroid_bl is None:
            if dbg:
                self.get_logger().info("funnel: TF to base_link FAILED",
                                       throttle_duration_sec=self._funnel_throttle_s)
            return

        if dbg:
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            wall_ms = (time.monotonic() - t0) * 1000.0
            # transport = how stale the cloud already was on arrival. Measured
            # 53-348 ms (typically >300) while compute ran 63-354 ms wall
            # against a 77 ms wall arrival period: the detector is ~2x slower
            # than its input, the queue backs up, and THAT backlog is the
            # transport figure. Transport is the symptom; compute is the cause.
            self.get_logger().info(
                f"timing: transport={((sim0 - stamp_s) * 1000.0):.0f} ms(sim) "
                f"compute={wall_ms:.0f} ms(wall, budget 77) raw={n_raw}",
                throttle_duration_sec=self._funnel_throttle_s)

        if d is not None:
            d["published"] = True
            d["centroid_bl"] = np.asarray(centroid_bl, dtype=np.float32)
        with prof.stage("publish"):
            self._publish_centroid(centroid_bl, msg.header.stamp)
            self._publish_marker(centroid_bl, msg.header.stamp)
        if dbg:
            self.get_logger().info(
                f"funnel: *** PUBLISHED *** best={best_cluster.shape[0]} score={score:.3f}",
                throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _write_dump(self, msg: PointCloud2) -> None:
        """Write one frame's per-stage record to debug_dump_dir.

        Keyed by the cloud's SIM stamp in nanoseconds, which is the only clock
        that joins this to the ball's true pose. Wall time cannot: the callback
        blocks the executor for ~80 ms, which is more than a frame period, so a
        wall-time join is ambiguous by a whole frame.

        Failures are logged, never raised — an instrument that can kill the
        pipeline it is measuring is worse than no instrument.
        """
        d = self._dump
        self._dump = None
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            # Uncompressed on purpose: this runs inside the callback that the
            # single-threaded executor uses to keep up with 15 Hz clouds, so
            # zlib on ~250 KB would cost more latency than the disk write it
            # saves — and added latency drops frames, corrupting exactly the
            # detection-consistency measurement this dump exists to make.
            np.savez(os.path.join(self._dump_dir, f"f_{stamp_ns}.npz"), **d)
        except Exception as e:  # noqa: BLE001 — diagnostic must not propagate
            self.get_logger().warn(f"debug dump write failed: {e}",
                                   throttle_duration_sec=5.0)

    def _lookup_matrix(self, target: str, source: str, stamp) -> Optional[np.ndarray]:
        """
        4x4 target<-source transform at `stamp`. Falls back to the latest
        available transform (≤1 odom period stale) when the exact stamp isn't
        bracketed yet, e.g. the newest cloud always arrives before the odom
        sample that would bracket it.
        Returns None if the chain isn't available at all.

        That staleness is NOT "cm-level at patrol speed" as this docstring used
        to claim: odom runs at 30 Hz and patrol translates at 2.5-3.2 m/s (the
        old 0.18-0.48 m/s figure was a two-competing-stacks artifact), so one
        period is ~0.10 m — exactly diff_threshold_m. It is still not what
        blinds the detector during patrol: the foreground floods to ~48k points
        even at 0.63 m/s, where one period is 0.02 m.
        "the detector goes blind while patrolling" — the cause is the 5-frame
        rolling background buffer, not this lookup.
        """
        try:
            ts = self._tf_buffer.lookup_transform(target, source,
                                                  Time.from_msg(stamp))
        except tf2_ros.TransformException:
            try:
                ts = self._tf_buffer.lookup_transform(target, source, Time())
            except tf2_ros.TransformException:
                return None
        tr, q = ts.transform.translation, ts.transform.rotation
        return make_transform((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM/SIGINT already shut the context down; a second shutdown
        # raises RCLError (seen as a traceback after every regression run).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
