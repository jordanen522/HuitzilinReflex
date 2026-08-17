"""synthetic_depth_publisher_node.py — a synthetic cloud for the REAL detector.

WHAT THIS IS. It publishes a `sensor_msgs/PointCloud2` on `/oak/points` that the
unmodified `detector_node.py` consumes, so the real pipeline — ROI gate, voxel
downsample, background differencing, Euclidean clustering, centroid extraction —
computes the centroid instead of it being asserted. **It is a synthetic cloud
through a real pipeline. It is not a rendered depth sensor.** Nothing measured
through this lane is evidence that a real camera achieves the modelled reach;
that remains a hardware measurement.

WHY IT EXISTS. Every result in docs/RESULTS.md was produced by
`oracle_detector_node.py`, which reads Gazebo truth and publishes
`/threat/centroid` directly. It subscribes to no cloud and shares no code with
the detector, so "dodges a 20 m/s projectile using depth sensing" was not true
*of that result* — the clustering pipeline never ran at 20 m/s. This node closes
that gap: the truth source stays (an 80 mm ball at 26 m is sub-pixel for any
depth camera this world can render, which is exactly why the oracle exists), but
the detection is now COMPUTED by the real algorithm rather than declared.

MUTUALLY EXCLUSIVE with `oracle_detector`, and with a second `detector`. Two
publishers on `/threat/centroid` give the tracker two uncorrelated views of one
ball. `week6_synthetic_depth.launch.py` enforces this by never including
`week3_perception` and never starting the oracle — there is no interlock here.

THE CLOUD CONTAINS THE BALL AND NOTHING ELSE — read this before quoting a number
------------------------------------------------------------------------------
No ground, no runway, no background return of any kind is synthesized. The
consequence for `detector_node`'s background-differencing stage is deliberate
and has to be carried with every result taken through this lane:

  * The stage still RUNS. Every return is novel, so the foreground mask is
    all-True and the whole cloud reaches the clusterer. Nothing is bypassed and
    `detector_node.py` is byte-identical.
  * It therefore rejects NOTHING. This lane measures the ROI / voxel / cluster /
    extent / centroid path. It says nothing whatever about false-positive
    rejection against real scene clutter — the Week 4 patrol figures in
    CLAUDE.md remain the only measurement of that, and a false-dodge count from
    this lane would be meaningless.
  * `synthetic_depth_detector.yaml` therefore ships `use_persistent_bg: false`.
    With a ball-only cloud the persistent voxel map has nothing to model and one
    way to do harm: it retains a throw's trail for `bg_map_ttl_s` 20 s, while
    `dodge_battery` re-throws after `settle_s` 6 s down the same corridor in the
    same odom frame, so throw 2 onward would be absorbed into throw 1's trail
    and read as background. The Week-3 rolling deque holds 5 frames and
    differences against the older 4 (0.067 s at 60 Hz), which cannot span two
    throws.

THE SLOW-BALL FLOOR THE DEQUE IMPOSES — this is not free, and it is silent
-------------------------------------------------------------------------
Frame differencing keeps a return only if it is further than `diff_threshold_m`
from every background return, so a ball must advance more than one threshold per
frame or it sits inside its own previous blob and the ENTIRE cloud reads as
background:

    v_min = diff_threshold_m * delivered_rate_hz = 0.10 * 60 = 6.0 m/s

Below `v_min` nothing is detected, at any range, with every stage upstream
looking healthy — measured through the real `voxel_downsample` /
`foreground_mask` / `cluster_and_split`: 20/14/8 m/s cluster on 100% of frames,
6 m/s on 20%, and 4 m/s (B01's speed) on 0 of 45. It does not bite at long range,
where the 0.30 m common-mode depth jitter dwarfs the 0.10 m threshold, only at
short range where sigma collapses to 0.005 m. The rendered lane never met it
because it runs at 15 Hz (floor 1.5 m/s); the 60 Hz default here is what makes it
reachable, and 60 Hz is kept because it is the configuration the headline 26 m /
20 m/s result was flown at.

This node cannot prevent it — the differencing is the detector's — so it does the
next best thing and makes it LOUD: the computed floor is printed as a WARN in the
startup banner, for the same reason CLAUDE.md makes `FRAME_CLASS=0` and the ball
prefixes loud. To fly a scenario slower than `v_min`, lower `rate_hz` (15 Hz
gives a 1.5 m/s floor) and quote the lower rate beside the result.

Frames. The ball is taken from Gazebo (both models, so the world/odom origin
offset cancels), rotated into base_link with the odom attitude — the same
quaternion `evasion_node` uses to rotate it back — and then shifted by the
camera mount offset so the round trip through the static TF chain is exact
rather than carrying a fixed 0.10 m forward bias. Points go out in the gz sensor
body convention (FLU: x forward, y left, z up) under the header frame
`camera_optical_frame`, which is what the real `ros_gz_bridge` produces and what
`detector.yaml`'s `cloud_convention: "gz_flu"` remaps at ingest. Publishing
optical-convention points instead would silently mirror the cloud.

Stamps come from the Gazebo pose message, never from arrival time: the filter
differentiates consecutive stamps, and arrival is not emission (CLAUDE.md).
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

from huitzilin_perception.cloud_geometry import is_valid_quat, quat_to_rot
from huitzilin_perception.oracle import (
    DEFAULT_BALL_PREFIXES,
    DelayLine,
    select_ball,
    to_body,
)
from huitzilin_perception.synthetic_depth import (
    DEFAULT_BALL_RADIUS_M,
    DEFAULT_DEPTH_SIGMA_M,
    DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    DEFAULT_FOV_HALF_H_DEG,
    DEFAULT_FOV_HALF_V_DEG,
    DEFAULT_IMAGE_HEIGHT_PX,
    DEFAULT_IMAGE_WIDTH_PX,
    DEFAULT_MAX_POINTS_PER_BALL,
    SCENE_BROADCASTER_HZ,
    camera_relative,
    emit_period_s,
    min_detectable_closing_speed_mps,
    quantize_rate_hz,
    synthesize_ball_cloud,
)
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

# Matches gz_pose_bridge's publisher. A RELIABLE subscription would not match it
# at all, and the topic would look alive while delivering nothing.
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# mav_bridge publishes odom RELIABLE; detector_node subscribes to it RELIABLE.
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# One float32 xyz triple per return: the only three fields detector_node reads
# (`pc2.read_points_numpy(..., field_names=("x", "y", "z"))`).
_XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP_BYTES = 12

# Roughly 20 s of 60 Hz pose messages: long enough that arming and takeoff pass
# without comment, short enough that a misconfigured prefix is loud inside the
# first scenario rather than after a battery has finished scoring a blind node.
UNMATCHED_WARN_AFTER = 2000

# ~2 s of 60 Hz poses. Enough to measure the stream's real cadence to well under
# a percent, early enough that the number is in the log before the first throw.
POSE_RATE_SAMPLE_MSGS = 120

# How far the measured pose cadence may sit from SCENE_BROADCASTER_HZ before the
# delivered-rate claim in the banner stops being true.
POSE_RATE_TOLERANCE = 0.05


class SyntheticDepthPublisherNode(Node):
    """Publishes the ball's synthetic returns on the detector's cloud topic."""

    def __init__(self) -> None:
        super().__init__("synthetic_depth_publisher")
        self._declare_params()
        self._read_params()
        self._reset_state()

        cloud_qos = QoSProfile(
            reliability=(QoSReliabilityPolicy.RELIABLE
                         if bool(self._p("cloud_reliable"))
                         else QoSReliabilityPolicy.BEST_EFFORT),
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=int(self._p("cloud_queue_depth")),
        )
        self._pub = self.create_publisher(
            PointCloud2, str(self._p("cloud_topic")), cloud_qos)
        self.create_subscription(Odometry, str(self._p("odom_topic")),
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, str(self._p("poses_topic")),
                                 self._poses_cb, POSE_QOS)
        self._log_banner(cloud_qos)

    def _read_params(self) -> None:
        """Snapshot every parameter into an attribute, exactly once.

        Read-once like the oracle and the detector: `ros2 param set` is accepted
        and ignored, so every configuration change needs a full restart.
        """
        self._drone_model = str(self._p("drone_model"))
        # Empty is rejected rather than tolerated: it matches no model, so the
        # node would run, log happily, publish empty clouds forever, and every
        # scenario would score NO-FIRE — indistinguishable from a trigger that
        # never fires. See oracle.DEFAULT_BALL_PREFIXES for what that cost once.
        self._ball_prefixes = [str(p) for p in (self._p("ball_name_prefixes")
                                                or []) if str(p)]
        if not self._ball_prefixes:
            raise ValueError(
                "ball_name_prefixes is empty — this node would match no ball "
                "and silently publish an empty cloud forever")

        self._range_m = float(self._p("detection_range_m"))
        self._min_range_m = float(self._p("min_range_m"))
        self._fov_h_deg = float(self._p("fov_half_h_deg"))
        self._fov_v_deg = float(self._p("fov_half_v_deg"))
        self._width_px = int(self._p("image_width_px"))
        self._height_px = int(self._p("image_height_px"))
        self._ball_radius_m = float(self._p("ball_radius_m"))
        self._max_points = int(self._p("max_points_per_ball"))
        self._sigma_ref_m = float(self._p("depth_sigma_m"))
        self._sigma_ref_range_m = float(self._p("depth_sigma_ref_range_m"))
        self._per_point_frac = float(self._p("per_point_depth_frac"))
        self._publish_empty = bool(self._p("publish_empty_clouds"))
        self._stamp_warn_s = float(self._p("stamp_offset_warn_s"))
        self._frame_id = str(self._p("cloud_frame_id"))
        self._camera_offset = self._read_camera_offset()

        self._requested_hz = float(self._p("rate_hz"))
        self._delivered_hz = quantize_rate_hz(self._requested_hz)
        self._period_s = emit_period_s(self._requested_hz)
        self._delay_line = DelayLine(float(self._p("detection_delay_s")))
        # Advisory only — this node runs no differencing. It exists so the
        # slow-ball floor is a number in the log rather than a zero-detection
        # run that reads as a marginal sensor. See _log_banner.
        self._min_speed_mps = min_detectable_closing_speed_mps(
            float(self._p("bg_diff_threshold_m")), self._delivered_hz)
        # Seeded so a battery is replayable. Seed 0 means "seed from entropy",
        # which is what you want when asking whether a result is robust rather
        # than whether it reproduces. Same convention as the oracle.
        self._seed = int(self._p("seed"))
        self._rng = np.random.default_rng(self._seed if self._seed else None)

    def _read_camera_offset(self) -> np.ndarray:
        offset = [float(v) for v in (self._p("camera_offset_xyz_m") or [])]
        if len(offset) != 3:
            raise ValueError(
                "camera_offset_xyz_m must be 3 values, got %d — it has to "
                "match the base_link->camera_link static TF the launch file "
                "publishes, or the centroid round trip carries a fixed bias"
                % len(offset))
        return np.asarray(offset, dtype=np.float64)

    def _reset_state(self) -> None:
        self._last_odom = None
        self._last_emit_t = None
        self._n_published = 0
        self._n_with_ball = 0
        self._warned_no_odom = False
        self._warned_bad_quat = False
        self._warned_unmatched = False
        self._checked_stamp = False
        self._n_unmatched_msgs = 0
        self._unmatched_names: set[str] = set()
        self._warned_no_drone = False
        self._n_no_drone_msgs = 0
        self._pose_t_first = None
        self._n_pose_msgs = 0
        self._pose_rate_logged = False

    def _log_banner(self, cloud_qos: QoSProfile) -> None:
        """Say what this is, and say every number a run can be wrong about.

        The DELIVERED rate, never the requested: dead time is 0.178 s at 60 Hz
        and 0.270 s at 12 Hz, so a report that quotes the requested rate beside
        a save figure quotes the wrong constant (docs/RESULTS.md §3).

        The MINIMUM CLOSING SPEED, because below it the detector's frame
        differencing reads the ball as its own background and the run scores
        zero with every stage upstream healthy. Same reasoning as the
        FRAME_CLASS=0 and ball-prefix traps in CLAUDE.md: a silent
        misconfiguration has to be made loud at startup.

        The DRONE MODEL and BALL PREFIXES, because a world that names the drone
        something else makes this node publish nothing at all — not even the
        empty-cloud heartbeat — and the names are what the fix is compared
        against.
        """
        self.get_logger().info(
            "synthetic_depth_publisher up — a SYNTHETIC CLOUD from Gazebo "
            "truth, not a rendered sensor. "
            f"reach {self._range_m} m (an INPUT, not a result), "
            f"frustum +/-{self._fov_h_deg}x{self._fov_v_deg} deg, "
            f"{self._width_px}x{self._height_px} px, "
            f"depth sigma {self._sigma_ref_m} m at {self._sigma_ref_range_m} m "
            f"(per-point frac {self._per_point_frac}), "
            f"rate {self._requested_hz} Hz requested -> {self._delivered_hz:g} "
            "Hz delivered (quote the delivered one), "
            f"delay {self._delay_line.delay_s * 1e3:.0f} ms, "
            f"seed {self._seed or 'entropy'} -> {str(self._p('cloud_topic'))} "
            f"[{cloud_qos.reliability.name} depth={cloud_qos.depth}] "
            f"frame '{self._frame_id}' in gz FLU, "
            f"ball prefixes {self._ball_prefixes} vs drone "
            f"'{self._drone_model}'"
        )
        self.get_logger().warn(
            "min detectable closing speed %.2f m/s = diff_threshold_m %.2f m x "
            "%g Hz. A ball SLOWER than this advances less than one differencing "
            "threshold per frame, sits inside its own previous blob, and is "
            "detected on NO frame at any range — silently. Lower rate_hz (or "
            "the detector's diff_threshold_m) before running a slow scenario."
            % (self._min_speed_mps, float(self._p("bg_diff_threshold_m")),
               self._delivered_hz))

    def _declare_params(self) -> None:
        """Every tunable, with detector-facing defaults. No magic numbers."""
        self.declare_parameter("poses_topic", "/gz/dynamic_poses")
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("cloud_topic", "/oak/points")
        self.declare_parameter("cloud_frame_id", "camera_optical_frame")
        self.declare_parameter("drone_model", "iris_depth")
        self.declare_parameter("ball_name_prefixes",
                               list(DEFAULT_BALL_PREFIXES))
        self.declare_parameter("camera_offset_xyz_m", [0.10, 0.0, 0.02])
        self.declare_parameter("ball_radius_m", DEFAULT_BALL_RADIUS_M)
        self.declare_parameter("detection_range_m",
                               DEFAULT_DEPTH_SIGMA_REF_RANGE_M)
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("fov_half_h_deg", DEFAULT_FOV_HALF_H_DEG)
        self.declare_parameter("fov_half_v_deg", DEFAULT_FOV_HALF_V_DEG)
        self.declare_parameter("image_width_px", DEFAULT_IMAGE_WIDTH_PX)
        self.declare_parameter("image_height_px", DEFAULT_IMAGE_HEIGHT_PX)
        self.declare_parameter("max_points_per_ball",
                               DEFAULT_MAX_POINTS_PER_BALL)
        self.declare_parameter("depth_sigma_m", DEFAULT_DEPTH_SIGMA_M)
        self.declare_parameter("depth_sigma_ref_range_m",
                               DEFAULT_DEPTH_SIGMA_REF_RANGE_M)
        self.declare_parameter("per_point_depth_frac", 0.0)
        self.declare_parameter("detection_delay_s", 0.0)
        self.declare_parameter("rate_hz", 60.0)
        # A MIRROR of the detector's diff_threshold_m, used for nothing except
        # computing and logging the minimum detectable closing speed. This node
        # does no differencing; the detector does, and it is the pairing of the
        # two configs that creates the floor. week6_synthetic_depth.launch.py
        # overrides this from the launched detector params file so the two
        # cannot drift.
        self.declare_parameter("bg_diff_threshold_m", 0.10)
        self.declare_parameter("publish_empty_clouds", True)
        self.declare_parameter("cloud_reliable", True)
        self.declare_parameter("cloud_queue_depth", 5)
        self.declare_parameter("seed", 0)
        self.declare_parameter("stamp_offset_warn_s", 0.05)

    def _p(self, name):
        return self.get_parameter(name).value

    # ── callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _poses_cb(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return
        stamp = msg.transforms[0].header.stamp
        t_pose = self._stamp_to_sec(stamp)

        # Release anything the pipeline is still holding BEFORE any early
        # return: frames already in flight must still arrive after the ball
        # leaves the frustum, exactly as a real pipeline delivers them. Inert
        # when detection_delay_s is 0.
        for pending in self._delay_line.due(t_pose):
            self._publish(pending)
        # Measured on EVERY pose message, not only emitted frames: it is the
        # input stream's cadence that is being checked, not this node's output.
        self._measure_pose_rate(t_pose)
        if not self._frame_due(t_pose):
            return
        self._check_stamp_source(t_pose)

        drone, candidates = self._split_models(msg)
        if drone is None:
            self._note_no_drone(msg)
            return
        self._last_emit_t = t_pose

        found = self._ball_returns(drone, candidates)
        if found is None and not self._publish_empty:
            return
        points, rng_m = (np.zeros((0, 3)), None) if found is None else found
        cloud = self._make_cloud(points, stamp)
        for pending in self._delay_line.push(t_pose, (cloud, rng_m)):
            self._publish(pending)

    def _frame_due(self, t_pose: float) -> bool:
        """Pace on the POSE stamp, never on arrival.

        Gazebo publishes dynamic poses faster than any camera runs, and pacing
        on arrival would make the emitted rate a function of machine load.
        """
        if self._period_s <= 0.0 or self._last_emit_t is None:
            return True
        # A sim-time rewind (a relaunch against the same clock) would otherwise
        # wedge this comparison forever.
        if t_pose < self._last_emit_t:
            self._last_emit_t = None
            return True
        return t_pose - self._last_emit_t >= self._period_s

    def _split_models(self, msg: TFMessage):
        """(drone world position or None, every other model as name/xyz)."""
        drone = None
        candidates = []
        for tr in msg.transforms:
            t = tr.transform.translation
            if tr.child_frame_id == self._drone_model:
                drone = np.array([t.x, t.y, t.z], dtype=np.float64)
            else:
                candidates.append((tr.child_frame_id, (t.x, t.y, t.z)))
        return drone, candidates

    def _ball_returns(self, drone: np.ndarray, candidates):
        """((N, 3) camera-FLU returns, true range) or None when there are none.

        None is "the sensor reports nothing this frame" — no ball in the world,
        outside the frustum, sub-pixel, or no attitude to express it with. The
        caller cannot tell those apart, exactly as it could not with a real one.
        """
        ball = select_ball(candidates, drone, self._ball_prefixes)
        if ball is None:
            self._note_unmatched(candidates)
            return None
        odom = self._last_odom
        if odom is None:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn("poses before first odom — waiting")
            return None
        q = odom.pose.pose.orientation
        if not is_valid_quat(q.x, q.y, q.z, q.w):
            if not self._warned_bad_quat:
                self._warned_bad_quat = True
                self.get_logger().warn(
                    "odom carries no valid orientation — cannot express the "
                    "ball in the camera frame; publishing empty clouds")
            return None

        rel_body = to_body(ball - drone, quat_to_rot(q.x, q.y, q.z, q.w))
        rel_cam = camera_relative(rel_body, self._camera_offset)
        points = synthesize_ball_cloud(
            rel_cam,
            detection_range_m=self._range_m,
            min_range_m=self._min_range_m,
            fov_half_h_deg=self._fov_h_deg,
            fov_half_v_deg=self._fov_v_deg,
            width_px=self._width_px,
            height_px=self._height_px,
            ball_radius_m=self._ball_radius_m,
            max_points=self._max_points,
            sigma_ref_m=self._sigma_ref_m,
            ref_range_m=self._sigma_ref_range_m,
            per_point_frac=self._per_point_frac,
            rng_gen=self._rng,
        )
        if points is None:
            return None
        return points, float(np.linalg.norm(rel_cam))

    # ── output ───────────────────────────────────────────────────────────────

    def _make_cloud(self, points: np.ndarray, stamp) -> PointCloud2:
        """An unorganized float32 xyz cloud in the gz sensor body convention.

        `is_dense` is True because every return is finite by construction —
        there is no sky pixel and no unmatched disparity to encode as inf.
        """
        xyz = np.ascontiguousarray(np.asarray(points, dtype=np.float32)
                                   .reshape(-1, 3))
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = int(xyz.shape[0])
        msg.fields = list(_XYZ_FIELDS)
        # tobytes() writes native order, so the flag has to follow the machine
        # rather than assume x86. read_points_numpy honours it.
        msg.is_bigendian = sys.byteorder != "little"
        msg.point_step = _POINT_STEP_BYTES
        msg.row_step = _POINT_STEP_BYTES * msg.width
        msg.data = xyz.tobytes()
        msg.is_dense = True
        return msg

    def _publish(self, pending) -> None:
        cloud, rng_m = pending
        self._pub.publish(cloud)
        self._n_published += 1
        if rng_m is None:
            return
        self._n_with_ball += 1
        self.get_logger().info(
            "synthetic cloud #%d: %d returns on the ball at %.2f m"
            % (self._n_with_ball, cloud.width, rng_m),
            throttle_duration_sec=1.0)

    # ── alarms ───────────────────────────────────────────────────────────────

    def _note_no_drone(self, msg: TFMessage) -> None:
        """Say once that the configured drone model is not in the world.

        The loudest failure this node has, because it is otherwise the
        quietest: with no drone there is no camera pose, so the callback
        returns before building anything and the topic carries NOTHING — not
        even the empty-cloud heartbeat that would show the node alive. The
        battery then reports NO-FIRE on every scenario, which reads as a
        trigger that never fires. `iris_depth` vs `iris_with_standoffs` is one
        typo away.
        """
        if self._warned_no_drone or self._n_published:
            return
        self._n_no_drone_msgs += 1
        self._unmatched_names.update(tr.child_frame_id for tr in msg.transforms)
        if self._n_no_drone_msgs < UNMATCHED_WARN_AFTER:
            return
        self._warned_no_drone = True
        self.get_logger().error(
            "drone model '%s' has matched NO model in %d pose messages — this "
            "node has published nothing at all, not even an empty cloud, and "
            "every scenario will score NO-FIRE. Models actually seen: %s"
            % (self._drone_model, self._n_no_drone_msgs,
               sorted(self._unmatched_names)))

    def _measure_pose_rate(self, t_pose: float) -> None:
        """Check the 60 Hz assumption the delivered-rate claim rests on.

        `quantize_rate_hz` and the emit period are both computed against
        SCENE_BROADCASTER_HZ. If the pose stream actually runs faster, this node
        emits ABOVE the rate its banner advertises and above the rate the cell
        is labelled with — and an implied rate above the launched one is
        CLAUDE.md's decisive contamination signature. That must not be something
        only a downstream CSV can reveal.
        """
        if self._pose_rate_logged:
            return
        if self._pose_t_first is None or t_pose < self._pose_t_first:
            # First message, or a sim-time rewind: restart the window.
            self._pose_t_first = t_pose
            self._n_pose_msgs = 0
            return
        self._n_pose_msgs += 1
        if self._n_pose_msgs < POSE_RATE_SAMPLE_MSGS:
            return
        self._pose_rate_logged = True
        span = t_pose - self._pose_t_first
        if span <= 0.0:
            self.get_logger().warn(
                "%d pose messages carried no elapsed sim time — the pose "
                "cadence cannot be measured, so the delivered rate in the "
                "banner is an assumption" % self._n_pose_msgs)
            return
        self._report_pose_rate(self._n_pose_msgs / span)

    def _report_pose_rate(self, measured_hz: float) -> None:
        drift = abs(measured_hz - SCENE_BROADCASTER_HZ) / SCENE_BROADCASTER_HZ
        if drift <= POSE_RATE_TOLERANCE:
            self.get_logger().info(
                "pose stream measured at %.2f Hz over %d messages — the "
                "assumed %g Hz holds, so %g Hz delivered is a fact"
                % (measured_hz, self._n_pose_msgs, SCENE_BROADCASTER_HZ,
                   self._delivered_hz))
            return
        self.get_logger().warn(
            "pose stream measured at %.2f Hz, not the assumed %g Hz. The rate "
            "quantisation and the '%g Hz delivered' banner are both computed "
            "against %g Hz, so this node is emitting at roughly %.2f Hz — "
            "quote THAT, and treat an implied rate above the launched one as "
            "contamination until proven otherwise."
            % (measured_hz, SCENE_BROADCASTER_HZ, self._delivered_hz,
               SCENE_BROADCASTER_HZ,
               quantize_rate_hz(self._requested_hz, measured_hz)))

    def _note_unmatched(self, candidates) -> None:
        """Say once that models are present but none of them look like a ball.

        A warning and not a fatal: a world can legitimately carry dynamic models
        that are not the ball. The names are echoed because the fix is always to
        compare them against the configured prefixes.
        """
        if self._warned_unmatched or self._n_with_ball or not candidates:
            return
        self._n_unmatched_msgs += 1
        self._unmatched_names.update(n for n, _ in candidates)
        if self._n_unmatched_msgs < UNMATCHED_WARN_AFTER:
            return
        self._warned_unmatched = True
        self.get_logger().warn(
            "no ball matched in %d pose messages — the cloud is empty and "
            "every scenario will score NO-FIRE. Configured prefixes %s; models "
            "actually seen: %s"
            % (self._n_unmatched_msgs, self._ball_prefixes,
               sorted(self._unmatched_names)))

    def _check_stamp_source(self, t_pose: float) -> None:
        """Say once whether the pose stamps agree with this node's clock.

        They are formally different clocks — Gazebo's sim time reaches ROS
        through the /clock bridge — and a systematic offset would show up as a
        constant lead/lag in every velocity the tracker estimates, which reads
        exactly like a tuning problem.

        WAITS FOR THE CLOCK GUARD, and the one-shot is not spent until it does.
        Under use_sim_time the node's clock reads exactly 0 until the first
        /clock message arrives, and poses routinely arrive first — Gazebo has
        usually been up for a while before this node starts, so t_pose is
        already tens of seconds in. Evaluating then measures the node's own
        clock startup, not a stamp offset, and it fired on every launch:
        `pose stamps lead/lag the node clock by -63.789 s` against a world that
        had been running 64 s, on a stack later verified to agree to within
        milliseconds. A check that cries wolf on every launch is a check nobody
        reads, which is the whole failure mode CLAUDE.md's loud-startup rules
        exist to prevent.

        `ros_time_ns > 0` is exactly clock_guard.evaluate_clock's OK condition
        for a sim-time node, so this defers to the guard rather than inventing
        a second notion of "the clock is up". On wall time it is always true and
        the behaviour is unchanged.
        """
        if self._checked_stamp:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        self._checked_stamp = True
        offset = now_ns * 1e-9 - t_pose
        if abs(offset) > self._stamp_warn_s:
            self.get_logger().warn(
                "pose stamps lead/lag the node clock by %.3f s — every cloud "
                "carries that offset" % offset)
        else:
            self.get_logger().info(
                "pose stamps agree with the node clock (%.4f s offset)" % offset)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticDepthPublisherNode()
    install_clock_guard(node)
    clock_failed = False
    try:
        rclpy.spin(node)
    except ClockGuardError:
        # Already logged fatal by the guard; exit non-zero so a launch file or
        # shell script cannot mistake this for a clean start.
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
