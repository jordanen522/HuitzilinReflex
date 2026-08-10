"""
oracle_detector_node.py — ground-truth stand-in for detector_node.

Publishes /threat/centroid from Gazebo truth instead of from a point cloud,
with detection range, rate, noise and dropout as parameters. Everything
downstream (evasion_node's tracker, trigger, dodge command, the bridge,
ArduPilot, the airframe) is untouched and unaware.

WHAT IT IS FOR. The 14 m/s failure is a sensing bound: the depth detector
first reports the ball at ~3.2-3.6 m, which at 20 m/s is 0.17 s of warning --
consumed entirely by three-frame confirmation plus pipeline, leaving nothing
for the vehicle to move in. So no sim run at 20 m/s can currently tell you
whether the tracker, the trigger, or the airframe would have coped, because
the dodge never fires at all. This node removes the sensing bound on purpose,
so those questions become measurable while the real answer -- what range the
OAK-D mono pair actually delivers -- is still a hardware measurement.

MUTUALLY EXCLUSIVE WITH detector_node. Both publish /threat/centroid; running
both gives the tracker two uncorrelated views of the same ball. The launch
files enforce this by never starting both, not by any interlock here.

HOW TO READ ITS RESULTS. detection_range_m is an INPUT. A 20 m/s dodge scored
here is a statement about the tracker and the airframe, conditional on a
sensor that can see 12 m. It is not evidence that such a sensor exists. Every
report must carry the range it was run at.

Frames: the drone->ball vector is taken from Gazebo (both models, so the
world/odom origin offset cancels) and rotated into base_link with the odom
attitude -- the same quaternion evasion_node uses to rotate it back, so any
attitude error round-trips out instead of injecting a bias the real pipeline
would not have.

Stamps come from the Gazebo pose message, never from arrival time: the filter
differentiates consecutive stamps, and arrival is not emission (CLAUDE.md).
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

from huitzilin_perception.cloud_geometry import is_valid_quat, quat_to_rot
from huitzilin_perception.oracle import (
    DEFAULT_BALL_PREFIXES,
    DelayLine,
    select_ball,
    synthesize,
)
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

# Matches gz_pose_bridge's publisher: a RELIABLE subscription would not match
# it at all, and the topic would look alive while delivering nothing.
POSE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# detector_node publishes centroids RELIABLE; evasion_node subscribes RELIABLE.
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class OracleDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("oracle_detector")

        self.declare_parameter("poses_topic", "/gz/dynamic_poses")
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("centroid_topic", "/threat/centroid")
        self.declare_parameter("drone_model", "iris_depth")
        # A LIST, because this repo has two ball-naming conventions and the
        # scoring harness uses the one the oracle originally did not match.
        # See DEFAULT_BALL_PREFIXES in oracle.py for what that cost.
        self.declare_parameter("ball_name_prefixes",
                               list(DEFAULT_BALL_PREFIXES))
        self.declare_parameter("detection_range_m", 12.0)
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("fov_half_angle_deg", 45.0)
        # Seconds of SIM time between the world moving and the centroid being
        # publishable: exposure + readout + transfer + detection. 0.0 = OFF =
        # the zero-latency oracle every recorded result was flown against, so
        # nothing already measured changes. See DelayLine for what is and is
        # not modelled -- in particular the stamp stays at exposure time.
        self.declare_parameter("detection_delay_s", 0.0)
        self.declare_parameter("rate_hz", 14.5)
        self.declare_parameter("noise_std_m", 0.15)
        # (forward, left, up) sigma in base_link, which is how a stereo
        # sensor's error actually decomposes. All-zero means "unspecified" and
        # falls back to the isotropic noise_std_m; for a genuinely noiseless
        # oracle set noise_std_m to 0.0 instead.
        #
        # NOT declared as an empty list: an empty list carries no element type,
        # so ROS 2 declares the parameter UNINITIALIZED and get_parameter()
        # raises ParameterUninitializedException at startup. The same is true
        # of `[]` written in the yaml, so both must carry three numbers.
        self.declare_parameter("noise_std_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("dropout_prob", 0.0)
        self.declare_parameter("seed", 0)
        self.declare_parameter("stamp_offset_warn_s", 0.05)

        self._drone_model = str(self._p("drone_model"))
        # Empty is rejected rather than tolerated: it matches no model, so the
        # node would run, log happily, publish nothing, and every scenario
        # would score NO-FIRE — indistinguishable from a trigger that never
        # fires. Failing at startup is the only way that mistake stays visible.
        self._ball_prefixes = [str(p) for p in (self._p("ball_name_prefixes")
                                                or []) if str(p)]
        if not self._ball_prefixes:
            raise ValueError(
                "ball_name_prefixes is empty — the oracle would match no ball "
                "and silently report zero detections")
        self._range = float(self._p("detection_range_m"))
        self._min_range = float(self._p("min_range_m"))
        self._fov = float(self._p("fov_half_angle_deg"))
        self._delay_line = DelayLine(float(self._p("detection_delay_s")))
        rate_hz = float(self._p("rate_hz"))
        self._period = 1.0 / rate_hz if rate_hz > 0.0 else 0.0
        self._dropout = float(self._p("dropout_prob"))
        self._stamp_warn = float(self._p("stamp_offset_warn_s"))

        xyz = [float(v) for v in (self._p("noise_std_xyz_m") or [])]
        if xyz and len(xyz) != 3:
            raise ValueError(
                "noise_std_xyz_m must be 3 values, got %d" % len(xyz))
        self._noise = (np.asarray(xyz, dtype=np.float64)
                       if any(v > 0.0 for v in xyz)
                       else np.full(3, float(self._p("noise_std_m"))))

        # Seeded so a battery is replayable. Seed 0 means "seed from entropy",
        # which is what you want when asking whether a result is robust rather
        # than whether it reproduces.
        seed = int(self._p("seed"))
        self._rng = np.random.default_rng(seed if seed else None)

        self._last_odom = None
        self._last_emit_t = None
        self._warned_no_odom = False
        self._warned_bad_quat = False
        self._checked_stamp = False
        self._n_published = 0
        # Unmatched-name alarm. The prefix bug published zero centroids while
        # logging nothing wrong, so a whole sweep scored 0/8 NO-FIRE before
        # anyone knew the oracle was blind. Counted in messages that carried
        # SOME non-drone model, so the quiet stretch between runs (no ball in
        # the world at all) can never trip it.
        self._n_unmatched_msgs = 0
        self._unmatched_names = set()
        self._warned_unmatched = False

        self._pub = self.create_publisher(
            PointStamped, str(self._p("centroid_topic")), RELIABLE_QOS)
        self.create_subscription(Odometry, str(self._p("odom_topic")),
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, str(self._p("poses_topic")),
                                 self._poses_cb, POSE_QOS)

        self.get_logger().info(
            "oracle_detector up — GROUND TRUTH, not a detector. "
            f"range {self._range} m (an INPUT, not a result), "
            f"fov +/-{self._fov} deg, {rate_hz} Hz, "
            f"noise {np.round(self._noise, 3).tolist()} m, "
            # After the noise triple on purpose: EXPECT_BANNER patterns in the
            # lab queues anchor on that field, and inserting ahead of it would
            # abort every queued cell.
            f"delay {self._delay_line.delay_s * 1e3:.0f} ms, "
            f"dropout {self._dropout}, seed {seed or 'entropy'}, "
            f"ball prefixes {self._ball_prefixes} vs drone "
            f"'{self._drone_model}'"
        )

    def _p(self, name):
        return self.get_parameter(name).value

    # ── callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _release_due(self, t_now: float) -> None:
        for out, rng_m in self._delay_line.due(t_now):
            self._publish(out, rng_m)

    def _publish(self, out: PointStamped, rng_m: float) -> None:
        self._pub.publish(out)
        self._n_published += 1
        self.get_logger().info(
            "oracle centroid #%d at %.2f m" % (self._n_published, rng_m),
            throttle_duration_sec=1.0)

    def _poses_cb(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return

        # Release anything the pipeline is still holding BEFORE any of the
        # early returns below. Frames already in flight must still arrive after
        # the ball leaves the frustum, or after a tick the rate limiter skips,
        # exactly as a real pipeline delivers them. Inert when delay is 0.
        self._release_due(self._stamp_to_sec(msg.transforms[0].header.stamp))

        drone = None
        candidates = []
        for tr in msg.transforms:
            name = tr.child_frame_id
            t = tr.transform.translation
            if name == self._drone_model:
                drone = np.array([t.x, t.y, t.z])
            else:
                candidates.append((name, (t.x, t.y, t.z)))
        if drone is None:
            return
        # Between runs there is legitimately no ball in the world, so None here
        # is "nothing to report this frame", not an error.
        ball = select_ball(candidates, drone, self._ball_prefixes)
        if ball is None:
            self._note_unmatched(candidates)
            return

        stamp = msg.transforms[0].header.stamp
        t_pose = self._stamp_to_sec(stamp)
        self._check_stamp_source(t_pose)

        # Rate limit on the POSE stamp, not on arrival: Gazebo publishes
        # dynamic poses far faster than any camera, and pacing on arrival
        # would make the emitted rate a function of machine load.
        if self._period > 0.0 and self._last_emit_t is not None:
            # A sim-time rewind (a relaunch against the same clock) would
            # otherwise wedge this comparison forever.
            if t_pose < self._last_emit_t:
                self._last_emit_t = None
            elif t_pose - self._last_emit_t < self._period:
                return

        odom = self._last_odom
        if odom is None:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn("poses before first odom — waiting")
            return
        q = odom.pose.pose.orientation
        if not is_valid_quat(q.x, q.y, q.z, q.w):
            if not self._warned_bad_quat:
                self._warned_bad_quat = True
                self.get_logger().warn(
                    "odom carries no valid orientation — cannot express the "
                    "ball in base_link; oracle inactive")
            return

        z_bl = synthesize(
            ball - drone,
            quat_to_rot(q.x, q.y, q.z, q.w),
            detection_range_m=self._range,
            min_range_m=self._min_range,
            fov_half_angle_deg=self._fov,
            noise_std_xyz=self._noise,
            dropout_prob=self._dropout,
            rng_gen=self._rng,
        )
        if z_bl is None:
            return

        self._last_emit_t = t_pose
        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        out.point.x, out.point.y, out.point.z = (float(v) for v in z_bl)
        # True range captured at GENERATION, so the throttled log reports where
        # the ball was when the frame was taken, not where it is when the frame
        # lands. With a delay configured those differ by delay * closing speed
        # — 0.6 m at 30 ms and 20 m/s.
        rng_m = float(np.linalg.norm(ball - drone))
        for pending_out, pending_rng in self._delay_line.push(t_pose,
                                                              (out, rng_m)):
            self._publish(pending_out, pending_rng)

    # Roughly 20 s of Gazebo pose messages. Long enough that arming, takeoff
    # and the first patrol leg pass without comment; short enough that a
    # misconfigured prefix is loud inside the first scenario rather than after
    # a 15-minute battery has finished scoring an unarmed oracle.
    UNMATCHED_WARN_AFTER = 2000

    def _note_unmatched(self, candidates) -> None:
        """Say once that models are present but none of them look like a ball.

        Deliberately a warning and not a fatal: a world can legitimately carry
        dynamic models that are not the ball, and the node must not refuse to
        run because of one. The names are echoed because the fix is always to
        compare them against the configured prefixes.
        """
        if self._warned_unmatched or self._n_published or not candidates:
            return
        self._n_unmatched_msgs += 1
        self._unmatched_names.update(n for n, _ in candidates)
        if self._n_unmatched_msgs < self.UNMATCHED_WARN_AFTER:
            return
        self._warned_unmatched = True
        self.get_logger().warn(
            "oracle has matched NO ball in %d pose messages — it is publishing "
            "nothing and every scenario will score NO-FIRE. Configured "
            "prefixes %s; models actually seen: %s"
            % (self._n_unmatched_msgs, self._ball_prefixes,
               sorted(self._unmatched_names)))

    def _check_stamp_source(self, t_pose: float) -> None:
        """Say once whether the pose stamps agree with this node's clock.

        They are formally different clocks -- Gazebo's sim time reaches ROS
        through the /clock bridge -- and a systematic offset between them
        would show up as a constant lead/lag in every velocity the tracker
        estimates, which is exactly the kind of error that reads as a tuning
        problem.
        """
        if self._checked_stamp:
            return
        self._checked_stamp = True
        offset = self.get_clock().now().nanoseconds * 1e-9 - t_pose
        if abs(offset) > self._stamp_warn:
            self.get_logger().warn(
                "pose stamps lead/lag the node clock by %.3f s — every "
                "oracle measurement carries that offset" % offset)
        else:
            self.get_logger().info(
                "pose stamps agree with the node clock (%.4f s offset)" % offset)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OracleDetectorNode()
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
