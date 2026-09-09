"""The guard alarm: is a person inside the box, and should the siren sound?

In:  detections_topic (std_msgs/String, JSON body joints from pose_detector)
     odom_topic       (nav_msgs/Odometry, ENU, the drone's own pose)
Out: alarm_topic      (std_msgs/Bool, consumed by alert_signal)
     status_topic     (std_msgs/String, JSON, 1 Hz)
Srv: arm_service      (std_srvs/SetBool) -- the ADT-style switch

THE ALARM NEVER COMMANDS FLIGHT. This node publishes a boolean and a status
string. It holds no MAVLink connection, imports no geometry_msgs and so
cannot construct a Twist, and names no flight topic; test_isolation.py
asserts all three against the source rather than trusting this paragraph.

The arm service is /guard/arm and is NOT interchangeable with the motor
arming service, which is a different subsystem entirely. Arming the alarm on
a disarmed aircraft is normal -- that is a guard sitting on the bench,
watching.

WHAT THIS ASSERTS, AND WHAT IT DOES NOT. Presence only: "a person is inside
the box". Nothing about who they are, what they are doing, or whether they
ought to be there. That is what makes it usable for a curfew on a block
without becoming a judgement about the individual crossing it. Identity is
not withheld by policy here, it is unavailable: pose_frame rejects any frame
carrying a face point, an image or an embedding, so no identifying data
reaches this node to be reasoned about.

While DISARMED the node still parses detections and reports person_in_box in
its status, and simply fires no alarm. Reporting an empty box while a person
stands in it would be a lie told by the only instrument an operator has.
"""

from __future__ import annotations

import json
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from huitzilin_guard import world
from huitzilin_guard.pose_frame import PoseFrameRejected, parse_pose_frame
from huitzilin_guard.presence import PresenceLatch, PresencePolicy
from huitzilin_sim.box import box_from_params
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class GuardNode(Node):

    def __init__(self):
        super().__init__("guard")

        self.declare_parameter("detections_topic", "/guard/detections")
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("alarm_topic", "/guard/alarm_request")
        self.declare_parameter("status_topic", "/guard/status")
        self.declare_parameter("arm_service", "/guard/arm")

        # The guarded rectangle, ENU metres relative to the arming point.
        # These five must match the patrol block in guard.yaml exactly;
        # test_guard_params.py asserts it, because a drone that patrols one
        # rectangle and guards another looks healthy while guarding nothing.
        self.declare_parameter("box_min_x", 0.0)
        self.declare_parameter("box_max_x", 5.0)
        self.declare_parameter("box_min_y", 0.0)
        self.declare_parameter("box_max_y", 5.0)
        self.declare_parameter("box_alt_m", 2.0)

        # Consecutive in-box detections before the siren sounds.
        self.declare_parameter("confirm_frames", 3)
        # Seconds of empty box before it stops. Not one empty frame: a
        # motionless person is what the detector misses.
        self.declare_parameter("clear_after_s", 3.0)
        # Seconds. A brief intrusion must still be perceptible.
        self.declare_parameter("min_alert_s", 3.0)
        # Seconds. Stuck-detector guard, long on purpose: an intruder
        # standing still must not be able to outwait the alarm.
        self.declare_parameter("max_alert_s", 300.0)
        # Seconds without a frame before a SOUNDING alarm gives up. See
        # presence.py: this is a dead-man, not a health check.
        self.declare_parameter("stale_input_s", 5.0)

        # Hz. Drives the time-based transitions above; they must happen when
        # no detections are arriving at all, which is the case that matters.
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("status_hz", 1.0)

        # Disarmed at startup, like any alarm panel. A guard that armed
        # itself on boot would sound during setup, and the person silencing
        # it is the person it exists to detect.
        self.declare_parameter("start_armed", False)

        def p(name):
            return self.get_parameter(name).value

        # Raises BoxError naming the offending axis. Deliberately before any
        # publisher exists: a bad box must stop the node, not run inside it.
        self.box = box_from_params(p)

        self._latch = PresenceLatch(PresencePolicy(
            confirm_frames=int(p("confirm_frames")),
            clear_after_s=float(p("clear_after_s")),
            min_alert_s=float(p("min_alert_s")),
            max_alert_s=float(p("max_alert_s")),
            stale_input_s=float(p("stale_input_s"))))

        self.armed = bool(p("start_armed"))
        self._position_enu = None
        self._quat_xyzw = None
        self._person_in_box = False
        self._frames_seen = 0
        self._frames_rejected = 0
        self._frames_unplaceable = 0

        self._alarm_pub = self.create_publisher(
            Bool, str(p("alarm_topic")), RELIABLE_QOS)
        self._status_pub = self.create_publisher(
            String, str(p("status_topic")), RELIABLE_QOS)

        self.create_subscription(String, str(p("detections_topic")),
                                 self._detection_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, str(p("odom_topic")),
                                 self._odom_cb, 10)
        self.create_service(SetBool, str(p("arm_service")), self._srv_arm)

        self.create_timer(1.0 / float(p("tick_hz")), self._tick)
        self.create_timer(1.0 / float(p("status_hz")), self._publish_status)

        # Resting state once at startup, so a subscriber that comes up later
        # sees "off" rather than nothing at all.
        self._alarm_pub.publish(Bool(data=False))

        self.get_logger().info(
            "guard up: box x [%g, %g] y [%g, %g] at %g m, %.1f x %.1f m, "
            "armed=%s. Presence only -- no identity, no recording."
            % (self.box.min_x, self.box.max_x, self.box.min_y, self.box.max_y,
               self.box.alt_m, self.box.width_m, self.box.depth_m, self.armed))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _srv_arm(self, req, resp):
        """Turn the alarm on or off. The whole operator interface."""
        want = bool(req.data)
        if want != self.armed:
            # Reset on either edge. Arming fresh means the confirm window
            # applies after arming rather than firing instantly on somebody
            # who was already standing there; disarming clears the state so a
            # later arm cannot inherit a stale in-box run.
            self._latch.reset()
            if not want:
                self._drive(False, force=True)
        self.armed = want
        resp.success = True
        resp.message = "guard armed" if want else "guard disarmed"
        self.get_logger().info(resp.message)
        return resp

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self._position_enu = (pos.x, pos.y, pos.z)
        self._quat_xyzw = (ori.x, ori.y, ori.z, ori.w)

    def _detection_cb(self, msg) -> None:
        self._frames_seen += 1

        try:
            frame = parse_pose_frame(json.loads(msg.data))
        except (ValueError, PoseFrameRejected) as exc:
            # frames_rejected is the privacy telemetry: non-zero means
            # something upstream is sending fields this subsystem refuses.
            self._frames_rejected += 1
            self.get_logger().warn("rejected a detection frame: %s" % exc,
                                   throttle_duration_sec=10.0)
            return

        point = world.person_point_optical(
            {name: (j.x, j.y, j.z) for name, j in frame.joints.items()})
        if point is None or self._position_enu is None:
            # No odometry means no way to say WHERE this person is. Counting
            # it as "not in the box" would be a guess in the unsafe
            # direction, so it is dropped and counted instead.
            self._frames_unplaceable += 1
            self.get_logger().warn(
                "a person was detected but could not be placed in the world "
                "(no odometry yet); not testing the box",
                throttle_duration_sec=10.0)
            return

        try:
            x, y, _z = world.camera_point_to_world(
                point, self._position_enu, self._quat_xyzw)
        except world.WorldTransformError as exc:
            self._frames_unplaceable += 1
            self.get_logger().warn(str(exc), throttle_duration_sec=10.0)
            return

        self._person_in_box = self.box.contains_xy(x, y)

        # Fed only while armed. A disarmed panel still reports what it sees
        # -- see _publish_status -- but must not accumulate toward a siren.
        if self.armed:
            self._drive(self._latch.on_frame(self._person_in_box,
                                             self._now_s()))

    def _tick(self) -> None:
        if not self.armed:
            return
        self._drive(self._latch.on_tick(self._now_s()))

    def _drive(self, action, force: bool = False) -> None:
        """Log any edge, then republish the current alarm state.

        The state is published EVERY time this runs, not only on an edge,
        and that is required rather than wasteful. alert_signal holds its own
        dead-man and clears the siren when this stream stops, which is what
        silences a siren if this node dies mid-alarm. An edge-only publisher
        is indistinguishable from a dead one, so the siren would clear itself
        stale_off_s after every alarm started while the guard still believed
        it was sounding.

        `action` is the latch's edge: True, False, or None for no change.
        """
        if action is not None:
            if action:
                self.get_logger().info(
                    "ALARM ON -- a person is inside the guarded area")
            else:
                self.get_logger().info(
                    "ALARM OFF: %s"
                    % (self._latch.last_stop_reason
                       or ("disarmed" if force else "clear")))
        self._alarm_pub.publish(Bool(data=bool(self._latch.is_on)))

    def _publish_status(self) -> None:
        """The 1 Hz heartbeat. Observable with `ros2 topic echo`.

        Use --full-length when reading it: echo truncates long strings, and a
        truncated JSON payload reads as a malformed one.
        """
        secs = self._latch.seconds_since_input(self._now_s())
        self._status_pub.publish(String(data=json.dumps({
            "armed": bool(self.armed),
            "alarm": bool(self._latch.is_on),
            "person_in_box": bool(self._person_in_box),
            "frames_seen": int(self._frames_seen),
            "frames_rejected": int(self._frames_rejected),
            "frames_unplaceable": int(self._frames_unplaceable),
            "consecutive_in_box": int(self._latch.consecutive_in_box),
            # None is the normal resting state on an empty box: the detector
            # publishes only when it sees somebody. Not a fault on its own.
            "secs_since_detection": (None if secs is None
                                     else round(float(secs), 2)),
            "have_odom": self._position_enu is not None,
            # Stated in the heartbeat rather than only in a document, so the
            # property is checkable at runtime by anyone with a terminal.
            "recording": False,
            "last_stop_reason": self._latch.last_stop_reason,
            "box": [self.box.min_x, self.box.max_x,
                    self.box.min_y, self.box.max_y],
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuardNode()
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
