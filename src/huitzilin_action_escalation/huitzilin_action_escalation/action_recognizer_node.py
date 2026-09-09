"""Scores body-joint frames for aggressive motion and requests an alert.

  /action/keypoints ---> [window -> score -> confirm/cooldown] ---> alert
                                                              ---> event
                                                              ---> status

In:  /action/keypoints (std_msgs/String, JSON, RELIABLE)
Out: /action/escalation_event (std_msgs/String, JSON, RELIABLE)
     /action/status           (std_msgs/String, JSON, RELIABLE, 1 Hz)
     /action/alert_request    (std_msgs/Bool, RELIABLE)

THIS FILE HAS NO FLIGHT OUTPUT AND NO SERVICE CLIENT, by construction rather
than by discipline: package.xml declares neither geometry_msgs nor std_srvs,
so a Twist publisher or a call to /huitzilin/arm cannot be written here
without a manifest change that shows up in review.

The alert is published on /action/alert_request and NOT on /payload/alarm.
That is not a naming preference. supervisor.py transitions PATROL -> EVADE on
/payload/alarm, and it is the only edge into EVADE in the whole state
machine, so publishing there would let a body-motion heuristic command
evasive flight.

QoS is RELIABLE on the input. Under BEST_EFFORT a dropped frame silently
shortens the confirmation window, which biases the system toward NOT alerting
in exactly the busy moments where frames are most likely to be dropped.

Thin by design: every decision lives in action_features.py and
escalation_policy.py, neither of which imports rclpy, and both of which are
unit-tested without a ROS graph.
"""

from __future__ import annotations

import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

from huitzilin_action_escalation import action_features as af
from huitzilin_action_escalation.escalation_policy import (
    EscalationLatch,
    EscalationPolicyConfig,
    EscState,
)
from huitzilin_action_escalation.pose_frame import (
    SCHEMA_VERSION,
    PoseFrameRejected,
    parse_pose_frame,
)
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

# Events and the alert are single edges, not streams. A dropped alert edge
# would be a siren that never fires, or never stops.
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class ActionRecognizerNode(Node):

    def __init__(self) -> None:
        super().__init__("action_recognizer")

        self.declare_parameter("keypoints_topic", "/action/keypoints")
        self.declare_parameter("window_s", 0.6)
        self.declare_parameter("min_joint_conf", 0.35)
        self.declare_parameter("enter_score", 0.70)
        self.declare_parameter("exit_score", 0.45)
        self.declare_parameter("min_confirm_frames", 4)
        self.declare_parameter("confirm_window_s", 0.6)
        self.declare_parameter("min_alert_s", 2.0)
        self.declare_parameter("max_alert_s", 10.0)
        self.declare_parameter("cooldown_s", 15.0)
        self.declare_parameter("stale_input_s", 2.0)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("status_period_s", 1.0)
        self.declare_parameter("publish_observations", False)
        self.declare_parameter("record_video", False)

        def p(name):
            return self.get_parameter(name).value

        if bool(p("record_video")):
            raise ValueError(
                "record_video is true. This subsystem has no video recording "
                "path and must not acquire one by configuration; raw frames "
                "are not part of its input contract.")
        if float(p("enter_score")) <= float(p("exit_score")):
            raise ValueError(
                "enter_score (%.3f) must exceed exit_score (%.3f) or the "
                "hysteresis gap inverts and the alert chatters."
                % (float(p("enter_score")), float(p("exit_score"))))

        self._feature_cfg = af.FeatureConfig(
            window_s=float(p("window_s")),
            min_joint_conf=float(p("min_joint_conf")))
        self._score_cfg = af.ScoreConfig()
        self._policy_cfg = EscalationPolicyConfig(
            enter_score=float(p("enter_score")),
            exit_score=float(p("exit_score")),
            min_confirm_frames=int(p("min_confirm_frames")),
            confirm_window_s=float(p("confirm_window_s")),
            min_alert_s=float(p("min_alert_s")),
            max_alert_s=float(p("max_alert_s")),
            cooldown_s=float(p("cooldown_s")),
            stale_input_s=float(p("stale_input_s")))

        self._latch = EscalationLatch(self._policy_cfg)
        self._windows = {}
        self._publish_observations = bool(p("publish_observations"))
        self._frames_seen = 0
        self._frames_rejected = 0
        self._last_reject = None
        self._last_latency_s = 0.0
        self._last_scores = {}

        self._event_pub = self.create_publisher(
            String, "/action/escalation_event", RELIABLE_QOS)
        self._status_pub = self.create_publisher(
            String, "/action/status", RELIABLE_QOS)
        self._alert_pub = self.create_publisher(
            Bool, "/action/alert_request", RELIABLE_QOS)
        self._obs_pub = self.create_publisher(
            String, "/action/observation", RELIABLE_QOS)

        self.create_subscription(String, str(p("keypoints_topic")),
                                 self._keypoints_cb, RELIABLE_QOS)

        self.create_timer(1.0 / float(p("tick_hz")), self._tick)
        self.create_timer(float(p("status_period_s")), self._publish_status)

        self.get_logger().info(
            "action_recognizer ready -- categories %s, enter %.2f / exit "
            "%.2f, %d frames in %.2f s, cooldown %.1f s. Scores are "
            "unvalidated heuristics, not probabilities. No flight output."
            % (", ".join(c.value for c in af.Category),
               self._policy_cfg.enter_score, self._policy_cfg.exit_score,
               self._policy_cfg.min_confirm_frames,
               self._policy_cfg.confirm_window_s,
               self._policy_cfg.cooldown_s))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _keypoints_cb(self, msg: String) -> None:
        now_s = self._now_s()
        try:
            frame = parse_pose_frame(json.loads(msg.data))
        except (ValueError, PoseFrameRejected) as exc:
            self._frames_rejected += 1
            self._last_reject = str(exc)
            # Throttled: a misconfigured upstream produces one of these per
            # frame. The count on /action/status is the durable signal.
            self.get_logger().warn("keypoint frame rejected: %s" % exc,
                                   throttle_duration_sec=5.0)
            return

        self._frames_seen += 1
        self._last_latency_s = now_s - frame.stamp_s

        window = self._windows.get(frame.subject)
        if window is None:
            window = af.FeatureWindow(self._feature_cfg)
            self._windows[frame.subject] = window
        window.push(frame)

        scores = af.score(window.features(), self._score_cfg)
        self._last_scores = {c.value: round(v, 4) for c, v in scores.items()}

        if self._publish_observations:
            self._obs_pub.publish(String(data=json.dumps({
                "schema": SCHEMA_VERSION, "t_s": round(now_s, 4),
                "subject": frame.subject, "scores": self._last_scores})))

        action = self._latch.on_scores(self._last_scores, now_s,
                                       stamp_s=frame.stamp_s)
        self._apply(action, now_s)

    def _tick(self) -> None:
        now_s = self._now_s()
        self._apply(self._latch.on_tick(now_s), now_s)

    def _apply(self, action, now_s: float) -> None:
        if action is None:
            return
        self._alert_pub.publish(Bool(data=bool(action)))
        self._publish_event("CONFIRM" if action else "CLEAR", now_s)
        self.get_logger().info(
            "%s -- category %s, %s"
            % ("ALERT" if action else "alert cleared",
               self._latch.category, self._latch.last_reason))

    def _publish_event(self, event: str, now_s: float) -> None:
        """The thresholds that fired travel inside the event.

        Same reason evade_event carries min_track_updates_used: a recorded
        event has to be attributable to the configuration it fired under, or
        events cannot be compared across runs.

        No identifier, no bounding box, no image reference, no joint
        positions. The event says what was DECIDED, never what was seen.
        """
        self._event_pub.publish(String(data=json.dumps({
            "schema": SCHEMA_VERSION,
            "t_event_s": round(now_s, 4),
            "event": event,
            "category": self._latch.category,
            "score": self._last_scores.get(self._latch.category),
            "provenance": af.PROVENANCE,
            "confirm_frames": self._policy_cfg.min_confirm_frames,
            "confirm_window_s": self._policy_cfg.confirm_window_s,
            "enter_score": self._policy_cfg.enter_score,
            "exit_score": self._policy_cfg.exit_score,
            "input_latency_s": round(self._last_latency_s, 4),
            "alert_requested": event == "CONFIRM",
            "reason": self._latch.last_reason,
            "scores": self._last_scores,
        })))

    def _publish_status(self) -> None:
        now_s = self._now_s()
        state = self._latch.state
        self._status_pub.publish(String(data=json.dumps({
            "schema": SCHEMA_VERSION,
            "t_s": round(now_s, 4),
            "state": state.value if isinstance(state, EscState) else None,
            "frames_seen": self._frames_seen,
            # Non-zero means something upstream is sending fields this
            # subsystem refuses. That belongs where topic echo can see it,
            # not only in a log nobody is tailing.
            "frames_rejected": self._frames_rejected,
            "last_reject_reason": self._last_reject,
            "alert_active": self._latch.is_alerting,
            "cooldown_remaining_s": round(
                self._latch.cooldown_remaining_s(now_s), 3),
            "input_latency_s": round(self._last_latency_s, 4),
            # Observable at runtime rather than only promised in a document.
            "recording": False,
            "reason": self._latch.last_reason,
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionRecognizerNode()
    install_clock_guard(node)
    clock_failed = False
    try:
        rclpy.spin(node)
    except ClockGuardError:
        # Already logged fatal by the guard; exit non-zero so a launch file
        # or shell script cannot mistake this for a clean start.
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
