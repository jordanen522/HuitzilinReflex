"""Drives the warning lights and siren from the escalation alert request.

In:  /action/alert_request (std_msgs/Bool, RELIABLE)
Out: /action/alert_state   (std_msgs/Bool, RELIABLE)

Separate from action_recognizer on purpose. The recogniser decides; this node
actuates, and holds its own dead-man so that a recogniser which dies mid-alert
cannot leave a siren latched on. That is the same split payload_node has from
evasion_node, and for the same reason.

There is no GPIO here. See alert_sink.py: payload_node already owns the LED
and siren lines, Linux GPIO line requests are exclusive, and a second process
taking them would silently disable the projectile alarm.

QoS is RELIABLE to match the recogniser's publisher. The alert is a single
edge rather than a stream, so a BEST_EFFORT subscription that dropped the
clear would hold the alert until the dead-man expired.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from huitzilin_action_escalation.alert_sink import (
    AlertLatch,
    AlertLatchPolicy,
    select_alert_sink,
)
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class AlertSignalNode(Node):

    def __init__(self):
        super().__init__("alert_signal")

        self.declare_parameter("alert_topic", "/action/alert_request")
        self.declare_parameter("state_topic", "/action/alert_state")
        self.declare_parameter("backend", "sim")
        self.declare_parameter("min_on_s", 2.0)
        self.declare_parameter("max_on_s", 12.0)
        self.declare_parameter("stale_off_s", 3.0)
        self.declare_parameter("tick_hz", 10.0)

        def p(name):
            return self.get_parameter(name).value

        self._state_pub = self.create_publisher(
            Bool, str(p("state_topic")), RELIABLE_QOS)

        self._sink, reasons = select_alert_sink(
            str(p("backend")),
            publish=lambda on: self._state_pub.publish(Bool(data=bool(on))))
        for reason in reasons:
            # error, not warning: unlike a missing library this is a
            # deliberate refusal, and it means the alert is inert.
            self.get_logger().error(reason)

        self._latch = AlertLatch(AlertLatchPolicy(
            min_on_s=float(p("min_on_s")),
            max_on_s=float(p("max_on_s")),
            stale_off_s=float(p("stale_off_s"))))

        self.create_subscription(Bool, str(p("alert_topic")),
                                 self._alert_cb, RELIABLE_QOS)
        self.create_timer(1.0 / float(p("tick_hz")), self._tick)

        # Publish the resting state once, so a subscriber that comes up later
        # sees "off" rather than nothing at all.
        self._state_pub.publish(Bool(data=False))

        self.get_logger().info(
            "alert_signal up on %s via backend %s -- min_on %.1f s, "
            "dead-man %.1f s"
            % (str(p("alert_topic")), str(p("backend")),
               float(p("min_on_s")), float(p("max_on_s"))))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _alert_cb(self, msg):
        self._drive(self._latch.on_message(bool(msg.data), self._now_s()))

    def _tick(self):
        self._drive(self._latch.on_tick(self._now_s()))

    def _drive(self, action):
        if action is None:
            return
        self._sink.set(bool(action))
        self.get_logger().info("alert %s" % ("ON" if action else "OFF"))

    def destroy_node(self):
        try:
            self._sink.set(False)
            self._sink.close()
        except Exception as exc:            # noqa: BLE001
            # Shutdown must continue, but this is the one failure at shutdown
            # worth hearing about: the siren may still be energised.
            self.get_logger().error("alert sink shutdown failed: %s" % exc)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlertSignalNode()
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
