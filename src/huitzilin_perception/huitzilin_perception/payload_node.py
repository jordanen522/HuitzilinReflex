"""Drives the LED strip and siren from /payload/alarm.

The payload is a SAFETY SIGNAL ONLY. It announces that the aircraft believes
it is under threat, so bystanders know why it is manoeuvring. It is never
used to follow, track, dazzle or harass a person (SAFETY_CASE.md section 4).

evasion_node has published /payload/alarm since Week 4 with no subscriber.
This is the consumer. It is deliberately hard to kill: SAFETY_CASE.md rates a
GPIO/payload fault Low -- log and continue -- so every hardware path degrades
to a logging no-op rather than propagating. A dead LED must not end a flight.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard
from huitzilin_perception.payload import AlarmLatch, AlarmPolicy, select_backend

# Must match evasion_node's publisher, which is RELIABLE. A BEST_EFFORT
# subscription would still match, but the alarm is a single edge rather than a
# stream -- losing it loses the whole signal.
RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class PayloadNode(Node):
    """Drives the alarm payload (buzzer/LED) from /payload/alarm.

    Every backend degrades to a null implementation rather than raising: a
    missing GPIO library or a failing channel must not take the flight stack
    down with it.
    """

    def __init__(self):
        super().__init__("payload")

        self.declare_parameter("alarm_topic", "/payload/alarm")
        self.declare_parameter("backend", "auto")   # auto | ws2812 | gpio | none
        self.declare_parameter("led_pin", 18)
        self.declare_parameter("led_count", 8)
        self.declare_parameter("siren_chip", "gpiochip0")
        self.declare_parameter("siren_line", 17)
        self.declare_parameter("min_on_s", 0.5)
        self.declare_parameter("max_on_s", 5.0)
        self.declare_parameter("stale_off_s", 3.0)
        self.declare_parameter("tick_hz", 10.0)

        def p(name):
            return self.get_parameter(name).value

        self._backend, reasons = select_backend(
            want=p("backend"),
            led_pin=int(p("led_pin")), led_count=int(p("led_count")),
            siren_chip=p("siren_chip"), siren_line=int(p("siren_line")))
        for why in reasons:
            # Warning, not error: this is the expected state on any machine
            # that is not the Pi, including every development box.
            self.get_logger().warning("payload degraded: %s" % why)

        self._latch = AlarmLatch(AlarmPolicy(min_on_s=float(p("min_on_s")),
                                             max_on_s=float(p("max_on_s")),
                                             stale_off_s=float(p("stale_off_s"))))

        self.create_subscription(Bool, p("alarm_topic"), self._on_alarm, RELIABLE)
        self.create_timer(1.0 / float(p("tick_hz")), self._tick)
        self.get_logger().info(
            "payload up on %s via %s"
            % (p("alarm_topic"), type(self._backend).__name__))

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _drive(self, action):
        if action is None:
            return
        try:
            self._backend.set(action)
        except Exception as e:
            # CompositeBackend already isolates per-channel faults; this is the
            # last line so that no payload problem can reach rclpy's executor.
            self.get_logger().warning("payload write failed: %s" % e)

    def _on_alarm(self, msg):
        self._drive(self._latch.on_message(bool(msg.data), self._now_s()))

    def _tick(self):
        self._drive(self._latch.on_tick(self._now_s()))

    def destroy_node(self):
        try:
            self._backend.set(False)
            self._backend.close()
        except Exception as exc:            # noqa: BLE001
            # Swallowed so shutdown always completes, but said out loud: this
            # is the path where the siren stays asserted after the node is
            # gone, and a bare `pass` left no evidence it had happened.
            self.get_logger().error(
                "payload backend did not shut down cleanly — the siren may "
                "still be energised: %s" % exc)
        super().destroy_node()


def main():
    rclpy.init()
    node = PayloadNode()
    install_clock_guard(node)
    clock_failed = False
    try:
        rclpy.spin(node)
    except ClockGuardError:
        clock_failed = True
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if clock_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
