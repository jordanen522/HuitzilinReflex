"""ROS wrapper around the supervisor transition table.

Thin by design: every decision lives in supervisor.py, which imports no rclpy
and is unit-tested exhaustively. This file only samples topics into an
Observation, applies the decision, and logs.

Node health is learned from message ages on topics that already exist. No
status topic is invented -- architecture.md's "node statuses" input is
satisfied by the fact that a node which has stopped publishing is a node that
has stopped.
"""

import json
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard
from huitzilin_sim.supervisor import (
    Limits,
    Observation,
    State,
    edge_only,
    next_state,
    watched_topics,
)

RELIABLE = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)
# The depth cloud is published best-effort. A RELIABLE subscription would
# silently never match it, and the supervisor would then read a perfectly
# healthy camera as a permanent sensor dropout.
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class SupervisorNode(Node):

    def __init__(self):
        super().__init__("supervisor")

        self.declare_parameter("tick_hz", 1.0)
        for name, default in Limits().__dict__.items():
            self.declare_parameter(name, default)
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("state_topic", "/huitzilin/state")
        self.declare_parameter("cloud_topic", "/oak/points")
        self.declare_parameter("patrol_state_topic", "/huitzilin/patrol_state")
        self.declare_parameter("cmd_vel_topic", "/huitzilin/cmd_vel")
        self.declare_parameter("alarm_topic", "/payload/alarm")
        self.declare_parameter("patrol_service", "/huitzilin/start_patrol")
        self.declare_parameter("mode_service", "/huitzilin/set_mode")
        self.declare_parameter("bridge_node", "mav_bridge")

        self._limits = Limits(**{k: float(self.get_parameter(k).value)
                                 for k in Limits().__dict__})

        self._state = State.DISARMED
        self._entered_s = self._now_s()
        self._last = {}          # topic key -> receipt time, seconds
        self._fc = {}            # last parsed /huitzilin/state JSON
        self._alarm_on = False

        def p(name):
            return self.get_parameter(name).value

        self.create_subscription(Odometry, p("odom_topic"),
                                 lambda _m: self._stamp("odom"), 10)
        self.create_subscription(String, p("state_topic"), self._on_state, 10)
        self.create_subscription(PointCloud2, p("cloud_topic"),
                                 lambda _m: self._stamp("cloud"), BEST_EFFORT)
        self.create_subscription(String, p("patrol_state_topic"),
                                 lambda _m: self._stamp("patrol_state"), 10)
        self.create_subscription(Twist, p("cmd_vel_topic"),
                                 lambda _m: self._stamp("cmd_vel"), 10)
        self.create_subscription(Bool, p("alarm_topic"), self._on_alarm, RELIABLE)

        self._patrol_cli = self.create_client(SetBool, p("patrol_service"))
        self._mode_cli = self.create_client(Trigger, p("mode_service"))
        self._bridge_param_cli = self.create_client(
            SetParameters, "/%s/set_parameters" % p("bridge_node"))

        self.create_timer(1.0 / float(p("tick_hz")), self._tick)

        # Which faults are actually live. A watch whose timeout is 0 is off, and
        # that has to be visible here rather than inferred from the yaml -- an
        # operator reading "watching: odom cloud" immediately knows cmd_vel is
        # not being monitored, instead of discovering it during a failure.
        armed = watched_topics(self._limits)
        off = [k for k in ("odom", "patrol_state", "cloud", "cmd_vel")
               if k not in armed]
        self.get_logger().info(
            "supervisor up in %s | watching: %s | disabled: %s"
            % (self._state.value,
               " ".join("%s(%.1fs)" % (k, v) for k, v in armed.items()) or "none",
               " ".join(off) or "none"))

    # -- plumbing ------------------------------------------------------------

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp(self, key):
        self._last[key] = self._now_s()

    def _on_state(self, msg):
        self._stamp("state")
        try:
            self._fc = json.loads(msg.data)
        except (ValueError, TypeError):
            # Malformed telemetry is not by itself a reason to bring the
            # aircraft down; the staleness check already covers a bridge that
            # has actually stopped.
            self.get_logger().warning("unparseable /huitzilin/state payload")

    def _on_alarm(self, msg):
        self._stamp("alarm")
        self._alarm_on = bool(msg.data)

    def _observe(self):
        now = self._now_s()
        n, e = self._fc.get("n"), self._fc.get("e")
        radius = None if n is None or e is None else (n * n + e * e) ** 0.5
        return Observation(
            now_s=now,
            armed=self._fc.get("armed"),
            mode=self._fc.get("mode"),
            alt_m=self._fc.get("alt"),
            radius_m=radius,
            batt_v=self._fc.get("batt_v"),
            fc_failsafe=self._fc.get("fc_failsafe"),
            landed=self._fc.get("armed") is False,
            alarm_on=self._alarm_on,
            state_age_s=now - self._entered_s,
            ages={k: now - t for k, t in self._last.items()},
        )

    # -- the tick ------------------------------------------------------------

    def _tick(self):
        previous = self._state
        decision = edge_only(previous,
                             next_state(previous, self._observe(), self._limits))
        if decision.state is not previous:
            self._state = decision.state
            self._entered_s = self._now_s()
            log = (self.get_logger().error if decision.fault
                   else self.get_logger().info)
            log("%s -> %s (%s)" % (previous.value, decision.state.value,
                                   decision.reason))

        if decision.start_patrol is not None:
            self._call_patrol(decision.start_patrol)
        if decision.set_mode is not None:
            self._request_mode(decision.set_mode)

    def _call_patrol(self, running):
        if not self._patrol_cli.service_is_ready():
            self.get_logger().warning("start_patrol service not ready")
            return
        req = SetBool.Request()
        req.data = bool(running)
        self._patrol_cli.call_async(req)

    def _request_mode(self, mode):
        """Ask mav_bridge for a flight mode.

        /huitzilin/set_mode is a Trigger that reads the mode from mav_bridge's
        own parameter, so the parameter has to be set first. Both calls are
        async on purpose: the bridge's set_mode blocks its single-threaded
        executor for up to 10 s, and a supervisor that blocked alongside it
        would stop monitoring faults for exactly as long.
        """
        if self._bridge_param_cli.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [
                Parameter("mode", Parameter.Type.STRING, mode).to_parameter_msg()]
            self._bridge_param_cli.call_async(req)
        if self._mode_cli.service_is_ready():
            self._mode_cli.call_async(Trigger.Request())
        else:
            self.get_logger().warning("set_mode service not ready (wanted %s)" % mode)


def main():
    rclpy.init()
    node = SupervisorNode()
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
