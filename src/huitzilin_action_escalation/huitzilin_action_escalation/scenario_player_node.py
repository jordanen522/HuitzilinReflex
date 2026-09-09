"""Replays a synthetic keypoint sequence, so the chain runs with no camera.

Out: /action/keypoints (std_msgs/String, JSON, RELIABLE)

This is a measurement-lane node in the sense week6_oracle.launch.py uses: it
fabricates an input so the rest of the chain is exercisable. It is not a
sensor and not a model of one. THERE IS NO POSE ESTIMATOR IN THIS PROJECT,
on the camera or on the Pi, so nothing downstream of this node has ever
consumed a real body.

Stamps are emitted as play_start + frame t_s, NOT as wall-clock arrival. The
scenarios have specific velocities authored into them, and publishing at
whatever rate the timer actually fires would rescale every one of those
velocities -- under Gazebo at 0.24 RTF it would compress the motion by a
factor of four and turn a walk into a lunge.

The consequence is that input_latency_s reads about zero under the player,
because the stamp and the clock come from the same place. That number
measures the player, not a camera, and must never be quoted as a perception
latency.

The day a real keypoint source exists it gets its own launch file, the way
week6_oracle.launch.py never includes week3_perception -- two publishers on
/action/keypoints would interleave two bodies into one window.
"""

from __future__ import annotations

import json
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from huitzilin_action_escalation.scenario import load_scenario
from huitzilin_sim.clock_guard import ClockGuardError, install_clock_guard

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class ScenarioPlayerNode(Node):

    def __init__(self):
        super().__init__("scenario_player")

        self.declare_parameter("keypoints_topic", "/action/keypoints")
        self.declare_parameter("scenario", "lunge.yaml")
        self.declare_parameter("loop", True)
        self.declare_parameter("loop_gap_s", 18.0)

        def p(name):
            return self.get_parameter(name).value

        path = str(p("scenario"))
        if not os.path.isabs(path):
            path = os.path.join(
                get_package_share_directory("huitzilin_action_escalation"),
                "scenarios", path)
        if not os.path.isfile(path):
            raise ValueError(
                "scenario %r not found. An uninstalled scenario does not fail "
                "loudly downstream: the player publishes nothing, and the "
                "silence reads as a recogniser that never fires." % path)

        with open(path, "r", encoding="utf-8") as handle:
            self._scenario = load_scenario(yaml.safe_load(handle))

        self._loop = bool(p("loop"))
        self._loop_gap_s = float(p("loop_gap_s"))
        self._index = 0
        self._play_start_s = None

        self._pub = self.create_publisher(String, str(p("keypoints_topic")),
                                          RELIABLE_QOS)
        self.create_timer(1.0 / self._scenario.rate_hz, self._tick)

        self.get_logger().info(
            "scenario_player ready -- %s (%d frames at %.1f Hz, %.2f s, "
            "expect %s). Synthetic input: no camera is involved."
            % (self._scenario.name, len(self._scenario.frames),
               self._scenario.rate_hz, self._scenario.duration_s,
               self._scenario.expect))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now_s = self._now_s()
        if self._play_start_s is None:
            self._play_start_s = now_s

        if self._index >= len(self._scenario.frames):
            if not self._loop:
                return
            # The gap must exceed the recogniser cooldown, or a looping
            # scenario can never produce a second alert.
            if now_s - self._play_start_s < (self._scenario.duration_s
                                             + self._loop_gap_s):
                return
            self._index = 0
            self._play_start_s = now_s
            self.get_logger().info("replaying %s" % self._scenario.name)

        frame = dict(self._scenario.frames[self._index])
        frame["stamp_s"] = round(self._play_start_s + frame["stamp_s"], 6)
        self._pub.publish(String(data=json.dumps(frame)))
        self._index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioPlayerNode()
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
