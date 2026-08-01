"""Startup guard against a node silently running on the wrong clock.

Weeks 1-4 ran entirely against Gazebo, so `use_sim_time` was true everywhere and
`/clock` always existed. On hardware nothing publishes `/clock`, and rclpy's
response to that is to sit on a ROS time that never leaves zero rather than to
complain. Every timing gate in the stack -- latency_budget_s, track_timeout_s,
patrol_handoff_s, cmd_timeout_s, bg_map_ttl_s -- is denominated in whichever
clock the node happens to be on, and Gazebo runs at 0.24-0.33 RTF, so a silent
fallback changes what those numbers mean by a factor of three or four without
changing a single line of config.

The rule this enforces: a node started against the wrong clock is either
correct or loudly dead, never quietly drifting.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

CLOCK_TOPIC = "/clock"

# How long to let a sim stack come up before declaring the clock missing.
# Gazebo + SITL + ros_gz_bridge routinely take a few seconds to produce the
# first /clock message, and killing a node during that window would be worse
# than the bug this guards against.
DEFAULT_GRACE_S = 5.0


class Verdict(Enum):
    OK = "ok"
    WAIT = "wait"      # inside the grace window, keep polling
    WARN = "warn"      # suspicious but legitimate; log once, do not die
    FAIL = "fail"      # unrecoverable, take the node down


@dataclass(frozen=True)
class ClockCheck:
    verdict: Verdict
    message: str


class ClockGuardError(RuntimeError):
    """Raised when a node's clock configuration cannot produce valid timestamps."""


def evaluate_clock(use_sim_time: bool,
                   clock_publisher_count: int,
                   ros_time_ns: int,
                   elapsed_wall_s: float,
                   grace_s: float = DEFAULT_GRACE_S) -> ClockCheck:
    """Classify a node's clock situation. Pure; the caller decides what to do.

    The frozen-at-zero row is not hypothetical: week3_perception.launch.py
    documents a stack that blocks on a /clock which never advances, which is
    indistinguishable from a missing one as far as message stamps are concerned
    but very much distinguishable when you are trying to work out why.
    """
    if not use_sim_time:
        if clock_publisher_count > 0:
            return ClockCheck(
                Verdict.WARN,
                f"{CLOCK_TOPIC} has {clock_publisher_count} publisher(s) but this node "
                "runs on wall time. Expected only in HITL, where the flight controller "
                "is real and the world is simulated. Confirm this node's clock is the "
                "one you meant.",
            )
        return ClockCheck(Verdict.OK, "wall clock, no simulated time present")

    expired = elapsed_wall_s >= grace_s

    if clock_publisher_count <= 0:
        if not expired:
            return ClockCheck(
                Verdict.WAIT,
                f"use_sim_time is true; waiting for a {CLOCK_TOPIC} publisher "
                f"({elapsed_wall_s:.1f}/{grace_s:.1f} s)",
            )
        return ClockCheck(
            Verdict.FAIL,
            f"use_sim_time is true but nothing publishes {CLOCK_TOPIC} after "
            f"{elapsed_wall_s:.1f} s. Every timing gate in this node would be measured "
            "against a clock frozen at t=0. Start Gazebo (or its clock bridge), or "
            "launch with use_sim_time:=false.",
        )

    if ros_time_ns > 0:
        return ClockCheck(Verdict.OK, "simulated time is publishing and advancing")

    if not expired:
        return ClockCheck(
            Verdict.WAIT,
            f"{CLOCK_TOPIC} has a publisher but ROS time is still 0 "
            f"({elapsed_wall_s:.1f}/{grace_s:.1f} s)",
        )
    return ClockCheck(
        Verdict.FAIL,
        f"{CLOCK_TOPIC} is advertised but ROS time is still frozen at 0 after "
        f"{elapsed_wall_s:.1f} s. The publisher is up but the simulation is paused "
        "or not stepping. Unpause Gazebo, or launch with use_sim_time:=false.",
    )


def install_clock_guard(node, grace_s: float = DEFAULT_GRACE_S,
                        period_s: float = 0.5) -> None:
    """Poll the clock situation on a `node` until it resolves, then stop.

    The timer must run on a STEADY clock. Scheduling it on the node's own clock
    would mean that under the exact failure being detected -- use_sim_time with
    no /clock -- the timer never fires and the guard never runs.
    """
    from rclpy.clock import Clock, ClockType

    steady = Clock(clock_type=ClockType.STEADY_TIME)
    started_ns = steady.now().nanoseconds
    state = {"timer": None, "warned": False}

    def _poll():
        elapsed_s = (steady.now().nanoseconds - started_ns) / 1e9
        check = evaluate_clock(
            use_sim_time=bool(node.get_parameter("use_sim_time").value),
            clock_publisher_count=node.count_publishers(CLOCK_TOPIC),
            ros_time_ns=node.get_clock().now().nanoseconds,
            elapsed_wall_s=elapsed_s,
            grace_s=grace_s,
        )
        if check.verdict is Verdict.WAIT:
            return
        if check.verdict is Verdict.WARN:
            if not state["warned"]:
                node.get_logger().warning(check.message)
                state["warned"] = True
            return
        if state["timer"] is not None:
            state["timer"].cancel()
        if check.verdict is Verdict.FAIL:
            node.get_logger().fatal(check.message)
            raise ClockGuardError(check.message)
        node.get_logger().info(check.message)

    state["timer"] = node.create_timer(period_s, _poll, clock=steady)
