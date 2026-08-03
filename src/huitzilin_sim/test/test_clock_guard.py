"""Clock-guard truth table.

The property under test is that the guard is decisive: it either passes, waits
inside a bounded grace window, warns once, or kills the node. There is no path
where a node keeps running on a clock nobody intended.
"""

import pytest

from huitzilin_sim.clock_guard import (
    ClockCheck,
    ClockGuardError,
    Verdict,
    evaluate_clock,
    install_clock_guard,
)

GRACE = 5.0
BEFORE = 1.0   # inside the grace window
AFTER = 9.0    # past it


# (use_sim_time, publishers, ros_time_ns, elapsed_wall_s) -> Verdict
TRUTH_TABLE = [
    # Wall time is the hardware default and needs no simulator.
    (False, 0, 0, AFTER, Verdict.OK),
    (False, 0, 123, AFTER, Verdict.OK),
    # A /clock exists but this node ignores it: the Week 7 HITL shape. Warn only.
    (False, 1, 500, AFTER, Verdict.WARN),
    (False, 3, 500, BEFORE, Verdict.WARN),
    # Sim time, publishing and advancing: the Weeks 1-4 nominal case.
    (True, 1, 1, BEFORE, Verdict.OK),
    (True, 1, 10 ** 9, AFTER, Verdict.OK),
    # Sim time, publisher up but time frozen at zero -> paused/unstepped sim.
    (True, 1, 0, BEFORE, Verdict.WAIT),
    (True, 1, 0, AFTER, Verdict.FAIL),
    # Sim time with nothing publishing /clock at all: the hardware trap.
    (True, 0, 0, BEFORE, Verdict.WAIT),
    (True, 0, 0, AFTER, Verdict.FAIL),
]


@pytest.mark.parametrize("sim,pubs,ros_ns,elapsed,expected", TRUTH_TABLE)
def test_truth_table(sim, pubs, ros_ns, elapsed, expected):
    assert evaluate_clock(sim, pubs, ros_ns, elapsed, GRACE).verdict is expected


def test_grace_boundary_is_inclusive():
    """Exactly at the grace deadline the verdict must have resolved, not hang."""
    assert evaluate_clock(True, 0, 0, GRACE, GRACE).verdict is Verdict.FAIL
    assert evaluate_clock(True, 0, 0, GRACE - 1e-9, GRACE).verdict is Verdict.WAIT


def test_hitl_shape_never_fails():
    """A stray /clock must never kill a wall-time node.

    Week 7 runs a real flight controller against a simulated world, so a node
    deliberately on wall time will see a /clock it is not using. Failing there
    would make the guard the thing that breaks HITL.
    """
    for pubs in range(0, 5):
        for ros_ns in (0, 1, 10 ** 12):
            for elapsed in (0.0, BEFORE, GRACE, AFTER):
                v = evaluate_clock(False, pubs, ros_ns, elapsed, GRACE).verdict
                assert v is not Verdict.FAIL


def test_never_raises_and_always_classifies():
    """Totality: no input combination escapes the table, including nonsense."""
    for sim in (True, False):
        for pubs in (-1, 0, 1, 7):
            for ros_ns in (-5, 0, 1, 10 ** 15):
                for elapsed in (-2.0, 0.0, BEFORE, AFTER, 1e9):
                    for grace in (0.0, GRACE, 1e6):
                        got = evaluate_clock(sim, pubs, ros_ns, elapsed, grace)
                        assert isinstance(got, ClockCheck)
                        assert got.verdict in set(Verdict)
                        assert got.message


def test_negative_publisher_count_is_treated_as_absent():
    """count_publishers cannot go negative, but the guard must not misread it
    as 'a publisher exists' if it ever did."""
    assert evaluate_clock(True, -1, 0, AFTER, GRACE).verdict is Verdict.FAIL


def test_failure_messages_name_the_escape_hatch():
    """A fatal message that does not say how to proceed just moves the problem."""
    for pubs in (0, 1):
        msg = evaluate_clock(True, pubs, 0, AFTER, GRACE).message
        assert "use_sim_time" in msg
        assert "/clock" in msg


def test_default_grace_is_used_when_omitted():
    assert evaluate_clock(True, 0, 0, 0.0).verdict is Verdict.WAIT
    assert evaluate_clock(True, 0, 0, 60.0).verdict is Verdict.FAIL


# ── install_clock_guard ──────────────────────────────────────────────────────
#
# evaluate_clock is pure and was already covered. install_clock_guard is the
# half that has effects -- it is what stops the timer and what actually kills
# the node -- and it had no test at all. It imports rclpy.clock internally, so
# these skip on a machine without ROS rather than failing there.


class _Logger:
    def __init__(self):
        self.calls = []

    def _record(self, level):
        return lambda msg: self.calls.append((level, msg))

    def __getattr__(self, level):
        return self._record(level)


class _Clock:
    def __init__(self, ns):
        self.nanoseconds = ns

    def now(self):
        return self


class _Param:
    def __init__(self, value):
        self.value = value


class _Node:
    """The narrowest node install_clock_guard actually uses."""

    def __init__(self, use_sim_time, publishers, ros_time_ns):
        self._use_sim_time = use_sim_time
        self._publishers = publishers
        self._ros_time_ns = ros_time_ns
        self._logger = _Logger()
        self.timers = []
        self.destroyed = []

    def get_parameter(self, name):
        assert name == "use_sim_time"
        return _Param(self._use_sim_time)

    def count_publishers(self, topic):
        assert topic == "/clock"
        return self._publishers

    def get_clock(self):
        return _Clock(self._ros_time_ns)

    def get_logger(self):
        return self._logger

    def create_timer(self, period_s, callback, clock=None):
        timer = ("timer", period_s, callback, clock)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)


def _install(use_sim_time, publishers, ros_time_ns, grace_s):
    pytest.importorskip("rclpy.clock")
    node = _Node(use_sim_time, publishers, ros_time_ns)
    install_clock_guard(node, grace_s=grace_s)
    assert len(node.timers) == 1
    return node, node.timers[0][2]          # the _poll callback


def test_the_guard_timer_runs_on_a_steady_clock():
    """Scheduled on the node's own clock, the timer would never fire under the
    exact failure being detected: use_sim_time with no /clock."""
    rclpy_clock = pytest.importorskip("rclpy.clock")
    node, _ = _install(True, 0, 0, grace_s=60.0)
    clock = node.timers[0][3]
    assert clock is not None
    assert clock.clock_type is rclpy_clock.ClockType.STEADY_TIME


def test_a_missing_clock_kills_the_node():
    node, poll = _install(True, 0, 0, grace_s=0.0)
    with pytest.raises(ClockGuardError):
        poll()
    assert [lvl for lvl, _ in node._logger.calls] == ["fatal"]


def test_waiting_leaves_the_timer_running():
    """WAIT is the one verdict that must NOT stop the poll -- a sim stack is
    allowed a few seconds to produce its first /clock."""
    node, poll = _install(True, 0, 0, grace_s=60.0)
    assert poll() is None
    assert node.destroyed == []
    assert node._logger.calls == []


def test_success_stops_the_timer():
    node, poll = _install(True, 1, 42, grace_s=5.0)
    poll()
    assert node.destroyed == node.timers
    assert [lvl for lvl, _ in node._logger.calls] == ["info"]


def test_warn_stops_the_timer_too():
    """The Week 7 HITL shape, and the case that runs longest. WARN used to log
    once and return with the 0.5 s timer still live, polling for the life of
    the process."""
    node, poll = _install(False, 1, 0, grace_s=5.0)
    poll()
    assert node.destroyed == node.timers
    assert [lvl for lvl, _ in node._logger.calls] == ["warning"]


def test_a_stopped_guard_does_not_destroy_its_timer_twice():
    """destroy_timer is idempotent-by-guard, not by rclpy: a second call on a
    destroyed timer is an error. Only reachable if a poll is already queued
    when the first one resolves."""
    node, poll = _install(False, 1, 0, grace_s=5.0)
    poll()
    poll()
    assert len(node.destroyed) == 1
