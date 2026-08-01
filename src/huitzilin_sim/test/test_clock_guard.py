"""Clock-guard truth table.

The property under test is that the guard is decisive: it either passes, waits
inside a bounded grace window, warns once, or kills the node. There is no path
where a node keeps running on a clock nobody intended.
"""

import pytest

from huitzilin_sim.clock_guard import (
    ClockCheck,
    Verdict,
    evaluate_clock,
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
