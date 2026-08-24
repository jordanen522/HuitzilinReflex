"""Setpoint arbitration: who owns the vehicle, and for exactly how long.

These rules used to live inside a timer callback, so the only way to exercise
them was to fly. The one that actually bites is the handback: ArduPilot coasts
on the last velocity setpoint it was given, so a dodge that expires without a
zero leaves the aircraft drifting at dodge speed until patrol's position
setpoints win the argument.

The evade path now has three callers -- receipt, fast timer, watchdog tick --
and "exactly one handback zero" has to survive all three.
"""

import pytest

from huitzilin_sim.cmd_router import (
    ZERO,
    Action,
    Route,
    RouterState,
    route,
)

TIMEOUT = 0.7
EVADE = (1.0, -2.0, 0.5, 0.0)
CRUISE = (3.0, 0.0, 0.0, 0.1)


def _state(**kw):
    return RouterState(**kw)


# priority

def test_fresh_evade_beats_fresh_cmd_vel():
    """The whole point of a separate evade topic."""
    r = route(_state(last_evade=EVADE, last_evade_t=10.0,
                     last_cmd=CRUISE, last_cmd_t=10.0), 10.0, TIMEOUT)
    assert r.action is Action.EVADE
    assert r.velocity == EVADE


def test_cmd_vel_flows_when_no_dodge_has_ever_fired():
    r = route(_state(last_cmd=CRUISE, last_cmd_t=10.0), 10.0, TIMEOUT)
    assert r.action is Action.CMD_VEL
    assert r.velocity == CRUISE


def test_evade_stays_fresh_right_up_to_the_timeout():
    r = route(_state(last_evade=EVADE, last_evade_t=0.0, evade_active=True),
              TIMEOUT, TIMEOUT)
    assert r.action is Action.EVADE


def test_evade_goes_stale_just_past_the_timeout():
    r = route(_state(last_evade=EVADE, last_evade_t=0.0, evade_active=True),
              TIMEOUT + 1e-6, TIMEOUT)
    assert r.action is Action.HANDBACK


# the handback edge

def test_expired_dodge_emits_exactly_one_zero():
    """Two zeros would be harmless; none would leave AP coasting at dodge
    speed. The edge must be consumed by whichever caller sees it first."""
    s = _state(last_evade=EVADE, last_evade_t=0.0, evade_active=True)
    first = route(s, 5.0, TIMEOUT)
    assert first.action is Action.HANDBACK
    assert first.velocity == ZERO
    assert first.evade_active is False

    # the caller commits the new evade_active, then routes again
    second = route(RouterState(last_evade=s.last_evade,
                               last_evade_t=s.last_evade_t,
                               last_cmd=s.last_cmd,
                               last_cmd_t=s.last_cmd_t,
                               evade_active=first.evade_active), 5.0, TIMEOUT)
    assert second.action is not Action.HANDBACK


def test_handback_outranks_cmd_vel():
    """Patrol must not get the vehicle back until the dodge has been zeroed."""
    r = route(_state(last_evade=EVADE, last_evade_t=0.0, evade_active=True,
                     last_cmd=CRUISE, last_cmd_t=5.0), 5.0, TIMEOUT)
    assert r.action is Action.HANDBACK


def test_no_handback_if_no_dodge_was_ever_active():
    """A stale evade that never became active must not manufacture a zero."""
    r = route(_state(last_evade=EVADE, last_evade_t=0.0, evade_active=False),
              99.0, TIMEOUT)
    assert r.action is Action.SILENCE


# silence vs zero-hold

def test_never_commanded_means_silence_not_a_zero_hold():
    """Position-mode patrol drives ArduPilot over its own MAVLink link. A
    zero-velocity stream from the bridge fights those position setpoints --
    the same coupling patrol_handoff_s > cmd_timeout_s protects."""
    r = route(_state(), 10.0, TIMEOUT)
    assert r.action is Action.SILENCE
    assert r.sends is False


def test_stale_cmd_vel_becomes_a_zero_hold_not_a_coast():
    r = route(_state(last_cmd=CRUISE, last_cmd_t=0.0), 10.0, TIMEOUT)
    assert r.action is Action.ZERO_HOLD
    assert r.velocity == ZERO
    assert r.sends is True


# what the fast evade timer is allowed to touch

def test_evade_route_is_idempotent_while_the_dodge_is_live():
    """The fast timer re-routes at 50 Hz. Each pass must yield the same
    command and leave evade_active set, or the retransmit would itself
    manufacture a handback."""
    s = _state(last_evade=EVADE, last_evade_t=10.0, evade_active=True)
    for now in (10.0, 10.02, 10.04, 10.5):
        r = route(s, now, TIMEOUT)
        assert r.action is Action.EVADE
        assert r.evade_active is True


def test_ignoring_a_non_evade_route_leaves_the_state_untouched():
    """_tick_evade acts on EVADE alone, so it must be safe for it to drop any
    other verdict on the floor: route() is a pure read, and the caller only
    commits evade_active when it acts."""
    s = _state(last_evade=EVADE, last_evade_t=0.0, evade_active=True)
    ignored = route(s, 5.0, TIMEOUT)          # HANDBACK, deliberately dropped
    assert ignored.action is not Action.EVADE
    assert s.evade_active is True             # unchanged: the edge survives
    assert route(s, 5.0, TIMEOUT).action is Action.HANDBACK


# guards on the checks themselves

def test_sends_is_false_only_for_silence():
    for action in Action:
        r = Route(action, ZERO, evade_active=False)
        assert r.sends is (action is not Action.SILENCE)


def test_router_state_is_immutable():
    """The node swaps whole snapshots under its lock rather than mutating a
    shared one; a mutable state would make the lock decorative."""
    s = _state()
    with pytest.raises(Exception):
        s.evade_active = True


# acceleration feedforward
#
# The accel arrives on its own topic, so it has its own freshness. The rules
# that matter are about what it must NOT be attached to.

ACCEL = (2.0, -1.0, 0.5)


def test_a_fresh_accel_rides_along_with_the_dodge():
    r = route(_state(last_evade=EVADE, last_evade_t=10.0,
                     last_accel=ACCEL, last_accel_t=10.0), 10.0, TIMEOUT)
    assert r.action is Action.EVADE
    assert r.accel == ACCEL


def test_no_accel_publisher_means_a_velocity_only_setpoint():
    """Nobody publishes the topic at all -- a bare bridge with no evasion_node.

    This is NOT the shipped flight configuration, though it was once believed
    to be; see test_the_shipped_zero_feedforward_sends_velocity_only.
    """
    r = route(_state(last_evade=EVADE, last_evade_t=10.0), 10.0, TIMEOUT)
    assert r.action is Action.EVADE
    assert r.accel is None


def test_the_shipped_zero_feedforward_sends_velocity_only():
    """One configuration must produce one command.

    evade_accel_ff_mps2 ships at 0.0, and evasion_node._publish_evade
    publishes an Accel on EVERY tick regardless -- deliberately, so a stale
    feedforward can never ride along with a fresh velocity. So the latched
    accel in flight is not absent, it is a FRESH ZERO, and this is the state
    every dodge actually routes through.

    Treating that as an acceleration command sets MASK_VEL_ACCEL, so the
    documented off-state would send a different MAVLink message than an
    absent publisher (MASK_VEL_ONLY). That inconsistency is what this pins.

    It does NOT pin a flight outcome. The zero-accel mask suppressed motion
    entirely at settled hover (28/28 trials at 0.00 m), but a controlled
    12-trial A/B in flight -- 8 m/s, detection_range_m 12.0 (an INPUT),
    pre-fix vs post-fix router -- found escape contribution 0.107 m pre-fix
    vs 0.091 m post-fix, both arms commanding ~1.5-1.9 m of displacement.
    The branch is correct on semantics; it recovers no dodges.
    """
    r = route(_state(last_evade=EVADE, last_evade_t=10.0,
                     last_accel=(0.0, 0.0, 0.0), last_accel_t=10.0),
              10.0, TIMEOUT)
    assert r.action is Action.EVADE
    assert r.velocity == EVADE      # the dodge still goes out, in full
    assert r.accel is None          # but as a velocity-only setpoint


@pytest.mark.parametrize("accel", [
    (1e-9, 0.0, 0.0),
    (0.0, -1e-9, 0.0),
    (0.0, 0.0, 1e-9),
])
def test_a_nonzero_feedforward_is_never_swallowed_as_zero(accel):
    """The zero check is exact equality, and must stay that way. Widening it
    into a tolerance would let a real -- if small -- feedforward be dropped
    silently, which is the same class of bug in the other direction."""
    r = route(_state(last_evade=EVADE, last_evade_t=10.0,
                     last_accel=accel, last_accel_t=10.0), 10.0, TIMEOUT)
    assert r.accel == accel


def test_a_stale_accel_degrades_to_velocity_only():
    """A dropped accel message must cost the feedforward for that tick, not
    re-apply the previous dodge's acceleration."""
    r = route(_state(last_evade=EVADE, last_evade_t=10.0,
                     last_accel=ACCEL, last_accel_t=5.0), 10.0, TIMEOUT)
    assert r.accel is None


def test_the_handback_zero_never_carries_an_acceleration():
    """The one that would actually hurt: a shove applied at the exact moment
    the aircraft is being handed back to patrol."""
    r = route(_state(last_evade=EVADE, last_evade_t=0.0, evade_active=True,
                     last_accel=ACCEL, last_accel_t=0.0), 5.0, TIMEOUT)
    assert r.action is Action.HANDBACK
    assert r.accel is None


@pytest.mark.parametrize("state,now", [
    (dict(last_cmd=CRUISE, last_cmd_t=10.0), 10.0),          # CMD_VEL
    (dict(last_cmd=CRUISE, last_cmd_t=0.0), 10.0),           # ZERO_HOLD
    (dict(), 10.0),                                          # SILENCE
])
def test_patrol_paths_never_carry_an_acceleration(state, now):
    r = route(_state(last_accel=ACCEL, last_accel_t=now, **state), now, TIMEOUT)
    assert r.action is not Action.EVADE
    assert r.accel is None
