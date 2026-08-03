"""Flight state machine and fault monitor. Pure logic; no rclpy.

docs/architecture.md has listed supervisor_node since Week 2 and
docs/state_machine.md describes a seven-state machine, but nothing implemented
either, so every fault response in SAFETY_CASE.md section 1 was documentation
rather than code.

The ordering inside next_state is the safety property, not a style choice:
faults are detected and answered before the threat branch is reachable at all.
That is what makes "any undefined fault -> FAILSAFE, never an evasive
manoeuvre" a structural guarantee instead of a rule someone must remember.
The reflex is for projectiles only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Optional


class State(Enum):
    DISARMED = "DISARMED"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    PATROL = "PATROL"
    EVADE = "EVADE"
    RTL_LAND = "RTL_LAND"
    FAILSAFE = "FAILSAFE"


class Fault(Enum):
    """One member per detection row of the SAFETY_CASE.md section 1 FMEA."""

    SENSOR_DROPOUT = "sensor_dropout"      # OAK-D disconnect -> topic timeout
    LINK_LOSS = "link_loss"                # RC/telemetry loss -> odom timeout
    COMPANION_LOSS = "companion_loss"      # Pi brownout -> publishers stop
    LOW_BATTERY = "low_battery"            # voltage monitor
    FC_FAILSAFE = "fc_failsafe"            # FC status message
    FENCE_BREACH = "fence_breach"          # flyaway -> geofence
    SETPOINT_STALL = "setpoint_stall"      # GUIDED setpoint stream stops
    UNKNOWN = "unknown"                    # anything not enumerated above


# Only the two faults whose correct answer is "come home" are listed. Every
# other fault, including any Fault member added later and forgotten here,
# resolves to FAILSAFE through the .get() default in fault_response(). A new
# fault is therefore safe by default rather than unhandled.
FAULT_RESPONSE = {
    Fault.LOW_BATTERY: State.RTL_LAND,
    Fault.FENCE_BREACH: State.RTL_LAND,
}

# Severity order, most serious first. detect_faults returns in this order so
# simultaneous faults do not resolve by dict insertion order.
FAULT_PRIORITY = (
    Fault.FC_FAILSAFE,
    Fault.LINK_LOSS,
    Fault.COMPANION_LOSS,
    Fault.FENCE_BREACH,
    Fault.LOW_BATTERY,
    Fault.SENSOR_DROPOUT,
    Fault.SETPOINT_STALL,
)

# The modes the supervisor may ask for. There is deliberately no mode meaning
# "dodge": evasion is commanded by evasion_node over /cmd/evade, never by the
# fault monitor.
ALLOWED_MODES = frozenset({"GUIDED", "LOITER", "RTL", "LAND"})


@dataclass(frozen=True)
class Limits:
    """Every threshold in one place, so none of them is a literal in the table."""

    # Topic staleness. Each is generous against its publisher rate: the depth
    # cloud runs at 15 Hz, odom at stream_rate_hz (30), patrol state at 10 Hz,
    # cmd_vel at cmd_rate_hz (10).
    #
    # A timeout of 0 (or negative) DISABLES that watch entirely. This is not a
    # convenience: a watch on a topic the running configuration never publishes
    # is indistinguishable from a dead publisher, because _age() reads a
    # never-seen topic as infinitely stale. cmd_vel_timeout_s defaults to 0
    # because position-mode patrol drives ArduPilot over MAVLink directly and
    # publishes no ROS setpoint stream -- see the comment in supervisor.yaml.
    sensor_timeout_s: float = 1.0
    odom_timeout_s: float = 1.0
    patrol_state_timeout_s: float = 2.0
    cmd_vel_timeout_s: float = 0.0

    batt_low_v: float = 21.0          # 6S at ~3.5 V/cell

    # Second line of defence only. The flight controller owns the real fence;
    # these exist so the companion notices a breach it caused itself.
    fence_radius_m: float = 10.0
    fence_alt_max_m: float = 5.0

    takeoff_alt_m: float = 2.0
    alt_tolerance_m: float = 0.3

    # EVADE must be timeout-bounded. evasion_node publishes alarm False only at
    # the end of a dodge, so one dropped message would otherwise strand the
    # machine in EVADE. Covers dodge_duration_s 1.0 + recover_hold_s 0.5 +
    # patrol_handoff_s 0.8, plus margin.
    evade_max_s: float = 3.5

    failsafe_settle_s: float = 3.0    # "stable hover achieved" -> RTL/LAND


@dataclass(frozen=True)
class Observation:
    """Everything the machine may look at, sampled at one instant."""

    now_s: float = 0.0
    armed: Optional[bool] = None
    mode: Optional[str] = None
    alt_m: Optional[float] = None
    radius_m: Optional[float] = None
    batt_v: Optional[float] = None
    fc_failsafe: Optional[bool] = None
    landed: bool = False
    arm_requested: bool = False
    alarm_on: bool = False
    state_age_s: float = 0.0          # seconds since this state was entered
    # Seconds since the last message on each watched topic. An absent key means
    # "never seen", which is not a fault until the aircraft is armed -- half
    # these topics are legitimately silent on the bench.
    ages: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class Decision:
    state: State
    reason: str
    fault: Optional[Fault] = None
    set_mode: Optional[str] = None
    start_patrol: Optional[bool] = None


def _age(obs: Observation, key: str) -> float:
    """A missing topic reads as infinitely stale, never as fresh."""
    v = obs.ages.get(key)
    return float("inf") if v is None else float(v)


def _stale(obs: Observation, key: str, timeout_s: float) -> bool:
    """Is `key` overdue, given that a non-positive timeout means "not watched"?

    The disable check comes first so a topic nothing publishes cannot fault.
    _age deliberately still reports a never-seen topic as infinitely stale --
    that is what catches a publisher which died before its first message -- so
    the only way to express "this configuration does not produce that topic" is
    to turn the watch off, not to soften the age.
    """
    return timeout_s > 0.0 and _age(obs, key) > timeout_s


def watched_topics(limits: Limits) -> dict:
    """Topic key -> timeout, for the watches that are armed. Used for logging.

    supervisor_node prints this at startup so an operator can see which faults
    are live without reverse-engineering it from the yaml.
    """
    return {key: timeout for key, timeout in (
        ("odom", limits.odom_timeout_s),
        ("patrol_state", limits.patrol_state_timeout_s),
        ("cloud", limits.sensor_timeout_s),
        ("cmd_vel", limits.cmd_vel_timeout_s),
    ) if timeout > 0.0}


def fault_response(fault: Fault) -> State:
    """Where a fault sends the machine. Anything unlisted goes to FAILSAFE."""
    return FAULT_RESPONSE.get(fault, State.FAILSAFE)


def detect_faults(obs: Observation, limits: Limits) -> list:
    """Faults present right now, most severe first.

    Returns nothing while disarmed. On the bench with props off most of these
    topics are legitimately quiet, and a supervisor that declared FAILSAFE
    before the aircraft ever armed would be switched off by the second day.

    Each staleness watch is skipped when its timeout is non-positive. Without
    that, the supervisor faults on any topic the launched configuration does
    not publish -- which is how a healthy aircraft used to get sent home one
    second after arming, because nothing publishes /huitzilin/cmd_vel in
    position mode.
    """
    if not obs.armed:
        return []

    found = set()

    if obs.fc_failsafe:
        found.add(Fault.FC_FAILSAFE)
    if _stale(obs, "odom", limits.odom_timeout_s):
        found.add(Fault.LINK_LOSS)
    if _stale(obs, "patrol_state", limits.patrol_state_timeout_s):
        found.add(Fault.COMPANION_LOSS)
    if _stale(obs, "cloud", limits.sensor_timeout_s):
        found.add(Fault.SENSOR_DROPOUT)
    if _stale(obs, "cmd_vel", limits.cmd_vel_timeout_s):
        found.add(Fault.SETPOINT_STALL)
    if obs.batt_v is not None and obs.batt_v < limits.batt_low_v:
        found.add(Fault.LOW_BATTERY)
    if obs.radius_m is not None and obs.radius_m > limits.fence_radius_m:
        found.add(Fault.FENCE_BREACH)
    if obs.alt_m is not None and obs.alt_m > limits.fence_alt_max_m:
        found.add(Fault.FENCE_BREACH)

    return [f for f in FAULT_PRIORITY if f in found]


def _terminal(state: State) -> bool:
    """States that keep the aircraft until they finish.

    Faults do not re-route them, because there is nowhere safer to go.
    """
    return state in (State.FAILSAFE, State.RTL_LAND, State.DISARMED)


def next_state(state: State, obs: Observation, limits: Limits) -> Decision:
    """The docs/state_machine.md transition table, faults first.

    Evaluation order is the safety property: the fault branch returns before
    the threat branch is reached, so no fault can produce EVADE. EVADE is
    reachable from exactly one place in this function.
    """
    if not _terminal(state):
        faults = detect_faults(obs, limits)
        if faults:
            fault = faults[0]
            target = fault_response(fault)
            return Decision(
                state=target,
                reason="%s -> %s" % (fault.value, target.value),
                fault=fault,
                set_mode="RTL" if target is State.RTL_LAND else "LOITER",
                # Stop patrol on the way out, except from EVADE where
                # evasion_node owns the service.
                start_patrol=None if state is State.EVADE else False,
            )

    if state is State.DISARMED:
        if obs.arm_requested:
            return Decision(State.ARMING, "arm requested, pre-arm checks pass")
        return Decision(State.DISARMED, "idle on the ground")

    if state is State.ARMING:
        if obs.armed:
            return Decision(State.TAKEOFF, "armed successfully", set_mode="GUIDED")
        return Decision(State.ARMING, "waiting for the arm to take")

    if state is State.TAKEOFF:
        reached = (obs.alt_m is not None
                   and obs.alt_m >= limits.takeoff_alt_m - limits.alt_tolerance_m)
        if reached:
            return Decision(State.PATROL, "target altitude reached",
                            set_mode="GUIDED", start_patrol=True)
        return Decision(State.TAKEOFF, "climbing")

    if state is State.PATROL:
        # The only edge into EVADE anywhere in this function.
        if obs.alarm_on:
            return Decision(State.EVADE, "threat detected")
        return Decision(State.PATROL, "patrolling")

    if state is State.EVADE:
        # Timeout-authoritative: a dropped alarm-off must not strand us here.
        if obs.state_age_s >= limits.evade_max_s:
            return Decision(State.PATROL,
                            "dodge window elapsed without an alarm clear")
        if not obs.alarm_on:
            return Decision(State.PATROL, "dodge complete")
        return Decision(State.EVADE, "evading")

    if state is State.FAILSAFE:
        if obs.state_age_s >= limits.failsafe_settle_s:
            return Decision(State.RTL_LAND, "stable hover achieved", set_mode="RTL")
        return Decision(State.FAILSAFE, "settling into a calm hover")

    if state is State.RTL_LAND:
        if obs.landed or obs.armed is False:
            return Decision(State.DISARMED, "landed")
        return Decision(State.RTL_LAND, "returning and landing")

    # Unreachable while the branches above stay exhaustive, but a State member
    # added later must fail safe rather than fall off the end.
    return Decision(State.FAILSAFE, "unhandled state %r" % (state,),
                    fault=Fault.UNKNOWN, set_mode="LOITER", start_patrol=False)


def edge_only(previous: State, decision: Decision) -> Decision:
    """Blank the commands unless the state actually changed.

    evasion_node already toggles /huitzilin/start_patrol either side of every
    dodge. A supervisor re-asserting it at 1 Hz would fight that handoff, and
    the patrol_handoff_s / cmd_timeout_s coupling exists precisely because
    that fight is expensive.
    """
    if decision.state is previous:
        return Decision(decision.state, decision.reason, decision.fault)
    if State.EVADE in (previous, decision.state):
        return Decision(decision.state, decision.reason, decision.fault,
                        set_mode=decision.set_mode, start_patrol=None)
    return decision
