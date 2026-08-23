"""Supervisor transition table.

The load-bearing test is test_no_fault_path_can_reach_evade. SAFETY_CASE.md
requires that any undefined fault goes to FAILSAFE and never to an evasive
manoeuvre, and that requirement is only worth writing down if something checks
it against every state and every fault rather than against the cases someone
happened to think of.
"""

import pytest

from huitzilin_sim.supervisor import (
    ALLOWED_MODES,
    Decision,
    Fault,
    FAULT_RESPONSE,
    Limits,
    Observation,
    State,
    detect_faults,
    edge_only,
    fault_response,
    next_state,
    watched_topics,
)

# The test bench arms EVERY watch, including cmd_vel, which the shipped
# supervisor.yaml disables. Without this the SETPOINT_STALL branch would be
# unreachable and test_the_fixture_really_induces_each_fault -- the guard that
# keeps the safety sweep from passing vacuously -- would itself go vacuous.
LIM = Limits(cmd_vel_timeout_s=1.0)

# What the shipped yaml actually produces: cmd_vel not watched.
SHIPPED = Limits()

# Every watched topic reporting fresh. Faults are opt-in from here.
FRESH = {"odom": 0.0, "patrol_state": 0.0, "cloud": 0.0, "cmd_vel": 0.0}

# What the running stack really looks like in position mode: nothing has ever
# published /huitzilin/cmd_vel, so the key is absent rather than stale.
SHIPPED_AGES = {"odom": 0.0, "patrol_state": 0.0, "cloud": 0.0}


def flying(**kw):
    """An armed, healthy, in-bounds aircraft. The baseline for fault tests."""
    base = dict(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                fc_failsafe=False, ages=dict(FRESH))
    base.update(kw)
    return Observation(**base)


def with_fault(fault, **kw):
    """An observation forcing exactly `fault` and nothing else."""
    ages = dict(FRESH)
    obs = dict(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
               fc_failsafe=False)
    if fault is Fault.FC_FAILSAFE:
        obs["fc_failsafe"] = True
    elif fault is Fault.LINK_LOSS:
        ages["odom"] = 99.0
    elif fault is Fault.COMPANION_LOSS:
        ages["patrol_state"] = 99.0
    elif fault is Fault.SENSOR_DROPOUT:
        ages["cloud"] = 99.0
    elif fault is Fault.SETPOINT_STALL:
        ages["cmd_vel"] = 99.0
    elif fault is Fault.LOW_BATTERY:
        obs["batt_v"] = 19.0
    elif fault is Fault.FENCE_BREACH:
        obs["radius_m"] = 50.0
    obs["ages"] = ages
    obs.update(kw)
    return Observation(**obs)


# --- the documented transition table, one case per row -----------------------

def test_disarmed_to_arming_on_arm_request():
    assert next_state(State.DISARMED, Observation(arm_requested=True),
                      LIM).state is State.ARMING


def test_arming_to_takeoff_once_armed():
    d = next_state(State.ARMING, Observation(armed=True, ages=dict(FRESH)), LIM)
    assert d.state is State.TAKEOFF
    assert d.set_mode == "GUIDED"


def test_takeoff_to_patrol_at_altitude():
    d = next_state(State.TAKEOFF, flying(alt_m=2.0), LIM)
    assert d.state is State.PATROL
    assert d.start_patrol is True


def test_takeoff_holds_below_altitude():
    assert next_state(State.TAKEOFF, flying(alt_m=0.5), LIM).state is State.TAKEOFF


def test_patrol_to_evade_on_threat():
    assert next_state(State.PATROL, flying(alarm_on=True), LIM).state is State.EVADE


def test_evade_to_patrol_when_dodge_completes():
    assert next_state(State.EVADE, flying(alarm_on=False),
                      LIM).state is State.PATROL


def test_fault_from_patrol_or_evade_goes_to_failsafe():
    for state in (State.PATROL, State.EVADE):
        d = next_state(state, with_fault(Fault.SENSOR_DROPOUT), LIM)
        assert d.state is State.FAILSAFE


def test_fence_breach_goes_to_rtl_not_failsafe():
    d = next_state(State.PATROL, with_fault(Fault.FENCE_BREACH), LIM)
    assert d.state is State.RTL_LAND
    assert d.set_mode == "RTL"


def test_failsafe_to_rtl_once_settled():
    d = next_state(State.FAILSAFE, flying(state_age_s=LIM.failsafe_settle_s), LIM)
    assert d.state is State.RTL_LAND


def test_rtl_to_disarmed_on_landing():
    assert next_state(State.RTL_LAND, flying(landed=True),
                      LIM).state is State.DISARMED


# --- the safety property -----------------------------------------------------

# Fault.UNKNOWN is a sentinel for the unreachable-state branch; no combination
# of sensor readings produces it, so with_fault() cannot induce it.
INDUCIBLE_FAULTS = [f for f in Fault if f is not Fault.UNKNOWN]


def test_the_fixture_really_induces_each_fault():
    """Guards the sweep below: an observation that failed to trigger its fault
    would make test_no_fault_path_can_reach_evade pass vacuously."""
    for fault in INDUCIBLE_FAULTS:
        assert detect_faults(with_fault(fault), LIM) == [fault]


# --- the watch gate ----------------------------------------------------------
# A watch on a topic the running configuration never publishes is
# indistinguishable from a dead publisher, because _age reads a never-seen
# topic as infinitely stale. That is not hypothetical: the shipped stack runs
# patrol in "position" mode, where patrol_node talks to ArduPilot over MAVLink
# and never creates the /huitzilin/cmd_vel publisher, so SETPOINT_STALL fired
# one second after every arm and drove FAILSAFE -> LOITER -> RTL_LAND.

def test_the_shipped_config_does_not_fault_a_healthy_aircraft():
    """The regression. cmd_vel is never published and must not be a fault."""
    obs = Observation(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                      fc_failsafe=False, ages=dict(SHIPPED_AGES))
    assert detect_faults(obs, SHIPPED) == []
    assert next_state(State.PATROL, obs, SHIPPED).state is State.PATROL


def test_shipped_config_leaves_cmd_vel_unwatched():
    assert SHIPPED.cmd_vel_timeout_s == 0.0
    assert "cmd_vel" not in watched_topics(SHIPPED)
    assert set(watched_topics(SHIPPED)) == {"odom", "patrol_state", "cloud"}


def test_a_zero_timeout_disables_its_fault():
    """Even an infinitely stale topic is silent when its watch is off."""
    dead = {"odom": 0.0, "patrol_state": 0.0, "cloud": 0.0, "cmd_vel": 99.0}
    obs = Observation(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                      fc_failsafe=False, ages=dead)
    assert Fault.SETPOINT_STALL not in detect_faults(obs, SHIPPED)
    # ...and the same observation still faults when the watch is armed, so the
    # disable is what silenced it rather than the fixture being broken.
    assert detect_faults(obs, LIM) == [Fault.SETPOINT_STALL]


def test_disabling_one_watch_does_not_disable_the_others():
    """SHIPPED still has to catch a genuinely dead camera and a dead link."""
    for fault, key in ((Fault.SENSOR_DROPOUT, "cloud"),
                       (Fault.LINK_LOSS, "odom"),
                       (Fault.COMPANION_LOSS, "patrol_state")):
        ages = dict(SHIPPED_AGES)
        ages[key] = 99.0
        obs = Observation(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                          fc_failsafe=False, ages=ages)
        assert detect_faults(obs, SHIPPED) == [fault]


def test_a_never_seen_topic_still_faults_when_its_watch_is_armed():
    """The disable is the only way to say "not published".

    _age must keep reporting a missing topic as infinitely stale, or a node
    that dies before its first message would read as healthy forever.
    """
    obs = Observation(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                      fc_failsafe=False, ages={"odom": 0.0, "cloud": 0.0})
    assert Fault.COMPANION_LOSS in detect_faults(obs, SHIPPED)


@pytest.mark.parametrize("timeout", [0.0, -1.0])
def test_non_positive_timeouts_are_all_treated_as_off(timeout):
    ages = {"odom": 0.0, "patrol_state": 0.0, "cloud": 99.0, "cmd_vel": 0.0}
    obs = Observation(armed=True, alt_m=2.0, radius_m=1.0, batt_v=24.0,
                      fc_failsafe=False, ages=ages)
    assert detect_faults(obs, Limits(sensor_timeout_s=timeout)) == []


def test_no_fault_path_can_reach_evade():
    """Every state crossed with every inducible fault. None of them may dodge.

    Swept over `armed` too. detect_faults returns [] while disarmed, so pinning
    armed=True proved the property only on the half where faults exist at all.
    The assertion is deliberately conditioned on a fault actually being present:
    while disarmed there is no fault, so "no FAULT path reaches EVADE" is
    vacuous there rather than false. What the disarmed half really guarantees is
    pinned by test_disarmed_threat_reaches_evade_but_commands_nothing below.
    """
    for state in State:
        for fault in INDUCIBLE_FAULTS:
            for armed in (True, False):
                obs = with_fault(fault, alarm_on=True, armed=armed)
                if not detect_faults(obs, LIM):
                    continue          # disarmed: nothing to answer, see above
                assert next_state(state, obs, LIM).state is not State.EVADE, (
                    "%s + %s (armed=%s) reached EVADE" % (state, fault, armed))


def test_disarmed_threat_reaches_evade_but_commands_nothing():
    """Documents the one way EVADE is reachable with every topic dead.

    Faults are gated on `armed` (bench topics are legitimately silent), so a
    disarmed PATROL + alarm_on takes the threat edge with no fault to pre-empt
    it. That is bookkeeping only: evasion_node owns /cmd/evade, the supervisor
    never commands a dodge, and the EVADE state is timeout-bounded. Pinned here
    so a future change to the fault gate cannot turn it into a real dodge.
    """
    obs = Observation(armed=False, alarm_on=True, ages={})
    assert detect_faults(obs, LIM) == []
    d = next_state(State.PATROL, obs, LIM)
    assert d.state is State.EVADE
    assert d.set_mode is None
    assert d.fault is None


def test_every_fault_including_future_ones_has_a_safe_response():
    """A Fault member added later and left out of FAULT_RESPONSE must still
    resolve to FAILSAFE rather than being silently unhandled."""
    for fault in Fault:
        assert fault_response(fault) in (State.FAILSAFE, State.RTL_LAND)
    for fault in set(Fault) - set(FAULT_RESPONSE):
        assert fault_response(fault) is State.FAILSAFE


def test_a_fault_beats_a_simultaneous_threat():
    """alarm_on and a dead camera at the same time is a failure, not a dodge."""
    d = next_state(State.PATROL, with_fault(Fault.SENSOR_DROPOUT, alarm_on=True), LIM)
    assert d.state is State.FAILSAFE
    assert d.fault is Fault.SENSOR_DROPOUT


def test_supervisor_never_commands_a_dodge():
    """The mode vocabulary is closed. Nothing in it means "evade"."""
    seen = set()
    for state in State:
        for obs in (flying(), flying(alarm_on=True), flying(landed=True),
                    Observation(arm_requested=True), Observation()):
            for fault in Fault:
                d = next_state(state, with_fault(fault), LIM)
                if d.set_mode:
                    seen.add(d.set_mode)
            d = next_state(state, obs, LIM)
            if d.set_mode:
                seen.add(d.set_mode)
    assert seen <= ALLOWED_MODES
    assert "EVADE" not in seen


def test_totality_never_raises_and_always_returns_a_state():
    corpus = [
        Observation(),
        Observation(arm_requested=True),
        Observation(armed=True),
        flying(),
        flying(alarm_on=True),
        flying(landed=True),
        flying(state_age_s=1e6),
        flying(alt_m=None, radius_m=None, batt_v=None),
        flying(batt_v=0.0),
        flying(alt_m=-5.0),
        flying(ages={}),
        Observation(armed=True, ages={"odom": float("inf")}),
    ]
    for state in State:
        for obs in corpus:
            d = next_state(state, obs, LIM)
            assert isinstance(d, Decision)
            assert d.state in set(State)
            assert d.reason


# --- faults are gated on being armed -----------------------------------------

def test_disarmed_aircraft_reports_no_faults():
    """Half these topics are legitimately silent on the bench with props off."""
    assert detect_faults(Observation(armed=False, ages={}), LIM) == []
    assert detect_faults(Observation(armed=None, ages={}), LIM) == []


def test_missing_topic_reads_as_stale_not_fresh():
    """An absent age key must not be mistaken for a fresh message."""
    faults = detect_faults(Observation(armed=True, ages={}), LIM)
    assert Fault.LINK_LOSS in faults
    assert Fault.SENSOR_DROPOUT in faults


def test_severity_order_is_deterministic():
    obs = Observation(armed=True, batt_v=19.0, fc_failsafe=True,
                      radius_m=50.0, ages=dict(FRESH))
    assert detect_faults(obs, LIM)[0] is Fault.FC_FAILSAFE


def test_unknown_battery_does_not_trip_low_battery():
    """batt_v is None until SYS_STATUS arrives, and None is not flat."""
    assert Fault.LOW_BATTERY not in detect_faults(flying(batt_v=None), LIM)


def test_altitude_ceiling_counts_as_a_fence_breach():
    assert Fault.FENCE_BREACH in detect_faults(flying(alt_m=99.0), LIM)


# --- EVADE is bounded --------------------------------------------------------

def test_evade_exits_on_timeout_even_if_the_alarm_never_clears():
    """evasion_node publishes alarm False once, at the end of a dodge. A single
    dropped message must not strand the machine in EVADE."""
    stuck = flying(alarm_on=True, state_age_s=LIM.evade_max_s)
    assert next_state(State.EVADE, stuck, LIM).state is State.PATROL


def test_evade_holds_inside_the_window():
    held = flying(alarm_on=True, state_age_s=LIM.evade_max_s - 0.1)
    assert next_state(State.EVADE, held, LIM).state is State.EVADE


# --- edge triggering ---------------------------------------------------------

def test_commands_are_suppressed_when_the_state_does_not_change():
    d = next_state(State.TAKEOFF, flying(alt_m=2.0), LIM)
    assert edge_only(State.TAKEOFF, d).start_patrol is True      # first tick
    same = next_state(State.PATROL, flying(), LIM)
    assert edge_only(State.PATROL, same).start_patrol is None    # every tick after


def test_start_patrol_is_never_asserted_around_a_dodge():
    """evasion_node owns /huitzilin/start_patrol for the whole dodge. A 1 Hz
    supervisor re-asserting it would fight the handoff that patrol_handoff_s
    and cmd_timeout_s are coupled to protect."""
    into = edge_only(State.PATROL, next_state(State.PATROL, flying(alarm_on=True), LIM))
    assert into.state is State.EVADE and into.start_patrol is None
    out = edge_only(State.EVADE, next_state(State.EVADE, flying(alarm_on=False), LIM))
    assert out.state is State.PATROL and out.start_patrol is None


def test_a_fault_out_of_evade_still_stops_patrol_via_the_mode_change():
    d = edge_only(State.EVADE, next_state(State.EVADE, with_fault(Fault.LINK_LOSS), LIM))
    assert d.state is State.FAILSAFE
    assert d.start_patrol is None
    assert d.set_mode == "LOITER"


def test_terminal_states_are_not_rerouted_by_faults():
    """FAILSAFE and RTL_LAND own the aircraft until they finish; there is
    nowhere safer to send them."""
    for state in (State.FAILSAFE, State.RTL_LAND):
        d = next_state(state, with_fault(Fault.SENSOR_DROPOUT), LIM)
        assert d.state in (state, State.RTL_LAND, State.DISARMED)
