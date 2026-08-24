"""get_state() dispatch for the telemetry the supervisor depends on.

SYS_STATUS was already being requested by request_streams() and HEARTBEAT
arrives unsolicited; both were drained and discarded. These tests pin the
sentinel handling, because ArduPilot reports "no reading" as an in-band value
that looks like a plausible measurement.
"""

import time

import pytest

from huitzilin_sim.mav_bridge import MavBridge

ARMED_FLAG = 128          # MAV_MODE_FLAG_SAFETY_ARMED
# Every real ArduPilot heartbeat sets this; without it pymavlink cannot
# resolve custom_mode and reports "Mode(0x...)" instead of a name.
CUSTOM_MODE = 1           # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
MAV_STATE_ACTIVE = 4
MAV_STATE_CRITICAL = 5
MAV_STATE_EMERGENCY = 6
MAV_TYPE_QUADROTOR = 2
MAV_TYPE_GCS = 6
AUTOPILOT_SYS = 1
GCS_SYS = 255


class _Msg:
    def __init__(self, kind, src=AUTOPILOT_SYS, **fields):
        self._kind = kind
        self._src = src
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._src


class _FakeMaster:
    """Hands out a scripted message queue, then None, like a drained socket."""

    def __init__(self, msgs):
        self._msgs = list(msgs)

    def recv_match(self, blocking=False, **_):
        return self._msgs.pop(0) if self._msgs else None


class _ArmFakeMaster:
    """A link whose vehicle never arms -- i.e. a refused arm.

    Models the case that actually mattered: ArduPilot answers heartbeats
    normally, so the link is plainly alive, while never setting the armed bit
    because a pre-arm check is failing. pymavlink's motors_armed_wait() loops
    on exactly that forever.
    """

    def __init__(self, armed=False):
        self._armed = armed
        self.heartbeats = 0
        self.sent = []
        self.mav = self

    def command_long_send(self, *args):
        self.sent.append(args)

    def wait_heartbeat(self, timeout=None):
        self.heartbeats += 1
        return object()

    def motors_armed(self):
        return 128 if self._armed else 0


def _arm_bridge(armed=False):
    b = object.__new__(MavBridge)
    b._state = {}
    b.master = _ArmFakeMaster(armed=armed)
    b.target_system = AUTOPILOT_SYS
    b.target_component = 1
    return b


def _bridge(msgs, target_system=AUTOPILOT_SYS):
    """A MavBridge without a live link. __init__ would open a real socket."""
    b = object.__new__(MavBridge)
    b._state = {}
    b.master = _FakeMaster(msgs)
    b.target_system = target_system
    b.target_component = 1
    return b


def _hb(src=AUTOPILOT_SYS, base_mode=CUSTOM_MODE, system_status=MAV_STATE_ACTIVE,
        custom_mode=4, mav_type=MAV_TYPE_QUADROTOR):
    return _Msg("HEARTBEAT", src=src, base_mode=base_mode,
                system_status=system_status, custom_mode=custom_mode,
                type=mav_type, autopilot=3)


def _sys_status(voltage_battery, battery_remaining):
    return _Msg("SYS_STATUS", voltage_battery=voltage_battery,
                battery_remaining=battery_remaining)


# battery sentinels

def test_battery_millivolts_become_volts():
    s = _bridge([_sys_status(22800, 76)]).get_state()
    assert s["batt_v"] == pytest.approx(22.8)
    assert s["batt_pct"] == pytest.approx(76.0)


def test_unknown_voltage_is_none_not_65_volts():
    """65535 mV is ArduPilot's 'unknown'. Read literally it is a 65.5 V pack,
    which is above any 6S threshold and would mask a genuinely flat battery."""
    assert _bridge([_sys_status(65535, 50)]).get_state()["batt_v"] is None


def test_zero_voltage_is_none_not_a_flat_pack():
    """SITL reports 0 before the battery monitor initialises. Taken at face
    value that is an instant low-battery RTL on every startup."""
    assert _bridge([_sys_status(0, 50)]).get_state()["batt_v"] is None


def test_unknown_battery_percent_is_none_not_minus_one():
    assert _bridge([_sys_status(22800, -1)]).get_state()["batt_pct"] is None


# heartbeat

def test_armed_flag_is_read_from_base_mode():
    assert _bridge([_hb(base_mode=CUSTOM_MODE | ARMED_FLAG)]).get_state()["armed"] is True
    assert _bridge([_hb(base_mode=CUSTOM_MODE)]).get_state()["armed"] is False


@pytest.mark.parametrize("status,expected", [
    (MAV_STATE_ACTIVE, False),
    (MAV_STATE_CRITICAL, True),
    (MAV_STATE_EMERGENCY, True),
])
def test_fc_failsafe_from_system_status(status, expected):
    assert _bridge([_hb(system_status=status)]).get_state()["fc_failsafe"] is expected


def test_gcs_heartbeat_does_not_overwrite_autopilot_state():
    """MAVProxy and the GCS heartbeat on the same link. A GCS heartbeat carries
    base_mode 0, so accepting it would report the aircraft disarmed forever."""
    msgs = [_hb(base_mode=CUSTOM_MODE | ARMED_FLAG),
            _hb(src=GCS_SYS, base_mode=0, mav_type=MAV_TYPE_GCS)]
    assert _bridge(msgs).get_state()["armed"] is True


def test_mode_name_survives_an_unresolvable_heartbeat():
    """An unknown mode must read as absent, not take the node down."""
    assert MavBridge.mode_name(_Msg("HEARTBEAT")) is None


def test_mode_name_resolves_a_real_copter_heartbeat():
    assert MavBridge.mode_name(_hb(custom_mode=4)) == "GUIDED"


# the existing contract must not move

def test_position_and_attitude_still_dispatch():
    msgs = [_Msg("LOCAL_POSITION_NED", x=1.0, y=2.0, z=-3.0, vx=0.1, vy=0.2, vz=0.3),
            _Msg("ATTITUDE", roll=0.01, pitch=0.02, yaw=1.5)]
    s = _bridge(msgs).get_state()
    assert (s["n"], s["e"], s["d"]) == (1.0, 2.0, -3.0)
    assert s["yaw"] == pytest.approx(1.5)


def test_new_types_do_not_starve_position():
    """The whole reason get_state drains untyped: a type-filtered read discards
    everything it scans past. Adding message types must not resurrect that."""
    msgs = [_hb(), _sys_status(22800, 80),
            _Msg("LOCAL_POSITION_NED", x=9.0, y=8.0, z=-7.0, vx=0, vy=0, vz=0),
            _Msg("VFR_HUD", airspeed=1.0),
            _Msg("ATTITUDE", roll=0, pitch=0, yaw=0.5)]
    s = _bridge(msgs).get_state()
    assert s["n"] == 9.0 and s["yaw"] == pytest.approx(0.5)
    assert s["armed"] is False and s["batt_v"] == pytest.approx(22.8)


def test_drain_is_still_capped():
    """A backlog must not stall the caller's timer tick."""
    from huitzilin_sim.mav_bridge import _DRAIN_MAX_MSGS
    flood = [_Msg("LOCAL_POSITION_NED", x=float(i), y=0.0, z=0.0,
                  vx=0, vy=0, vz=0) for i in range(_DRAIN_MAX_MSGS + 50)]
    b = _bridge(flood)
    b.get_state()
    assert len(b.master._msgs) == 50


def test_absent_telemetry_stays_absent():
    """Keys must not appear with default values; a consumer has to be able to
    tell 'never reported' from 'reported false'."""
    s = _bridge([_Msg("LOCAL_POSITION_NED", x=0.0, y=0.0, z=0.0,
                      vx=0, vy=0, vz=0)]).get_state()
    for key in ("armed", "mode", "batt_v", "batt_pct", "fc_failsafe"):
        assert key not in s


# arm: the bound, and the command it actually sends

def test_a_refused_arm_times_out_instead_of_blocking_forever():
    """The whole point of T3.

    _srv_arm calls this from a service callback on a single-threaded executor,
    so an unbounded wait does not just fail to arm -- it stops odom,
    /huitzilin/state and the setpoint stream for the life of the process.
    """
    b = _arm_bridge(armed=False)
    t0 = time.monotonic()
    with pytest.raises(TimeoutError):
        b.arm(True, timeout=0.3)
    assert time.monotonic() - t0 < 5.0, "arm() did not honour its timeout"
    assert b.master.heartbeats > 0, "never polled; the bound is vacuous"


def test_arm_returns_as_soon_as_the_vehicle_reports_armed():
    b = _arm_bridge(armed=True)
    assert b.arm(True, timeout=5) is True


def test_disarm_is_bounded_too():
    b = _arm_bridge(armed=True)          # stays armed, so disarm never confirms
    with pytest.raises(TimeoutError):
        b.arm(False, timeout=0.3)


def test_arm_never_sends_the_force_arm_magic_value():
    """param2 must be 0. 21196 is ArduPilot's force-arm, which CLAUDE.md
    forbids because it hides the frame/EKF fault you need to see; the field
    previously held 21, which is not a recognised value at all."""
    b = _arm_bridge(armed=True)
    b.arm(True, timeout=5)
    (args,) = b.master.sent
    param2 = args[5]          # target_sys, target_comp, cmd, confirmation, p1, p2
    assert param2 == 0
    assert param2 != 21196
