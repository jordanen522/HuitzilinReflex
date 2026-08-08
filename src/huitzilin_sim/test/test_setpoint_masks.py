"""SET_POSITION_TARGET_LOCAL_NED type_masks, checked as bit arithmetic.

A type_mask is the least forgiving thing in this codebase: a set bit means
"ignore this field", so one wrong bit does not raise, does not warn, and does
not appear in any log -- it silently drops the term you were trying to add,
or worse, un-ignores a position field full of zeros and commands the vehicle
to the EKF origin.

Bit layout (0-indexed): 0..2 pos x/y/z, 3..5 vel x/y/z, 6..8 acc x/y/z,
9 force, 10 yaw, 11 yaw_rate.
"""

import pytest

from huitzilin_sim.mav_bridge import (
    MASK_POS_ONLY,
    MASK_VEL_ACCEL,
    MASK_VEL_ONLY,
    MavBridge,
)

POS_BITS = (0, 1, 2)
VEL_BITS = (3, 4, 5)
ACC_BITS = (6, 7, 8)
YAW_RATE_BIT = 11


def ignored(mask, bit):
    return bool(mask & (1 << bit))


class _FakeMav:
    def __init__(self):
        self.sent = []

    def set_position_target_local_ned_send(self, *args):
        self.sent.append(args)


def _bridge():
    """A MavBridge without a live link; __init__ would open a real socket."""
    b = object.__new__(MavBridge)
    b._state = {}
    b.master = type("M", (), {})()
    b.master.mav = _FakeMav()
    b.target_system = 1
    b.target_component = 1
    return b


# ── the masks themselves ─────────────────────────────────────────────────────

def test_the_shipped_masks_have_the_documented_values():
    """These are quoted as decimals throughout the docs and the journal."""
    assert MASK_VEL_ONLY == 4039
    assert MASK_VEL_ACCEL == 3591
    assert MASK_POS_ONLY == 4088


@pytest.mark.parametrize("mask", [MASK_VEL_ONLY, MASK_VEL_ACCEL])
def test_every_velocity_mask_ignores_position(mask):
    """An un-ignored position field would be read as (0,0,0) — a command to
    fly to the EKF origin, i.e. back to the takeoff point."""
    assert all(ignored(mask, b) for b in POS_BITS)


@pytest.mark.parametrize("mask", [MASK_VEL_ONLY, MASK_VEL_ACCEL])
def test_every_velocity_mask_uses_velocity(mask):
    assert not any(ignored(mask, b) for b in VEL_BITS)


def test_the_velocity_only_mask_ignores_acceleration():
    assert all(ignored(MASK_VEL_ONLY, b) for b in ACC_BITS)


def test_the_feedforward_mask_uses_acceleration():
    """The single bit-level difference between the two commands."""
    assert not any(ignored(MASK_VEL_ACCEL, b) for b in ACC_BITS)


def test_the_two_masks_differ_only_in_the_acceleration_bits():
    diff = MASK_VEL_ONLY ^ MASK_VEL_ACCEL
    assert diff == sum(1 << b for b in ACC_BITS)


# ── what actually goes on the wire ───────────────────────────────────────────

def test_acceleration_lands_in_the_acceleration_fields():
    """Field order is positional in pymavlink: putting accel where velocity
    belongs would send a 4 m/s dodge as a 4 m/s^2 one, or the reverse."""
    b = _bridge()
    b.send_velocity_accel_body(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    (args,) = b.master.mav.sent
    # (time, sys, comp, frame, mask, x,y,z, vx,vy,vz, ax,ay,az, yaw, yaw_rate)
    assert args[4] == MASK_VEL_ACCEL
    assert args[5:8] == (0, 0, 0)          # position ignored
    assert args[8:11] == (1.0, 2.0, 3.0)   # velocity
    assert args[11:14] == (4.0, 5.0, 6.0)  # acceleration


def test_the_velocity_only_path_still_sends_zero_acceleration():
    b = _bridge()
    b.send_velocity_body(1.0, 2.0, 3.0)
    (args,) = b.master.mav.sent
    assert args[4] == MASK_VEL_ONLY
    assert args[8:11] == (1.0, 2.0, 3.0)
    assert args[11:14] == (0, 0, 0)


def test_a_nonzero_yaw_rate_is_un_ignored_on_both_paths():
    b = _bridge()
    b.send_velocity_body(0.0, 0.0, 0.0, 0.5)
    b.send_velocity_accel_body(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, yaw_rate=0.5)
    for args in b.master.mav.sent:
        assert not ignored(args[4], YAW_RATE_BIT)
        assert args[15] == 0.5


def test_a_zero_yaw_rate_stays_ignored():
    """Un-ignoring it would command "hold zero yaw rate", which fights any
    yaw the flight controller wants during the maneuver."""
    b = _bridge()
    b.send_velocity_accel_body(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    (args,) = b.master.mav.sent
    assert ignored(args[4], YAW_RATE_BIT)


def test_both_setpoints_use_the_body_offset_frame():
    """LOCAL_NED would reinterpret a body-forward dodge as a north-east one,
    so the escape direction would be wrong by the vehicle's heading.

    Asserted against the enum, not a literal: MAV_FRAME_BODY_NED (8) and
    MAV_FRAME_BODY_OFFSET_NED (9) are adjacent and easy to transpose, and a
    hardcoded number here would pin whichever one happened to be written."""
    from pymavlink import mavutil

    b = _bridge()
    b.send_velocity_body(1.0, 0.0, 0.0)
    b.send_velocity_accel_body(1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    frames = {args[3] for args in b.master.mav.sent}
    assert frames == {mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED}
