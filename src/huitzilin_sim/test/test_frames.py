"""Unit tests for MavBridge frame conversions (NED<->ENU, attitude).

pymavlink required (it is a dependency of huitzilin_sim); no SITL needed.
"""

import math

import numpy as np

from huitzilin_sim.mav_bridge import MavBridge


def _rot_from_quat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _body_x_in_enu(roll, pitch, yaw):
    q = MavBridge.ned_rpy_to_enu_quat(roll, pitch, yaw)
    return _rot_from_quat(*q) @ np.array([1.0, 0.0, 0.0])


def test_position_round_trip():
    n, e, d = 10.0, -3.0, -2.0
    assert MavBridge.enu_to_ned(*MavBridge.ned_to_enu(n, e, d)) == (n, e, d)


def test_yaw_north():
    # NED yaw 0 = facing North = ENU +y
    fwd = _body_x_in_enu(0.0, 0.0, 0.0)
    np.testing.assert_allclose(fwd, [0, 1, 0], atol=1e-9)


def test_yaw_east():
    # NED yaw +90° = facing East = ENU +x
    fwd = _body_x_in_enu(0.0, 0.0, math.pi / 2)
    np.testing.assert_allclose(fwd, [1, 0, 0], atol=1e-9)


def test_pitch_nose_up():
    # NED pitch +30° nose-up, facing North: forward gains +z (up) in ENU
    fwd = _body_x_in_enu(0.0, math.radians(30), 0.0)
    np.testing.assert_allclose(fwd, [0, math.cos(math.radians(30)),
                                     math.sin(math.radians(30))], atol=1e-9)


def test_roll_right_wing_down():
    # NED roll +20° (right wing down), facing North: body-left axis (ENU West
    # at yaw North) tilts UP — gains +z. Roll sign passes through unflipped.
    r = math.radians(20)
    q = MavBridge.ned_rpy_to_enu_quat(r, 0.0, 0.0)
    left = _rot_from_quat(*q) @ np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(left, [-math.cos(r), 0, math.sin(r)], atol=1e-9)


def test_quat_is_unit():
    q = MavBridge.ned_rpy_to_enu_quat(0.3, -0.2, 1.1)
    assert abs(sum(c * c for c in q) - 1.0) < 1e-9
