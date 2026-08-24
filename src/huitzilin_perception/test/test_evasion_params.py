"""The evasion node's live parameter surface.

`ros2 param set` is all-or-nothing: if the callback returns unsuccessful, ROS
rolls back every declared parameter in the request. The shadow dict `self._p`
is what the node actually reads at runtime, so it has to honour the same
all-or-nothing rule -- otherwise a rejected set leaves the two disagreeing and
`ros2 param get` reports something the node is not using.

This matters because the dodge battery sweeps by sending several keys in one
call; a half-applied set would mislabel a scored row with no visible symptom.
"""

import math

import numpy as np
import pytest

from huitzilin_perception.cloud_geometry import quat_to_rot, rotate_covariance
from huitzilin_perception.evasion_node import (
    FIXED_AT_START,
    EvasionNode,
    _resolve_meas_std_xyz,
)
from huitzilin_perception.kalman import ProjectileTracker


class _Prm:
    """Stands in for rcl_interfaces Parameter: only .name and .value are read."""

    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(msg)


def _node(**overrides):
    """An EvasionNode with only the parameter surface wired up.

    __init__ would open subscriptions, publishers and a service client, none
    of which _on_param_set touches.
    """
    n = object.__new__(EvasionNode)
    n._p = {
        "dodge_speed_mps": 1.5,
        "trigger_horizon_s": 1.5,
        "min_track_updates": 3,
        "meas_std_m": 0.15,          # declared, but FIXED_AT_START
    }
    n._p.update(overrides)
    n.get_logger = lambda: _Logger()
    return n


def test_a_valid_multi_key_set_applies_every_key():
    n = _node()
    res = n._on_param_set([_Prm("dodge_speed_mps", 2.5),
                           _Prm("trigger_horizon_s", 2.0)])
    assert res.successful is True
    assert n._p["dodge_speed_mps"] == 2.5
    assert n._p["trigger_horizon_s"] == 2.0


def test_a_rejected_set_applies_nothing_even_before_the_bad_key():
    """The regression. dodge_speed_mps is legal and comes first; meas_std_m is
    fixed at start and comes second. ROS rolls both back, so self._p must too.
    """
    n = _node()
    res = n._on_param_set([_Prm("dodge_speed_mps", 4.0),
                           _Prm("meas_std_m", 0.99)])
    assert res.successful is False
    assert "meas_std_m" in res.reason
    assert n._p["dodge_speed_mps"] == 1.5, (
        "a legal key applied despite the request being rejected -- self._p and "
        "ros2 param get now disagree")
    assert n._p["meas_std_m"] == 0.15


def test_the_fixture_really_uses_a_fixed_key():
    """Guards the test above: if meas_std_m stopped being fixed at start, the
    rejection would never fire and the assertion would pass vacuously."""
    assert "meas_std_m" in FIXED_AT_START
    assert "dodge_speed_mps" not in FIXED_AT_START


@pytest.mark.parametrize("name", sorted(FIXED_AT_START))
def test_every_fixed_key_is_refused_on_its_own(name):
    n = _node(**{name: "sentinel"})
    res = n._on_param_set([_Prm(name, "changed")])
    assert res.successful is False
    assert n._p[name] == "sentinel"


def test_unknown_keys_are_accepted_but_not_added_to_the_shadow_dict():
    """ROS only routes declared parameters here, so an unknown name means the
    declaration list and self._p have drifted apart. Silently creating the key
    would hide that; the node reads self._p, not the declaration."""
    n = _node()
    res = n._on_param_set([_Prm("not_a_parameter", 1)])
    assert res.successful is True
    assert "not_a_parameter" not in n._p


# anisotropic measurement covariance (meas_std_xyz_m)
#
# The gap this closes: nothing ever passed the tracker a per-measurement R,
# so the filter always trusted an isotropic meas_std_m even when the real
# sensor's depth and bearing noise differ by an order of magnitude. See
# test_kalman.py's own "per-measurement covariance" section for the filter
# math -- everything below is evasion_node's WIRING of it, which is the part
# that did not exist.

class _FakeTracker:
    """Stands in for MultiHypothesisTracker: only .process() is read, and
    only to record the exact positional args it was called with."""

    def __init__(self):
        self.calls = []

    def process(self, *args):
        self.calls.append(args)


def _tracker_node(meas_std_xyz):
    n = object.__new__(EvasionNode)
    n._meas_std_xyz = meas_std_xyz
    n._tracker = _FakeTracker()
    return n


def test_resolve_meas_std_xyz_all_zero_disables_the_anisotropic_path():
    assert _resolve_meas_std_xyz([0.0, 0.0, 0.0]) is None


def test_resolve_meas_std_xyz_any_nonzero_component_enables_it():
    result = _resolve_meas_std_xyz([0.0, 0.02, 0.0])
    np.testing.assert_array_equal(result, [0.0, 0.02, 0.0])


@pytest.mark.parametrize("n_values", [2, 4])
def test_resolve_meas_std_xyz_wrong_length_raises_at_startup(n_values):
    with pytest.raises(ValueError, match="meas_std_xyz_m"):
        _resolve_meas_std_xyz([0.1] * n_values)


def test_meas_std_xyz_m_is_fixed_at_start():
    """Guards the parametrized FIXED_AT_START sweep below: if this key ever
    stopped being fixed, that test would pass vacuously."""
    assert "meas_std_xyz_m" in FIXED_AT_START


def test_anisotropic_r_disabled_by_default_reaches_the_tracker_unset():
    """The acceptance criterion for this whole feature: with the feature off
    the tracker gets EXACTLY the old two-argument call -- never R=None, never
    a computed-but-equal R. Asserted on the mock's recorded call, not on
    floats."""
    n = _tracker_node(None)
    z = np.array([1.0, 2.0, 3.0])
    n._update_tracker(1.23, z, np.eye(3))

    assert len(n._tracker.calls) == 1
    args = n._tracker.calls[0]
    assert len(args) == 2, "R was passed even though meas_std_xyz_m is disabled"
    t, z_out = args
    assert t == 1.23
    np.testing.assert_array_equal(z_out, z)


def test_anisotropic_r_enabled_rotates_the_body_std_into_odom():
    """When enabled, the tracker gets a THIRD argument: the body-frame std,
    diagonalised and rotated into odom by the SAME rotation used to lift the
    point itself."""
    n = _tracker_node(np.array([0.30, 0.02, 0.02]))
    z = np.array([1.0, 2.0, 3.0])
    yaw = math.radians(90)
    Rot = quat_to_rot(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

    n._update_tracker(4.56, z, Rot)

    assert len(n._tracker.calls) == 1
    args = n._tracker.calls[0]
    assert len(args) == 3
    t, z_out, R = args
    assert t == 4.56
    np.testing.assert_array_equal(z_out, z)
    expected = rotate_covariance(np.diag([0.30 ** 2, 0.02 ** 2, 0.02 ** 2]), Rot)
    np.testing.assert_allclose(R, expected)


def test_a_loose_body_forward_axis_is_still_corrected_slowly_after_rotation():
    """End-to-end through the wiring this feature adds, not just the isolated
    filter math (already covered in test_kalman.py): a LOOSE forward (body x)
    std, rotated 90 deg into odom where forward becomes odom +y, must still
    correct more slowly than a TIGHT axis -- proving the covariance is
    rotated in the same sense as the point, not left in the body frame or
    rotated the wrong way."""
    truth = np.array([0.0, 10.0, 2.0])
    offset = np.array([0.0, 1.0, 1.0])  # 1 m disagreement along odom y and z

    tr = ProjectileTracker()
    tr.process(0.0, truth)

    yaw = math.radians(90)
    Rot = quat_to_rot(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    meas_std_xyz = np.array([5.0, 0.02, 0.02])  # forward loose; left, up tight
    R_body = np.diag(np.square(meas_std_xyz))
    R_odom = rotate_covariance(R_body, Rot)

    tr.process(1.0 / 15.0, truth + offset, R_odom)

    pos, _ = tr.state()
    moved_y = abs(pos[1] - truth[1])  # odom y <- body forward (loose)
    moved_z = abs(pos[2] - truth[2])  # odom z <- body up (tight)
    assert moved_z > moved_y * 5.0, (
        "loose forward-mapped axis moved %.4f, tight axis moved %.4f"
        % (moved_y, moved_z))
