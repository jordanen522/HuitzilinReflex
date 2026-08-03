"""The evasion node's live parameter surface.

`ros2 param set` is all-or-nothing: if the callback returns unsuccessful, ROS
rolls back every declared parameter in the request. The shadow dict `self._p`
is what the node actually reads at runtime, so it has to honour the same
all-or-nothing rule -- otherwise a rejected set leaves the two disagreeing and
`ros2 param get` reports something the node is not using.

This matters because the dodge battery sweeps by sending several keys in one
call; a half-applied set would mislabel a scored row with no visible symptom.
"""

import pytest

from huitzilin_perception.evasion_node import FIXED_AT_START, EvasionNode


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
