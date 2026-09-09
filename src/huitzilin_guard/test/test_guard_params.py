"""guard.yaml as data: the two box blocks must agree, and must be flyable.

Runs without ROS (yaml only):
    python -m pytest src/huitzilin_guard/test/test_guard_params.py

The failure this module exists to prevent has no symptom. If the patrol block
and the guard block disagree, the drone flies one rectangle and alarms on
another: it takes off, follows a circuit, the detector detects, the siren
stays quiet, and every log line looks correct while the area nobody asked
about is the one being guarded.
"""

import pathlib

import pytest
import yaml

from huitzilin_sim.box import (
    BOX_PARAM_NAMES,
    box_from_params,
    check_fence_radius,
)

PKG = pathlib.Path(__file__).resolve().parents[1]
GUARD_YAML = PKG / "params" / "guard.yaml"
PARAMS_FILES = sorted((PKG / "params").glob("*.yaml"))


def load(node):
    doc = yaml.safe_load(GUARD_YAML.read_text(encoding="utf-8"))
    return doc[node]["ros__parameters"]


def test_the_config_file_exists_and_declares_both_nodes():
    """Guards every check below: a renamed node key would otherwise make the
    comparisons pass by comparing nothing."""
    doc = yaml.safe_load(GUARD_YAML.read_text(encoding="utf-8"))
    assert set(doc) == {"patrol", "guard"}, sorted(doc)


def test_the_patrol_and_guard_blocks_describe_exactly_the_same_box():
    """The headline assertion. One rectangle, written twice because two
    nodes need it, and the two copies must never drift."""
    patrol, guard = load("patrol"), load("guard")
    for name in BOX_PARAM_NAMES:
        assert name in patrol, "patrol is missing %s" % name
        assert name in guard, "guard is missing %s" % name
        assert patrol[name] == guard[name], (
            "%s is %r for patrol and %r for guard. The drone would patrol "
            "one rectangle and guard another, with nothing looking wrong."
            % (name, patrol[name], guard[name]))


def test_the_comparison_would_actually_catch_a_disagreement():
    """Positive control. If BOX_PARAM_NAMES were ever emptied the test above
    would pass while comparing nothing at all."""
    assert len(BOX_PARAM_NAMES) == 5
    patrol = dict(load("patrol"))
    patrol["box_max_x"] = patrol["box_max_x"] + 1.0
    assert patrol["box_max_x"] != load("guard")["box_max_x"]


def test_the_shipped_box_is_a_valid_rectangle():
    """box_from_params raises on an inverted, zero-area or non-finite box."""
    box = box_from_params(load("guard").__getitem__)
    assert box.width_m > 0.0
    assert box.depth_m > 0.0


def test_the_shipped_waypoints_are_the_four_corners_of_the_shipped_box():
    """The patrol flies the perimeter and nothing else: four waypoints, one
    per corner, in the order corners_enu gives them."""
    box = box_from_params(load("patrol").__getitem__)
    flat = box.to_waypoints_ned()
    assert len(flat) == 12, "four corners, three values each"

    corners = box.corners_enu()
    assert len(corners) == 4
    assert len(set(corners)) == 4, "a repeated corner is a degenerate circuit"


def test_the_shipped_box_fits_inside_the_declared_geofence():
    """Shipping a default that breaches the fence would mean the very first
    flight anybody attempts ends in an RTL halfway round."""
    patrol = load("patrol")
    box = box_from_params(patrol.__getitem__)
    check_fence_radius(box, float(patrol["fence_radius_m"]))


def test_the_declared_fence_matches_the_flight_controller_parameter():
    """fence_radius_m is a mirror of FENCE_RADIUS, not an independent knob.
    A looser mirror silently permits a box the aircraft will not fly."""
    parm = PKG.parent / "huitzilin_sim" / "params" / "hw_frame.parm"
    if not parm.is_file():
        pytest.skip("hw_frame.parm not present in this checkout")
    declared = None
    for line in parm.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("FENCE_RADIUS"):
            declared = float(line.split()[1])
    assert declared is not None, "FENCE_RADIUS not found in hw_frame.parm"
    assert float(load("patrol")["fence_radius_m"]) == declared


def test_the_box_is_actually_switched_on_in_the_shipped_config():
    """box_enabled defaults FALSE in patrol_node so the dodge battery keeps
    its measured circuit. guard.yaml is the file that turns it on, and a
    false here would fly the old literal waypoints while guarding the box."""
    assert load("patrol")["box_enabled"] is True


def test_patrol_does_not_start_itself():
    """Setpoints sent during takeoff flood GUIDED and the aircraft never
    leaves the ground."""
    assert load("patrol")["autostart"] is False


def test_the_guard_starts_disarmed():
    """Like any alarm panel. A guard that armed itself on boot would sound
    during setup, and the person silencing it is the person it exists to
    detect."""
    assert load("guard")["start_armed"] is False


def test_the_alarm_windows_are_ordered_so_the_dead_man_cannot_cut_it_short():
    guard = load("guard")
    assert guard["max_alert_s"] > guard["min_alert_s"]
    assert guard["stale_input_s"] >= guard["min_alert_s"]
    assert guard["confirm_frames"] >= 1


@pytest.mark.parametrize("path", PARAMS_FILES, ids=lambda p: p.name)
def test_no_params_file_pins_use_sim_time(path):
    """use_sim_time is a launch argument, never a yaml key. Pinned in a
    params file it silently overrides the launch, and a node with sim time
    and no /clock freezes at t=0 instead of failing."""
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for node, body in doc.items():
        assert "use_sim_time" not in ((body or {}).get("ros__parameters")
                                      or {}), "%s:%s" % (path.name, node)
