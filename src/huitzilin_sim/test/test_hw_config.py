"""The shipped parameter files, checked as data.

None of the hardware config can be validated by flying yet, so the invariants
that would otherwise only surface as a crash are asserted here.
"""

import pathlib

import pytest
import yaml

from huitzilin_sim.param_audit import check_fence_consistency, check_overlay
from huitzilin_sim.parm_file import parse_parm

SIM = pathlib.Path(__file__).resolve().parents[1]          # src/huitzilin_sim
SRC = SIM.parent                                            # src/
PERCEPTION = SRC / "huitzilin_perception"


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def params(doc, node):
    return doc[node]["ros__parameters"]


# ── .parm files ──────────────────────────────────────────────────────────────

# Every .parm in the package, discovered rather than listed. The hardcoded
# ["hw_frame.parm", "sitl_frame.parm"] looked exhaustive but only covered
# params/, so config/huitzilin_3p5_6s.parm -- the largest of the three, and the
# one that actually had bad content -- was never parsed. A new .parm must not be
# able to join the tree unchecked.
ALL_PARM_FILES = sorted(SIM.glob("params/*.parm")) + sorted(SIM.glob("config/*.parm"))


def test_the_parm_glob_finds_every_shipped_file():
    """Guards the parametrize below: an empty or short glob would make
    test_parm_files_parse_clean pass by testing nothing."""
    found = {p.name for p in ALL_PARM_FILES}
    assert {"hw_frame.parm", "sitl_frame.parm",
            "huitzilin_3p5_6s.parm"} <= found
    assert len(ALL_PARM_FILES) == len(found)      # no duplicate basenames


@pytest.mark.parametrize("path", ALL_PARM_FILES, ids=lambda p: p.name)
def test_parm_files_parse_clean(path):
    """No inline comments, no duplicate keys, every value numeric."""
    parse_parm(path.read_text())


@pytest.mark.parametrize("path", ALL_PARM_FILES, ids=lambda p: p.name)
def test_no_parm_file_sets_rtl_above_the_hardware_fence(path):
    """RTL_ALT is centimetres; FENCE_ALT_MAX is metres.

    Checked across every file, not only the one that declares a fence. A .parm
    with no FENCE_* lines still sets RTL_ALT on the same vehicle, so loading it
    after hw_frame.parm silently overwrites the deliberate 400 cm. That is how
    config/huitzilin_3p5_6s.parm carried RTL_ALT,500 -- equal to the 5 m ceiling
    -- while every fence-consistency check passed, because it had no fence of
    its own to be inconsistent with.
    """
    parms = parse_parm(path.read_text())
    if "RTL_ALT" not in parms:
        pytest.skip("no RTL_ALT in %s" % path.name)
    ceiling_m = parse_parm(
        (SIM / "params" / "hw_frame.parm").read_text())["FENCE_ALT_MAX"]
    assert parms["RTL_ALT"] / 100.0 < ceiling_m, (
        "%s sets RTL_ALT %s cm, not below the %s m hardware fence"
        % (path.name, parms["RTL_ALT"], ceiling_m))


def test_hw_frame_geofence_is_self_consistent():
    """The headline assertion: RTL_ALT (cm) must sit below FENCE_ALT_MAX (m).

    SAFETY_CASE.md section 2 sets a 5 m ceiling and separately describes RTL
    climbing to RTL_ALT, whose ArduPilot default is 1500 cm = 15 m. Left as
    written, a fence-breach RTL climbs through the fence it is responding to.
    Note the /100: a naive comparison would happily accept RTL_ALT 4, meaning
    4 cm.
    """
    parms = parse_parm((SIM / "params" / "hw_frame.parm").read_text())
    assert check_fence_consistency(parms) == []
    assert parms["RTL_ALT"] / 100.0 < parms["FENCE_ALT_MAX"]


def test_hw_frame_never_disables_arming_checks():
    parms = parse_parm((SIM / "params" / "hw_frame.parm").read_text())
    assert parms.get("ARMING_CHECK") != 0


def test_hw_frame_maps_a_kill_switch():
    """SAFETY_CASE.md section 3: motor emergency stop on a dedicated channel."""
    parms = parse_parm((SIM / "params" / "hw_frame.parm").read_text())
    assert any(k.startswith("RC") and k.endswith("_OPTION") and v == 31
               for k, v in parms.items())


def test_a_broken_fence_set_is_actually_caught():
    """Guards the check itself: an assertion that cannot fail proves nothing."""
    parms = parse_parm((SIM / "params" / "hw_frame.parm").read_text())
    parms["RTL_ALT"] = 1500.0          # the ArduPilot default, 15 m
    assert any("FENCE_ALT_MAX" in p for p in check_fence_consistency(parms))


# ── yaml overlays ────────────────────────────────────────────────────────────

OVERLAYS = [
    (SIM / "params" / "bridge.yaml", SIM / "params" / "hw_bridge.yaml"),
    (PERCEPTION / "params" / "detector.yaml",
     PERCEPTION / "params" / "hw_detector.yaml"),
    (PERCEPTION / "params" / "evasion.yaml",
     PERCEPTION / "params" / "hw_evasion.yaml"),
]


@pytest.mark.parametrize("base,overlay", OVERLAYS, ids=lambda p: p.name)
def test_overlay_matches_the_file_it_overlays(base, overlay):
    """Both failure modes are silent in ROS 2 at runtime: a mismatched node
    name loads none of the overlay, and an undeclared parameter name is simply
    ignored."""
    assert check_overlay(load_yaml(base), load_yaml(overlay)) == []


def test_overlay_check_catches_a_typoed_node_name():
    """Guards the check itself."""
    base = {"mav_bridge": {"ros__parameters": {"cmd_rate_hz": 10.0}}}
    typo = {"mav_brige": {"ros__parameters": {"cmd_rate_hz": 10.0}}}
    assert check_overlay(base, typo) != []


def test_no_params_yaml_bakes_in_use_sim_time():
    """Permanent enforcement of the S5-4 clock audit. A yaml that pins
    use_sim_time overrides the launch argument and re-creates exactly the
    silent wrong-clock failure the guard exists to prevent."""
    offenders = []
    for pkg in (SIM, PERCEPTION):
        for path in sorted((pkg / "params").glob("*.yaml")):
            for node, body in (load_yaml(path) or {}).items():
                if isinstance(body, dict) and "use_sim_time" in (
                        body.get("ros__parameters") or {}):
                    offenders.append("%s:%s" % (path.name, node))
    assert offenders == []


# ── cross-file couplings ─────────────────────────────────────────────────────

@pytest.mark.parametrize("bridge_name,evasion_name", [
    ("bridge.yaml", "evasion.yaml"),
    ("hw_bridge.yaml", "hw_evasion.yaml"),
])
def test_patrol_handoff_exceeds_cmd_timeout(bridge_name, evasion_name):
    """If handoff is not the longer of the two, the zero-velocity handback
    fights patrol's position setpoints. evasion.yaml documents the coupling;
    this is what stops the pair drifting apart."""
    bridge = params(load_yaml(SIM / "params" / bridge_name), "mav_bridge")
    evasion = params(load_yaml(PERCEPTION / "params" / evasion_name), "evasion")
    assert evasion["patrol_handoff_s"] > bridge["cmd_timeout_s"]


def test_hw_dodge_floor_is_raised_off_the_sim_value():
    """The sim floor of 1.0 m exists because a descending throw drove the
    drone into the runway at 2 m AGL. The real value comes from the enclosure
    (H8-1); only the direction can be asserted today."""
    sim = params(load_yaml(PERCEPTION / "params" / "evasion.yaml"), "evasion")
    hw = params(load_yaml(PERCEPTION / "params" / "hw_evasion.yaml"), "evasion")
    assert hw["dodge_floor_m"] > sim["dodge_floor_m"]


def test_hw_bridge_uses_a_stable_serial_path():
    """/dev/ttyACM0 renumbers when anything else enumerates first. The real id
    is unknown until H5-2, so only the shape can be asserted."""
    hw = params(load_yaml(SIM / "params" / "hw_bridge.yaml"), "mav_bridge")
    assert hw["connection"].startswith("/dev/serial/by-id/")


def test_supervisor_fence_matches_the_flight_controller_fence():
    """The supervisor's fence is a second line of defence. If it is looser
    than the FC's own, it never fires; if tighter, it pre-empts it."""
    sup = params(load_yaml(SIM / "params" / "supervisor.yaml"), "supervisor")
    parms = parse_parm((SIM / "params" / "hw_frame.parm").read_text())
    assert sup["fence_radius_m"] == parms["FENCE_RADIUS"]
    assert sup["fence_alt_max_m"] == parms["FENCE_ALT_MAX"]
