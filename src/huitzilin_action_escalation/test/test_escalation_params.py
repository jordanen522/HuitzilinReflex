"""The shipped parameter files, checked as data.

Runs without ROS (yaml only):
    python -m pytest src/huitzilin_action_escalation/test/test_escalation_params.py

None of this configuration can be validated against a real body yet, so the
invariants that would otherwise surface as a misbehaving alert are asserted
here instead.
"""

import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
PARAMS = sorted((PKG / "params").glob("*.yaml"))


def load(name):
    return yaml.safe_load((PKG / "params" / name).read_text(encoding="utf-8"))


def params(name, node):
    return load(name)[node]["ros__parameters"]


def test_the_params_glob_finds_every_shipped_file():
    """Guards the parametrize below."""
    found = {p.name for p in PARAMS}
    assert {"action_recognizer.yaml", "alert_signal.yaml",
            "scenario_player.yaml"} <= found
    assert len(PARAMS) == len(found)


@pytest.mark.parametrize("path", PARAMS, ids=lambda p: p.name)
def test_no_params_yaml_bakes_in_use_sim_time(path):
    """use_sim_time is a launch argument only. A yaml that pins it overrides
    the launch argument and re-creates the silent wrong-clock failure the
    clock guard exists to prevent."""
    for node, body in (yaml.safe_load(path.read_text(encoding="utf-8"))
                       or {}).items():
        assert "use_sim_time" not in ((body or {}).get("ros__parameters")
                                      or {}), "%s:%s" % (path.name, node)


def test_the_hysteresis_gap_is_the_right_way_round():
    """Inverted, the alert chatters on and off at the threshold."""
    p = params("action_recognizer.yaml", "action_recognizer")
    assert p["enter_score"] > p["exit_score"]
    assert 0.0 < p["exit_score"] < 1.0
    assert 0.0 < p["enter_score"] <= 1.0


def test_the_alert_is_bounded_at_both_ends():
    p = params("action_recognizer.yaml", "action_recognizer")
    assert p["min_alert_s"] < p["max_alert_s"]
    assert p["cooldown_s"] > 0.0
    assert p["min_confirm_frames"] >= 2, (
        "a single confirming frame would make every jitter spike an alert")


def test_staleness_outlasts_several_ticks():
    """A stale timeout shorter than a few tick periods turns a momentary gap
    into a dropout fault."""
    p = params("action_recognizer.yaml", "action_recognizer")
    assert p["stale_input_s"] > 3.0 / p["tick_hz"]


def test_the_alert_signal_dead_man_outlasts_the_recogniser_alert_cap():
    """If alert_signal cleared first the two would race at the boundary and
    the alert would flicker."""
    rec = params("action_recognizer.yaml", "action_recognizer")
    sig = params("alert_signal.yaml", "alert_signal")
    assert sig["max_on_s"] > rec["max_alert_s"]
    assert sig["stale_off_s"] > 3.0 / sig["tick_hz"]


def test_the_alert_backend_does_not_ship_as_hardware():
    """hardware is refused at runtime, but it must not be the shipped default
    either -- that would make every start log an error."""
    assert params("alert_signal.yaml", "alert_signal")["backend"] in (
        "sim", "none")


def test_the_recogniser_and_the_alert_agree_on_the_topic():
    """A typo here is silent: the alert node subscribes to a topic nothing
    publishes, and simply never fires."""
    assert (params("alert_signal.yaml", "alert_signal")["alert_topic"]
            == "/action/alert_request")


def test_the_player_loop_gap_exceeds_the_recogniser_cooldown():
    """Otherwise a looping scenario can never produce a second alert, and the
    loop reads as broken."""
    rec = params("action_recognizer.yaml", "action_recognizer")
    play = params("scenario_player.yaml", "scenario_player")
    assert play["loop_gap_s"] > rec["cooldown_s"]


def test_the_confirm_window_can_hold_the_required_frames():
    """min_confirm_frames must fit inside confirm_window_s at a plausible
    input rate, or confirmation is unreachable by construction."""
    p = params("action_recognizer.yaml", "action_recognizer")
    slowest_plausible_hz = 15.0
    assert (p["min_confirm_frames"] / slowest_plausible_hz
            <= p["confirm_window_s"] + 1e-9)
