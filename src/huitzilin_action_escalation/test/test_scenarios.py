"""The shipped scenarios, end to end through window, score and policy.

Runs without ROS (yaml only):
    python -m pytest src/huitzilin_action_escalation/test/test_scenarios.py

WHAT A GREEN RUN HERE DOES AND DOES NOT MEAN. These scenarios are authored by
the same hand that set the thresholds, so the three `confirm` cases passing
is not a recall figure and never becomes one. The load-bearing cases are the
`no_confirm` controls: they are the only ones that can fail for a reason
nobody intended.
"""

import pathlib

import pytest
import yaml

from huitzilin_action_escalation import action_features as af
from huitzilin_action_escalation.escalation_policy import (
    EscalationLatch,
    EscalationPolicyConfig,
)
from huitzilin_action_escalation.pose_frame import parse_pose_frame
from huitzilin_action_escalation.scenario import (
    ScenarioRejected,
    load_scenario,
)

PKG = pathlib.Path(__file__).resolve().parents[1]
SCENARIO_FILES = sorted((PKG / "scenarios").glob("*.yaml"))


def test_the_scenario_glob_finds_every_shipped_file():
    """Guards every parametrize below: an empty or short glob would make the
    whole module pass by testing nothing."""
    found = {p.stem for p in SCENARIO_FILES}
    assert {"lunge", "strike", "shove", "benign_wave",
            "ambiguous_approach"} <= found
    assert len(SCENARIO_FILES) == len(found)


def test_the_suite_contains_negative_controls():
    """A suite of positives only cannot fail informatively. At least two
    scenarios must assert that nothing fires."""
    expects = [load(p).expect for p in SCENARIO_FILES]
    assert expects.count("no_confirm") >= 2
    assert expects.count("confirm") >= 1


def load(path):
    return load_scenario(yaml.safe_load(path.read_text(encoding="utf-8")))


@pytest.mark.parametrize("path", SCENARIO_FILES, ids=lambda p: p.stem)
def test_every_scenario_loads_and_expands(path):
    scenario = load(path)
    assert scenario.frames
    assert scenario.duration_s > 0.0


@pytest.mark.parametrize("path", SCENARIO_FILES, ids=lambda p: p.stem)
def test_every_generated_frame_survives_the_strict_parser(path):
    """The player publishes these verbatim, so a scenario that cannot pass
    the privacy parser would be dropped frame by frame at runtime."""
    for raw in load(path).frames:
        parse_pose_frame(raw)


def run(scenario):
    """Window -> score -> policy, on the scenario's own stamps."""
    window = af.FeatureWindow()
    latch = EscalationLatch(EscalationPolicyConfig())
    confirmed = False
    peak = 0.0
    for raw in scenario.frames:
        frame = parse_pose_frame(raw)
        window.push(frame)
        scores = af.score(window.features())
        peak = max(peak, max(scores.values()))
        action = latch.on_scores({c.value: v for c, v in scores.items()},
                                 frame.stamp_s, stamp_s=frame.stamp_s)
        if action is True:
            confirmed = True
    return confirmed, peak


@pytest.mark.parametrize("path", SCENARIO_FILES, ids=lambda p: p.stem)
def test_every_scenario_meets_its_declared_expectation(path):
    scenario = load(path)
    confirmed, peak = run(scenario)
    assert confirmed == (scenario.expect == "confirm"), (
        "%s expected %s but confirmed=%s (peak score %.2f)"
        % (scenario.name, scenario.expect, confirmed, peak))


def test_a_wave_scores_lower_than_a_strike_despite_a_faster_hand():
    """The conjunctive design, demonstrated on the shipped data.

    benign_wave reaches a HIGHER peak wrist speed than strike does. It scores
    near zero anyway, because a wave moves an already-extended arm instead of
    opening it. Under disjunctive scoring the wave would win.
    """
    def peaks(name):
        window = af.FeatureWindow()
        best_speed = 0.0
        best_strike = 0.0
        for raw in load(PKG / "scenarios" / ("%s.yaml" % name)).frames:
            window.push(parse_pose_frame(raw))
            feats = window.features()
            best_speed = max(best_speed, feats.wrist_speed_max_mps)
            best_strike = max(best_strike, af.score(feats)[af.Category.STRIKE])
        return best_speed, best_strike

    wave_speed, wave_strike = peaks("benign_wave")
    strike_speed, strike_strike = peaks("strike")
    assert wave_speed > strike_speed
    assert wave_strike < 0.3
    assert strike_strike > 0.9


def test_a_brisk_walk_does_not_read_as_a_lunge_or_a_shove():
    """The control that matters. Approach speed alone is not aggression."""
    scenario = load(PKG / "scenarios" / "ambiguous_approach.yaml")
    window = af.FeatureWindow()
    worst = {af.Category.LUNGE: 0.0, af.Category.SHOVE: 0.0}
    saw_real_approach = False
    for raw in scenario.frames:
        window.push(parse_pose_frame(raw))
        feats = window.features()
        if feats.approach_speed_mps > 1.2:
            saw_real_approach = True
        scores = af.score(feats)
        for cat in worst:
            worst[cat] = max(worst[cat], scores[cat])
    # Without this the test could pass on a scenario where nobody moves.
    assert saw_real_approach, "the control never actually closed distance"
    assert worst[af.Category.LUNGE] < 0.7
    assert worst[af.Category.SHOVE] < 0.7


@pytest.mark.parametrize("doc,fragment", [
    ({"name": "x", "expect": "maybe", "rate_hz": 15.0,
      "base": {"wrist_l": [0, 0, 0, 1]}, "keyframes": [{"t_s": 0.0}]},
     "expect"),
    ({"name": "x", "expect": "confirm", "rate_hz": 0,
      "base": {"wrist_l": [0, 0, 0, 1]}, "keyframes": [{"t_s": 0.0}]},
     "rate_hz"),
    ({"name": "x", "expect": "confirm", "rate_hz": 15.0,
      "base": {"nose": [0, 0, 0, 1]},
      "keyframes": [{"t_s": 0.0}, {"t_s": 1.0}]}, "whitelist"),
    ({"name": "x", "expect": "confirm", "rate_hz": 15.0, "surprise": 1,
      "base": {"wrist_l": [0, 0, 0, 1]}, "keyframes": [{"t_s": 0.0}]},
     "unknown key"),
])
def test_the_scenario_loader_rejects_malformed_input(doc, fragment):
    with pytest.raises(ScenarioRejected) as exc:
        load_scenario(doc)
    assert fragment in str(exc.value)
