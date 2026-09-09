"""Kinematics and the conjunctive aggression score.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_action_escalation/test/test_action_features.py
"""

import pytest

from huitzilin_action_escalation import action_features as af
from huitzilin_action_escalation.pose_frame import PoseFrame

BASE = {
    "shoulder_l": (-0.20, -0.50, 4.00),
    "shoulder_r": (0.20, -0.50, 4.00),
    "elbow_l": (-0.25, -0.22, 3.98),
    "elbow_r": (0.25, -0.22, 3.98),
    "wrist_l": (-0.28, 0.05, 3.96),
    "wrist_r": (0.28, 0.05, 3.96),
    "hip_l": (-0.15, 0.00, 4.00),
    "hip_r": (0.15, 0.00, 4.00),
}


def test_a_single_frame_yields_no_rates():
    win = af.FeatureWindow()
    win.push(_pf(0.0))
    feats = win.features()
    assert feats.n_frames == 1
    assert feats.approach_speed_mps == 0.0
    assert feats.wrist_speed_max_mps == 0.0


def test_a_non_advancing_stamp_contributes_no_rate_and_never_divides_by_zero():
    """The frozen-frame case. A repeated stamp must not produce an infinity.

    This is distinct from the missing-frame case: a producer republishing an
    identical frame keeps arriving, so only the stamp distinguishes it.
    """
    win = af.FeatureWindow()
    win.push(_pf(1.0))
    win.push(_pf(1.0, dz=-1.0))
    feats = win.features()
    for value in (feats.approach_speed_mps, feats.body_speed_mps,
                  feats.wrist_speed_max_mps, feats.torso_lean_rate_radps,
                  feats.wrist_extension_rate_mps,
                  feats.bilateral_reach_rate_mps):
        assert value == 0.0


def test_frames_outside_the_window_are_evicted():
    win = af.FeatureWindow(af.FeatureConfig(window_s=0.2))
    for i in range(10):
        win.push(_pf(i * 0.1))
    assert len(win) <= 3
    assert win.features().span_s <= 0.2 + 1e-9


def test_low_confidence_joints_are_treated_as_absent():
    """A pose estimator reports a confidence for an occluded limb rather than
    omitting it, and a hallucinated wrist moves fast."""
    win = af.FeatureWindow(af.FeatureConfig(min_joint_conf=0.5))
    win.push(_pf(0.0, conf=0.1))
    win.push(_pf(0.1, dz=-1.0, conf=0.1))
    assert win.features().approach_speed_mps == 0.0


def test_approach_speed_measures_closing_not_receding():
    win = af.FeatureWindow()
    win.push(_pf(0.0))
    win.push(_pf(0.1, dz=-0.2))
    assert win.features().approach_speed_mps == pytest.approx(2.0, rel=1e-6)

    away = af.FeatureWindow()
    away.push(_pf(0.0))
    away.push(_pf(0.1, dz=0.2))
    assert away.features().approach_speed_mps == 0.0


def test_scores_are_clamped_to_the_unit_interval():
    feats = af.Features(approach_speed_mps=99.0, body_speed_mps=99.0,
                        torso_lean_rate_radps=99.0,
                        wrist_speed_max_mps=99.0,
                        wrist_extension_rate_mps=99.0,
                        bilateral_reach_rate_mps=99.0, n_frames=5)
    for value in af.score(feats).values():
        assert 0.0 <= value <= 1.0
    zero = af.score(af.Features(n_frames=5))
    assert set(zero.values()) == {0.0}


def test_the_category_set_is_aggressive_actions_only():
    """Scope guard. No benign class, and none of the two cut categories.

    A benign category would turn this from an alarm into an activity log,
    which is a different system with different privacy properties.
    """
    assert {c.value for c in af.Category} == {"LUNGE", "STRIKE", "SHOVE"}


def test_scoring_is_conjunctive_so_one_strong_term_cannot_fire_a_category():
    """Fast approach alone is a brisk walk, not a lunge.

    Under disjunctive scoring this returns ~1.0 and every walking bystander
    trips the siren.
    """
    walk = af.Features(approach_speed_mps=5.0, body_speed_mps=5.0,
                       torso_lean_rate_radps=0.0, n_frames=5)
    assert af.score(walk)[af.Category.LUNGE] == 0.0

    wave = af.Features(wrist_speed_max_mps=9.0,
                       wrist_extension_rate_mps=0.0, n_frames=5)
    assert af.score(wave)[af.Category.STRIKE] == 0.0


def test_a_one_armed_extension_is_not_a_shove():
    """SHOVE takes the minimum of the two arms, so a single arm scores zero
    however fast it moves. That is what separates a strike from a shove."""
    one_arm = af.Features(bilateral_reach_rate_mps=0.0,
                          approach_speed_mps=2.0, n_frames=5)
    assert af.score(one_arm)[af.Category.SHOVE] == 0.0


def test_the_strike_extension_ramp_stays_within_arm_reach():
    """Guards against a threshold no anatomy can satisfy.

    Shoulder-to-wrist travel is bounded by arm length, about 0.6 m, so an
    extension-rate ceiling above roughly 3 m/s cannot be reached by a real
    strike and would silently make STRIKE unfirable.
    """
    lo, hi = af.ScoreConfig().extension_rate_strike
    assert lo < hi <= 3.5


def test_top_category_is_stable_under_ties():
    scores = {af.Category.LUNGE: 0.5, af.Category.STRIKE: 0.5,
              af.Category.SHOVE: 0.5}
    assert af.top_category(scores)[0] is af.top_category(scores)[0]
    assert af.top_category({})[0] is None


def _pf(t, dx=0.0, dy=0.0, dz=0.0, conf=0.9):
    from huitzilin_action_escalation.pose_frame import Joint
    joints = {name: Joint(x + dx, y + dy, z + dz, conf)
              for name, (x, y, z) in BASE.items()}
    return PoseFrame(stamp_s=t, subject=0, joints=joints)
