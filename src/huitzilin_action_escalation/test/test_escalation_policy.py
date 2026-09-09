"""Temporal confirmation, hysteresis, dead-man, cooldown and staleness.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_action_escalation/test/test_escalation_policy.py
"""

import pytest

from huitzilin_action_escalation.escalation_policy import (
    EscalationLatch,
    EscalationPolicyConfig,
    EscState,
)

CFG = EscalationPolicyConfig(
    enter_score=0.70, exit_score=0.45,
    min_confirm_frames=4, confirm_window_s=0.60,
    min_alert_s=2.0, max_alert_s=10.0,
    cooldown_s=15.0, stale_input_s=2.0)


def hot(value=0.9):
    return {"LUNGE": value, "STRIKE": 0.0, "SHOVE": 0.0}


def feed(latch, values, t0=0.0, dt=0.1):
    """Push a sequence of top-scores, returning every non-None action."""
    actions = []
    for i, value in enumerate(values):
        t = t0 + i * dt
        result = latch.on_scores(hot(value), t, stamp_s=t)
        if result is not None:
            actions.append((t, result))
    return actions


def test_a_single_high_frame_does_not_alert():
    latch = EscalationLatch(CFG)
    assert feed(latch, [0.9]) == []
    assert latch.is_alerting is False


def test_confirmation_needs_the_full_frame_count():
    latch = EscalationLatch(CFG)
    assert feed(latch, [0.9, 0.9, 0.9]) == []
    assert latch.state is EscState.CONFIRMING
    latch.on_scores(hot(), 0.3, stamp_s=0.3)
    assert latch.is_alerting is True


def test_frames_spread_wider_than_the_window_never_confirm():
    """Four hits at 0.5 s apart span 1.5 s, so the window never holds four."""
    latch = EscalationLatch(CFG)
    assert feed(latch, [0.9] * 6, dt=0.5) == []
    assert latch.is_alerting is False


def test_the_confirming_category_is_the_one_that_dominated_the_window():
    latch = EscalationLatch(CFG)
    for i in range(3):
        latch.on_scores({"LUNGE": 0.9, "STRIKE": 0.0}, i * 0.1,
                        stamp_s=i * 0.1)
    latch.on_scores({"LUNGE": 0.0, "STRIKE": 0.95}, 0.3, stamp_s=0.3)
    assert latch.category == "LUNGE"


def test_hysteresis_holds_the_alert_between_exit_and_enter():
    """A score at 0.60 is below enter but above exit: it must neither
    re-confirm nor clear, or the siren chatters."""
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    assert latch.is_alerting is True
    assert latch.on_scores(hot(0.60), 5.0, stamp_s=5.0) is None
    assert latch.is_alerting is True


def test_the_minimum_alert_time_survives_an_immediate_drop():
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    assert latch.on_scores(hot(0.0), 0.5, stamp_s=0.5) is None
    assert latch.is_alerting is True
    assert latch.on_scores(hot(0.0), 3.0, stamp_s=3.0) is False
    assert latch.is_alerting is False


def test_the_dead_man_clears_an_alert_the_input_never_clears():
    """A scorer stuck high must not latch the siren on indefinitely.

    Scores are kept flowing on purpose. Letting them stop would trip the
    staleness rule first, and this test would silently be checking that
    instead of max_alert_s.
    """
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    confirmed_at = 0.3
    t = confirmed_at
    while t < confirmed_at + CFG.max_alert_s - 1.0:
        t += 0.5
        latch.on_scores(hot(), t, stamp_s=t)
        assert latch.on_tick(t) is None, "cleared early at t=%.1f" % t
    late = confirmed_at + CFG.max_alert_s + 0.2
    latch.on_scores(hot(), late, stamp_s=late)
    assert latch.on_tick(late) is False
    assert latch.is_alerting is False


def test_cooldown_blocks_an_immediate_second_alert():
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    latch.on_scores(hot(0.0), 3.0, stamp_s=3.0)
    assert feed(latch, [0.9] * 6, t0=3.1) == []
    assert latch.state is EscState.COOLDOWN
    assert latch.cooldown_remaining_s(4.0) > 0.0
    assert latch.cooldown_remaining_s(100.0) == 0.0


def test_a_second_alert_is_possible_once_cooldown_elapses():
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    latch.on_scores(hot(0.0), 3.0, stamp_s=3.0)
    actions = feed(latch, [0.9] * 4, t0=30.0)
    assert [a for _, a in actions] == [True]


def test_silence_forces_degraded_and_drops_the_alert():
    """Silence is not consent to keep a siren running."""
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    assert latch.on_tick(0.5) is None
    assert latch.on_tick(2.5) is False
    assert latch.state is EscState.DEGRADED
    assert latch.is_alerting is False


def test_a_frozen_producer_reads_as_stale_not_as_a_sustained_alert():
    """A repeated identical stamp carries no new evidence.

    This is the case a message-age check alone gets wrong: the frames keep
    arriving, so liveness looks fine while the last score is held forever.
    """
    latch = EscalationLatch(CFG)
    for i in range(3):
        latch.on_scores(hot(), i * 0.1, stamp_s=i * 0.1)
    for i in range(20):
        assert latch.on_scores(hot(), 0.3 + i * 0.1, stamp_s=0.2) is None
    assert latch.is_alerting is False
    assert "did not advance" in latch.last_reason
    assert latch.on_tick(5.0) is None
    assert latch.state is EscState.DEGRADED


def test_input_resuming_after_degraded_can_alert_again():
    latch = EscalationLatch(CFG)
    latch.on_scores(hot(), 0.0, stamp_s=0.0)
    latch.on_tick(5.0)
    assert latch.state is EscState.DEGRADED
    actions = feed(latch, [0.9] * 4, t0=6.0)
    assert [a for _, a in actions] == [True]


def test_the_latch_never_re_drives_a_state_it_is_already_in():
    """None means no change, so the node never re-drives an output."""
    latch = EscalationLatch(CFG)
    feed(latch, [0.9] * 4)
    for i in range(10):
        assert latch.on_scores(hot(), 1.0 + i * 0.1,
                               stamp_s=1.0 + i * 0.1) is None
    assert latch.on_tick(1.5) is None


def test_an_empty_score_map_is_inert():
    latch = EscalationLatch(CFG)
    for i in range(10):
        assert latch.on_scores({}, i * 0.1, stamp_s=i * 0.1) is None


def test_the_configured_hysteresis_gap_is_the_right_way_round():
    assert CFG.enter_score > CFG.exit_score
    assert CFG.min_alert_s < CFG.max_alert_s
