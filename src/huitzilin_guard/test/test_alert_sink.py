"""The alert output, including the deliberate hardware refusal.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_guard/test/test_alert_sink.py
"""

import pytest

from huitzilin_guard.alert_sink import (
    HARDWARE_REFUSAL,
    AlertLatch,
    AlertLatchPolicy,
    NullAlertSink,
    SimAlertSink,
    select_alert_sink,
)


def test_the_sim_sink_publishes_what_it_is_given():
    seen = []
    sink, reasons = select_alert_sink("sim", publish=seen.append)
    assert isinstance(sink, SimAlertSink)
    assert reasons == []
    sink.set(True)
    sink.set(False)
    assert seen == [True, False]


def test_the_none_backend_is_inert_and_silent():
    sink, reasons = select_alert_sink("none", publish=lambda on: None)
    assert isinstance(sink, NullAlertSink)
    assert reasons == []


def test_the_hardware_backend_is_refused_and_says_why():
    """A second process on GPIO 17 would silently stop the
    PROJECTILE alarm from firing. The refusal has to name that, because a
    generic 'not supported' would read as unfinished work."""
    sink, reasons = select_alert_sink("hardware", publish=lambda on: None)
    assert isinstance(sink, NullAlertSink)
    assert len(reasons) == 1
    for fragment in ("payload_node", "GPIO 10", "GPIO 17", "exclusive"):
        assert fragment in reasons[0], fragment
    assert "projectile" in reasons[0].lower()


def test_the_refusal_never_raises_and_the_node_can_still_run():
    """SAFETY_CASE section 1 rates a payload fault log-and-continue. A node
    that died on a config value would invert that posture."""
    sink, _ = select_alert_sink("hardware", publish=lambda on: None)
    sink.set(True)
    sink.set(False)
    sink.close()
    assert sink.calls == [True, False]


def test_an_unknown_backend_degrades_to_sim_rather_than_failing():
    seen = []
    sink, reasons = select_alert_sink("ws2812", publish=seen.append)
    assert isinstance(sink, SimAlertSink)
    assert any("unknown backend" in r for r in reasons)


def test_a_missing_publish_callback_degrades_instead_of_raising():
    sink, reasons = select_alert_sink("sim", publish=None)
    assert isinstance(sink, NullAlertSink)
    assert any("publish callback" in r for r in reasons)


def test_the_hardware_refusal_constant_points_at_the_documented_fix():
    assert "docs/guard.md" in HARDWARE_REFUSAL


POLICY = AlertLatchPolicy(min_on_s=2.0, max_on_s=12.0, stale_off_s=3.0)


def test_the_latch_returns_none_rather_than_re_driving_a_held_state():
    latch = AlertLatch(POLICY)
    assert latch.on_message(True, 0.0) is True
    assert latch.on_message(True, 0.5) is None
    assert latch.on_message(True, 1.0) is None


def test_a_lone_true_still_produces_a_perceptible_pulse():
    latch = AlertLatch(POLICY)
    latch.on_message(True, 0.0)
    assert latch.on_message(False, 0.1) is None
    assert latch.is_on is True
    assert latch.on_message(False, 2.5) is False


def test_the_dead_man_bounds_an_alert_that_is_never_cleared():
    """max_on_s must bound the alert even while the publisher stays alive.

    The input is kept fresh on purpose. Letting it go quiet would clear the
    alert on stale_off_s first, and this test would silently be checking
    staleness rather than the dead-man.
    """
    latch = AlertLatch(POLICY)
    latch.on_message(True, 0.0)
    t = 0.0
    while t < POLICY.max_on_s - 1.0:
        t += 1.0
        latch.on_message(True, t)
        assert latch.on_tick(t) is None, "cleared early at t=%.1f" % t
    late = POLICY.max_on_s + 0.1
    latch.on_message(True, late)
    assert latch.on_tick(late) is False
    assert latch.is_on is False


def test_silence_turns_the_alert_off():
    latch = AlertLatch(POLICY)
    latch.on_message(True, 0.0)
    assert latch.on_tick(1.0) is None
    assert latch.on_tick(3.5) is False


def test_off_is_idempotent_from_rest():
    latch = AlertLatch(POLICY)
    assert latch.on_message(False, 0.0) is None
    assert latch.on_tick(1.0) is None


def test_the_alert_dead_man_outlasts_the_guards_alert_cap():
    """alert_signal must not clear before the guard does.

    The guard publishes a LEVEL, not a repeated edge, so if this node's
    dead-man expired first it would silence the siren and nothing would ever
    turn it back on: the guard would still be publishing True, and
    AlertLatch.on_message returns None for a True it is already holding. The
    SHIPPED values have to satisfy this, not merely the dataclass defaults,
    so both params files are read rather than the defaults trusted.
    """
    import pathlib

    import yaml

    from huitzilin_guard.presence import PresencePolicy

    params = pathlib.Path(__file__).resolve().parents[1] / "params"
    shipped = yaml.safe_load(
        (params / "alert_signal.yaml").read_text(encoding="utf-8"))
    max_on_s = shipped["alert_signal"]["ros__parameters"]["max_on_s"]
    guard = yaml.safe_load(
        (params / "guard.yaml").read_text(encoding="utf-8"))
    max_alert_s = guard["guard"]["ros__parameters"]["max_alert_s"]

    assert max_on_s > max_alert_s
    assert max_on_s > PresencePolicy().max_alert_s
