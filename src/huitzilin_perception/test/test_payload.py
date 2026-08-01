"""Payload backend selection and the alarm latch.

Neither rpi_ws281x nor gpiod is installed on the development machines, so the
degradation path is not a corner case here -- it is the default path, and that
is deliberate. SAFETY_CASE.md section 1 rates a GPIO/payload fault Low: log and
continue. A dead LED must never take the flight stack with it.
"""

import pytest

from huitzilin_perception.payload import (
    AlarmLatch,
    AlarmPolicy,
    CompositeBackend,
    NullBackend,
    select_backend,
)


def raising(exc):
    def _importer(name):
        raise exc("no %s here" % name)
    return _importer


# ── backend selection never raises ───────────────────────────────────────────

def test_auto_degrades_to_a_null_backend_without_the_libraries():
    backend, reasons = select_backend("auto", importer=raising(ImportError))
    assert isinstance(backend, NullBackend)
    assert len(reasons) == 2
    assert any("rpi_ws281x" in r for r in reasons)
    assert any("gpiod" in r for r in reasons)


def test_a_permission_error_degrades_rather_than_crashes():
    """rpi_ws281x imports fine and then fails on /dev/mem unprivileged."""
    backend, reasons = select_backend("ws2812", importer=raising(PermissionError))
    assert isinstance(backend, NullBackend)
    assert reasons


@pytest.mark.parametrize("exc", [ImportError, OSError, PermissionError,
                                 AttributeError, ValueError])
def test_every_plausible_hardware_error_degrades(exc):
    backend, _ = select_backend("auto", importer=raising(exc))
    assert isinstance(backend, NullBackend)


def test_none_never_touches_the_importer():
    def explode(name):
        raise AssertionError("importer must not be called for want='none'")
    backend, reasons = select_backend("none", importer=explode)
    assert isinstance(backend, NullBackend)
    assert reasons == []


def test_a_degraded_backend_still_accepts_calls():
    """The node keeps driving the payload; it just goes nowhere."""
    backend, _ = select_backend("auto", importer=raising(ImportError))
    backend.set(True)
    backend.set(False)
    backend.close()
    assert backend.calls == [True, False]


# ── one failing channel must not silence the other ───────────────────────────

class _Boom:
    def set(self, on):
        raise RuntimeError("wiring fault")

    def close(self):
        pass


def test_a_failing_backend_does_not_stop_the_others():
    good = NullBackend()
    comp = CompositeBackend([_Boom(), good])
    comp.set(True)
    assert good.calls == [True]
    assert comp.error_count == 1


def test_composite_never_propagates():
    comp = CompositeBackend([_Boom(), _Boom()])
    comp.set(True)
    comp.close()
    assert comp.error_count >= 2


# ── the alarm latch ──────────────────────────────────────────────────────────

POLICY = AlarmPolicy(min_on_s=0.5, max_on_s=5.0, stale_off_s=3.0)


def test_a_single_true_still_produces_a_visible_pulse():
    """evasion_node could publish True then False almost immediately. A
    signal nobody can see is not a signal."""
    latch = AlarmLatch(POLICY)
    assert latch.on_message(True, 0.0) is True
    assert latch.on_message(False, 0.1) is None      # held
    assert latch.is_on
    assert latch.on_message(False, 0.6) is False     # released past min_on_s


def test_repeated_true_does_not_re_drive_the_gpio():
    latch = AlarmLatch(POLICY)
    assert latch.on_message(True, 0.0) is True
    assert latch.on_message(True, 0.1) is None
    assert latch.on_message(True, 1.0) is None


def test_off_is_idempotent():
    latch = AlarmLatch(POLICY)
    assert latch.on_message(False, 0.0) is None
    assert latch.on_message(False, 1.0) is None


def test_dead_man_clears_a_still_publishing_but_stuck_alarm():
    """The alarm is being refreshed, so staleness cannot explain the clear --
    this isolates max_on_s. A node that keeps asserting True past the longest
    possible dodge is stuck, and the siren still has to stop."""
    latch = AlarmLatch(POLICY)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0, 4.9):
        latch.on_message(True, t)
    assert latch.on_tick(4.9) is None
    latch.on_message(True, 5.0)
    assert latch.on_tick(5.0) is False
    assert not latch.is_on


def test_silence_clears_before_the_dead_man_does():
    """stale_off_s (3 s) is deliberately shorter than max_on_s (5 s): a
    publisher that has died is detectable sooner than one that is merely
    stuck, and the quieter failure should not wait for the louder bound."""
    assert POLICY.stale_off_s < POLICY.max_on_s
    latch = AlarmLatch(POLICY)
    latch.on_message(True, 0.0)
    assert latch.on_tick(2.9) is None
    assert latch.on_tick(3.0) is False


def test_silence_clears_the_alarm():
    """No messages at all means the publisher is gone. Silence is not consent
    to keep the siren running."""
    latch = AlarmLatch(POLICY)
    latch.on_message(True, 0.0)
    assert latch.on_tick(2.9) is None
    assert latch.on_tick(3.0) is False


def test_a_refreshed_alarm_is_not_treated_as_stale():
    latch = AlarmLatch(POLICY)
    latch.on_message(True, 0.0)
    for t in (1.0, 2.0, 3.0, 4.0):
        latch.on_message(True, t)
        assert latch.on_tick(t) is None
    assert latch.is_on


def test_ticking_while_off_does_nothing():
    latch = AlarmLatch(POLICY)
    for t in (0.0, 10.0, 1000.0):
        assert latch.on_tick(t) is None


def test_the_latch_can_be_re_armed_after_a_dead_man_clear():
    """A second throw after a stuck first one must still raise the alarm."""
    latch = AlarmLatch(POLICY)
    latch.on_message(True, 0.0)
    assert latch.on_tick(5.0) is False
    assert latch.on_message(True, 6.0) is True
