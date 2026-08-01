"""LED strip and siren driving, with the hardware made optional.

Payload is a SAFETY SIGNAL ONLY. It announces that the aircraft believes it is
under threat. It is never used to follow, track, dazzle or harass a person
(SAFETY_CASE.md section 4).

Two pure pieces, both testable without a Pi:

`select_backend` never raises. SAFETY_CASE.md section 1 rates a GPIO/payload
fault Low -- log and continue -- so a missing rpi_ws281x, a missing gpiod or a
permission error must degrade to a no-op rather than take the flight stack
down with it. On any machine without the libraries, that degradation is the
path the tests exercise.

`AlarmLatch` is where the real safety content sits. evasion_node publishes the
alarm clear exactly once, at the end of a dodge, so a naive pass-through leaves
the siren latched on forever if that node dies mid-dodge.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Callable, Optional


class PayloadBackend:
    """Anything that can be turned on and off."""

    def set(self, on: bool) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class NullBackend(PayloadBackend):
    """Records calls and does nothing else. Never raises."""

    def __init__(self, why: str = ""):
        self.why = why
        self.calls = []

    def set(self, on: bool) -> None:
        self.calls.append(bool(on))


class Ws2812Backend(PayloadBackend):
    """WS2812B strip over rpi_ws281x. Imported lazily: the module only exists
    on a Pi and only imports successfully with /dev/mem access."""

    def __init__(self, strip):
        self._strip = strip

    def set(self, on: bool) -> None:
        colour = (255, 40, 0) if on else (0, 0, 0)
        for i in range(self._strip.numPixels()):
            self._strip.setPixelColorRGB(i, *colour)
        self._strip.show()


class GpioBuzzerBackend(PayloadBackend):
    """Piezo siren through a transistor switch on a GPIO line."""

    def __init__(self, line):
        self._line = line

    def set(self, on: bool) -> None:
        self._line.set_value(1 if on else 0)

    def close(self) -> None:
        try:
            self._line.release()
        except Exception:
            pass


class CompositeBackend(PayloadBackend):
    """Fan out to several backends, isolating each one's failures.

    A dead LED must not silence the siren, and neither must propagate into the
    node.
    """

    def __init__(self, backends):
        self.backends = list(backends)
        self.error_count = 0

    def set(self, on: bool) -> None:
        for b in self.backends:
            try:
                b.set(on)
            except Exception:
                self.error_count += 1

    def close(self) -> None:
        for b in self.backends:
            try:
                b.close()
            except Exception:
                self.error_count += 1


# Everything that can go wrong reaching for optional hardware. PermissionError
# matters as much as ImportError: rpi_ws281x imports fine and then fails on
# /dev/mem when the process is not privileged.
_BACKEND_ERRORS = (ImportError, OSError, PermissionError, AttributeError, ValueError)


def select_backend(want: str = "auto",
                   importer: Callable = importlib.import_module,
                   led_pin: int = 18, led_count: int = 8,
                   siren_chip: str = "gpiochip0", siren_line: int = 17):
    """Build the payload backend. Returns (backend, reasons).

    `reasons` lists every channel that degraded and why, so the node can log
    it once at startup instead of failing. This function never raises.
    """
    reasons = []

    if want == "none":
        return NullBackend("disabled by configuration"), reasons

    backends = []

    if want in ("auto", "ws2812"):
        try:
            mod = importer("rpi_ws281x")
            strip = mod.PixelStrip(led_count, led_pin)
            strip.begin()
            backends.append(Ws2812Backend(strip))
        except _BACKEND_ERRORS as e:
            reasons.append("rpi_ws281x unavailable (%s: %s)" % (type(e).__name__, e))

    if want in ("auto", "gpio"):
        try:
            mod = importer("gpiod")
            chip = mod.Chip(siren_chip)
            line = chip.get_line(siren_line)
            line.request(consumer="huitzilin_siren",
                         type=mod.LINE_REQ_DIR_OUT)
            backends.append(GpioBuzzerBackend(line))
        except _BACKEND_ERRORS as e:
            reasons.append("gpiod unavailable (%s: %s)" % (type(e).__name__, e))

    if not backends:
        return NullBackend("; ".join(reasons) or "no backend selected"), reasons
    return CompositeBackend(backends), reasons


@dataclass(frozen=True)
class AlarmPolicy:
    # A single True must still produce a signal a human can see and hear, even
    # if False follows on the next message.
    min_on_s: float = 0.5
    # Dead-man. evasion_node publishes the clear exactly once, at the end of a
    # dodge; if it dies mid-dodge nothing ever turns the siren off.
    max_on_s: float = 5.0
    # No message at all for this long means the publisher is gone. Silence is
    # not consent to keep the siren running.
    stale_off_s: float = 3.0


class AlarmLatch:
    """Turns a stream of alarm booleans into on/off actions.

    Returns True (turn on), False (turn off) or None (no change), so a caller
    never re-drives a GPIO line that is already in the right state.
    """

    def __init__(self, policy: Optional[AlarmPolicy] = None):
        self.policy = policy or AlarmPolicy()
        self._on = False
        self._on_since = 0.0
        self._last_msg_s = None

    @property
    def is_on(self) -> bool:
        return self._on

    def on_message(self, value: bool, now_s: float):
        self._last_msg_s = now_s
        if value:
            if self._on:
                return None                 # already on; no GPIO churn
            self._on = True
            self._on_since = now_s
            return True
        if not self._on:
            return None                     # already off; off is idempotent
        if now_s - self._on_since < self.policy.min_on_s:
            return None                     # hold the minimum visible pulse
        self._on = False
        return False

    def on_tick(self, now_s: float):
        """Time-driven clears the message stream cannot provide."""
        if not self._on:
            return None
        if now_s - self._on_since >= self.policy.max_on_s:
            self._on = False
            return False
        if (self._last_msg_s is not None
                and now_s - self._last_msg_s >= self.policy.stale_off_s):
            self._on = False
            return False
        return None
