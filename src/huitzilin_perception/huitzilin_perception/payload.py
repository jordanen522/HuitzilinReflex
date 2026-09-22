"""LED strip and siren driving, with the hardware made optional.

Payload is a SAFETY SIGNAL ONLY. It announces that the aircraft believes it is
under threat. It is never used to follow, track, dazzle or harass a person
(SAFETY_CASE.md section 4).

Two pure pieces, both testable without a Pi:

`select_backend` never raises. SAFETY_CASE.md section 1 rates a GPIO/payload
fault Low -- log and continue -- so a missing spidev, a missing gpiod or a
permission error must degrade to a no-op rather than take the flight stack
down with it.

Both channels target the Raspberry Pi 5. rpi_ws281x, the usual WS2812 library,
does not support the Pi 5's RP1 I/O chip, so the strip is driven over SPI.
libgpiod changed its Python API completely between v1 and v2 and a distro may
ship either, so the siren speaks both. On any machine without the libraries, that degradation is the
path the tests exercise.

`AlarmLatch` is where the real safety content sits. evasion_node publishes the
alarm clear exactly once, at the end of a dodge, so a naive pass-through leaves
the siren latched on forever if that node dies mid-dodge.
"""

from __future__ import annotations

import importlib
import os
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


# WS2812 over SPI: one SPI byte per LED bit at 6.4 MHz (156 ns per SPI bit).
# A 1 is high for 5 SPI bits (781 ns), a 0 for 2 (312 ns); both inside the
# WS2812B windows. A whole byte per bit means any gap the SPI controller
# inserts between bytes only stretches a low period, which the LED tolerates.
WS2812_SPI_HZ = 6_400_000
_WS_ONE = 0b11111000
_WS_ZERO = 0b11000000
# Latch: the line held low for > 280 us (newer WS2812B parts) = 240 bytes.
_WS_RESET = bytes(240)
ALARM_RGB = (255, 40, 0)


def ws2812_spi_frame(pixels) -> bytes:
    """SPI bytes for a list of (r, g, b). The LED wants GRB, MSB first."""
    out = bytearray()
    for r, g, b in pixels:
        for byte in (g, r, b):
            for bit in range(7, -1, -1):
                out.append(_WS_ONE if (byte >> bit) & 1 else _WS_ZERO)
    return bytes(out) + _WS_RESET


class Ws2812Backend(PayloadBackend):
    """WS2812B strip on SPI0 MOSI (GPIO10) through spidev."""

    def __init__(self, spi, count: int):
        self._spi = spi
        self._count = count

    def set(self, on: bool) -> None:
        colour = ALARM_RGB if on else (0, 0, 0)
        self._spi.writebytes2(ws2812_spi_frame([colour] * self._count))

    def close(self) -> None:
        self._spi.close()


class _GpiodV2Line:
    """libgpiod v2 line request, shaped like a v1 line (set_value/release)."""

    def __init__(self, mod, request, offset):
        self._value = mod.line.Value
        self._request = request
        self._offset = offset

    def set_value(self, v: int) -> None:
        self._request.set_value(
            self._offset, self._value.ACTIVE if v else self._value.INACTIVE)

    def release(self) -> None:
        self._request.release()


# The Pi 5's 40-pin header lives on the RP1 chip. Which /dev/gpiochipN that is
# depends on the kernel (4 on older ones, 0 on newer), and on the wrong chip
# line 17 is a different pin -- the request succeeds and the siren never sounds.
RP1_LABEL = "pinctrl-rp1"


def _chip_label(mod, path: str) -> str:
    if hasattr(mod, "request_lines"):                        # v2
        with mod.Chip(path) as chip:
            return chip.get_info().label
    return mod.Chip(path).label()                            # v1


def find_gpiochip(mod, label: str = RP1_LABEL, listdir=os.listdir) -> str:
    """/dev/gpiochipN whose label matches. Raises OSError if none does."""
    for name in sorted(listdir("/dev")):
        if not name.startswith("gpiochip"):
            continue
        path = "/dev/" + name
        try:
            if _chip_label(mod, path) == label:
                return path
        except OSError:
            continue
    raise OSError("no gpiochip labelled %r" % label)


def request_output_line(mod, chip: str, line: int, consumer: str):
    """An output line, driven low, from either libgpiod API."""
    path = chip if chip.startswith("/dev/") else "/dev/" + chip
    if hasattr(mod, "request_lines"):                        # v2
        settings = mod.LineSettings(direction=mod.line.Direction.OUTPUT,
                                    output_value=mod.line.Value.INACTIVE)
        request = mod.request_lines(path, consumer=consumer,
                                    config={line: settings})
        return _GpiodV2Line(mod, request, line)
    handle = mod.Chip(path).get_line(line)                   # v1
    handle.request(consumer=consumer, type=mod.LINE_REQ_DIR_OUT,
                   default_vals=[0])
    return handle


class GpioBuzzerBackend(PayloadBackend):
    """Piezo siren through a transistor switch on a GPIO line."""

    def __init__(self, line):
        self._line = line
        self.close_error = None

    def set(self, on: bool) -> None:
        self._line.set_value(1 if on else 0)

    def close(self) -> None:
        try:
            self._line.release()
        except Exception as exc:            # noqa: BLE001
            # Still swallowed -- shutdown must continue -- but recorded rather
            # than discarded. A line that will not release may be leaving the
            # siren asserted, and a bare `pass` left no evidence of the one
            # failure at shutdown you would actually want to hear about.
            self.close_error = exc


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


# Everything that can go wrong reaching for optional hardware. The failures
# that matter most are the ones *after* a clean import: opening /dev/spidev0.0
# or a gpiochip without the right group fails as PermissionError (an OSError),
# a busy line as OSError, and an unexpected library version as AttributeError
# or TypeError. Any of them escaping would kill node construction, against
# this function's "never raises" contract and SAFETY_CASE.md, which rates a
# dead payload log-and-continue.
_BACKEND_ERRORS = (ImportError, OSError, RuntimeError, AttributeError,
                   ValueError, TypeError)


def select_backend(want: str = "auto",
                   importer: Callable = importlib.import_module,
                   led_spi_bus: int = 0, led_count: int = 8,
                   siren_chip: str = "auto", siren_line: int = 17):
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
            mod = importer("spidev")
            spi = mod.SpiDev()
            spi.open(led_spi_bus, 0)
            spi.max_speed_hz = WS2812_SPI_HZ
            spi.mode = 0
            backends.append(Ws2812Backend(spi, led_count))
        except _BACKEND_ERRORS as e:
            reasons.append("spidev unavailable (%s: %s)" % (type(e).__name__, e))

    if want in ("auto", "gpio"):
        try:
            mod = importer("gpiod")
            chip = find_gpiochip(mod) if siren_chip == "auto" else siren_chip
            line = request_output_line(mod, chip, siren_line, "huitzilin_siren")
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
