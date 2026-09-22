"""The Pi 5 payload drivers, against fake spidev and gpiod modules.

No Pi is needed: these pin the byte stream sent to the WS2812 strip and the
calls made to each libgpiod API. What they cannot show is that the strip lights
or the siren sounds -- that is bench stage 4 in docs/HARDWARE.md.
"""

import types

import pytest

from huitzilin_perception.payload import (
    CompositeBackend,
    GpioBuzzerBackend,
    NullBackend,
    Ws2812Backend,
    find_gpiochip,
    select_backend,
    ws2812_spi_frame,
)


# WS2812 over SPI

ONE, ZERO = 0b11111000, 0b11000000


def test_frame_is_grb_msb_first_one_byte_per_bit():
    frame = ws2812_spi_frame([(0xFF, 0x00, 0x80)])       # r, g, b
    g, r, b = frame[0:8], frame[8:16], frame[16:24]
    assert list(g) == [ZERO] * 8
    assert list(r) == [ONE] * 8
    assert list(b) == [ONE] + [ZERO] * 7


def test_frame_ends_with_a_latch_of_low_bytes():
    frame = ws2812_spi_frame([(1, 2, 3)] * 8)
    assert len(frame) == 8 * 24 + 240
    assert set(frame[8 * 24:]) == {0}


class _FakeSpi:
    def __init__(self):
        self.opened = None
        self.writes = []
        self.closed = False

    def open(self, bus, dev):
        self.opened = (bus, dev)

    def writebytes2(self, data):
        self.writes.append(bytes(data))

    def close(self):
        self.closed = True


def test_led_backend_writes_the_alarm_colour_then_black():
    spi = _FakeSpi()
    led = Ws2812Backend(spi, count=2)
    led.set(True)
    led.set(False)
    assert spi.writes[0] == ws2812_spi_frame([(255, 40, 0)] * 2)
    assert spi.writes[1] == ws2812_spi_frame([(0, 0, 0)] * 2)


def test_select_opens_spi_at_the_ws2812_clock():
    spi = _FakeSpi()
    fake = types.SimpleNamespace(SpiDev=lambda: spi)
    backend, reasons = select_backend(
        "ws2812", importer=lambda name: fake, led_spi_bus=0, led_count=8)
    assert isinstance(backend, CompositeBackend)
    assert reasons == []
    assert spi.opened == (0, 0)
    assert spi.max_speed_hz == 6_400_000


# libgpiod v2 (request_lines) and v1 (Chip.get_line)

class _V2Request:
    def __init__(self):
        self.values = []
        self.released = False

    def set_value(self, offset, value):
        self.values.append((offset, value))

    def release(self):
        self.released = True


def _gpiod_v2(labels):
    """A fake libgpiod v2 module. labels: path -> chip label."""
    req = _V2Request()
    Value = types.SimpleNamespace(ACTIVE="ACTIVE", INACTIVE="INACTIVE")

    class Chip:
        def __init__(self, path):
            if path not in labels:
                raise OSError("no such chip")
            self._label = labels[path]

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_info(self):
            return types.SimpleNamespace(label=self._label)

    mod = types.SimpleNamespace(
        Chip=Chip,
        LineSettings=lambda **kw: kw,
        line=types.SimpleNamespace(
            Value=Value, Direction=types.SimpleNamespace(OUTPUT="OUTPUT")),
        requests=[],
    )

    def request_lines(path, consumer, config):
        mod.requests.append((path, consumer, config))
        return req

    mod.request_lines = request_lines
    mod.req = req
    return mod


def _gpiod_v1():
    class Line:
        def __init__(self):
            self.values = []
            self.requested = None

        def request(self, consumer, type, default_vals):
            self.requested = (consumer, type, default_vals)

        def set_value(self, v):
            self.values.append(v)

        def release(self):
            pass

    line = Line()

    class Chip:
        opened = []

        def __init__(self, path):
            Chip.opened.append(path)

        def get_line(self, n):
            line.offset = n
            return line

    return types.SimpleNamespace(Chip=Chip, LINE_REQ_DIR_OUT="OUT", line_obj=line)


def test_v2_siren_requests_line_17_low_and_drives_it():
    mod = _gpiod_v2({"/dev/gpiochip0": "pinctrl-rp1"})
    backend, reasons = select_backend(
        "gpio", importer=lambda name: mod, siren_chip="gpiochip0")
    assert reasons == []
    (path, consumer, config), = mod.requests
    assert path == "/dev/gpiochip0"
    assert config[17]["output_value"] == "INACTIVE"
    backend.set(True)
    backend.set(False)
    assert mod.req.values == [(17, "ACTIVE"), (17, "INACTIVE")]


def test_v1_siren_still_works():
    mod = _gpiod_v1()
    backend, reasons = select_backend(
        "gpio", importer=lambda name: mod, siren_chip="gpiochip4")
    assert reasons == []
    assert mod.Chip.opened == ["/dev/gpiochip4"]
    assert mod.line_obj.offset == 17
    assert mod.line_obj.requested[2] == [0]
    backend.set(True)
    assert mod.line_obj.values == [1]


def test_auto_chip_finds_the_rp1_header_not_gpiochip0():
    """On older Pi 5 kernels the header is gpiochip4 and gpiochip0 is another
    controller. Asking for line 17 there succeeds on the wrong pin."""
    mod = _gpiod_v2({"/dev/gpiochip0": "gpio-brcmstb@107d508500",
                     "/dev/gpiochip4": "pinctrl-rp1"})
    listdir = lambda _d: ["gpiochip0", "gpiochip4", "tty0"]
    assert find_gpiochip(mod, listdir=listdir) == "/dev/gpiochip4"


def test_auto_chip_with_no_rp1_degrades_instead_of_guessing():
    mod = _gpiod_v2({"/dev/gpiochip0": "something-else"})
    backend, reasons = select_backend("gpio", importer=lambda name: mod)
    assert isinstance(backend, NullBackend)
    assert reasons


def test_buzzer_backend_is_version_agnostic():
    mod = _gpiod_v2({"/dev/gpiochip0": "pinctrl-rp1"})
    backend, _ = select_backend("gpio", importer=lambda name: mod,
                                siren_chip="/dev/gpiochip0")
    (siren,) = backend.backends
    assert isinstance(siren, GpioBuzzerBackend)
    backend.close()
    assert mod.req.released is True


@pytest.mark.parametrize("want", ["auto", "ws2812", "gpio"])
def test_a_library_of_the_wrong_shape_degrades(want):
    """An unexpected API version surfaces as AttributeError or TypeError."""
    backend, reasons = select_backend(want, importer=lambda name: object())
    assert isinstance(backend, NullBackend)
    assert reasons
