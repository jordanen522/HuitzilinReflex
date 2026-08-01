"""Reading and comparing ArduPilot .parm files.

Exists so the sharp edges around these files are enforced by a test instead of
remembered. CLAUDE.md records that an inline comment inside a .parm breaks
MAVProxy; that is a parse error here. A duplicated key is also an error,
because a duplicate is exactly how a safety parameter gets silently overridden
by a later line nobody scrolled down to.

The comparison half is shared with scripts/hw_param_readback.py so that
readback-vs-expected logic lives in tested Python rather than in bash.
"""

from __future__ import annotations

from dataclasses import dataclass


class ParmSyntaxError(ValueError):
    """A .parm file MAVProxy would mis-read or reject."""


@dataclass(frozen=True)
class Mismatch:
    name: str
    expected: float | None
    actual: float | None

    @property
    def kind(self) -> str:
        if self.actual is None:
            return "missing"
        if self.expected is None:
            return "unexpected"
        return "differs"

    def __str__(self) -> str:
        return "%s: %s (expected %r, got %r)" % (
            self.name, self.kind, self.expected, self.actual)


def parse_parm(text: str) -> dict:
    """Parse `NAME VALUE` or `NAME,VALUE` lines into {name: float}.

    Accepts blank lines and whole-line comments only. Anything else raises,
    including the inline comment that silently breaks MAVProxy.
    """
    out: dict = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "#" in line:
            raise ParmSyntaxError(
                "line %d: inline comment breaks MAVProxy, use a comment-only "
                "line: %r" % (lineno, raw))
        parts = line.replace(",", " ").split()
        if len(parts) != 2:
            raise ParmSyntaxError(
                "line %d: expected 'NAME VALUE' or 'NAME,VALUE', got %r"
                % (lineno, raw))
        name, value = parts[0].upper(), parts[1]
        try:
            number = float(value)
        except ValueError:
            raise ParmSyntaxError(
                "line %d: %s value %r is not a number" % (lineno, name, value))
        if name in out:
            raise ParmSyntaxError(
                "line %d: %s set twice; the later value silently wins, which is "
                "how a safety parameter goes missing" % (lineno, name))
        out[name] = number
    return out


def diff_params(expected: dict, actual: dict, tol: float = 1e-6) -> list:
    """Every way `actual` fails to match `expected`, missing keys included.

    Extra keys in `actual` are not reported: a live flight controller has
    hundreds of parameters this file does not speak for.
    """
    out = []
    for name, want in sorted(expected.items()):
        if name not in actual:
            out.append(Mismatch(name, want, None))
        elif abs(float(actual[name]) - float(want)) > tol:
            out.append(Mismatch(name, want, float(actual[name])))
    return out
