"""The warning-light and siren interface for the escalation alert.

There is no GPIO backend here, and its absence is a safety decision rather
than unfinished work.

huitzilin_perception's payload_node already owns the physical annunciator:
WS2812B data on GPIO 18, siren on gpiochip0 line 17. Linux GPIO line requests
are EXCLUSIVE. If this node started first and took line 17, payload_node would
fail to acquire it -- so a discretionary body-motion warning would have
silently disabled the projectile threat annunciator, which is the
safety-critical one. That is a priority inversion, and it is not acceptable in
either direction of timing luck.

So `select_alert_sink("hardware")` refuses, returning a null sink and a reason
naming the conflict. It does NOT raise: payload.select_backend documents a
never-raises contract and SAFETY_CASE.md section 1 rates a payload/GPIO fault
Low -- log and continue. A node that dies on a config value would invert that
posture, and a node killed by a params typo is worse than one running inert
with a loud warning.

The correct hardware design, deferred and deliberately not built here:
payload_node grows a SECOND, lower-priority input, so one process continues to
own the line and the projectile alarm always wins arbitration. Implementing
that from this package would put escalation logic inside the projectile
package and destroy the isolation this subsystem exists to keep.

Until then the alert is observable as a ROS topic and in the log, which is
enough to exercise and review the policy without any hardware at all.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

HARDWARE_REFUSAL = (
    "backend 'hardware' refused: payload_node already owns the WS2812B data "
    "line (GPIO 18) and the siren line (gpiochip0 line 17), and Linux GPIO "
    "line requests are exclusive. A second process taking those lines would "
    "silently prevent the projectile alarm from firing. Hardware alerting "
    "needs payload_node to gain a second, lower-priority input; see "
    "docs/action_escalation.md."
)


class AlertSink:
    """Anything the escalation alert can be driven into."""

    def set(self, on: bool) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class NullAlertSink(AlertSink):
    """Records calls and does nothing else. Never raises."""

    def __init__(self, why: str = ""):
        self.why = why
        self.calls: List[bool] = []

    def set(self, on: bool) -> None:
        self.calls.append(bool(on))


class SimAlertSink(AlertSink):
    """Publishes the alert state instead of energising anything.

    This is the default, and on any machine without the deferred hardware
    arbitration it is the only correct backend.
    """

    def __init__(self, publish: Callable[[bool], None]):
        self._publish = publish
        self.calls: List[bool] = []

    def set(self, on: bool) -> None:
        self.calls.append(bool(on))
        self._publish(bool(on))


def select_alert_sink(want: str = "sim",
                      publish: Optional[Callable[[bool], None]] = None
                      ) -> Tuple[AlertSink, List[str]]:
    """Build the alert sink. Returns (sink, reasons). Never raises.

    `reasons` lists every degradation so the node can log it once at startup,
    matching payload.select_backend's contract.
    """
    reasons: List[str] = []

    if want == "none":
        return NullAlertSink("disabled by configuration"), reasons

    if want == "hardware":
        reasons.append(HARDWARE_REFUSAL)
        return NullAlertSink(HARDWARE_REFUSAL), reasons

    if want != "sim":
        reasons.append("unknown backend %r; falling back to sim" % (want,))

    if publish is None:
        reasons.append("no publish callback supplied; alert is inert")
        return NullAlertSink("no publish callback"), reasons

    return SimAlertSink(publish), reasons


@dataclass(frozen=True)
class AlertLatchPolicy:
    # A lone True must still produce a perceptible signal.
    min_on_s: float = 2.0
    # Dead-man. The recogniser publishes a clear, but it may die first.
    max_on_s: float = 12.0
    # Silence means the publisher is gone, not that the alert should persist.
    stale_off_s: float = 3.0


class AlertLatch:
    """Turns a stream of alert booleans into on/off actions.

    Returns True (turn on), False (turn off) or None (no change), so the node
    never re-drives a sink already in the right state.

    Duplicates payload.AlarmLatch on purpose -- see the module docstring. The
    dead-man here is longer than the projectile one because an escalation
    alert is not bounded by a dodge duration.
    """

    def __init__(self, policy: Optional[AlertLatchPolicy] = None):
        self.policy = policy or AlertLatchPolicy()
        self._on = False
        self._on_since = 0.0
        self._last_msg_s: Optional[float] = None

    @property
    def is_on(self) -> bool:
        return self._on

    def on_message(self, value: bool, now_s: float):
        self._last_msg_s = now_s
        if value:
            if self._on:
                return None
            self._on = True
            self._on_since = now_s
            return True
        if not self._on:
            return None
        if now_s - self._on_since < self.policy.min_on_s:
            return None
        self._on = False
        return False

    def on_tick(self, now_s: float):
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
