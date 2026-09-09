"""Turns a stream of per-frame scores into a conservative alert decision.

Structurally this is payload.py's AlarmLatch: on_scores and on_tick both
return True (turn the alert on), False (turn it off) or None (no change), so a
caller never re-drives an output already in the right state. The duplication
is deliberate rather than an import. Coupling this dead-man to the projectile
siren's file would mean a threshold changed for one silently changes the
other, and these two alarms answer to completely different evidence.

What makes it conservative, in the order the checks apply:

  Temporal confirmation -- a single high-scoring frame does nothing. It takes
  min_confirm_frames frames above enter_score, all inside confirm_window_s.
  A jitter spike cannot reach that; a sustained action can.

  Hysteresis -- confirmation needs enter_score, but staying confirmed only
  needs exit_score. Without the gap a score hovering at the threshold chatters
  the siren on and off, which is worse than either state.

  Minimum on time -- once on, the alert holds for min_alert_s so a human can
  actually see and hear it.

  Dead-man -- max_alert_s caps it regardless of input, so a stuck-high scorer
  cannot latch the siren on indefinitely.

  Cooldown -- after clearing, cooldown_s must pass before anything can
  confirm again. This is what stops one ambiguous episode producing a burst of
  repeated alerts.

  Staleness -- stale_input_s with no NEW frame forces DEGRADED and drops the
  alert. Silence is not consent to keep a siren running. A frame whose stamp
  has not advanced does not count as new: a producer republishing an identical
  frame is indistinguishable from a dead one as far as evidence goes, and
  treating it as live would hold the last high score forever.

This module decides whether to signal. It has no concept of flight, and
nothing it returns is a command to move.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple


class EscState(Enum):
    IDLE = "IDLE"
    CONFIRMING = "CONFIRMING"
    ALERTING = "ALERTING"
    COOLDOWN = "COOLDOWN"
    DEGRADED = "DEGRADED"


@dataclass(frozen=True)
class EscalationPolicyConfig:
    # Confirmation. enter_score must exceed exit_score or the hysteresis gap
    # inverts and the latch chatters; the params test asserts the ordering.
    enter_score: float = 0.70
    exit_score: float = 0.45
    min_confirm_frames: int = 4
    confirm_window_s: float = 0.60

    # A lone confirmation must still produce a signal a person can perceive.
    min_alert_s: float = 2.0
    # Dead-man. Nothing upstream is trusted to publish a clear.
    max_alert_s: float = 10.0
    # Silence between episodes, so one incident is one alert.
    cooldown_s: float = 15.0
    # No new frame for this long is a fault, not a quiet scene.
    stale_input_s: float = 2.0


class EscalationLatch:
    """Conservative alert decision over a stream of scores.

    All times are seconds on the caller's clock. The caller passes both the
    node clock (now_s) and the frame stamp (stamp_s) because they answer
    different questions: now_s is when the decision is being made, stamp_s is
    whether the evidence is new.
    """

    def __init__(self, config: Optional[EscalationPolicyConfig] = None):
        self.config = config or EscalationPolicyConfig()
        self._state = EscState.IDLE
        self._category: Optional[str] = None
        self._reason = "no input yet"
        self._alert = False
        self._alert_since = 0.0
        self._cleared_at: Optional[float] = None
        self._last_input_s: Optional[float] = None
        self._last_stamp_s: Optional[float] = None
        # (time, category) for frames that cleared enter_score.
        self._hits: List[Tuple[float, str]] = []

    @property
    def state(self) -> EscState:
        return self._state

    @property
    def category(self) -> Optional[str]:
        return self._category

    @property
    def last_reason(self) -> str:
        return self._reason

    @property
    def is_alerting(self) -> bool:
        return self._alert

    @property
    def alert_on_s(self) -> float:
        return 0.0 if not self._alert else self._alert_since

    def cooldown_remaining_s(self, now_s: float) -> float:
        if self._cleared_at is None:
            return 0.0
        return max(0.0, self.config.cooldown_s - (now_s - self._cleared_at))

    def on_scores(self, scores: Dict[str, float], now_s: float,
                  stamp_s: Optional[float] = None):
        """Fold one frame of scores in. Returns True, False or None.

        `scores` is keyed by category name so this module never imports the
        feature module; the two are independent by construction.
        """
        cfg = self.config

        # A stamp that has not advanced carries no new evidence. Refusing to
        # refresh liveness here is what makes a frozen producer look stale
        # rather than permanently alarming.
        if stamp_s is not None and self._last_stamp_s is not None:
            if stamp_s <= self._last_stamp_s:
                self._reason = "frame stamp did not advance"
                return None
        if stamp_s is not None:
            self._last_stamp_s = stamp_s
        self._last_input_s = now_s

        if self._state is EscState.DEGRADED:
            # Live input again; fall back to IDLE and re-earn any alert.
            self._state = EscState.IDLE
            self._reason = "input resumed"

        top_cat, top_score = None, 0.0
        for name, value in sorted(scores.items()):
            if value > top_score:
                top_cat, top_score = name, value

        if self._alert:
            held_s = now_s - self._alert_since
            if top_score >= cfg.exit_score:
                self._reason = "sustained above exit_score"
                return None
            if held_s < cfg.min_alert_s:
                self._reason = "holding minimum visible alert"
                return None
            return self._clear(now_s, "score fell below exit_score")

        if self.cooldown_remaining_s(now_s) > 0.0:
            self._state = EscState.COOLDOWN
            self._reason = "in cooldown"
            return None

        if top_score >= cfg.enter_score and top_cat is not None:
            self._hits.append((now_s, top_cat))
        self._hits = [(t, c) for (t, c) in self._hits
                      if now_s - t <= cfg.confirm_window_s]

        if len(self._hits) >= cfg.min_confirm_frames:
            # The category reported is the one that dominated the confirming
            # window, not merely the last frame.
            counts: Dict[str, int] = {}
            for _, c in self._hits:
                counts[c] = counts.get(c, 0) + 1
            self._category = max(sorted(counts), key=lambda c: counts[c])
            self._state = EscState.ALERTING
            self._alert = True
            self._alert_since = now_s
            self._hits = []
            self._reason = ("confirmed on %d frames within %.2f s"
                            % (cfg.min_confirm_frames, cfg.confirm_window_s))
            return True

        if self._hits:
            self._state = EscState.CONFIRMING
            self._reason = ("%d/%d confirming frames"
                            % (len(self._hits), cfg.min_confirm_frames))
        else:
            self._state = EscState.IDLE
            self._reason = "below enter_score"
        return None

    def on_tick(self, now_s: float):
        """Time-driven transitions the score stream cannot supply."""
        cfg = self.config

        if self._alert and now_s - self._alert_since >= cfg.max_alert_s:
            return self._clear(now_s, "max_alert_s dead-man expired")

        if self._last_input_s is None:
            return None

        if now_s - self._last_input_s >= cfg.stale_input_s:
            was_alerting = self._alert
            self._state = EscState.DEGRADED
            self._reason = ("no new frame for %.1f s"
                            % (now_s - self._last_input_s))
            self._hits = []
            if was_alerting:
                self._alert = False
                self._cleared_at = now_s
                return False
            return None

        if (not self._alert and self._state is EscState.COOLDOWN
                and self.cooldown_remaining_s(now_s) <= 0.0):
            self._state = EscState.IDLE
            self._reason = "cooldown elapsed"
        return None

    def _clear(self, now_s: float, why: str):
        self._alert = False
        self._cleared_at = now_s
        self._state = EscState.COOLDOWN
        self._hits = []
        self._reason = why
        return False
