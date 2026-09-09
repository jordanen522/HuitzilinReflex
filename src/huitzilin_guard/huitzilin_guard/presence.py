"""Turns a noisy stream of in-box detections into a stable alarm decision.

Pure logic, no rclpy, so the whole decision can be tested without a ROS graph
or a camera.

The asymmetry between sounding and stopping is the entire point of this
module, and it is deliberate in both directions:

  * Sounding needs confirm_frames CONSECUTIVE in-box detections. One frame is
    a detector artefact often enough that a single-frame trigger would make
    the siren untrustworthy, and an alarm nobody believes is worse than none.

  * Stopping needs clear_after_s of an EMPTY box -- not one empty frame. A
    person standing still is exactly the case the detector is worst at, and
    one missed frame must never silence an alarm that is correctly sounding.

Returns True (sound it), False (stop it) or None (no change), so the node
never re-drives an output already in the right state.

There is NO COOLDOWN. Someone still inside the box is still an intruder, and
a post-alarm quiet period is a window in which the system deliberately does
not do its job.

A NOTE ON STALENESS, which is subtler than it looks. pose_detector publishes
only when it SEES somebody, so an empty box and a dead detector both look
like silence. Silence therefore cannot be treated as a fault on its own:
doing so would leave the system permanently faulted on any quiet night, which
is exactly how a real fault gets trained out of an operator's attention -- the
same trap CLAUDE.md records for supervisor watches on topics nothing
publishes. So stale_input_s is a DEAD-MAN, not a health check: it clears an
alarm that is currently sounding when the frames feeding it stop arriving,
because a siren meaning "I have no idea" is worse than silence. While the
alarm is off, silence is reported in status as a plain observation and is
never escalated.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class PresencePolicy:
    """Timings for the alarm decision. Seconds, except confirm_frames."""

    # Consecutive in-box detections before the siren sounds. At the
    # detector's 10 Hz ceiling this is well under a second of real time.
    confirm_frames: int = 3
    # Empty box before the siren stops. Covers a run of missed detections on
    # a person who has stopped moving.
    clear_after_s: float = 3.0
    # A brief intrusion must still produce a signal somebody can perceive.
    min_alert_s: float = 3.0
    # Stuck-detector guard. Deliberately long: a real intruder who stands
    # still must not be able to outwait the alarm.
    max_alert_s: float = 300.0
    # Dead-man on the frames feeding a SOUNDING alarm. See the module
    # docstring for why this is not a health check.
    stale_input_s: float = 5.0

    def __post_init__(self) -> None:
        if self.confirm_frames < 1:
            raise ValueError(
                "confirm_frames is %r; it must be at least 1. Zero would arm "
                "the siren before any detection had been made."
                % (self.confirm_frames,))
        if self.max_alert_s <= self.min_alert_s:
            raise ValueError(
                "max_alert_s (%g) must exceed min_alert_s (%g), or the "
                "stuck-detector guard fires before the alarm has been "
                "audible for its minimum duration."
                % (self.max_alert_s, self.min_alert_s))


class PresenceLatch:
    """The alarm decision for one box.

    Fed a stream of "is a person in the box right now" observations, one per
    detection frame, plus a periodic tick so the time-based transitions
    happen even when no frames are arriving at all.
    """

    def __init__(self, policy: Optional[PresencePolicy] = None):
        self.policy = policy or PresencePolicy()
        self._on = False
        self._on_since_s = 0.0
        self._consecutive_in_box = 0
        self._last_in_box_s: Optional[float] = None
        self._last_input_s: Optional[float] = None
        # Why the alarm last stopped on its own, for the status heartbeat.
        # Empty string means it has not stopped on its own yet.
        self.last_stop_reason = ""

    @property
    def is_on(self) -> bool:
        return self._on

    @property
    def consecutive_in_box(self) -> int:
        """How far through the confirm window we are. Reported in status."""
        return self._consecutive_in_box

    def seconds_since_input(self, now_s: float) -> Optional[float]:
        """Age of the last detection frame, or None if none has arrived.

        None is not a fault: on an empty box the detector publishes nothing,
        so this is the normal resting state.
        """
        if self._last_input_s is None:
            return None
        return now_s - self._last_input_s

    def reset(self) -> None:
        """Forget everything, without driving an output.

        Used on the disarmed-to-armed edge so the confirm window always
        applies fresh after arming. Without it, arming while somebody was
        already standing in the box would sound the siren on the first tick
        with no confirmation at all.
        """
        self._on = False
        self._on_since_s = 0.0
        self._consecutive_in_box = 0
        self._last_in_box_s = None
        self._last_input_s = None

    def on_frame(self, person_in_box: bool, now_s: float) -> Optional[bool]:
        """One detection frame. Returns True, False or None."""
        self._last_input_s = now_s

        if not person_in_box:
            # Reset the confirm run, but do NOT clear here. Clearing is
            # time-based, in on_tick, so a single missed frame on a
            # motionless person cannot silence a correct alarm.
            self._consecutive_in_box = 0
            return None

        self._last_in_box_s = now_s
        self._consecutive_in_box += 1
        if self._on:
            return None
        if self._consecutive_in_box < self.policy.confirm_frames:
            return None

        self._on = True
        self._on_since_s = now_s
        self.last_stop_reason = ""
        return True

    def on_tick(self, now_s: float) -> Optional[bool]:
        """Time-based transitions. Called even when no frames are arriving."""
        if not self._on:
            return None

        on_for = now_s - self._on_since_s

        # Checked before min_alert_s: this is the stuck-detector guard, and a
        # minimum duration must not be able to extend a runaway alarm.
        if on_for >= self.policy.max_alert_s:
            return self._stop(
                "max_alert_s (%g s) reached; the detector has reported a "
                "person in the box continuously for that long, which is more "
                "likely stuck than true" % self.policy.max_alert_s)

        # Also checked before min_alert_s. stale_input_s is longer than
        # min_alert_s in the shipped config, so this does not cut a real
        # alarm short; it means the frames justifying the siren have stopped.
        age = self.seconds_since_input(now_s)
        if age is not None and age >= self.policy.stale_input_s:
            return self._stop(
                "no detection frame for %.1f s while sounding; the alarm is "
                "no longer supported by anything being measured" % age)

        if on_for < self.policy.min_alert_s:
            return None

        if self._last_in_box_s is None:
            return None
        if now_s - self._last_in_box_s < self.policy.clear_after_s:
            return None

        # The clear window has elapsed, but elapsed time alone is not
        # evidence that the box is empty. At least one frame must have
        # ARRIVED since the last in-box detection, showing nobody there.
        #
        # Without this condition an alarm whose detector had simply died
        # would stop with the reason "box clear", which is a measurement
        # nothing made: no frames were arriving to observe an empty box.
        # In that case the dead-man above is the correct exit, two seconds
        # later and honest about why. Stopping a sounding alarm needs
        # justification; running out of data is not the same as an all-clear.
        if (self._last_input_s is not None
                and self._last_input_s > self._last_in_box_s):
            return self._stop("box clear for %g s" % self.policy.clear_after_s)

        return None

    def _stop(self, reason: str) -> bool:
        self._on = False
        self._consecutive_in_box = 0
        self.last_stop_reason = reason
        return False
