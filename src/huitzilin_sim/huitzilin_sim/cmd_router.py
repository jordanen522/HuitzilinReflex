"""Which setpoint the bridge should send right now. Pure logic; no rclpy.

This was inline in mav_bridge_node._tick_setpoint, which meant the priority
rules -- fresh evade beats cmd_vel, a finished dodge emits exactly one zero,
silence is different from a zero-hold -- could only be exercised by flying.
They are the rules a late or duplicated dodge command breaks, so they are
worth pinning as data.

The reason this module exists at all is timing. The bridge used to send only
on its 10 Hz watchdog tick, so a dodge command published the instant the
trigger fired still waited up to 100 ms for the next tick before reaching
ArduPilot. That is a tenth of the whole 150 ms budget spent on nothing, and it
matters most at the speeds where the envelope is already tight. The node now
routes on three occasions -- on receipt, on a fast evade timer, and on the
slow watchdog tick -- and all three go through route(), so the "exactly one
handback zero" property survives having three callers.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

# Body-NED velocity plus yaw rate: (vx, vy, vz, yaw_rate).
Velocity = Tuple[float, float, float, float]
# Body-NED acceleration feedforward: (ax, ay, az).
Accel = Tuple[float, float, float]

ZERO: Velocity = (0.0, 0.0, 0.0, 0.0)
ZERO_ACCEL: Accel = (0.0, 0.0, 0.0)


class Action(Enum):
    EVADE = "evade"           # a dodge is live and owns the vehicle
    HANDBACK = "handback"     # the dodge just expired: one zero, then let go
    CMD_VEL = "cmd_vel"       # patrol's velocity stream
    ZERO_HOLD = "zero_hold"   # patrol went quiet: calm hold, never a coast
    SILENCE = "silence"       # nothing to say; position-mode patrol owns it


@dataclass(frozen=True)
class RouterState:
    """Latched commands and their arrival times. A time of None means never.

    Never-received and long-ago are deliberately different: before patrol has
    ever commanded anything the bridge must stay silent, because a zero-hold
    would fight the position setpoints patrol sends over its own MAVLink link.
    """

    last_evade: Velocity = ZERO
    last_evade_t: Optional[float] = None
    last_cmd: Velocity = ZERO
    last_cmd_t: Optional[float] = None
    evade_active: bool = False
    # Acceleration feedforward arrives on its own topic, so it has its own
    # freshness: a dropped accel message must degrade to a plain velocity
    # setpoint, never to the previous dodge's acceleration.
    last_accel: Accel = ZERO_ACCEL
    last_accel_t: Optional[float] = None


@dataclass(frozen=True)
class Route:
    action: Action
    velocity: Velocity
    # What evade_active becomes IF the caller acts on this route. The fast
    # evade timer deliberately acts on nothing but EVADE, so it must not be
    # able to consume the handback edge -- that belongs to the watchdog tick,
    # which is guaranteed to run within one of its own periods.
    evade_active: bool
    # None means "send a velocity-only setpoint". Never carried on anything
    # but EVADE: an acceleration attached to a handback zero would command a
    # shove at precisely the moment the aircraft is being handed back to
    # patrol, which is the opposite of what that zero is for.
    #
    # An all-zero acceleration is also None. The reason is SEMANTIC, not
    # dynamic: it picks the MAVLink mask, and "the feedforward is off" must
    # send the same setpoint as "nothing publishes a feedforward at all".
    # Without this branch the shipped default evade_accel_ff_mps2: 0.0 sends
    # MASK_VEL_ACCEL while an absent publisher sends MASK_VEL_ONLY -- two
    # different commands for what is documented as one configuration.
    #
    # What this branch is NOT: a recovery for lost dodges. At settled hover a
    # zero-accel setpoint measured 28/28 trials at 0.00 m against ~1.5 m
    # velocity-only, which looked like ArduPilot discarding the whole
    # setpoint. That does NOT reproduce in flight. Controlled 12-trial A/B on
    # one battery cell (8 m/s, detection_range_m 12.0 -- an INPUT), pre-fix
    # vs post-fix, differing only in this branch: escape contribution 0.107 m
    # pre-fix vs 0.091 m post-fix (pre-fix marginally LARGER, i.e. no gain),
    # and both arms commanded real motion (1.89 m and 1.53 m of displacement
    # against the extrapolated track). Whatever suppressed motion at hover,
    # the mask does not cost the dodge while the vehicle is cruising. Keep
    # this for the semantics above; never credit it with saving a dodge.
    accel: Optional[Accel] = None

    @property
    def sends(self) -> bool:
        return self.action is not Action.SILENCE


def _fresh(stamp: Optional[float], now_s: float, timeout_s: float) -> bool:
    return stamp is not None and (now_s - stamp) <= timeout_s


def route(state: RouterState, now_s: float, timeout_s: float) -> Route:
    """Priority: fresh evade > handback zero > cmd_vel > zero-hold > silence."""
    if _fresh(state.last_evade_t, now_s, timeout_s):
        accel = state.last_accel
        if (not _fresh(state.last_accel_t, now_s, timeout_s)
                or accel == ZERO_ACCEL):
            accel = None
        return Route(Action.EVADE, state.last_evade, evade_active=True,
                     accel=accel)

    # The dodge has expired. Send one zero so ArduPilot does not coast on the
    # last dodge velocity, then fall through on every subsequent call.
    if state.evade_active:
        return Route(Action.HANDBACK, ZERO, evade_active=False)

    if state.last_cmd_t is None:
        return Route(Action.SILENCE, ZERO, evade_active=False)

    if _fresh(state.last_cmd_t, now_s, timeout_s):
        return Route(Action.CMD_VEL, state.last_cmd, evade_active=False)

    return Route(Action.ZERO_HOLD, ZERO, evade_active=False)
