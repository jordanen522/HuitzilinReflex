"""Throw-window gate: is the drone on enough straight leg to aim a throw at it?

`compute_spawn` leads the target by extrapolating the drone at CONSTANT
velocity across the ball's flight. `patrol_node` is a corner follower flown in
position mode, so ArduPilot decelerates into each waypoint and accelerates out
along a new heading — a discontinuity no motion model can predict across.
Fitting a better model is the wrong fix; spending the throw only when the
remaining straight leg covers the whole flight is the right one:

    leg_time = (dist_to_waypoint - accept_radius) / cruise_speed

`accept_radius` is subtracted because patrol retargets the moment it lands
inside it (`patrol_node.py`: `if dist < self.accept`), so those last metres are
not straight flight.

Both conditions in `throw_window_ok` are necessary. Gating on leg time alone,
at *instantaneous* speed, measured worse than no gate at all: near a corner the
speed collapses and the quotient inflates into a huge "safe" window at exactly
the least predictable moment.

This never adjusts aim. An unaimable moment is skipped, not compensated, so a
scenario whose flight time exceeds the longest available leg is structurally
unmeasurable on that patrol loop and the battery reports it as such.
"""

from __future__ import annotations

import math

# Below this speed the drone is effectively stationary: a constant-velocity
# lead is then exact, there is no corner to cross, and dividing by the speed
# would manufacture a huge window out of sensor noise.
HOVER_SPEED_MPS = 0.05

# Slack required on top of the bare flight time. The detector needs the
# approach *observed* over several frames before impact (min_track_updates=3
# at 15 Hz = 0.2 s), so a leg that ends exactly at impact is still too short.
DEFAULT_MARGIN_S = 0.30

# Fraction of cruise the drone must be doing before a throw is worth spending.
# DERIVED, not guessed: any shortfall becomes aim error, because the lead
# extrapolates the low current velocity while the drone accelerates away.
# Bounding that under the 0.5 m off-target tolerance at the worst flight time
# (1.5 s) against the 12 m loop's 5.26 m/s cruise needs
# (1 - frac) * 5.26 * 1.5 < 0.5, i.e. frac > 0.94. Re-derive if the loop
# geometry or cruise speed changes; do not round it down for throughput.
DEFAULT_MIN_CRUISE_FRAC = 0.95


def straight_leg_time_s(dist_to_wp_m: float,
                        accept_radius_m: float,
                        cruise_mps: float) -> float:
    """Seconds of straight flight left before patrol snaps to the next waypoint.

    cruise_mps must be the drone's CRUISE speed, not its instantaneous speed
    (see the module docstring). Cruise is the conservative choice — the drone
    accelerates back up to it during the ball's flight, so it reaches the
    corner at least this soon.

    Returns ``inf`` when cruise is below HOVER_SPEED_MPS: the drone is parked,
    there is no corner ahead, and the lead is trivially exact.
    """
    if cruise_mps < HOVER_SPEED_MPS:
        return math.inf
    remaining_m = max(0.0, float(dist_to_wp_m) - float(accept_radius_m))
    return remaining_m / float(cruise_mps)


def throw_window_ok(*,
                    dist_to_wp_m: float | None,
                    accept_radius_m: float,
                    speed_mps: float,
                    cruise_mps: float,
                    t_flight_s: float,
                    margin_s: float = DEFAULT_MARGIN_S,
                    min_cruise_frac: float = DEFAULT_MIN_CRUISE_FRAC,
                    patrol_running: bool = True) -> tuple[bool, str]:
    """Decide whether to spend a throw now, with a reason for the report.

    Two independent conditions, both measured as necessary:

    1. **At cruise** — ``speed_mps >= min_cruise_frac * cruise_mps``. A drone
       accelerating out of a corner breaks the constant-velocity lead even on
       a long leg, because the lead extrapolates from the *current* (low)
       velocity and the drone then speeds up and walks away from the aim point.
    2. **Enough leg** — the remaining straight run, evaluated at cruise,
       covers the flight plus margin.

    cruise_mps is a required keyword rather than defaulting to speed_mps, so
    passing instantaneous speed for both roles is not expressible.

    The reason string is carried into the battery row so a refusal is never
    scored as a trigger failure.
    """
    if not patrol_running:
        return True, "patrol idle — no corner to cross, lead is exact"

    if dist_to_wp_m is None:
        # Refuse rather than assume. Throwing blind here is precisely the v9
        # behaviour this gate replaces.
        return False, ("no patrol state on /huitzilin/patrol_state — cannot "
                       "tell how much straight leg remains")

    needed_s = float(t_flight_s) + float(margin_s)
    leg_s = straight_leg_time_s(dist_to_wp_m, accept_radius_m, cruise_mps)
    if leg_s == math.inf:
        return True, "drone hovering — lead is exact"

    floor_mps = float(min_cruise_frac) * float(cruise_mps)
    if speed_mps < floor_mps:
        return False, (f"speed {speed_mps:.2f} m/s is below "
                       f"{floor_mps:.2f} m/s ({min_cruise_frac:.0%} of cruise "
                       f"{cruise_mps:.2f}) — still accelerating out of a "
                       f"corner, the lead would aim short")
    if leg_s >= needed_s:
        return True, f"leg {leg_s:.2f} s >= needed {needed_s:.2f} s"
    return False, (f"only {leg_s:.2f} s of straight leg left at cruise "
                   f"{cruise_mps:.2f} m/s, need {needed_s:.2f} s "
                   f"({t_flight_s:.2f} s flight + {margin_s:.2f} s margin) "
                   f"— throw would span a corner")
