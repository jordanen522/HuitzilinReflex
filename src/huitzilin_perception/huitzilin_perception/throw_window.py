"""Throw-window gate: is the drone on enough straight leg to aim a throw at it?

Why this exists (battery v9, 2026-07-26): 11 of 17 throws landed off-target,
aim error mean 1.50 m / max 3.79 m, so the battery could not measure dodge
performance — two thirds of its throws missed by design.

Root cause: compute_spawn leads the target by extrapolating the drone at
CONSTANT velocity across the ball's flight (t = offset_forward_m / speed_mps).
patrol_node is a corner follower — a closed loop of waypoints flown in
position mode — so ArduPilot decelerates into each waypoint and accelerates
out along a new heading. That is a discontinuity in the commanded path, not a
smooth curve: no constant-velocity *or* constant-acceleration lead can predict
across it. Fitting a better motion model is the wrong fix.

The right fix is to spend the throw only when the remaining straight leg
covers the whole flight, which is pure geometry:

    leg_time = (dist_to_waypoint - accept_radius) / speed

The accept radius is subtracted because patrol reassigns its target the moment
it lands inside it (patrol_node.py `if dist < self.accept`), so those last
metres are not straight flight.

Two conditions, not one — learned the hard way. Battery v10 gated on leg time
alone, computed as (dist - accept)/*instantaneous* speed, and came out WORSE
than v9: 16/20 off-target, aim error mean 1.60 m / max 4.96 m, with the gate
never once refusing a throw. The quotient is inverted near a corner, where the
speed collapses toward zero and inflates the window:

    window_s  aim_err_m
      0.76      0.18      <- fast, mid-leg: best throws of the run
      2.48      0.21
      5.86      3.22      <- slow, just out of a corner: worst
      8.20      0.94

So the gate also requires the drone to be AT CRUISE, and computes leg time at
the cruise estimate rather than the current speed. A drone accelerating out of
a corner breaks the lead even on a long leg, because compute_spawn extrapolates
from the velocity it is handed and the drone then speeds up and leaves.

Note what this does NOT do: it never adjusts the aim. An unaimable moment is
skipped, not compensated. A scenario whose flight time exceeds the longest
available leg is therefore unmeasurable on that patrol loop, and the battery
reports it as such rather than throwing anyway — see
test_b01_on_a_five_metre_leg_is_structurally_unmeasurable.
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

# Fraction of cruise the drone must actually be doing before a throw is worth
# spending. Measured on the Dell over 45 s of patrol (1350 odom samples):
# median 2.09 m/s, p90 3.35, max 3.49 — and only 30% of the time above 80% of
# max. The other 70% is corner transitions, which is why v10 threw 16/20
# off-target while believing every window was safe.
#
# DERIVED, not guessed. Battery v12 (12 m loop, this gate at 0.80) collapsed
# the per-scenario spread to +-0.04 m and exposed a systematic bias
# proportional to flight time — ~0.9 m/s * t_flight:
#     B03 t=0.43 s -> 0.39, 0.36, 0.44 m
#     B06 t=0.75 s -> 0.66, 0.66, 0.63 m
# That residual IS the 20% of cruise this floor was letting through: the drone
# is still accelerating, so the lead extrapolates a velocity that low and the
# ball arrives behind. Bounding the error under the 0.5 m off-target tolerance
# at the worst flight time (1.5 s) against the 12 m loop's 5.26 m/s cruise
# needs (1 - frac) * 5.26 * 1.5 < 0.5, i.e. frac > 0.94.
DEFAULT_MIN_CRUISE_FRAC = 0.95


def straight_leg_time_s(dist_to_wp_m: float,
                        accept_radius_m: float,
                        cruise_mps: float) -> float:
    """Seconds of straight flight left before patrol snaps to the next waypoint.

    cruise_mps must be the drone's CRUISE speed, not its instantaneous speed.
    Dividing by the instantaneous speed is what made battery v10 worse than v9
    (see the module docstring): near a corner the speed collapses, so the
    quotient inflates into a huge "safe" window at exactly the moment the aim
    is least predictable. Cruise is the conservative choice — the drone will
    accelerate back up to it during the ball's flight, so it reaches the corner
    at least this soon.

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

    cruise_mps is passed in rather than defaulted so the broken v10 usage
    (instantaneous speed for both roles) is not expressible.

    The reason string is carried into the battery row so a refusal is never
    read as a trigger failure — the v9 report conflated the two.
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
