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


def straight_leg_time_s(dist_to_wp_m: float,
                        accept_radius_m: float,
                        speed_mps: float) -> float:
    """Seconds of straight flight left before patrol snaps to the next waypoint.

    Returns ``inf`` when the drone is hovering (see HOVER_SPEED_MPS): there is
    no corner ahead, so the window is not a constraint.
    """
    if speed_mps < HOVER_SPEED_MPS:
        return math.inf
    remaining_m = max(0.0, float(dist_to_wp_m) - float(accept_radius_m))
    return remaining_m / float(speed_mps)


def throw_window_ok(*,
                    dist_to_wp_m: float | None,
                    accept_radius_m: float,
                    speed_mps: float,
                    t_flight_s: float,
                    margin_s: float = DEFAULT_MARGIN_S,
                    patrol_running: bool = True) -> tuple[bool, str]:
    """Decide whether to spend a throw now, with a reason for the report.

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
    leg_s = straight_leg_time_s(dist_to_wp_m, accept_radius_m, speed_mps)
    if leg_s == math.inf:
        return True, "drone hovering — lead is exact"
    if leg_s >= needed_s:
        return True, f"leg {leg_s:.2f} s >= needed {needed_s:.2f} s"
    return False, (f"only {leg_s:.2f} s of straight leg left, need "
                   f"{needed_s:.2f} s ({t_flight_s:.2f} s flight + "
                   f"{margin_s:.2f} s margin) — throw would span a corner")
