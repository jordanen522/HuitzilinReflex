"""Unit tests for the W4 throw-window gate (aiming under patrol).

Runs without ROS (pure arithmetic):
    python -m pytest src/huitzilin_perception/test/test_throw_window.py

Failure being encoded (docs/JOURNAL.md, battery v9, 2026-07-26): 11 of 17
throws landed off-target, aim error mean 1.50 m / max 3.79 m. The lead in
compute_spawn extrapolates the drone at CONSTANT velocity across the ball's
flight, but patrol_node is a *corner* follower — a 5 m square in position
mode, so ArduPilot decelerates into each waypoint and accelerates out. No
constant-velocity (or constant-acceleration) lead can predict across that
discontinuity. The only sound fix is to spend the throw when enough straight
leg remains to cover the whole flight.

The old gate (_wait_steady_velocity, |dv| <= 0.35 m/s over 0.15 s) cannot
catch this: see test_sustained_turn_passes_a_dv_gate_but_fails_the_window.
"""

import math

from huitzilin_perception.throw_window import (
    straight_leg_time_s,
    throw_window_ok,
)


# ── straight_leg_time_s ───────────────────────────────────────────────────────

def test_leg_time_is_remaining_distance_over_speed():
    # 4.4 m of straight run left at 2.2 m/s = 2.0 s.
    assert straight_leg_time_s(5.0, 0.6, 2.2) == 2.0


def test_accept_radius_shortens_the_usable_leg():
    # patrol snaps to the next WP at accept_radius, so the last 0.6 m is not
    # straight flight — counting it would over-promise the window.
    full = straight_leg_time_s(5.0, 0.0, 2.0)
    gated = straight_leg_time_s(5.0, 0.6, 2.0)
    assert full > gated
    assert gated == (5.0 - 0.6) / 2.0


def test_inside_accept_radius_is_zero_window():
    # Already at the corner: the next control tick reassigns the target.
    assert straight_leg_time_s(0.4, 0.6, 2.0) == 0.0


def test_hovering_drone_has_an_unbounded_window():
    # A stationary drone's constant-velocity lead is trivially exact, so the
    # gate must not stall the battery waiting for a leg that never starts.
    assert straight_leg_time_s(3.0, 0.6, 0.0) == math.inf
    assert straight_leg_time_s(3.0, 0.6, 0.01) == math.inf


# ── throw_window_ok ──────────────────────────────────────────────────────────

def test_early_in_a_leg_is_throwable():
    ok, reason = throw_window_ok(dist_to_wp_m=5.0, accept_radius_m=0.6,
                                 speed_mps=2.5, t_flight_s=0.75, margin_s=0.3)
    assert ok, reason


def test_approaching_a_corner_is_refused():
    # 1.0 m from the WP at 2.5 m/s = 0.16 s of leg; a 0.75 s flight spans the
    # corner and the lead aims at a point the drone never reaches.
    ok, reason = throw_window_ok(dist_to_wp_m=1.0, accept_radius_m=0.6,
                                 speed_mps=2.5, t_flight_s=0.75, margin_s=0.3)
    assert not ok
    assert "0.16" in reason and "1.05" in reason, reason


def test_margin_is_required_not_just_bare_flight_time():
    # Exactly enough leg for the flight but no slack -> refused, because the
    # detector needs the approach observed before impact, not just at it.
    ok, _ = throw_window_ok(dist_to_wp_m=2.1, accept_radius_m=0.6,
                            speed_mps=1.0, t_flight_s=1.5, margin_s=0.3)
    assert not ok
    ok, _ = throw_window_ok(dist_to_wp_m=2.5, accept_radius_m=0.6,
                            speed_mps=1.0, t_flight_s=1.5, margin_s=0.3)
    assert ok


def test_idle_patrol_always_throwable():
    # Hover instrument: no corners exist, so the window is not a constraint.
    ok, reason = throw_window_ok(dist_to_wp_m=0.1, accept_radius_m=0.6,
                                 speed_mps=0.0, t_flight_s=1.5,
                                 patrol_running=False)
    assert ok
    assert "idle" in reason.lower()


def test_missing_patrol_state_is_refused_not_assumed_ok():
    # If /huitzilin/patrol_state never arrives we must not silently fall back
    # to "throw anyway" — that is exactly the v9 behaviour being fixed.
    ok, reason = throw_window_ok(dist_to_wp_m=None, accept_radius_m=0.6,
                                 speed_mps=2.5, t_flight_s=0.75)
    assert not ok
    assert "no patrol state" in reason.lower()


def test_b01_on_a_five_metre_leg_is_structurally_unmeasurable():
    """
    The measurement that decides whether the patrol loop needs enlarging.

    B01 is 4 m/s over offset_forward_m 6.0 => 1.50 s of flight. The best
    possible window on a 5 m leg with a 0.6 m accept radius, at the measured
    2.5-3.2 m/s patrol speed, is (5.0-0.6)/2.5 = 1.76 s — less than
    1.50 + 0.30 of margin. So no amount of waiting produces a valid B01 throw
    on this loop; the gate will correctly refuse every one.
    """
    t_flight = 6.0 / 4.0
    best_window = straight_leg_time_s(5.0, 0.6, 2.5)
    assert round(best_window, 6) == 1.76
    ok, _ = throw_window_ok(dist_to_wp_m=5.0, accept_radius_m=0.6,
                            speed_mps=2.5, t_flight_s=t_flight, margin_s=0.3)
    assert not ok, "B01 must be reported unmeasurable, not silently thrown"

    # A 10 m leg makes the same scenario measurable — this is the harness fix
    # the report should justify, not a guess.
    ok, _ = throw_window_ok(dist_to_wp_m=10.0, accept_radius_m=0.6,
                            speed_mps=2.5, t_flight_s=t_flight, margin_s=0.3)
    assert ok


def test_b03_fast_throw_fits_comfortably():
    # 14 m/s over 6 m = 0.43 s; fits in most of a 5 m leg.
    ok, _ = throw_window_ok(dist_to_wp_m=3.0, accept_radius_m=0.6,
                            speed_mps=2.5, t_flight_s=6.0 / 14.0, margin_s=0.3)
    assert ok


def test_sustained_turn_passes_a_dv_gate_but_fails_the_window():
    """
    Why the |dv| gate was not enough (it is what battery v9 shipped).

    ArduPilot slows into a corner, so close to the waypoint |dv| per 0.15 s
    sample falls under the 0.35 m/s steady gate while the path is still
    bending. The window gate keys off geometry instead and refuses regardless
    of how smooth v looks.
    """
    speed_near_corner = 0.8          # decelerating into the WP
    dv_per_sample = 0.12             # well under the 0.35 m/s steady gate
    assert dv_per_sample <= 0.35

    ok, reason = throw_window_ok(dist_to_wp_m=0.9, accept_radius_m=0.6,
                                 speed_mps=speed_near_corner,
                                 t_flight_s=0.75, margin_s=0.3)
    assert not ok, reason
