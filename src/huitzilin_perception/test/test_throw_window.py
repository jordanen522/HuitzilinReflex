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
leg remains AND the drone is already at cruise.

Both halves are load-bearing, and the second was learned by shipping without
it: v10 gated on leg time alone and was WORSE than v9 (16/20 off-target).
See test_v10_inversion_slow_near_corner_must_not_look_safe.
"""

import math

from huitzilin_perception.throw_window import (
    DEFAULT_MIN_CRUISE_FRAC,
    straight_leg_time_s,
    throw_window_ok,
)

# Measured on the Dell over 45 s of patrol, 1350 odom samples:
# median 2.09 m/s, p90 3.35, max 3.49.
CRUISE = 3.4


def _ok(**kw):
    """throw_window_ok with the measured cruise and a drone at cruise."""
    kw.setdefault("cruise_mps", CRUISE)
    kw.setdefault("speed_mps", kw["cruise_mps"])
    kw.setdefault("accept_radius_m", 0.6)
    return throw_window_ok(**kw)


# ── straight_leg_time_s ───────────────────────────────────────────────────────

def test_leg_time_is_remaining_distance_over_cruise():
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


def test_parked_drone_has_an_unbounded_window():
    # Cruise below the hover floor means no corner is ever reached, so the
    # gate must not stall the battery waiting for a leg that never starts.
    assert straight_leg_time_s(3.0, 0.6, 0.0) == math.inf
    assert straight_leg_time_s(3.0, 0.6, 0.01) == math.inf


# ── the v10 inversion ─────────────────────────────────────────────────────────

def test_v10_inversion_slow_near_corner_must_not_look_safe():
    """
    The regression that made v10 worse than v9.

    Just out of a corner: 4.9 m to the next waypoint but only 0.63 m/s so far.
    Dividing by the INSTANTANEOUS speed reports (4.9-0.6)/0.63 = 6.8 s of
    window — the run that actually measured 3.22 m of aim error. Evaluated at
    cruise the same instant is 1.26 s, and the at-cruise check refuses it
    outright because the lead would extrapolate from 0.63 m/s and aim short.
    """
    slow = 0.63
    inflated = straight_leg_time_s(4.9, 0.6, slow)
    honest = straight_leg_time_s(4.9, 0.6, CRUISE)
    assert inflated > 6.0, inflated
    assert round(honest, 2) == 1.26
    assert inflated > 5 * honest        # the inversion, quantified

    ok, reason = throw_window_ok(dist_to_wp_m=4.9, accept_radius_m=0.6,
                                 speed_mps=slow, cruise_mps=CRUISE,
                                 t_flight_s=0.75, margin_s=0.3)
    assert not ok
    assert "accelerating out of a corner" in reason, reason


def test_at_cruise_mid_leg_is_throwable():
    ok, reason = _ok(dist_to_wp_m=5.0, t_flight_s=6.0 / 14.0, margin_s=0.3)
    assert ok, reason


def test_cruise_floor_is_the_measured_eighty_percent():
    # 0.8 * 3.4 = 2.72 m/s. Just under must refuse, just over must pass.
    assert DEFAULT_MIN_CRUISE_FRAC == 0.80
    ok, _ = _ok(dist_to_wp_m=5.0, speed_mps=2.70, t_flight_s=0.43)
    assert not ok
    ok, _ = _ok(dist_to_wp_m=5.0, speed_mps=2.75, t_flight_s=0.43)
    assert ok


# ── leg-length gate ──────────────────────────────────────────────────────────

def test_approaching_a_corner_is_refused():
    # 1.0 m from the WP at cruise 3.4 = 0.12 s of leg; a 0.75 s flight spans
    # the corner and the lead aims at a point the drone never reaches.
    ok, reason = _ok(dist_to_wp_m=1.0, t_flight_s=0.75, margin_s=0.3)
    assert not ok
    assert "0.12" in reason and "1.05" in reason, reason


def test_margin_is_required_not_just_bare_flight_time():
    # Exactly enough leg for the flight but no slack -> refused, because the
    # detector needs the approach observed before impact, not just at it.
    ok, _ = throw_window_ok(dist_to_wp_m=2.1, accept_radius_m=0.6,
                            speed_mps=1.0, cruise_mps=1.0,
                            t_flight_s=1.5, margin_s=0.3)
    assert not ok
    ok, _ = throw_window_ok(dist_to_wp_m=2.5, accept_radius_m=0.6,
                            speed_mps=1.0, cruise_mps=1.0,
                            t_flight_s=1.5, margin_s=0.3)
    assert ok


def test_idle_patrol_always_throwable():
    # Hover instrument: no corners exist, so the window is not a constraint.
    ok, reason = _ok(dist_to_wp_m=0.1, speed_mps=0.0, t_flight_s=1.5,
                     patrol_running=False)
    assert ok
    assert "idle" in reason.lower()


def test_missing_patrol_state_is_refused_not_assumed_ok():
    # If /huitzilin/patrol_state never arrives we must not silently fall back
    # to "throw anyway" — that is exactly the v9 behaviour being fixed.
    ok, reason = _ok(dist_to_wp_m=None, t_flight_s=0.75)
    assert not ok
    assert "no patrol state" in reason.lower()


# ── what the measurement says about the loop itself ──────────────────────────

def test_five_metre_loop_cannot_measure_the_slow_scenario():
    """
    The measurement that justifies enlarging the patrol loop.

    B01 is 4 m/s over offset_forward_m 6.0 => 1.50 s of flight, so it needs
    1.80 s of straight leg. The BEST a 5 m leg can offer at the measured
    3.4 m/s cruise is (5.0-0.6)/3.4 = 1.29 s. No amount of waiting produces a
    valid B01 throw on this loop, so the gate must refuse every one and the
    report must say the scenario is unmeasurable rather than emit a miss.
    """
    t_flight = 6.0 / 4.0
    best = straight_leg_time_s(5.0, 0.6, CRUISE)
    assert round(best, 2) == 1.29
    ok, _ = _ok(dist_to_wp_m=5.0, t_flight_s=t_flight, margin_s=0.3)
    assert not ok, "B01 must be reported unmeasurable, not silently thrown"


def test_ten_metre_loop_makes_the_slow_scenario_measurable():
    # (10.0-0.6)/3.4 = 2.76 s, comfortably over B01's 1.80 s requirement.
    # This is the harness fix the report should justify, not a guess.
    assert round(straight_leg_time_s(10.0, 0.6, CRUISE), 2) == 2.76
    ok, _ = _ok(dist_to_wp_m=10.0, t_flight_s=6.0 / 4.0, margin_s=0.3)
    assert ok


def test_medium_scenario_is_marginal_on_a_five_metre_leg():
    # 8 m/s => 0.75 s flight, needs 1.05 s of the 1.29 s a 5 m leg offers.
    # Throwable only in the first quarter of the leg — which is why B02/B04-B07
    # were erratic rather than uniformly bad.
    ok, _ = _ok(dist_to_wp_m=5.0, t_flight_s=0.75, margin_s=0.3)
    assert ok
    ok, _ = _ok(dist_to_wp_m=4.0, t_flight_s=0.75, margin_s=0.3)
    assert not ok


def test_sustained_turn_passes_a_dv_gate_but_fails_the_window():
    """
    Why the |dv| gate was not enough (it is what battery v9 shipped).

    ArduPilot slows into a corner, so close to the waypoint |dv| per 0.15 s
    sample falls under the 0.35 m/s steady gate while the path is still
    bending. The window gate keys off geometry instead and refuses regardless
    of how smooth v looks.
    """
    dv_per_sample = 0.12             # well under the 0.35 m/s steady gate
    assert dv_per_sample <= 0.35

    ok, reason = throw_window_ok(dist_to_wp_m=0.9, accept_radius_m=0.6,
                                 speed_mps=0.8, cruise_mps=CRUISE,
                                 t_flight_s=0.75, margin_s=0.3)
    assert not ok, reason
