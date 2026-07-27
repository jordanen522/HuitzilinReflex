"""Unit tests for ballistics.py — pure math, no ROS."""
import math

import numpy as np
import pytest

from huitzilin_perception.ballistics import (
    G_MPS2,
    ballistic_positions,
    compute_spawn,
)


def test_vertical_drop_matches_half_g_t_squared():
    pts = ballistic_positions([0.0, 0.0, 10.0], [0.0, 0.0, 0.0], [0.0, 1.0])
    assert pts.shape == (2, 3)
    np.testing.assert_allclose(pts[0], [0.0, 0.0, 10.0])
    assert pts[1][2] == pytest.approx(10.0 - 0.5 * G_MPS2)


def test_scalar_time_gives_single_row():
    pts = ballistic_positions([0.0, 0.0, 2.0], [1.0, 0.0, 0.0], 2.0)
    assert pts.shape == (1, 3)
    assert pts[0][0] == pytest.approx(2.0)


def test_flat_throw_matches_week3_spawn_geometry():
    # Drone at origin, yaw 0 (facing +x ENU): spawn 6 m ahead, miss 0.5 m
    # lands at y = -0.5 (Week 3 formula: spawn_y = dy + fwd*sin(yaw) - miss*cos(yaw)).
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0, miss_distance_m=0.5)
    np.testing.assert_allclose(plan.position, [6.0, -0.5, 2.0], atol=1e-12)
    np.testing.assert_allclose(plan.velocity, [-8.0, 0.0, 0.0], atol=1e-12)


def test_oblique_velocity_direction():
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0, approach_angle_deg=30.0)
    assert plan.velocity[0] == pytest.approx(-8.0 * math.cos(math.radians(30.0)))
    assert plan.velocity[1] == pytest.approx(-8.0 * math.sin(math.radians(30.0)))
    assert plan.velocity[2] == pytest.approx(0.0)


def test_gravity_compensated_throw_returns_to_aim_altitude():
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0, compensate_gravity=True)
    t_flight = 6.0 / 8.0
    p = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    assert p[0] == pytest.approx(0.0, abs=1e-9)   # arrives at the drone x
    assert p[2] == pytest.approx(2.0, abs=1e-9)   # ...at drone altitude


def test_gravity_compensation_nulls_vertical_offset():
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0,
                         offset_vertical_m=-1.0, compensate_gravity=True)
    t_flight = 6.0 / 8.0
    p = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    assert p[2] == pytest.approx(2.0, abs=1e-9)   # climbs back to drone altitude


def test_gravity_comp_requires_positive_speed():
    with pytest.raises(ValueError):
        compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=0.0, compensate_gravity=True)


def test_aim_at_drone_oblique_throw_actually_hits():
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0,
                         approach_angle_deg=30.0, compensate_gravity=True,
                         aim_at_drone=True)
    t_flight = 6.0 / 8.0
    p = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    np.testing.assert_allclose(p, [0.0, 0.0, 2.0], atol=1e-9)


def test_aim_at_drone_preserves_miss_distance():
    plan = compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=8.0,
                         approach_angle_deg=30.0, miss_distance_m=0.5,
                         aim_at_drone=True)
    t_flight = 6.0 / 8.0
    p = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    assert np.linalg.norm(p[:2]) == pytest.approx(0.5, abs=1e-9)


def test_aim_at_drone_false_is_week3_identical():
    a = compute_spawn((0.0, 0.0, 2.0), 0.3, speed_mps=8.0, approach_angle_deg=30.0)
    b = compute_spawn((0.0, 0.0, 2.0), 0.3, speed_mps=8.0, approach_angle_deg=30.0,
                      aim_at_drone=False)
    np.testing.assert_allclose(a.position, b.position)
    np.testing.assert_allclose(a.velocity, b.velocity)


# ── Target lead (W4 live bring-up, 2026-07-26) ────────────────────────────
# Aiming at where the drone IS misses a patrolling drone by |v| * t_flight.
# Measured on the Dell with a single clean stack: patrol flies 2.5-3.2 m/s and
# 8 m/s "direct hit" runs missed by 1.3-2.6 m.


def test_lead_with_zero_velocity_changes_nothing():
    kw = dict(speed_mps=8.0, approach_angle_deg=20.0, miss_distance_m=0.3,
              compensate_gravity=True, aim_at_drone=True)
    a = compute_spawn((1.0, 2.0, 2.0), 0.4, **kw)
    b = compute_spawn((1.0, 2.0, 2.0), 0.4, target_vel_enu=(0.0, 0.0, 0.0),
                      spawn_latency_s=0.5, **kw)
    np.testing.assert_allclose(a.position, b.position, atol=1e-12)
    np.testing.assert_allclose(a.velocity, b.velocity, atol=1e-12)


def test_lead_omitted_is_identical_to_before():
    """target_vel_enu=None must leave every existing caller byte-identical."""
    kw = dict(speed_mps=8.0, approach_angle_deg=30.0, compensate_gravity=True)
    a = compute_spawn((0.0, 0.0, 2.0), 0.3, **kw)
    b = compute_spawn((0.0, 0.0, 2.0), 0.3, target_vel_enu=None, **kw)
    np.testing.assert_allclose(a.position, b.position, atol=1e-12)
    np.testing.assert_allclose(a.velocity, b.velocity, atol=1e-12)


def test_lead_ball_arrives_where_the_drone_will_be():
    # The measured failure: drone translating at 2.5 m/s across its own yaw.
    p0 = np.array([1.0, 5.0, 2.0])
    vel = np.array([2.4, -0.7, 0.0])
    latency, speed = 0.5, 8.0
    plan = compute_spawn(p0, 0.34, speed_mps=speed, compensate_gravity=True,
                         aim_at_drone=True, target_vel_enu=vel,
                         spawn_latency_s=latency)
    t_flight = 6.0 / speed
    ball = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    # The ball is launched `latency` after the odom sample and flies t_flight,
    # so the drone has moved for (latency + t_flight) by the time it arrives.
    drone_then = p0 + vel * (latency + t_flight)
    np.testing.assert_allclose(ball, drone_then, atol=1e-9)


def test_lead_without_latency_still_leads_by_flight_time():
    p0 = np.array([0.0, 0.0, 2.0])
    vel = np.array([0.0, 3.0, 0.0])
    plan = compute_spawn(p0, 0.0, speed_mps=8.0, compensate_gravity=True,
                         aim_at_drone=True, target_vel_enu=vel)
    t_flight = 6.0 / 8.0
    ball = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    np.testing.assert_allclose(ball, p0 + vel * t_flight, atol=1e-9)


def test_lead_preserves_miss_distance_about_the_predicted_point():
    p0 = np.array([0.0, 0.0, 2.0])
    vel = np.array([2.5, 0.0, 0.0])
    latency, speed, miss = 0.4, 8.0, 0.5
    plan = compute_spawn(p0, 0.0, speed_mps=speed, miss_distance_m=miss,
                         aim_at_drone=True, compensate_gravity=True,
                         target_vel_enu=vel, spawn_latency_s=latency)
    t_flight = 6.0 / speed
    ball = ballistic_positions(plan.position, plan.velocity, t_flight)[0]
    drone_then = p0 + vel * (latency + t_flight)
    assert np.linalg.norm(ball[:2] - drone_then[:2]) == pytest.approx(miss, abs=1e-9)


def test_lead_requires_positive_speed():
    with pytest.raises(ValueError):
        compute_spawn((0.0, 0.0, 2.0), 0.0, speed_mps=0.0,
                      target_vel_enu=(1.0, 0.0, 0.0))


def test_lead_scales_with_latency():
    p0, vel = (0.0, 0.0, 2.0), (3.0, 0.0, 0.0)
    near = compute_spawn(p0, 0.0, speed_mps=8.0, target_vel_enu=vel,
                         spawn_latency_s=0.0)
    far = compute_spawn(p0, 0.0, speed_mps=8.0, target_vel_enu=vel,
                        spawn_latency_s=0.5)
    # Both spawn 6 m ahead of their aim point, so the difference is exactly the
    # extra 0.5 s of predicted drone travel.
    assert far.position[0] - near.position[0] == pytest.approx(0.5 * 3.0)
