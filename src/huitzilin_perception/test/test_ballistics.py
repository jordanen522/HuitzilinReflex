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
