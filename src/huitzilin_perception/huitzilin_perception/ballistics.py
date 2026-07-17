"""
ballistics.py — pure-numpy drag-free projectile math for Week 4.

Shared by spawn_projectile.py (spawn planning), dodge_battery.py (analytic
cross-checks), and the unit tests. No ROS imports — unit-testable on any
machine (mirrors the cloud_geometry.py pattern).

The Gazebo projectile model (models/projectile/model.sdf) has gravity ON and
no aero-drag plugin, so a spawned ball flies an exact parabola: these
closed-form trajectories match the simulator, not approximate it.
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

G_MPS2 = 9.80665  # Gazebo Harmonic default world gravity


class SpawnPlan(NamedTuple):
    """World-ENU spawn state for one projectile throw."""
    position: np.ndarray   # (3,) ENU spawn point [m]
    velocity: np.ndarray   # (3,) ENU initial velocity [m/s]


def ballistic_positions(p0, v0, ts, g: float = G_MPS2) -> np.ndarray:
    """
    Drag-free trajectory samples: p(t) = p0 + v0*t + 0.5*(0,0,-g)*t^2.
    p0, v0: (3,) ENU. ts: scalar or (N,) seconds since launch.
    Returns (N, 3) float64 (N=1 for scalar ts).
    """
    ts = np.atleast_1d(np.asarray(ts, dtype=np.float64)).reshape(-1, 1)
    p0 = np.asarray(p0, dtype=np.float64)
    v0 = np.asarray(v0, dtype=np.float64)
    grav = np.array([0.0, 0.0, -g])
    return p0 + ts * v0 + 0.5 * ts * ts * grav


def compute_spawn(
    drone_enu,
    drone_yaw: float,
    *,
    speed_mps: float,
    approach_angle_deg: float = 0.0,
    miss_distance_m: float = 0.0,
    offset_forward_m: float = 6.0,
    offset_vertical_m: float = 0.0,
    compensate_gravity: bool = False,
    g: float = G_MPS2,
) -> SpawnPlan:
    """
    Spawn pose + velocity for one scenario, world ENU.

    Replicates the Week 3 spawn_projectile geometry exactly: spawn ahead of
    the drone along its yaw, offset laterally by miss_distance_m, velocity
    pointed back at the drone tilted by approach_angle_deg.

    compensate_gravity=False -> flat throw (vz=0), identical to Week 3: at
      8 m/s over 6 m the ball passes ~2.8 m BELOW a target at spawn altitude.
    compensate_gravity=True  -> lofts the throw so the parabola returns to
      the aim altitude after the straight-line flight time
      t = offset_forward_m / speed_mps:
          vz0 = 0.5*g*t - offset_vertical_m/t
      With offset_vertical_m=0 the ball arrives AT drone altitude — a true
      hit trajectory for the Week 4 dodge battery.
    """
    dx, dy, dz = (float(v) for v in drone_enu)
    yaw = float(drone_yaw)
    angle_rad = math.radians(approach_angle_deg)

    spawn = np.array([
        dx + offset_forward_m * math.cos(yaw) + miss_distance_m * math.sin(yaw),
        dy + offset_forward_m * math.sin(yaw) - miss_distance_m * math.cos(yaw),
        dz + offset_vertical_m,
    ])

    vel = speed_mps * np.array([
        -math.cos(yaw + angle_rad),
        -math.sin(yaw + angle_rad),
        0.0,
    ])

    if compensate_gravity:
        if speed_mps <= 0.0:
            raise ValueError("compensate_gravity requires speed_mps > 0")
        t_flight = offset_forward_m / speed_mps
        vel[2] = 0.5 * g * t_flight - offset_vertical_m / t_flight

    return SpawnPlan(position=spawn, velocity=vel)
