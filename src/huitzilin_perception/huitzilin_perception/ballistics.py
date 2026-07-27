"""
ballistics.py — pure-numpy drag-free projectile math for Week 4.

Shared by spawn_projectile.py (spawn planning), dodge_battery.py (analytic
cross-checks), and the unit tests. No ROS imports — unit-testable on any
machine (mirrors the cloud_geometry.py pattern).

The Gazebo projectile model (models/projectile/model.sdf) has no aero-drag
plugin, so a thrown ball flies an exact parabola: these closed-form
trajectories match the simulator, not approximate it.

The model ships with link gravity OFF so the ball cannot fall during the
~0.5 s of sim time a `gz` CLI call costs; spawn_projectile restores it at
throw time as a persistent -mass*g wrench, which gives exactly a = -g. If a
throw ever flies straight, that wrench was dropped — the parabola assumed
here is only as good as that restore.
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

G_MPS2 = 9.80665  # standard gravity; Gazebo's default world gravity is 9.8 — the
                  # 0.07% delta is ~2 mm over a battery throw, absorbed by the KF
                  # process noise


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
    aim_at_drone: bool = False,
    target_vel_enu=None,
    spawn_latency_s: float = 0.0,
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

    aim_at_drone=False (default) -> Week 3-compatible, BYTE-IDENTICAL geometry:
      the spawn ray (and its lateral-miss perpendicular) uses drone_yaw, while
      the velocity is independently tilted by approach_angle_deg. This means
      approach_angle_deg tilts the velocity but does NOT aim the throw at the
      drone — a nominal 30 deg "hit" scenario actually passes
      ~offset_forward_m * sin(angle_rad) away from the drone (the Week 3
      crossing-trajectory semantic; this bit us once and is why Week 4 needs
      aim_at_drone=True for oblique scenarios).
    aim_at_drone=True  -> the spawn ray AND the lateral-miss perpendicular use
      (yaw + angle_rad) instead of yaw, so the ball's straight-line path
      genuinely closes on the drone with closest approach == miss_distance_m,
      regardless of approach_angle_deg. Velocity direction is unchanged
      (still -speed * unit(yaw + angle_rad)).

    target_vel_enu (ENU m/s, e.g. odom twist.linear) -> LEAD the shot: aim at
      where the drone WILL be, not where it is. Without this the whole geometry
      above is exact about a stale point, which is only correct for a hovering
      target. Measured on the Dell 2026-07-26 with a single clean stack: patrol
      translates at 2.5-3.2 m/s, and 8 m/s runs specifying
      miss_distance_m=0.0 measured closest approaches of 1.3-2.6 m -- almost
      exactly |v| * t_flight. The same throw against a hovering drone measured
      0.114 m, which is how we know the parabola and the gravity wrench were
      never at fault.
      The aim point is advanced by target_vel_enu * (spawn_latency_s +
      offset_forward_m / speed_mps). No iteration is needed: offset_forward_m
      is measured from the *aim point*, so the flight time stays exactly
      offset_forward_m / speed_mps however far the lead moves it.
      Pass None (default) to keep the legacy stale-point behaviour.
    spawn_latency_s -> dead time between sampling the drone state and the ball
      actually launching. The `gz` create call costs ~0.5 s of SIM time
      (see spawn_projectile), during which the drone keeps flying, so this is
      not negligible next to a 0.75 s flight. Folded into the lead above.
    """
    dx, dy, dz = (float(v) for v in drone_enu)
    yaw = float(drone_yaw)

    if target_vel_enu is not None:
        if speed_mps <= 0.0:
            raise ValueError("target_vel_enu lead requires speed_mps > 0")
        vx, vy, vz = (float(v) for v in target_vel_enu)
        t_lead = float(spawn_latency_s) + offset_forward_m / speed_mps
        dx += vx * t_lead
        dy += vy * t_lead
        dz += vz * t_lead

    angle_rad = math.radians(approach_angle_deg)
    ray = yaw + (angle_rad if aim_at_drone else 0.0)

    spawn = np.array([
        dx + offset_forward_m * math.cos(ray) + miss_distance_m * math.sin(ray),
        dy + offset_forward_m * math.sin(ray) - miss_distance_m * math.cos(ray),
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
