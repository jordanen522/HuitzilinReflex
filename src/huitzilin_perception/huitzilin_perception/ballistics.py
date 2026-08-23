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
    # The point the throw was aimed at: the drone's position advanced by the
    # lead, or its raw position when not leading. Exposed so a caller can score
    # its own prediction. Comparing a measured miss against the scenario SPEC
    # conflates "the lead predicted the wrong place" with "the geometry about
    # the predicted place was wrong", and the hover control
    # established the geometry is exact to a few cm — so the aim point is the
    # quantity still worth measuring. Appended last: every existing caller
    # reads .position / .velocity by attribute, never by unpacking.
    aim_point: np.ndarray  # (3,) ENU predicted target position [m]


class MissVector(NamedTuple):
    """A closest-approach miss resolved in the ball's own path frame.

    Supplements the scalar `min_dist`, which is the PERPENDICULAR distance from
    the path to the drone and therefore cannot distinguish a lead that fired
    early from a path that drifted sideways from a gravity-compensation error —
    three different bugs with three different fixes.
    """
    along_m: float   # + = ball ended up BEYOND the drone along its own heading
    cross_m: float   # + = ball passed to the LEFT of the drone (ball's heading)
    vert_m: float    # + = ball passed ABOVE the drone (world z)
    dist_m: float    # |ball - drone|; == hypot of the three above


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
      target. Measured on the Dell with a single clean stack: patrol
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
        if offset_forward_m <= 0.0:
            # t_flight is the whole compensation. At offset_forward_m == 0 the
            # ball starts on top of the drone, so there is no flight to
            # compensate -- and the line below divides by t_flight, making this
            # a ZeroDivisionError from inside the maths rather than a rejected
            # scenario. Only speed_mps was guarded.
            raise ValueError(
                "compensate_gravity requires offset_forward_m > 0 "
                "(got %r): flight time is offset_forward_m / speed_mps"
                % (offset_forward_m,))
        t_flight = offset_forward_m / speed_mps
        vel[2] = 0.5 * g * t_flight - offset_vertical_m / t_flight

    return SpawnPlan(position=spawn, velocity=vel,
                     aim_point=np.array([dx, dy, dz]))


def miss_components(ball_enu, drone_enu, ball_vel_enu) -> MissVector:
    """Decompose a closest-approach miss into the ball's path frame.

    Returns the vector (ball - drone) resolved as:
      along : along the ball's HORIZONTAL heading, + = ball went past the drone
      cross : to the left of that heading,         + = ball passed on its left
      vert  : world z,                             + = ball passed above

    Why these axes. The three components map one-to-one onto three distinct
    failure modes, which the scalar min_dist cannot separate:
      - `along` is a timing/lead-magnitude error: the aim point was right but
        reached at the wrong moment, or the target's speed was mispredicted.
        Note it is near-zero AT the true closest approach of a straight path by
        definition, so what it really reports is the sampling residual plus any
        curvature — a large |along| means the samples bracket the approach
        coarsely, not that the lead is fine.
      - `cross` is a heading error: the target turned, or the lead's direction
        was wrong. This is the component a head-on throw cannot explain away,
        and the one the +30/-30 deg asymmetry pointed at.
      - `vert` isolates gravity compensation, an entirely world-z quantity.
    Vertical is deliberately WORLD vertical rather than perpendicular to a
    lofted path: mixing the loft angle into the axes would smear a gravity error
    across two components and destroy exactly the separation this exists for.

    Raises ValueError for a path with no horizontal component — a purely
    vertical throw has no heading, so "left of it" is undefined. Returning a
    silent zero there would read as a perfect cross-track score.
    """
    ball = np.asarray(ball_enu, dtype=np.float64)
    drone = np.asarray(drone_enu, dtype=np.float64)
    vel = np.asarray(ball_vel_enu, dtype=np.float64)

    horiz = float(math.hypot(vel[0], vel[1]))
    if horiz < 1e-9:
        raise ValueError(
            "ball_vel_enu has no horizontal component: a vertical path has no "
            "heading, so along/cross-track are undefined")

    fwd = np.array([vel[0] / horiz, vel[1] / horiz, 0.0])
    left = np.array([-fwd[1], fwd[0], 0.0])   # up x fwd, still horizontal
    miss = ball - drone
    return MissVector(
        along_m=float(miss @ fwd),
        cross_m=float(miss @ left),
        vert_m=float(miss[2]),
        dist_m=float(np.linalg.norm(miss)),
    )
