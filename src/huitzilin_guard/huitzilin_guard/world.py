"""Where a detected person is standing, in the same frame as the box.

Pure arithmetic, no rclpy and no numpy: three frame changes and an addition.

    camera optical  ->  body FLU  ->  world ENU

pose_detector reports joints in the CAMERA OPTICAL frame, which is the
computer-vision convention and not the robotics one: +X right, +Y DOWN, +Z
along the lens axis away from the camera. The body frame is FLU (+X forward,
+Y left, +Z up), so the mapping is a pure axis relabel with two sign flips
and no trigonometry:

    forward = +z_optical      left = -x_optical      up = -y_optical

Then rotate into the world by the drone's attitude and add its position.

FULL ATTITUDE, NOT YAW ONLY. This is the part that is tempting to skip and
must not be. The aircraft banks into every turn on a rectangular circuit, and
a detection is displaced by roughly tilt x range when roll and pitch are
dropped: 10 degrees of bank at 6 m is about a metre of lateral error, which
is more than enough to move somebody across a box edge and either raise a
false alarm or miss a real one. The error is largest exactly at the corners,
where the bank is largest -- and a perimeter patrol spends much of its time
turning.

Duplicating huitzilin_perception's cloud_geometry.quat_to_rot was not an
option: that package is on this one's forbidden-import list, which is what
keeps the guard subsystem unable to reach the projectile pipeline. The
rotation below is written as a quaternion-vector product rather than a matrix
so it stays short enough to check by eye.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

# Below this the quaternion is not a rotation. ArduPilot's EKF publishes
# (0, 0, 0, 0) before it has converged, and rotating by that collapses every
# detection onto the drone's own position -- which, on a patrol that flies
# the perimeter, is a point ON the box edge. The alarm would then fire on the
# aircraft's own location. Rejecting the frame is the only safe reading.
MIN_QUAT_NORM = 1e-6


class WorldTransformError(ValueError):
    """The detection cannot be placed in the world. Never guessed at."""


def optical_to_flu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Camera optical (right, down, forward) -> body FLU (forward, left, up)."""
    return (z, -x, -y)


def rotate_by_quat(v: Sequence[float],
                   quat_xyzw: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate a body-frame vector into the world by an ENU orientation.

    quat_xyzw is (x, y, z, w), the order nav_msgs/Odometry uses. It is
    normalised here rather than trusted: an unnormalised quaternion scales
    the vector as well as rotating it, which reads downstream as a person at
    the wrong range rather than as a bad orientation.
    """
    qx, qy, qz, qw = (float(c) for c in quat_xyzw)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < MIN_QUAT_NORM:
        raise WorldTransformError(
            "orientation quaternion is (%g, %g, %g, %g), which is not a "
            "rotation. The EKF publishes this before it converges; rotating "
            "by it would place every detection at the drone's own position."
            % (qx, qy, qz, qw))
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    vx, vy, vz = (float(c) for c in v)
    # v' = v + 2 * qvec x (qvec x v + w * v)
    tx = qy * vz - qz * vy + qw * vx
    ty = qz * vx - qx * vz + qw * vy
    tz = qx * vy - qy * vx + qw * vz
    return (vx + 2.0 * (qy * tz - qz * ty),
            vy + 2.0 * (qz * tx - qx * tz),
            vz + 2.0 * (qx * ty - qy * tx))


def camera_point_to_world(point_optical: Sequence[float],
                          drone_position_enu: Sequence[float],
                          drone_quat_xyzw: Sequence[float]
                          ) -> Tuple[float, float, float]:
    """One camera-frame point to world ENU metres."""
    flu = optical_to_flu(*(float(c) for c in point_optical))
    rx, ry, rz = rotate_by_quat(flu, drone_quat_xyzw)
    px, py, pz = (float(c) for c in drone_position_enu)
    return (px + rx, py + ry, pz + rz)


# Hips and shoulders. The torso is the most reliably detected part of a body
# and the most stable stand-in for where somebody is standing: limbs swing,
# and a single wrist can be a metre from the person it belongs to.
TORSO_JOINTS = ("hip_l", "hip_r", "shoulder_l", "shoulder_r")


def person_point_optical(joints: Dict[str, Sequence[float]]
                         ) -> Optional[Tuple[float, float, float]]:
    """Centroid of one body in the camera frame, or None if it has no joints.

    Torso joints if any are present, otherwise every joint that is. The
    fallback matters: a person half behind a parked car has no hips in frame,
    and refusing to place them would make partial visibility a blind spot
    exactly where somebody trying not to be seen would stand.
    """
    chosen = [joints[name] for name in TORSO_JOINTS if name in joints]
    if not chosen:
        chosen = list(joints.values())
    if not chosen:
        return None

    count = float(len(chosen))
    return (sum(float(j[0]) for j in chosen) / count,
            sum(float(j[1]) for j in chosen) / count,
            sum(float(j[2]) for j in chosen) / count)
