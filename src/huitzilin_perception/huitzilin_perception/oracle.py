"""
oracle.py — synthetic detections from ground truth. Pure numpy, no ROS.

Why this exists. The measured envelope closes at 14 m/s, and the arithmetic
says the binding term is SENSING: the sim depth camera first reports the
80 mm ball at ~3.2-3.6 m, so at 20 m/s the whole time of flight inside
detection range is 0.17 s -- less than confirmation plus pipeline, before the
airframe is asked to move at all. Every downstream question (is the tracker
fast enough, does the trigger fire, can the vehicle clear 0.30 m) is therefore
unanswerable in sim today, because nothing ever gets that far.

This module fabricates the measurement stream a longer-ranged sensor would
produce, from Gazebo truth, so the rest of the chain can be scored at 20 m/s
now. It is a REFEREE, not a detector: it says what happens IF detection range
were R, and deliberately says nothing about whether any real sensor can
achieve R. That is a hardware measurement (Week 6, mono-pair bring-up).

Read every number it produces with that caveat attached. In particular
`detection_range_m` is an input, never a result.

Fidelity choices, and why each is here rather than "detect everything":
  * range gates at BOTH ends -- a near limit exists too (roi_min_range_m 0.30)
  * a field-of-view gate -- a real camera cannot see behind itself, and a
    dodge planned on a ball outside the frustum would be free warning that no
    sensor could ever have given
  * PER-AXIS noise -- stereo error is not round. Depth error grows as z^2
    while lateral error grows as z, so an isotropic sigma either flatters the
    depth axis or slanders the bearing axes
  * uniform dropout only -- deliberately NOT range-dependent. Making detection
    probability fall with range would confound the one variable this harness
    exists to isolate.

Gates are evaluated on the true geometry and noise is applied afterwards:
visibility is a property of where the ball is, not of where the sensor
mistakenly reports it.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

# base_link is FLU (x forward, y left, z up), so the camera boresight is +x.
BORESIGHT = np.array([1.0, 0.0, 0.0])

# BOTH spawners' conventions, because this repo has two and they disagree:
#   spawn_projectile.py names its ball  projectile_<scenario>_<epoch>
#   dodge_battery.py    names its ball  ball_<run>_r<rep>_<epoch>
# The oracle used to match only "projectile_", so under the battery -- the one
# harness that scores anything -- it matched no model, published zero
# centroids, and every scenario came back NO-FIRE. That reads exactly like a
# trigger that never fires, which is why this must be a LIST and never a
# scalar: one prefix silently answers the wrong question instead of failing.
DEFAULT_BALL_PREFIXES = ("ball_", "projectile_")


def is_ball_name(name: str,
                 prefixes: Iterable[str] = DEFAULT_BALL_PREFIXES) -> bool:
    """Does this Gazebo model name belong to a thrown ball?"""
    return any(name.startswith(p) for p in prefixes if p)


def select_ball(
    candidates: Iterable[Tuple[str, Sequence[float]]],
    drone_world,
    prefixes: Iterable[str] = DEFAULT_BALL_PREFIXES,
) -> Optional[np.ndarray]:
    """Pick THE ball out of every model in one /gz/dynamic_poses message.

    Nearest-to-the-drone, not first-match, because matching two prefixes
    widens the set of things that can match. A run whose ball failed to
    despawn (gz_remove can fail -- spawn_projectile logs "Could NOT remove"
    and asks for it by hand) leaves a stale model in the world under the very
    same prefix, and first-match would then hand the tracker whichever one the
    message happened to list first. Nearest is also the benign choice when it
    is wrong: a stale ball sits still, so the tracker sees no closing velocity
    and never triggers, whereas silently tracking the wrong ball would score.

    Returns None when nothing matches, which the caller must treat as "no
    detection this frame" rather than as an error -- between runs there is
    legitimately no ball in the world at all.
    """
    drone = np.asarray(drone_world, dtype=np.float64).reshape(3)
    best = None
    best_d = float("inf")
    for name, pos in candidates:
        if not is_ball_name(name, prefixes):
            continue
        p = np.asarray(pos, dtype=np.float64).reshape(3)
        d = float(np.linalg.norm(p - drone))
        if d < best_d:
            best, best_d = p, d
    return best


def to_body(rel_world, R_body_to_world) -> np.ndarray:
    """World-frame drone->ball vector, expressed in base_link.

    R is the drone's body->world rotation, so its transpose maps world into
    body. Only the DIFFERENCE of two world positions is passed in, which is
    why no origin offset appears: the Gazebo world origin and the odom origin
    (the EKF's takeoff point) differ by a translation that cancels.
    """
    R = np.asarray(R_body_to_world, dtype=np.float64).reshape(3, 3)
    return R.T @ np.asarray(rel_world, dtype=np.float64).reshape(3)


def in_view(
    rel_body,
    *,
    detection_range_m: float,
    min_range_m: float = 0.30,
    fov_half_angle_deg: float = 45.0,
) -> bool:
    """Would a sensor with this range and frustum see the ball at all?"""
    rel_body = np.asarray(rel_body, dtype=np.float64).reshape(3)
    rng = float(np.linalg.norm(rel_body))
    if rng <= 0.0 or rng < min_range_m or rng > detection_range_m:
        return False
    cos_off = float(rel_body @ BORESIGHT) / rng
    return cos_off >= math.cos(math.radians(fov_half_angle_deg))


def apply_noise(rel_body, noise_std_xyz: Sequence[float], rng_gen) -> np.ndarray:
    """Add per-axis Gaussian error in base_link (forward, left, up).

    Zero sigma on an axis must reproduce that axis exactly, so a noiseless
    oracle is genuinely noiseless rather than merely nearly so.
    """
    rel_body = np.asarray(rel_body, dtype=np.float64).reshape(3)
    sigma = np.asarray(noise_std_xyz, dtype=np.float64).reshape(3)
    if not np.any(sigma > 0.0):
        return rel_body.copy()
    return rel_body + rng_gen.normal(0.0, 1.0, 3) * sigma


def measurement_covariance(noise_std_xyz: Sequence[float],
                           floor_m: float = 1e-3) -> np.ndarray:
    """The 3x3 R matching this oracle's own noise, in base_link.

    Feeds the tracker's per-measurement covariance hook. Floored so a
    noiseless oracle still yields an invertible innovation covariance.
    """
    sigma = np.maximum(
        np.asarray(noise_std_xyz, dtype=np.float64).reshape(3), floor_m)
    return np.diag(sigma ** 2)


def synthesize(
    rel_world,
    R_body_to_world,
    *,
    detection_range_m: float,
    min_range_m: float = 0.30,
    fov_half_angle_deg: float = 45.0,
    noise_std_xyz: Sequence[float] = (0.0, 0.0, 0.0),
    dropout_prob: float = 0.0,
    rng_gen: Optional[np.random.Generator] = None,
) -> Optional[np.ndarray]:
    """One synthetic base_link detection, or None if the ball is not reported.

    None covers both "outside the sensor's reach" and "dropped this frame";
    the caller cannot tell them apart, exactly as it cannot with a real one.
    """
    rng_gen = np.random.default_rng() if rng_gen is None else rng_gen
    rel_body = to_body(rel_world, R_body_to_world)

    if not in_view(rel_body,
                   detection_range_m=detection_range_m,
                   min_range_m=min_range_m,
                   fov_half_angle_deg=fov_half_angle_deg):
        return None

    if dropout_prob > 0.0 and float(rng_gen.random()) < dropout_prob:
        return None

    return apply_noise(rel_body, noise_std_xyz, rng_gen)
