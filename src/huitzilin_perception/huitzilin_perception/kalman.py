"""
kalman.py — pure-numpy 6-state ballistic Kalman filter + dodge planning (Week 4).

No ROS imports: everything is unit-testable anywhere (test/test_kalman.py),
following the cloud_geometry.py pattern. evasion_node.py is the ROS consumer.

Frame contract: ALL vectors here live in the fixed `odom` ENU frame. The
evasion node re-expresses /threat/centroid (base_link) into odom before
calling process() — filtering in the moving body frame would alias the
drone's own motion into the projectile velocity estimate (the same failure
mode as the Week 3 egomotion regression, one layer up).

Model: constant velocity + known gravity as control input. The sim
projectile is drag-free (models/projectile/model.sdf), so gravity-as-input
is exact; process_accel_std absorbs the residual (real-world drag, stereo
jitter) and becomes the Week 8 re-tuning knob.
"""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

import numpy as np

GRAVITY_ENU = np.array([0.0, 0.0, -9.80665])
CHI2_GATE_3DOF_99 = 11.345  # chi-square 99% quantile, 3 DOF


class DodgePlan(NamedTuple):
    tca_s: float           # time to closest approach from "now" [s]
    miss_m: float          # predicted closest range drone<->projectile [m]
    miss_vec: np.ndarray   # (3,) drone->projectile vector at closest approach
    direction: np.ndarray  # (3,) unit ENU dodge direction


class ProjectileTracker:
    """
    Kalman filter over x = [px py pz vx vy vz] (odom ENU).

    Measurements are 3D centroid positions; timestamps are sim-time seconds.
    dt is derived from consecutive measurement stamps, so irregular detector
    output (dropped frames) is handled naturally.
    """

    def __init__(
        self,
        *,
        process_accel_std: float = 3.0,
        meas_std_m: float = 0.15,
        init_vel_std: float = 15.0,
        gate_chi2: float = CHI2_GATE_3DOF_99,
        track_timeout_s: float = 0.5,
        max_consecutive_rejects: int = 3,
    ) -> None:
        self._q = float(process_accel_std)
        self._R = np.eye(3) * float(meas_std_m) ** 2
        self._init_vel_var = float(init_vel_std) ** 2
        self._gate = float(gate_chi2)
        self._timeout = float(track_timeout_s)
        self._max_rejects = int(max_consecutive_rejects)
        self._H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.reset()

    # ── track lifecycle ──────────────────────────────────────────────────

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None
        self._t = float("nan")
        self._n_updates = 0
        self._rejects = 0

    @property
    def has_track(self) -> bool:
        return self._x is not None

    @property
    def n_updates(self) -> int:
        return self._n_updates

    @property
    def last_update_t(self) -> float:
        return self._t

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        """(position (3,), velocity (3,)) of the current track estimate."""
        if self._x is None:
            raise RuntimeError("no active track")
        return self._x[:3].copy(), self._x[3:].copy()

    # ── filtering ────────────────────────────────────────────────────────

    def process(self, t: float, z) -> bool:
        """
        Feed one measurement. Returns True if it initialised or updated the
        track, False if rejected (gated outlier / out-of-order stamp).
        A stale track (> track_timeout_s since last stamp) resets first, so
        a second throw starts a fresh track instead of dragging the old one.
        """
        z = np.asarray(z, dtype=np.float64).reshape(3)

        if self._x is not None and (t - self._t) > self._timeout:
            self.reset()

        if self._x is None:
            self._x = np.concatenate([z, np.zeros(3)])
            self._P = np.diag([self._R[0, 0]] * 3 + [self._init_vel_var] * 3)
            self._t = t
            self._n_updates = 1
            self._rejects = 0
            return True

        dt = t - self._t
        if dt <= 0.0:
            return False  # out-of-order / duplicate stamp

        x_pred, P_pred = self._predict(dt)

        # Mahalanobis gate on the innovation
        nu = z - self._H @ x_pred
        S = self._H @ P_pred @ self._H.T + self._R
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 > self._gate:
            self._rejects += 1
            if self._rejects >= self._max_rejects:
                # Persistent disagreement -> different object; reseed from it.
                self.reset()
                return self.process(t, z)
            # Keep the prediction, drop the outlier.
            self._x, self._P, self._t = x_pred, P_pred, t
            return False

        K = P_pred @ self._H.T @ np.linalg.inv(S)
        self._x = x_pred + K @ nu
        self._P = (np.eye(6) - K @ self._H) @ P_pred
        self._t = t
        self._n_updates += 1
        self._rejects = 0
        return True

    def _predict(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        u = np.concatenate([0.5 * dt * dt * GRAVITY_ENU, dt * GRAVITY_ENU])
        q2 = self._q ** 2
        Q = np.block([
            [np.eye(3) * (dt ** 4 / 4.0 * q2), np.eye(3) * (dt ** 3 / 2.0 * q2)],
            [np.eye(3) * (dt ** 3 / 2.0 * q2), np.eye(3) * (dt * dt * q2)],
        ])
        return F @ self._x + u, F @ self._P @ F.T + Q


# ── Dodge planning ────────────────────────────────────────────────────────


def predict_closest_approach(
    p_rel,
    v_rel,
    *,
    horizon_s: float = 3.0,
    step_s: float = 0.01,
    gravity=GRAVITY_ENU,
) -> tuple[float, float, np.ndarray]:
    """
    Closest approach of a ballistic projectile RELATIVE to the drone.
    p_rel / v_rel: projectile state minus drone state (ENU). The drone is
    assumed constant-velocity over the horizon, so the relative acceleration
    is exactly gravity. Sampled numerically (vectorised): gravity makes the
    closed form a quartic, and 300 samples cost microseconds.
    Returns (tca_s, miss_m, miss_vec) with miss_vec = drone->projectile at
    closest approach.
    """
    ts = np.arange(0.0, horizon_s + step_s, step_s)
    rel = (np.asarray(p_rel, dtype=np.float64)
           + ts[:, None] * np.asarray(v_rel, dtype=np.float64)
           + 0.5 * ts[:, None] ** 2 * np.asarray(gravity, dtype=np.float64))
    d = np.linalg.norm(rel, axis=1)
    i = int(np.argmin(d))
    return float(ts[i]), float(d[i]), rel[i]


def dodge_direction(miss_vec, approach_vel, *, min_offset_m: float = 0.05) -> np.ndarray:
    """
    Unit ENU dodge direction: perpendicular to the approach axis, pointing
    AWAY from where the projectile will pass (miss_vec is drone->projectile
    at closest approach, so -miss_vec_perp opens the gap fastest).
    Dead-centre hits have no defined "away" side — fall back to a horizontal
    perpendicular (deterministic, so battery runs are repeatable).
    """
    a = np.asarray(approach_vel, dtype=np.float64)
    a_norm = np.linalg.norm(a)
    a_hat = a / a_norm if a_norm > 1e-9 else np.array([1.0, 0.0, 0.0])

    m = np.asarray(miss_vec, dtype=np.float64)
    m_perp = m - (m @ a_hat) * a_hat
    m_perp_norm = np.linalg.norm(m_perp)
    if m_perp_norm >= min_offset_m:
        return -m_perp / m_perp_norm

    lateral = np.cross(a_hat, np.array([0.0, 0.0, 1.0]))
    lat_norm = np.linalg.norm(lateral)
    if lat_norm < 1e-6:  # vertical approach
        return np.array([1.0, 0.0, 0.0])
    return lateral / lat_norm


def clamp_dodge_to_clearance(
    direction,
    altitude_m: float,
    *,
    floor_m: float,
    descent_len_m: float,
) -> np.ndarray:
    """
    Re-aim a dodge that would fly the drone into the ground. Returns a unit
    ENU vector.

    dodge_direction() only sees the approach geometry, and it is *usually*
    right to escape downward: a gravity-compensated throw arrives descending,
    so the fastest-opening perpendicular points down. That is also how the
    drone kills itself. Measured live 2026-07-26 hovering at 2 m:

        dir_body=(+0.01,+0.68,-0.74)
        dir_body=(+0.03,+0.56,-0.83)

    (body FLU, so z is up). At dodge_speed_mps x dodge_duration_s = 1.5 m
    that is >1.2 m of descent from 2 m — MAVProxy logged Crash then Disarm,
    and the rest of the battery aborted on the MIN_SPAWN_Z guard because the
    vehicle was lying on the runway.

    descent_len_m is how far the maneuver travels if the whole command is
    spent going down (dodge_speed_mps * dodge_duration_s). The downward
    component is capped at whatever fraction of that fits in the headroom
    above floor_m, and the freed budget is re-spent along the *same*
    horizontal escape bearing, so escape speed is preserved and the drone
    still leaves the projectile's pass side.

    Escape is never flipped upward. The ball is descending through that
    space; climbing into it trades a ground strike for a hit.
    """
    d = np.asarray(direction, dtype=np.float64)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    d = d / n
    if d[2] >= 0.0:
        return d

    headroom_m = max(float(altitude_m) - float(floor_m), 0.0)
    max_down = (min(1.0, headroom_m / descent_len_m)
                if descent_len_m > 1e-9 else 0.0)
    if -d[2] <= max_down:
        return d

    horiz = np.array([d[0], d[1], 0.0])
    h_norm = np.linalg.norm(horiz)
    if h_norm < 1e-6:
        # Straight down with no headroom: any horizontal beats descending.
        # Deterministic pick so battery runs stay repeatable.
        return np.array([1.0, 0.0, 0.0])
    h_keep = math.sqrt(max(1.0 - max_down ** 2, 0.0))
    return np.array([horiz[0] / h_norm * h_keep,
                     horiz[1] / h_norm * h_keep,
                     -max_down])


def should_dodge(
    n_updates: int,
    miss_m: float,
    tca_s: float,
    *,
    min_updates: int,
    threat_radius_m: float,
    trigger_horizon_s: float,
) -> bool:
    """Trigger policy: confirmed track + predicted pass inside the threat
    radius + impact soon enough that dodging now is the right call."""
    return (
        n_updates >= min_updates
        and miss_m < threat_radius_m
        and 0.0 <= tca_s <= trigger_horizon_s
    )


def plan_dodge(
    p_proj,
    v_proj,
    p_drone,
    v_drone,
    *,
    horizon_s: float = 3.0,
    altitude_m: Optional[float] = None,
    floor_m: float = 1.0,
    descent_len_m: float = 0.0,
) -> DodgePlan:
    """Convenience wrapper: absolute states in odom ENU -> full dodge plan.

    Pass altitude_m (metres above the ground, not above the odom origin) to
    apply the ground-clearance clamp — see clamp_dodge_to_clearance. Flight
    callers must pass it; omitting it returns the raw geometric escape, which
    can point at the ground.
    """
    p_rel = np.asarray(p_proj, dtype=np.float64) - np.asarray(p_drone, dtype=np.float64)
    v_rel = np.asarray(v_proj, dtype=np.float64) - np.asarray(v_drone, dtype=np.float64)
    tca, miss, miss_vec = predict_closest_approach(p_rel, v_rel, horizon_s=horizon_s)
    approach_vel = v_rel + tca * GRAVITY_ENU
    direction = dodge_direction(miss_vec, approach_vel)
    if altitude_m is not None:
        direction = clamp_dodge_to_clearance(
            direction, altitude_m, floor_m=floor_m, descent_len_m=descent_len_m)
    return DodgePlan(tca, miss, miss_vec, direction)
