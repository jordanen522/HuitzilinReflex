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
