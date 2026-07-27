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
        # Lifetime reseed totals. Set BEFORE reset(), which must not clear them,
        # so a reseed cannot erase its own evidence.
        #
        # Why they are worth counting: a reseed returns the velocity state to
        # ZERO, and predict_closest_approach works on v_rel = v_proj - v_drone,
        # so a zero v_proj makes the ball look like it is receding — the relative
        # minimum lands at t~0, giving tca~0 and a predicted miss equal to the
        # current range. should_dodge then reads a genuine intercept as "not a
        # threat" until the velocity re-converges. Measured 2026-07-27: the
        # published predicted miss does track the ball's current range on inbound
        # throws, which is that signature — these counters say whether mid-flight
        # reseeds are the cause. The two paths are separated because their fixes
        # differ: rejects point at the detector's false-positive stream stealing
        # the track, timeouts at gaps in its output.
        self._n_reseeds_reject = 0
        self._n_reseeds_timeout = 0
        self.reset()

    # ── track lifecycle ──────────────────────────────────────────────────

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None
        self._t = float("nan")
        self._t_start = float("nan")
        self._n_updates = 0
        self._rejects = 0

    @property
    def has_track(self) -> bool:
        return self._x is not None

    @property
    def n_updates(self) -> int:
        return self._n_updates

    @property
    def track_age_s(self) -> float:
        """Measurement-time span of the CURRENT track, nan without one.

        The discriminator for why a dodge commits late. Pair it with n_updates
        and with how long the ball has been visible:
          - age much shorter than the ball's visibility -> the track was
            RESTARTED mid-flight (see the reseed counters)
          - age comparable to visibility but n_updates small -> detection is
            SPARSE, i.e. the detector is not reporting the ball every frame
        Those have different fixes, and lifetime reseed totals cannot tell them
        apart because the false-positive stream reseeds constantly between
        throws.
        """
        if self._x is None:
            return float("nan")
        return self._t - self._t_start

    @property
    def n_reseeds_reject(self) -> int:
        """Tracks abandoned after max_consecutive_rejects disagreements."""
        return self._n_reseeds_reject

    @property
    def n_reseeds_timeout(self) -> int:
        """Tracks abandoned after track_timeout_s of silence."""
        return self._n_reseeds_timeout

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
            self._n_reseeds_timeout += 1
            self.reset()

        if self._x is None:
            self._x = np.concatenate([z, np.zeros(3)])
            self._P = np.diag([self._R[0, 0]] * 3 + [self._init_vel_var] * 3)
            self._t = t
            self._t_start = t
            self._n_updates = 1
            self._rejects = 0
            return True

        dt = t - self._t
        if dt <= 0.0:
            return False  # out-of-order / duplicate stamp

        x_pred, P_pred = self._predict(dt)
        nu, S, d2 = self._innovation(x_pred, P_pred, z)
        if d2 > self._gate:
            self._rejects += 1
            if self._rejects >= self._max_rejects:
                # Persistent disagreement -> different object; reseed from it.
                self._n_reseeds_reject += 1
                self.reset()
                return self.process(t, z)
            # Keep the prediction, drop the outlier.
            self._x, self._P, self._t = x_pred, P_pred, t
            return False

        self._apply(x_pred, P_pred, S, nu, t)
        return True

    # ── association primitives (used by MultiHypothesisTracker) ──────────

    def association_cost(self, t: float, z, *, cov_cap: float) -> float:
        """How well this track explains z, as a negative log-likelihood.
        +inf means "not mine" — out of order, or outside the gate.

        Two deliberate choices, both of which exist because the single-target
        filter's failure mode is a track claiming a measurement that is not
        its own:

        * The GATE is evaluated against a covariance whose scale is capped at
          cov_cap. An uninformed track (init_vel_std 15 m/s) honestly believes
          its object could be metres away after a few frames, and a gate that
          wide accepts anything in the room — which is exactly how a false
          positive swallowed the ball. cov_cap makes the gate a physical
          association window instead of a purely statistical one.
        * The COST includes log|S|, so it ranks by likelihood rather than by
          Mahalanobis distance. Raw distance rewards a bloated covariance: a
          track that has coasted without evidence would out-bid the mature
          track that actually owns the measurement.
        """
        if self._x is None:
            return float("inf")
        dt = t - self._t
        if dt <= 0.0:
            return float("inf")
        x_pred, P_pred = self._predict(dt)
        nu, S, d2 = self._innovation(x_pred, P_pred, z)

        scale = float(np.max(np.diag(S)))
        if scale > cov_cap:
            # Shrink S isotropically rather than clipping the diagonal: that
            # bounds the gate volume while preserving the innovation's shape,
            # so an elongated uncertainty stays elongated.
            d2_gate = d2 * scale / cov_cap
        else:
            d2_gate = d2
        if d2_gate > self._gate:
            return float("inf")

        _, logdet = np.linalg.slogdet(S)
        return 0.5 * (d2 + float(logdet))

    def associate(self, t: float, z) -> bool:
        """Fold in a measurement already chosen for this track by the
        associator. No gate: association_cost() applied it."""
        z = np.asarray(z, dtype=np.float64).reshape(3)
        if self._x is None:
            return self.process(t, z)
        dt = t - self._t
        if dt <= 0.0:
            return False
        x_pred, P_pred = self._predict(dt)
        nu, S, _ = self._innovation(x_pred, P_pred, z)
        self._apply(x_pred, P_pred, S, nu, t)
        return True

    def is_stale(self, t: float) -> bool:
        return self._x is not None and (t - self._t) > self._timeout

    # ── filter internals ─────────────────────────────────────────────────

    def _innovation(self, x_pred, P_pred, z):
        nu = z - self._H @ x_pred
        S = self._H @ P_pred @ self._H.T + self._R
        d2 = float(nu @ np.linalg.solve(S, nu))
        return nu, S, d2

    def _apply(self, x_pred, P_pred, S, nu, t: float) -> None:
        K = P_pred @ self._H.T @ np.linalg.inv(S)
        self._x = x_pred + K @ nu
        self._P = (np.eye(6) - K @ self._H) @ P_pred
        self._t = t
        self._n_updates += 1
        self._rejects = 0

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


class MultiHypothesisTracker:
    """A small set of candidate ProjectileTracker hypotheses.

    Why this exists (measured 2026-07-27, docs/JOURNAL.md): a single filter fed
    every centroid is forced to represent whatever arrived last. The detector
    emits ~1.4 false positives per second, so a stale FP track is usually alive
    when a ball arrives — and after ONE rejection its covariance has inflated
    enough to swallow the ball as a valid update, fitting a ~38 m/s velocity
    along the line from the false positive to the ball. Three more true
    detections are then spent contradicting that fiction before the track
    reseeds. Cost: 4 frames, 0.267 s, against a measured tca of 0.18-0.26 s at
    the moment a dodge commits. That is the whole of the missing warning; see
    test_the_incumbent_costs_four_frames_of_warning.

    Here every centroid goes to the hypothesis that explains it best, and
    anything unexplained starts its own. The ball's opening detections
    accumulate on their own track instead of correcting someone else's.

    The trigger must act on a CONFIRMED track (confirmed()); an unconfirmed one
    still has a near-zero velocity estimate, which reads an inbound ball as
    receding.
    """

    def __init__(
        self,
        *,
        max_tracks: int = 6,
        max_assoc_sigma_m: float = 0.75,
        track_timeout_s: float = 0.5,
        **filter_kwargs,
    ) -> None:
        self._max_tracks = int(max_tracks)
        # Widest position uncertainty any hypothesis may claim when bidding for
        # a measurement — see ProjectileTracker.association_cost. Scaled to the
        # ball's per-frame travel: 0.75 m covers 11 m/s at the 15 Hz detector
        # rate with the chi2 gate's margin on top, and is well under the
        # distance at which a false positive and a ball are distinct objects.
        self._cov_cap = float(max_assoc_sigma_m) ** 2
        self._timeout = float(track_timeout_s)
        self._kw = dict(filter_kwargs, track_timeout_s=track_timeout_s)
        self._tracks: list[ProjectileTracker] = []
        self._n_spawned = 0
        self._n_pruned = 0

    # ── introspection ────────────────────────────────────────────────────

    @property
    def tracks(self) -> list[ProjectileTracker]:
        return list(self._tracks)

    @property
    def n_tracks(self) -> int:
        return len(self._tracks)

    @property
    def n_spawned(self) -> int:
        """Lifetime hypotheses started — a direct read on the detector's
        false-positive rate."""
        return self._n_spawned

    @property
    def n_pruned(self) -> int:
        """Lifetime hypotheses dropped for going quiet past track_timeout_s."""
        return self._n_pruned

    def confirmed(self, *, min_updates: int) -> list[ProjectileTracker]:
        return [tr for tr in self._tracks if tr.n_updates >= int(min_updates)]

    def reset(self) -> None:
        self._tracks.clear()

    # ── association ──────────────────────────────────────────────────────

    def process(self, t: float, z) -> bool:
        """Route one measurement. Always True: a centroid that no hypothesis
        claims is not an outlier to be argued with, it is a new object."""
        z = np.asarray(z, dtype=np.float64).reshape(3)
        self._prune(t)

        best, best_cost = None, float("inf")
        for tr in self._tracks:
            cost = tr.association_cost(t, z, cov_cap=self._cov_cap)
            if cost < best_cost:
                best, best_cost = tr, cost
        if best is not None:
            return best.associate(t, z)

        self._spawn(t, z)
        return True

    def _prune(self, t: float) -> None:
        keep = [tr for tr in self._tracks if not tr.is_stale(t)]
        self._n_pruned += len(self._tracks) - len(keep)
        self._tracks = keep

    def _spawn(self, t: float, z) -> None:
        tr = ProjectileTracker(**self._kw)
        tr.process(t, z)
        self._tracks.append(tr)
        self._n_spawned += 1
        while len(self._tracks) > self._max_tracks:
            # Evict least-evidenced first, oldest breaking the tie. A burst of
            # false positives must never push out the mature track that is
            # about to fire a dodge.
            weakest = min(self._tracks,
                          key=lambda x: (x.n_updates, x.last_update_t))
            self._tracks.remove(weakest)


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
