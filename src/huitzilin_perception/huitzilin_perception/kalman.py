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
        # threat" until the velocity re-converges. Measured: the
        # published predicted miss does track the ball's current range on inbound
        # throws, which is that signature — these counters say whether mid-flight
        # reseeds are the cause. The two paths are separated because their fixes
        # differ: rejects point at the detector's false-positive stream stealing
        # the track, timeouts at gaps in its output.
        self._n_reseeds_reject = 0
        self._n_reseeds_timeout = 0
        self.reset()

    # track lifecycle

    def reset(self) -> None:
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None
        self._t = float("nan")
        self._t_start = float("nan")
        self._n_updates = 0
        self._rejects = 0
        # Per-track, unlike the reseed totals: the question these answer is
        # "which sensor built THIS track", which a new track re-asks.
        self._last_source: Optional[str] = None
        self._source_counts: dict[str, int] = {}

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

    @property
    def last_source(self) -> Optional[str]:
        """Tag of the most recent measurement folded in, or None if untagged."""
        return self._last_source

    @property
    def source_counts(self) -> dict[str, int]:
        """Updates on this track per source tag.

        The diagnostic a fused pipeline needs: a track that fired on three
        long-range triangulations is a different claim from one that fired on
        three near-field depth centroids, and the dodge outcome cannot be
        attributed without knowing which.
        """
        return dict(self._source_counts)

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        """(position (3,), velocity (3,)) of the current track estimate."""
        if self._x is None:
            raise RuntimeError("no active track")
        return self._x[:3].copy(), self._x[3:].copy()

    # filtering

    def process(self, t: float, z, R=None, source: Optional[str] = None) -> bool:
        """
        Feed one measurement. Returns True if it initialised or updated the
        track, False if rejected (gated outlier / out-of-order stamp).
        A stale track (> track_timeout_s since last stamp) resets first, so
        a second throw starts a fresh track instead of dragging the old one.

        R, when given, is that measurement's own 3x3 covariance in odom ENU
        and replaces the configured meas_std_m for this update only. Passing
        None keeps the isotropic default, so every existing call site behaves
        exactly as before.
        """
        z = np.asarray(z, dtype=np.float64).reshape(3)
        R_eff = self._as_R(R)

        if self._x is not None and (t - self._t) > self._timeout:
            self._n_reseeds_timeout += 1
            self.reset()

        if self._x is None:
            self._x = np.concatenate([z, np.zeros(3)])
            # Seed position variance from the measurement that actually seeded
            # the track, not from the configured scalar: a triangulated
            # far-field detection is metres uncertain in depth and centimetres
            # across, and starting it round would understate one and overstate
            # the other. Identical to the old diagonal for an isotropic R.
            self._P = np.diag(np.concatenate(
                [np.diag(R_eff), np.full(3, self._init_vel_var)]))
            self._t = t
            self._t_start = t
            self._n_updates = 1
            self._rejects = 0
            self._note_source(source)
            return True

        dt = t - self._t
        if dt <= 0.0:
            return False  # out-of-order / duplicate stamp

        x_pred, P_pred = self._predict(dt)
        nu, S, d2 = self._innovation(x_pred, P_pred, z, R_eff)
        if d2 > self._gate:
            self._rejects += 1
            if self._rejects >= self._max_rejects:
                # Persistent disagreement -> different object; reseed from it.
                self._n_reseeds_reject += 1
                self.reset()
                return self.process(t, z, R, source)
            # Keep the prediction, drop the outlier.
            self._x, self._P, self._t = x_pred, P_pred, t
            return False

        self._apply(x_pred, P_pred, S, nu, t, source)
        return True

    # association primitives (used by MultiHypothesisTracker)

    def association_cost(self, t: float, z, *, cov_cap: float, R=None) -> float:
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
        nu, S, d2 = self._innovation(x_pred, P_pred, z, self._as_R(R))

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

    def associate(self, t: float, z, R=None, source: Optional[str] = None) -> bool:
        """Fold in a measurement already chosen for this track by the
        associator. No gate: association_cost() applied it."""
        z = np.asarray(z, dtype=np.float64).reshape(3)
        if self._x is None:
            return self.process(t, z, R, source)
        dt = t - self._t
        if dt <= 0.0:
            return False
        x_pred, P_pred = self._predict(dt)
        nu, S, _ = self._innovation(x_pred, P_pred, z, self._as_R(R))
        self._apply(x_pred, P_pred, S, nu, t, source)
        return True

    def is_stale(self, t: float) -> bool:
        return self._x is not None and (t - self._t) > self._timeout

    # filter internals

    def _as_R(self, R) -> np.ndarray:
        """This measurement's covariance, or the configured default.

        Validated rather than trusted: a (3,) variance vector or a wrongly
        shaped array would broadcast into S silently and produce a filter that
        looks like it is working.
        """
        if R is None:
            return self._R
        R = np.asarray(R, dtype=np.float64)
        if R.shape != (3, 3):
            raise ValueError("measurement covariance must be 3x3, got %r"
                             % (R.shape,))
        return R

    def _note_source(self, source: Optional[str]) -> None:
        self._last_source = source
        if source is not None:
            self._source_counts[source] = self._source_counts.get(source, 0) + 1

    def _innovation(self, x_pred, P_pred, z, R=None):
        nu = z - self._H @ x_pred
        S = self._H @ P_pred @ self._H.T + (self._R if R is None else R)
        d2 = float(nu @ np.linalg.solve(S, nu))
        return nu, S, d2

    def _apply(self, x_pred, P_pred, S, nu, t: float,
               source: Optional[str] = None) -> None:
        K = P_pred @ self._H.T @ np.linalg.inv(S)
        self._x = x_pred + K @ nu
        self._P = (np.eye(6) - K @ self._H) @ P_pred
        self._t = t
        self._n_updates += 1
        self._rejects = 0
        self._note_source(source)

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

    Why this exists (measured): a single filter fed
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

    # introspection

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

    # association

    def process(self, t: float, z, R=None, source: Optional[str] = None) -> bool:
        """Route one measurement. Always True: a centroid that no hypothesis
        claims is not an outlier to be argued with, it is a new object.

        R and source are forwarded untouched to whichever hypothesis wins, so
        a fused stream can hand each measurement its own covariance without
        the associator needing to know where it came from.
        """
        z = np.asarray(z, dtype=np.float64).reshape(3)
        self._prune(t)

        best, best_cost = None, float("inf")
        for tr in self._tracks:
            cost = tr.association_cost(t, z, cov_cap=self._cov_cap, R=R)
            if cost < best_cost:
                best, best_cost = tr, cost
        if best is not None:
            return best.associate(t, z, R, source)

        self._spawn(t, z, R, source)
        return True

    def _prune(self, t: float) -> None:
        keep = [tr for tr in self._tracks if not tr.is_stale(t)]
        self._n_pruned += len(self._tracks) - len(keep)
        self._tracks = keep

    def _spawn(self, t: float, z, R=None, source: Optional[str] = None) -> None:
        tr = ProjectileTracker(**self._kw)
        tr.process(t, z, R, source)
        self._tracks.append(tr)
        self._n_spawned += 1
        while len(self._tracks) > self._max_tracks:
            # Evict least-evidenced first, oldest breaking the tie. A burst of
            # false positives must never push out the mature track that is
            # about to fire a dodge.
            weakest = min(self._tracks,
                          key=lambda x: (x.n_updates, x.last_update_t))
            self._tracks.remove(weakest)


# Dodge planning


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


def effective_min_updates(
    now_s: float,
    last_cue_s: Optional[float],
    *,
    cue_timeout_s: float,
    patrol_updates: int,
    alert_updates: int,
) -> int:
    """How many track updates a dodge needs right now.

    Confirmation is the single most expensive term in the reaction budget:
    three updates at the measured 14.5 Hz cost 0.21 s, which at 14 m/s is
    most of the ball's time of flight inside detection range. Dropping to two
    unconditionally was MEASURED as a bad trade -- it bought 0.044 s of tca
    and lost more in dodge success than it gained (13/18 -> 11/16).

    This is a different proposition, and the difference is the reason the
    earlier null does not settle it: the relaxed threshold applies ONLY while
    an external cue says a fast object is probably inbound, and reverts by
    itself when the cue goes stale. The null measured a permanently weaker
    filter against a false-positive stream running the whole time; this
    weakens it for a bounded window that something else has already vouched
    for. Whether that trade is good is an open measurement, not a settled one.

    No cue ever published => patrol_updates forever, i.e. today's behaviour.
    """
    if last_cue_s is None or cue_timeout_s <= 0.0:
        return int(patrol_updates)
    if not (0.0 <= now_s - last_cue_s <= cue_timeout_s):
        # Also catches a cue from the future (a sim-time rewind), which would
        # otherwise hold the machine in ALERT indefinitely.
        return int(patrol_updates)
    return int(alert_updates)


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
    allow_upward: bool = False,
    threat_descending: bool = True,
    ceiling_m: Optional[float] = None,
    climb_len_m: Optional[float] = None,
    away_hint=None,
) -> np.ndarray:
    """
    Re-aim a dodge that would fly the drone into the ground. Returns a unit
    ENU vector.

    dodge_direction() only sees the approach geometry, and it is *usually*
    right to escape downward: a gravity-compensated throw arrives descending,
    so the fastest-opening perpendicular points down. That is also how the
    drone kills itself. Measured live hovering at 2 m:

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

    By default escape is never flipped upward. The ball is usually descending
    through that space, and climbing into it trades a ground strike for a hit.

    allow_upward opts into the exception, and it is an exception worth having
    because vertical is the axis a quadrotor is FASTEST on. A lateral escape
    has to tilt before it can translate -- that is the ~2 m/s^2 deviation rise
    measured over 14 dodges, against a tilt ceiling that would permit 5.66.
    Collective thrust needs no tilt at all and a 2:1 thrust-to-weight airframe
    has most of g in hand. That matters only at the speeds where the budget is
    tightest, which is exactly where the envelope currently closes.

    It is gated on two conditions, both of which must hold:
      * threat_descending is False -- the FMEA rule survives as a predicate
        rather than as a blanket refusal. A ball coming down on top of the
        drone is still answered horizontally.
      * ceiling_m leaves room to climb -- SAFETY_CASE.md sets a 5 m ceiling
        and FENCE_ACTION 1 turns a breach into an RTL, so a dodge that busts
        the fence has ended the flight even if it cleared the ball.

    Note that passing ceiling_m ALSO clamps an escape that already pointed
    upward on its own. That path was previously unclamped in every direction,
    which was survivable only because the geometric escape never reached the
    fence from 2 m AGL; it is not survivable with more vertical authority.

    away_hint is a horizontal ENU bearing known to point AWAY from the threat,
    supplied by plan_dodge from the approach geometry. It is used only where a
    re-aim has no horizontal bearing of its own to keep -- a straight-down or
    straight-up escape. Those branches used to invent a hardcoded [1, 0, 0],
    which is a fixed compass direction with no relation to where the ball is:
    roughly half such dodges would fly TOWARD it. This function is otherwise
    threat-blind by construction -- it receives a direction, not a geometry --
    so the hint is the only way it can pick a defensible fallback.

    Passing no hint keeps the old deterministic [1, 0, 0], so nothing that
    called this before changes behaviour.
    """
    d = np.asarray(direction, dtype=np.float64)
    n = np.linalg.norm(d)
    fallback = _unit_horizontal(away_hint)
    if n < 1e-9:
        return fallback
    d = d / n

    climb_len = descent_len_m if climb_len_m is None else float(climb_len_m)
    max_up = _vertical_budget(
        headroom_m=(None if ceiling_m is None
                    else max(float(ceiling_m) - float(altitude_m), 0.0)),
        travel_len_m=climb_len)

    if d[2] >= 0.0:
        # Already climbing. Only a stated ceiling can constrain it.
        if max_up is None or d[2] <= max_up:
            return d
        return _reaim(d, max_up, fallback)

    headroom_m = max(float(altitude_m) - float(floor_m), 0.0)
    max_down = _vertical_budget(headroom_m=headroom_m,
                                travel_len_m=descent_len_m) or 0.0
    if -d[2] <= max_down:
        return d

    if allow_upward and not threat_descending:
        # The ball is not coming down on us and the floor will not let us go
        # under it, so go over it instead of settling for horizontal-only.
        up = 1.0 if max_up is None else max_up
        if up > 0.0:
            return _reaim(d, up, fallback)

    horiz = np.array([d[0], d[1], 0.0])
    h_norm = np.linalg.norm(horiz)
    if h_norm < 1e-6:
        # Straight down with no headroom: any horizontal beats descending, but
        # not ANY horizontal will do -- see away_hint. Without a hint this is
        # still the old deterministic pick, so battery runs stay repeatable.
        return fallback
    h_keep = math.sqrt(max(1.0 - max_down ** 2, 0.0))
    return np.array([horiz[0] / h_norm * h_keep,
                     horiz[1] / h_norm * h_keep,
                     -max_down])


def _vertical_budget(*, headroom_m: Optional[float],
                     travel_len_m: float) -> Optional[float]:
    """Fraction of a unit escape that may point vertically, or None for
    'unconstrained' when no limit was stated."""
    if headroom_m is None:
        return None
    if travel_len_m <= 1e-9:
        return 0.0
    return min(1.0, headroom_m / travel_len_m)


def _unit_horizontal(hint) -> np.ndarray:
    """A unit horizontal ENU bearing from a hint, or the legacy [1, 0, 0].

    Kept deliberately forgiving: callers pass whatever the geometry gave them,
    including a vertical or zero vector when the geometry was degenerate, and
    a fallback that itself needs a fallback helps nobody.
    """
    if hint is None:
        return np.array([1.0, 0.0, 0.0])
    h = np.asarray(hint, dtype=np.float64)
    h = np.array([h[0], h[1], 0.0])
    n = np.linalg.norm(h)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    return h / n


def horizontal_away_bearing(miss_vec, approach_vel) -> np.ndarray:
    """Unit horizontal ENU bearing that leaves the projectile's pass side.

    Used as clamp_dodge_to_clearance's away_hint. Preference order:

      1. the horizontal part of -miss_vec, i.e. straight away from where the
         ball will pass. Correct whenever the pass is offset horizontally;
      2. failing that (a dead-overhead or dead-under pass, where the horizontal
         offset is nil), the horizontal perpendicular to the approach axis --
         the same construction dodge_direction falls back to, and the fastest
         way off a line you cannot get off sideways.

    Both are derived from the threat. That is the entire point: the branch this
    feeds used to answer "which way is away?" with a hardcoded compass bearing.
    """
    m = np.asarray(miss_vec, dtype=np.float64)
    away = np.array([-m[0], -m[1], 0.0])
    if np.linalg.norm(away) >= 1e-6:
        return away / np.linalg.norm(away)

    a = np.asarray(approach_vel, dtype=np.float64)
    a_norm = np.linalg.norm(a)
    a_hat = a / a_norm if a_norm > 1e-9 else np.array([1.0, 0.0, 0.0])
    lateral = np.cross(a_hat, np.array([0.0, 0.0, 1.0]))
    lat_norm = np.linalg.norm(lateral)
    if lat_norm < 1e-6:            # vertical approach: no lateral is preferred
        return np.array([1.0, 0.0, 0.0])
    return lateral / lat_norm


def _reaim(d: np.ndarray, vertical: float, fallback=None) -> np.ndarray:
    """Rebuild a unit vector with a given vertical component, keeping the
    horizontal bearing and re-spending the freed budget along it.

    A purely vertical input has no bearing to keep, so one is supplied -- the
    caller's threat-derived fallback where there is one, else the legacy
    deterministic [1, 0, 0], so battery runs stay repeatable. Returning the
    input unchanged instead would silently ignore the cap, which is the whole
    reason this function is called.
    """
    horiz = np.array([d[0], d[1], 0.0])
    h_norm = np.linalg.norm(horiz)
    if h_norm < 1e-6:
        horiz, h_norm = _unit_horizontal(fallback), 1.0
    h_keep = math.sqrt(max(1.0 - vertical ** 2, 0.0))
    return np.array([horiz[0] / h_norm * h_keep,
                     horiz[1] / h_norm * h_keep,
                     vertical])


def dodge_velocity_command(
    direction,
    v_drone,
    *,
    dodge_speed_mps: float,
    max_speed_mps: Optional[float] = None,
) -> np.ndarray:
    """
    ENU velocity to command so the drone leaves its OWN predicted track at
    dodge_speed_mps, whichever way it was already flying.

    A thrown ball is aimed where the drone is predicted to be, and the
    prediction is the drone's current velocity extrapolated (that is exactly
    what the battery's lead does, and what a thrower does). Escape is therefore
    deviation from that extrapolation -- not ground speed, and not distance
    from where the drone was when the dodge fired.

    Commanding an absolute dodge_speed_mps * direction gets this wrong,
    measured over 16 dodges (probe deleted in 28f0f40; recover it with
    `git show 28f0f40^:scripts/hz_cmd_path_probe.py`): it
    REPLACES the ~4.2 m/s patrol cruise with a 1.5 m/s command, so the drone
    sheds 2.5-3.9 m/s within a second and the deviation is dominated by
    -v_cruise. Its component along the escape direction is

        dodge_speed_mps - v_drone . direction

    which collapses toward zero as the escape lines up with the cruise, and
    reverses once the cruise outruns the command. Escape at 1.0 s correlated
    r = -0.911 with that alignment, and battery min_dist r = -0.67. The
    "dodges that never moved the airframe" were this: the airframe moved, it
    just stayed on the line the ball was thrown at.

    Adding the escape to the current velocity instead makes the deviation
    exactly dodge_speed_mps * direction * t for any cruise.

    max_speed_mps caps the result. The cap is spent on the CRUISE term, never
    on the escape term -- a capped dodge should stop chasing its waypoint, not
    stop dodging.
    """
    d = np.asarray(direction, dtype=np.float64)
    n = np.linalg.norm(d)
    if n < 1e-9:
        return np.zeros(3)
    d = d / n
    escape = float(dodge_speed_mps) * d
    v = np.asarray(v_drone, dtype=np.float64)

    if max_speed_mps is None:
        return v + escape

    cap = float(max_speed_mps)
    if np.linalg.norm(v + escape) <= cap:
        return v + escape
    # Keep the escape whole and scale the cruise until the sum fits. A cap
    # tight enough to bite into the escape itself leaves nothing to preserve.
    esc_norm = np.linalg.norm(escape)
    if esc_norm >= cap:
        return escape * (cap / esc_norm)
    # The cruise component ALONG the escape axis is the term that cancels the
    # escape, so drop it entirely and keep what fits perpendicular.
    v_perp = v - (v @ d) * d
    perp_budget = math.sqrt(max(cap ** 2 - esc_norm ** 2, 0.0))
    perp_norm = np.linalg.norm(v_perp)
    if perp_norm > perp_budget:
        v_perp = v_perp * (perp_budget / perp_norm)
    return v_perp + escape


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
    allow_upward: bool = False,
    ceiling_m: Optional[float] = None,
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
            direction, altitude_m, floor_m=floor_m,
            descent_len_m=descent_len_m,
            allow_upward=allow_upward,
            # The ball's own vertical motion at closest approach. Climbing is
            # refused when it is coming down onto us -- that is the FMEA rule,
            # evaluated rather than assumed.
            threat_descending=bool(approach_vel[2] < 0.0),
            ceiling_m=ceiling_m,
            # The clamp only ever sees a direction, so it cannot work out for
            # itself which way is away from the ball. Where it has to invent a
            # horizontal bearing, this is the one it should invent.
            away_hint=horizontal_away_bearing(miss_vec, approach_vel))
    return DodgePlan(tca, miss, miss_vec, direction)
