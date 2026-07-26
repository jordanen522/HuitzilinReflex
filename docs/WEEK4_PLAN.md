# Week 4 — Kalman Filter + Dodge Trigger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the sense→dodge loop in SITL: `/threat/centroid` → odom-frame ballistic Kalman track → intercept prediction → dodge trigger → velocity-spike through `mav_bridge` → mocked alarm — plus a ground-truth-scored dodge battery and parameter sweep.

**Architecture:** Pure-numpy math modules (`ballistics.py`, `kalman.py` — no ROS imports, unit-tested anywhere, mirroring `cloud_geometry.py`) wrapped by a thin `evasion_node.py`. The dodge preempts patrol by pausing `/huitzilin/start_patrol` (patrol's position setpoints ride a separate MAVLink connection and would fight a velocity spike) and streams `/cmd/evade`, which `mav_bridge_node` prioritizes over `/huitzilin/cmd_vel` while fresh. A battery harness spawns gravity-compensated throws (today's "direct hits" actually pass 0.7–2.8 m below the drone — the model is drag-free with gravity ON) and scores hit/miss from the bridged Gazebo dynamic-pose stream.

**Tech Stack:** ROS 2 Jazzy, numpy, pytest; Gazebo Harmonic + ArduPilot SITL on the Dell only.

## Global Constraints

- Work on branch `week4/kalman-dodge` (push of `main` is blocked; Week 3 used the same pattern).
- **Never stage `CLAUDE.md` or `docs/hardware_bringup.md`** — they carry uncommitted work from the hardware-bringup session. Always `git add` explicit paths, never `-A`/`.`.
- Commit messages: `W4-<nn>: <description>` (Week 3 convention). **No Co-Authored-By trailer** (attribution disabled in user settings).
- Tests run on this Windows box: `cd src/huitzilin_perception && python -m pytest test/ -v` (Python 3.14, numpy 2.5.1, scipy 1.18 verified present). Do **not** run `test_frames.py` here (imports pymavlink, not installed on Windows).
- Pure-math modules (`ballistics.py`, `kalman.py`) must have **zero ROS imports**.
- Every tunable is a ROS param backed by a yaml key — no magic numbers in node files (project convention from `detector_node.py`).
- All timing in **sim time**, never wall-clock (CLAUDE.md sharp edge). Frames: ROS topics ENU/FLU only; NED↔ENU conversion exists **only** in `mav_bridge` (`docs/frames.md`).
- ROS-node files can't be imported on Windows (no rclpy) — syntax-check them with `python -m py_compile <file>`; runtime verification happens on the Dell per `docs/week4_dodge_runbook.md` (Task 9).
- Sweepable-at-runtime evasion params (set via `ros2 param set`): `dodge_speed_mps`, `threat_radius_m`, `trigger_horizon_s`, `dodge_duration_s`, `min_track_updates`.
- No hard success-rate gate in the battery (user decision 2026-07-16): report measured rates; exit non-zero **only** on harness errors.

---

### Task 1: Branch + `ballistics.py` (shared drag-free projectile math)

**Files:**
- Create: `src/huitzilin_perception/huitzilin_perception/ballistics.py`
- Test: `src/huitzilin_perception/test/test_ballistics.py`

**Interfaces:**
- Consumes: nothing (pure numpy).
- Produces (used by Tasks 4, 7, and tests):
  - `G_MPS2: float` (9.80665)
  - `SpawnPlan(NamedTuple)` with `.position: np.ndarray (3,)`, `.velocity: np.ndarray (3,)`
  - `ballistic_positions(p0, v0, ts, g=G_MPS2) -> np.ndarray (N,3)`
  - `compute_spawn(drone_enu, drone_yaw, *, speed_mps, approach_angle_deg=0.0, miss_distance_m=0.0, offset_forward_m=6.0, offset_vertical_m=0.0, compensate_gravity=False, g=G_MPS2) -> SpawnPlan`

- [ ] **Step 1: Create the branch**

```bash
git checkout -b week4/kalman-dodge
```

- [ ] **Step 2: Write the failing tests**

Create `src/huitzilin_perception/test/test_ballistics.py`:

```python
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
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd src/huitzilin_perception && python -m pytest test/test_ballistics.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'huitzilin_perception.ballistics'`

- [ ] **Step 4: Write the implementation**

Create `src/huitzilin_perception/huitzilin_perception/ballistics.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd src/huitzilin_perception && python -m pytest test/test_ballistics.py -v`
Expected: 7 passed

- [ ] **Step 6: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/ballistics.py src/huitzilin_perception/test/test_ballistics.py
git commit -m "W4-01: ballistics.py — shared drag-free projectile math + gravity-compensated spawn"
```

---

### Task 2: `kalman.py` — ProjectileTracker (6-state ballistic KF)

**Files:**
- Create: `src/huitzilin_perception/huitzilin_perception/kalman.py`
- Test: `src/huitzilin_perception/test/test_kalman.py`

**Interfaces:**
- Consumes: `ballistics.ballistic_positions`, `ballistics.G_MPS2` (tests only).
- Produces (used by Tasks 3 and 6):
  - `GRAVITY_ENU: np.ndarray (3,)` = `[0, 0, -9.80665]`
  - `CHI2_GATE_3DOF_99: float` = `11.345`
  - `DodgePlan(NamedTuple)` with `.tca_s: float`, `.miss_m: float`, `.miss_vec: np.ndarray (3,)`, `.direction: np.ndarray (3,)`
  - `class ProjectileTracker` with:
    - `__init__(*, process_accel_std=3.0, meas_std_m=0.15, init_vel_std=15.0, gate_chi2=CHI2_GATE_3DOF_99, track_timeout_s=0.5, max_consecutive_rejects=3)`
    - `process(t: float, z) -> bool` — feed one measurement (sim-time seconds, (3,) position)
    - `reset() -> None`
    - `has_track: bool` (property), `n_updates: int` (property), `last_update_t: float` (property)
    - `state() -> tuple[np.ndarray, np.ndarray]` — (position (3,), velocity (3,)); raises `RuntimeError` if no track

- [ ] **Step 1: Write the failing tests**

Create `src/huitzilin_perception/test/test_kalman.py` (Task 3 appends more tests to this file):

```python
"""Unit tests for kalman.py — pure math, no ROS."""
import numpy as np
import pytest

from huitzilin_perception.ballistics import G_MPS2, ballistic_positions
from huitzilin_perception.kalman import (
    CHI2_GATE_3DOF_99,
    GRAVITY_ENU,
    ProjectileTracker,
)

RATE_HZ = 15.0  # detector centroid rate in sim


def _feed_ballistic(tracker, p0, v0, *, n, noise_std=0.0, seed=42, t0=0.0):
    """Feed n samples of an exact ballistic track (+ optional noise)."""
    rng = np.random.default_rng(seed)
    ts = t0 + np.arange(n) / RATE_HZ
    pts = ballistic_positions(p0, v0, ts - t0)
    if noise_std > 0.0:
        pts = pts + rng.normal(0.0, noise_std, pts.shape)
    for t, z in zip(ts, pts):
        tracker.process(float(t), z)
    return float(ts[-1])


def test_first_measurement_initialises_track():
    tr = ProjectileTracker()
    assert not tr.has_track
    assert tr.process(0.0, [6.0, 0.0, 2.0])
    assert tr.has_track
    assert tr.n_updates == 1
    pos, vel = tr.state()
    np.testing.assert_allclose(pos, [6.0, 0.0, 2.0])
    np.testing.assert_allclose(vel, [0.0, 0.0, 0.0])


def test_state_raises_without_track():
    with pytest.raises(RuntimeError):
        ProjectileTracker().state()


def test_velocity_converges_on_noisy_ballistic_track():
    tr = ProjectileTracker(meas_std_m=0.05)
    p0, v0 = [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9]
    t_last = _feed_ballistic(tr, p0, v0, n=10, noise_std=0.05)
    pos, vel = tr.state()
    truth_pos = ballistic_positions(p0, v0, t_last)[0]
    truth_vel = np.asarray(v0) + t_last * GRAVITY_ENU
    assert np.linalg.norm(pos - truth_pos) < 0.15
    assert np.linalg.norm(vel - truth_vel) < 1.0


def test_gravity_is_modelled_not_estimated():
    # Free-fall from rest, exact measurements: velocity must track -g*t
    tr = ProjectileTracker()
    t_last = _feed_ballistic(tr, [0.0, 0.0, 30.0], [0.0, 0.0, 0.0], n=8)
    _, vel = tr.state()
    assert vel[2] == pytest.approx(-G_MPS2 * t_last, abs=0.5)


def test_outlier_is_gated_but_track_survives():
    tr = ProjectileTracker(meas_std_m=0.05)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=6)
    n_before = tr.n_updates
    accepted = tr.process(t_last + 1.0 / RATE_HZ, [50.0, 50.0, 50.0])
    assert not accepted
    assert tr.has_track
    assert tr.n_updates == n_before


def test_persistent_disagreement_reseeds_track():
    tr = ProjectileTracker(meas_std_m=0.05, max_consecutive_rejects=3)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=6)
    for k in range(1, 4):
        tr.process(t_last + k / RATE_HZ, [80.0, 80.0, 10.0])
    assert tr.has_track
    assert tr.n_updates == 1                      # reseeded from the new object
    pos, _ = tr.state()
    np.testing.assert_allclose(pos, [80.0, 80.0, 10.0])


def test_track_times_out_and_reseeds():
    tr = ProjectileTracker(track_timeout_s=0.5)
    t_last = _feed_ballistic(tr, [6.0, 0.0, 2.0], [-8.0, 0.0, 2.9], n=5)
    assert tr.process(t_last + 1.0, [1.0, 2.0, 3.0])   # 1 s silence > timeout
    assert tr.n_updates == 1
    pos, _ = tr.state()
    np.testing.assert_allclose(pos, [1.0, 2.0, 3.0])


def test_out_of_order_stamp_rejected():
    tr = ProjectileTracker()
    tr.process(1.0, [6.0, 0.0, 2.0])
    assert not tr.process(0.9, [5.9, 0.0, 2.0])
    assert tr.n_updates == 1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/huitzilin_perception && python -m pytest test/test_kalman.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'huitzilin_perception.kalman'`

- [ ] **Step 3: Write the implementation**

Create `src/huitzilin_perception/huitzilin_perception/kalman.py`:

```python
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
```

(The `DodgePlan` NamedTuple is defined now but only exercised in Task 3.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/huitzilin_perception && python -m pytest test/test_kalman.py -v`
Expected: 8 passed

- [ ] **Step 5: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/kalman.py src/huitzilin_perception/test/test_kalman.py
git commit -m "W4-02: kalman.py — 6-state ballistic KF with gravity input, gating, timeout reseed"
```

---

### Task 3: `kalman.py` — dodge planning (closest approach, direction, trigger policy)

**Files:**
- Modify: `src/huitzilin_perception/huitzilin_perception/kalman.py` (append functions)
- Test: `src/huitzilin_perception/test/test_kalman.py` (append tests)

**Interfaces:**
- Consumes: `GRAVITY_ENU`, `DodgePlan` from Task 2.
- Produces (used by Task 6):
  - `predict_closest_approach(p_rel, v_rel, *, horizon_s=3.0, step_s=0.01, gravity=GRAVITY_ENU) -> tuple[float, float, np.ndarray]` — `(tca_s, miss_m, miss_vec)`
  - `dodge_direction(miss_vec, approach_vel, *, min_offset_m=0.05) -> np.ndarray` — unit (3,)
  - `should_dodge(n_updates, miss_m, tca_s, *, min_updates, threat_radius_m, trigger_horizon_s) -> bool`
  - `plan_dodge(p_proj, v_proj, p_drone, v_drone, *, horizon_s=3.0) -> DodgePlan`

- [ ] **Step 1: Append the failing tests**

Append to `src/huitzilin_perception/test/test_kalman.py`:

```python
# ── Task 3: dodge planning ────────────────────────────────────────────────

from huitzilin_perception.kalman import (  # noqa: E402
    dodge_direction,
    plan_dodge,
    predict_closest_approach,
    should_dodge,
)


def test_tca_and_miss_without_gravity():
    tca, miss, miss_vec = predict_closest_approach(
        [6.0, 0.5, 0.0], [-8.0, 0.0, 0.0], gravity=np.zeros(3))
    assert tca == pytest.approx(0.75, abs=0.02)
    assert miss == pytest.approx(0.5, abs=0.02)
    assert miss_vec[1] == pytest.approx(0.5, abs=0.02)


def test_tca_gravity_compensated_throw_is_a_hit():
    # Lofted throw: vz0 = 0.5*g*t_flight returns to aim altitude at t=0.75 s.
    v_rel = np.array([-8.0, 0.0, 0.5 * G_MPS2 * 0.75])
    tca, miss, _ = predict_closest_approach([6.0, 0.0, 0.0], v_rel)
    assert tca == pytest.approx(0.75, abs=0.02)
    assert miss < 0.06  # bounded by the 10 ms sampling step


def test_miss_beyond_horizon_reports_horizon_edge():
    # Ball flying AWAY: closest approach is now (t=0).
    tca, miss, _ = predict_closest_approach([6.0, 0.0, 0.0], [8.0, 0.0, 0.0],
                                            gravity=np.zeros(3))
    assert tca == pytest.approx(0.0)
    assert miss == pytest.approx(6.0)


def test_dodge_direction_moves_away_from_pass_side():
    # Ball passes 0.4 m to the drone's +y: dodge must be -y.
    d = dodge_direction([0.0, 0.4, 0.0], [-8.0, 0.0, 0.0])
    np.testing.assert_allclose(d, [0.0, -1.0, 0.0], atol=1e-9)
    assert np.linalg.norm(d) == pytest.approx(1.0)


def test_dodge_direction_dead_centre_falls_back_to_lateral():
    d = dodge_direction([0.01, 0.0, 0.0], [-8.0, 0.0, 0.0])
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert abs(d @ np.array([-1.0, 0.0, 0.0])) < 1e-9   # perpendicular to approach
    assert d[2] == pytest.approx(0.0)                    # horizontal fallback


def test_dodge_direction_vertical_approach_still_defined():
    d = dodge_direction([0.0, 0.0, 0.01], [0.0, 0.0, -10.0])
    assert np.linalg.norm(d) == pytest.approx(1.0)
    assert abs(d[2]) < 1e-9


@pytest.mark.parametrize("n,miss,tca,expected", [
    (3, 0.5, 1.0, True),
    (2, 0.5, 1.0, False),   # not enough confirmations
    (3, 0.9, 1.0, False),   # passes outside threat radius
    (3, 0.5, 2.0, False),   # too far in the future
    (3, 0.5, -0.1, False),  # already passed
])
def test_should_dodge_thresholds(n, miss, tca, expected):
    assert should_dodge(
        n, miss, tca,
        min_updates=3, threat_radius_m=0.75, trigger_horizon_s=1.5,
    ) is expected


def test_plan_dodge_end_to_end_hit_geometry():
    v0 = np.array([-8.0, 0.0, 0.5 * G_MPS2 * 0.75])
    plan = plan_dodge([6.0, 0.0, 2.0], v0, [0.0, 0.0, 2.0], [0.0, 0.0, 0.0])
    assert plan.tca_s == pytest.approx(0.75, abs=0.02)
    assert plan.miss_m < 0.06
    assert np.linalg.norm(plan.direction) == pytest.approx(1.0)
```

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `cd src/huitzilin_perception && python -m pytest test/test_kalman.py -v`
Expected: 8 pass (Task 2), new ones FAIL with `ImportError: cannot import name 'dodge_direction'`

- [ ] **Step 3: Append the implementation**

Append to `src/huitzilin_perception/huitzilin_perception/kalman.py`:

```python
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


def plan_dodge(p_proj, v_proj, p_drone, v_drone, *, horizon_s: float = 3.0) -> DodgePlan:
    """Convenience wrapper: absolute states in odom ENU -> full dodge plan."""
    p_rel = np.asarray(p_proj, dtype=np.float64) - np.asarray(p_drone, dtype=np.float64)
    v_rel = np.asarray(v_proj, dtype=np.float64) - np.asarray(v_drone, dtype=np.float64)
    tca, miss, miss_vec = predict_closest_approach(p_rel, v_rel, horizon_s=horizon_s)
    approach_vel = v_rel + tca * GRAVITY_ENU
    return DodgePlan(tca, miss, miss_vec, dodge_direction(miss_vec, approach_vel))
```

- [ ] **Step 4: Run the full perception test suite**

Run: `cd src/huitzilin_perception && python -m pytest test/ -v`
Expected: all pass (ballistics 7, kalman 16, cloud_geometry existing suite)

- [ ] **Step 5: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/kalman.py src/huitzilin_perception/test/test_kalman.py
git commit -m "W4-03: dodge planning — numeric closest-approach, away-side dodge direction, trigger policy"
```

---

### Task 4: `spawn_projectile.py` — gravity compensation + reusable gz helpers

**Files:**
- Modify: `src/huitzilin_perception/huitzilin_perception/spawn_projectile.py`

**Interfaces:**
- Consumes: `ballistics.compute_spawn` (Task 1).
- Produces (used by Task 7):
  - `MIN_SPAWN_Z: float` (existing, stays)
  - `gz_spawn(world: str, model_uri: str, name: str, position, velocity, timeout_s=5.0) -> tuple[bool, str]`
  - `gz_remove(world: str, name: str, timeout_s=5.0) -> tuple[bool, str]`
  - New ROS param on the node: `compensate_gravity` (bool, default **false** — Week 3 behavior unchanged)

- [ ] **Step 1: Add the module-level gz helpers**

In `spawn_projectile.py`, add after the `MIN_SPAWN_Z` constant (keep all existing imports; none new are needed):

> **SUPERSEDED (2026-07-26) — do not copy this snippet.** The JSON bodies below do
> not work: `gz service --req` parses protobuf **text** format only, and rejects JSON
> with empty stdout and exit code 0 (silent failure). `gz.msgs.EntityFactory` also has
> no `initial_linear_velocity` field in Harmonic — the throw is a separate one-physics-step
> `gz.msgs.EntityWrench` on `/world/<world>/wrench` (needs `gz-sim-apply-link-wrench-system`),
> and the world must be paused across create+impulse or the ball free-falls and ground
> contact eats the kick. See the shipped `gz_spawn` / `gz_world_control` / `gz_impulse` /
> `gz_remove` / `_run_gz` in `src/huitzilin_perception/huitzilin_perception/spawn_projectile.py`
> for the authoritative form.

```python
def gz_spawn(world: str, model_uri: str, name: str, position, velocity,
             timeout_s: float = 5.0) -> tuple[bool, str]:
    """Spawn `model_uri` as `name` at ENU `position` with initial `velocity`
    via the Gazebo Harmonic EntityFactory service. Returns (ok, message).
    Module-level so dodge_battery.py can reuse it without a node."""
    factory_json = json.dumps({
        "sdf_filename": model_uri.replace("model://", ""),
        "name": name,
        "pose": {
            "position": {"x": float(position[0]), "y": float(position[1]),
                         "z": float(position[2])},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        },
        "initial_linear_velocity": {"x": float(velocity[0]),
                                    "y": float(velocity[1]),
                                    "z": float(velocity[2])},
    })
    cmd = [
        "gz", "service", "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", factory_json,
    ]
    return _run_gz(cmd, timeout_s)


def gz_remove(world: str, name: str, timeout_s: float = 5.0) -> tuple[bool, str]:
    """Remove model `name` from the world (gz.msgs.Entity type 2 = MODEL).
    The dodge battery calls this between runs so spent balls don't litter
    the ground plane and confuse the depth background model."""
    req = json.dumps({"name": name, "type": 2})
    cmd = [
        "gz", "service", "-s", f"/world/{world}/remove",
        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    return _run_gz(cmd, timeout_s)


def _run_gz(cmd: list[str], timeout_s: float) -> tuple[bool, str]:
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, "gz service timed out"
    except FileNotFoundError:
        return False, "'gz' command not found — is Gazebo Harmonic installed?"
    if result.returncode != 0:
        return False, f"gz service failed: {result.stderr.strip()}"
    return True, result.stdout.strip()
```

- [ ] **Step 2: Declare the new param and rewire `_do_spawn`**

In `SpawnProjectileNode.__init__`, after the `model_uri` declaration, add:

```python
        self.declare_parameter("compensate_gravity", False)  # loft to arrive at drone altitude (Week 4 battery)
```

and cache it with the other params:

```python
        self._comp_gravity  = self.get_parameter("compensate_gravity").value
```

Add the import at the top (with the other project imports):

```python
from huitzilin_perception.ballistics import compute_spawn
```

Replace the body of `_do_spawn` from the line `odom = self._latest_odom` down to (and including) the `except FileNotFoundError` block with:

```python
        odom = self._latest_odom
        # Drone ENU position
        dx = odom.pose.pose.position.x
        dy = odom.pose.pose.position.y
        dz = odom.pose.pose.position.z

        # Drone yaw from quaternion (ENU)
        q = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        plan = compute_spawn(
            (dx, dy, dz), yaw,
            speed_mps=self._speed,
            approach_angle_deg=self._angle_deg,
            miss_distance_m=self._miss_dist,
            offset_forward_m=self._offset_fwd,
            offset_vertical_m=self._offset_vert,
            compensate_gravity=self._comp_gravity,
        )
        spawn_x, spawn_y, spawn_z = plan.position

        if spawn_z < MIN_SPAWN_Z:
            self.get_logger().error(
                f"NOT spawning: z={spawn_z:.2f} m is below ground clearance "
                f"({MIN_SPAWN_Z} m) — drone z={dz:.2f}, "
                f"offset_vertical_m={self._offset_vert}. Fly higher first "
                f"(need altitude >= {MIN_SPAWN_Z - self._offset_vert:.1f} m, "
                f"e.g. raise takeoff_alt_m) and rerun."
            )
            return

        model_name = f"projectile_{self._scenario_id}_{int(time.time())}"
        vx, vy, vz = plan.velocity
        self.get_logger().info(
            f"Spawning '{model_name}' at ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f}) "
            f"vel=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s "
            f"(gravity compensation {'ON' if self._comp_gravity else 'off'})"
        )
        ok, msg = gz_spawn(self._world, self._model_uri, model_name,
                           plan.position, plan.velocity)
        if ok:
            self.get_logger().info(f"Spawn OK: {msg}")
        else:
            self.get_logger().error(msg)
```

Also update the module docstring's DESIGN NOTES to mention that the geometry math lives in `ballistics.compute_spawn` (shared with the Week 4 battery) and that `compensate_gravity:=true` lofts throws into true hit trajectories.

- [ ] **Step 3: Verify — pure functions still tested, node file compiles**

Run: `cd src/huitzilin_perception && python -m pytest test/test_ballistics.py -v && python -m py_compile huitzilin_perception/spawn_projectile.py && echo COMPILED`
Expected: 7 passed, `COMPILED`

- [ ] **Step 4: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/spawn_projectile.py
git commit -m "W4-04: spawn_projectile — geometry via ballistics.compute_spawn, opt-in gravity loft, reusable gz_spawn/gz_remove"
```

---

### Task 5: `mav_bridge_node.py` — `/cmd/evade` priority path

**Files:**
- Modify: `src/huitzilin_sim/huitzilin_sim/mav_bridge_node.py`

**Interfaces:**
- Consumes: existing `MavBridge.send_velocity_body`.
- Produces: bridge subscribes `/cmd/evade` (`geometry_msgs/Twist`, body FLU). While an evade command is fresher than `cmd_timeout_s`, it overrides `/huitzilin/cmd_vel`; on handback the bridge sends one zero-velocity setpoint, then returns to the previous behavior (including full silence if no `cmd_vel` was ever received — patrol position mode must not be fought with zero-velocity spam).

- [ ] **Step 1: Add evade state + subscription**

In `MavBridgeNode.__init__`, after the `self._cmd_ever_received = False` line, add:

```python
        # evade override state: /cmd/evade preempts /huitzilin/cmd_vel while
        # fresh (Week 4). Separate from cmd_vel so a finished dodge hands
        # control back cleanly instead of leaving a zero-velocity stream
        # fighting patrol's position setpoints.
        self._last_evade = (0.0, 0.0, 0.0, 0.0)
        self._last_evade_t = self.get_clock().now()
        self._evade_ever_received = False
        self._evade_active = False
```

After the existing `create_subscription(Twist, "/huitzilin/cmd_vel", ...)` line, add:

```python
        self.create_subscription(Twist, "/cmd/evade", self._on_evade, 10)
```

- [ ] **Step 2: Add the evade callback (below `_on_cmd_vel`)**

```python
    # -- /cmd/evade: same FLU->NED mapping as cmd_vel, but takes priority
    def _on_evade(self, msg: Twist):
        vx = msg.linear.x
        vy = -msg.linear.y                 # FLU y(left) -> NED y(right)
        vz = -msg.linear.z                 # up -> down
        yaw_rate = -msg.angular.z          # ENU yaw(ccw+) -> NED yaw(cw+)
        with self._lock:
            self._last_evade = (vx, vy, vz, yaw_rate)
            self._last_evade_t = self.get_clock().now()
            self._evade_ever_received = True
```

- [ ] **Step 3: Replace `_tick_setpoint` entirely**

```python
    def _tick_setpoint(self):
        """Stream the freshest command at a fixed rate.

        Priority: fresh /cmd/evade > /huitzilin/cmd_vel. When an evade goes
        stale, send ONE zero setpoint (so ArduPilot doesn't coast on the last
        dodge velocity), then fall back — to cmd_vel zero-hold if patrol ever
        commanded velocity, or to full silence (patrol position mode owns the
        vehicle again)."""
        now = self.get_clock().now()
        with self._lock:
            evade_fresh = False
            if self._evade_ever_received:
                e_age = (now - self._last_evade_t).nanoseconds * 1e-9
                evade_fresh = e_age <= self.cmd_timeout
            evx, evy, evz, eyr = self._last_evade
            cmd_age = (now - self._last_cmd_t).nanoseconds * 1e-9
            vx, vy, vz, yr = self._last_cmd

        if evade_fresh:
            self._evade_active = True
            self.bridge.send_velocity_body(evx, evy, evz, eyr)
            return
        if self._evade_active:
            self._evade_active = False
            self.bridge.send_velocity_body(0.0, 0.0, 0.0, 0.0)  # handback
            return

        if not self._cmd_ever_received:
            return
        if cmd_age > self.cmd_timeout:
            vx = vy = vz = yr = 0.0        # dropout -> calm hold, never a coast/lunge
        self.bridge.send_velocity_body(vx, vy, vz, yr)
```

Also update the node docstring (line 2) to: `"""ROS 2 Jazzy wrapper around MavBridge: cmd_vel/evade in, odom/state out, services."""`

- [ ] **Step 4: Syntax check**

Run: `cd src/huitzilin_sim && python -m py_compile huitzilin_sim/mav_bridge_node.py && echo COMPILED`
Expected: `COMPILED`

- [ ] **Step 5: Commit**

```bash
git add src/huitzilin_sim/huitzilin_sim/mav_bridge_node.py
git commit -m "W4-05: mav_bridge — /cmd/evade priority over cmd_vel with clean one-zero handback"
```

---

### Task 6: `evasion_node.py` + `params/evasion.yaml` + entry point

**Files:**
- Create: `src/huitzilin_perception/huitzilin_perception/evasion_node.py`
- Create: `src/huitzilin_perception/params/evasion.yaml`
- Modify: `src/huitzilin_perception/setup.py`

**Interfaces:**
- Consumes: `ProjectileTracker`, `plan_dodge`, `should_dodge`, `GRAVITY_ENU` (Tasks 2–3); `is_valid_quat`, `make_transform`, `apply_transform`, `quat_to_rot` from `cloud_geometry.py`.
- Produces:
  - Topics out: `/cmd/evade` (Twist, body FLU), `/payload/alarm` (Bool), `/threat/intercept` (PointStamped, base_link), `/threat/evade_event` (String, JSON: keys `t_trigger_s`, `latency_s`, `tca_s`, `miss_m`, `dodge_body`, `dodge_enu`, `over_budget`), `/threat/intercept_marker` (Marker)
  - Console entry point `evasion` (used by Task 8's launch file and the runbook)
  - Runtime-settable params (Task 7's sweep): `dodge_speed_mps`, `threat_radius_m`, `trigger_horizon_s`, `dodge_duration_s`, `min_track_updates`

- [ ] **Step 1: Write `params/evasion.yaml`**

```yaml
# evasion.yaml — Week 4 evasion-node tuning (see docs/WEEK4_PLAN.md).
#
# Sweepable at runtime via `ros2 param set /evasion <key> <val>` (the dodge
# battery's sweep mode uses this): dodge_speed_mps, threat_radius_m,
# trigger_horizon_s, dodge_duration_s, min_track_updates.
# Kalman keys are fixed at node start (tracker is constructed once).

evasion:
  ros__parameters:
    # ── Kalman filter (fixed at node start) ──────────────────────────────
    process_accel_std: 3.0     # m/s^2 accel noise; absorbs drag/stereo mismatch
    meas_std_m: 0.15           # centroid measurement noise (stereo + clustering)
    init_vel_std: 15.0         # m/s prior; covers throws up to 14 m/s
    track_timeout_s: 0.5       # sim s of silence before a track is dropped
    # ── Trigger policy ───────────────────────────────────────────────────
    min_track_updates: 3       # confirmations before a dodge may fire
    threat_radius_m: 0.75      # predicted pass inside this radius -> dodge
    trigger_horizon_s: 1.5     # only dodge if TCA is sooner than this
    prediction_horizon_s: 3.0  # numeric closest-approach search window
    latency_budget_s: 0.15     # centroid stamp -> dodge command, sim s
    # ── Dodge maneuver ───────────────────────────────────────────────────
    dodge_speed_mps: 1.5       # master doc: ~1.5 m/s velocity spike
    dodge_duration_s: 1.0      # how long the spike streams
    recover_hold_s: 0.5        # zero-velocity settle before patrol resumes
    evade_cmd_rate_hz: 20.0    # /cmd/evade stream rate while evading
    auto_resume_patrol: true   # call /huitzilin/start_patrol true after recovery
```

- [ ] **Step 2: Write `evasion_node.py`**

```python
"""
evasion_node.py — HuitzilinReflex Week 4: Kalman filter + dodge trigger.

Closes the sense->dodge loop:
  /threat/centroid (base_link) --re-express in odom--> ballistic KF track
  -> predicted closest approach vs the drone -> trigger policy
  -> pause patrol + stream /cmd/evade (body FLU) + /payload/alarm (mock)
  -> recover -> resume patrol.

State machine (docs/state_machine.md PATROL->EVADE->PATROL):
  TRACKING -> EVADING -> RECOVERING -> TRACKING

Patrol preemption: patrol_node (position mode) streams position setpoints on
its OWN MAVLink connection, so a dodge velocity spike would fight it. The
dodge therefore calls /huitzilin/start_patrol false first; mav_bridge gives
/cmd/evade priority over /huitzilin/cmd_vel for velocity-mode patrol too.

All timing is SIM time (launch sets use_sim_time; latency is measured
against message stamps) — never wall-clock (CLAUDE.md sharp edge).

Frames: centroids arrive in base_link; the filter runs in fixed odom ENU
(filtering in the moving body frame would alias drone motion into the
projectile velocity — Week 3's egomotion lesson, one layer up). Dodge
commands go out in body FLU; mav_bridge owns the only NED conversion.
"""

from __future__ import annotations

import json
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

from huitzilin_perception.cloud_geometry import (
    apply_transform,
    is_valid_quat,
    make_transform,
    quat_to_rot,
)
from huitzilin_perception.kalman import (
    GRAVITY_ENU,
    ProjectileTracker,
    plan_dodge,
    should_dodge,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Params the dodge battery sweeps at runtime (ros2 param set /evasion ...).
SWEEPABLE = {
    "dodge_speed_mps", "threat_radius_m", "trigger_horizon_s",
    "dodge_duration_s", "min_track_updates",
}


class EvadeState(Enum):
    TRACKING = "TRACKING"
    EVADING = "EVADING"
    RECOVERING = "RECOVERING"


class EvasionNode(Node):
    """Kalman tracker + dodge trigger. See module docstring."""

    def __init__(self) -> None:
        super().__init__("evasion")

        # ── Params (values from params/evasion.yaml) ─────────────────────
        self.declare_parameter("centroid_topic", "/threat/centroid")
        self.declare_parameter("odom_topic", "/huitzilin/odom")
        self.declare_parameter("evade_topic", "/cmd/evade")
        self.declare_parameter("alarm_topic", "/payload/alarm")
        self.declare_parameter("intercept_topic", "/threat/intercept")
        self.declare_parameter("event_topic", "/threat/evade_event")
        self.declare_parameter("marker_topic", "/threat/intercept_marker")
        self.declare_parameter("patrol_service", "/huitzilin/start_patrol")
        self.declare_parameter("process_accel_std", 3.0)
        self.declare_parameter("meas_std_m", 0.15)
        self.declare_parameter("init_vel_std", 15.0)
        self.declare_parameter("track_timeout_s", 0.5)
        self.declare_parameter("min_track_updates", 3)
        self.declare_parameter("threat_radius_m", 0.75)
        self.declare_parameter("trigger_horizon_s", 1.5)
        self.declare_parameter("prediction_horizon_s", 3.0)
        self.declare_parameter("latency_budget_s", 0.15)
        self.declare_parameter("dodge_speed_mps", 1.5)
        self.declare_parameter("dodge_duration_s", 1.0)
        self.declare_parameter("recover_hold_s", 0.5)
        self.declare_parameter("evade_cmd_rate_hz", 20.0)
        self.declare_parameter("auto_resume_patrol", True)

        self._p = {
            name: self.get_parameter(name).value
            for name in (
                "centroid_topic", "odom_topic", "evade_topic", "alarm_topic",
                "intercept_topic", "event_topic", "marker_topic",
                "patrol_service", "process_accel_std", "meas_std_m",
                "init_vel_std", "track_timeout_s", "min_track_updates",
                "threat_radius_m", "trigger_horizon_s", "prediction_horizon_s",
                "latency_budget_s", "dodge_speed_mps", "dodge_duration_s",
                "recover_hold_s", "evade_cmd_rate_hz", "auto_resume_patrol",
            )
        }
        self.add_on_set_parameters_callback(self._on_param_set)

        # ── Tracker + state machine ──────────────────────────────────────
        self._tracker = ProjectileTracker(
            process_accel_std=self._p["process_accel_std"],
            meas_std_m=self._p["meas_std_m"],
            init_vel_std=self._p["init_vel_std"],
            track_timeout_s=self._p["track_timeout_s"],
        )
        self._state = EvadeState.TRACKING
        self._phase_end = None            # rclpy Time when current phase ends
        self._dodge_cmd_body = np.zeros(3)
        self._last_odom = None
        self._warned_no_odom = False
        self._warned_bad_quat = False

        # ── ROS interfaces ───────────────────────────────────────────────
        self.create_subscription(PointStamped, self._p["centroid_topic"],
                                 self._centroid_cb, RELIABLE_QOS)
        self.create_subscription(Odometry, self._p["odom_topic"],
                                 self._odom_cb, RELIABLE_QOS)
        self._evade_pub = self.create_publisher(
            Twist, self._p["evade_topic"], RELIABLE_QOS)
        self._alarm_pub = self.create_publisher(
            Bool, self._p["alarm_topic"], RELIABLE_QOS)
        self._intercept_pub = self.create_publisher(
            PointStamped, self._p["intercept_topic"], RELIABLE_QOS)
        self._event_pub = self.create_publisher(
            String, self._p["event_topic"], RELIABLE_QOS)
        self._marker_pub = self.create_publisher(
            Marker, self._p["marker_topic"], RELIABLE_QOS)
        self._patrol_cli = self.create_client(SetBool, self._p["patrol_service"])

        self.create_timer(1.0 / float(self._p["evade_cmd_rate_hz"]),
                          self._evade_tick)

        self.get_logger().info(
            "evasion_node ready — "
            f"threat_radius {self._p['threat_radius_m']} m, "
            f"trigger_horizon {self._p['trigger_horizon_s']} s, "
            f"dodge {self._p['dodge_speed_mps']} m/s "
            f"for {self._p['dodge_duration_s']} s"
        )

    # ── Param updates (sweep support) ────────────────────────────────────

    def _on_param_set(self, params) -> SetParametersResult:
        for prm in params:
            if prm.name in SWEEPABLE:
                self._p[prm.name] = prm.value
                self.get_logger().info(f"param {prm.name} -> {prm.value}")
        return SetParametersResult(successful=True)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _centroid_cb(self, msg: PointStamped) -> None:
        if self._state is not EvadeState.TRACKING:
            return  # mid-dodge/recovery: the tracker restarts fresh afterwards

        odom = self._last_odom
        if odom is None:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn("centroid before first odom — waiting")
            return
        q = odom.pose.pose.orientation
        if not is_valid_quat(q.x, q.y, q.z, q.w):
            if not self._warned_bad_quat:
                self._warned_bad_quat = True
                self.get_logger().warn(
                    "odom carries no valid orientation — cannot lift centroids "
                    "into odom; evasion inactive (pre-b0eedd5-style stream?)")
            return

        # base_link -> odom (odom pose IS T_odom_baselink)
        p = odom.pose.pose.position
        T_odom_bl = make_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        z_bl = np.array([msg.point.x, msg.point.y, msg.point.z])
        z_odom = apply_transform(T_odom_bl, z_bl[None, :])[0].astype(np.float64)

        if not self._tracker.process(self._stamp_to_sec(msg.header.stamp), z_odom):
            return
        if self._tracker.n_updates < int(self._p["min_track_updates"]):
            return

        pos_proj, vel_proj = self._tracker.state()
        p_drone = np.array([p.x, p.y, p.z])
        tw = odom.twist.twist.linear
        v_drone = np.array([tw.x, tw.y, tw.z])  # world ENU (bridge converts NED)

        plan = plan_dodge(pos_proj, vel_proj, p_drone, v_drone,
                          horizon_s=float(self._p["prediction_horizon_s"]))
        self._publish_intercept(plan, pos_proj, vel_proj, T_odom_bl,
                                msg.header.stamp)

        if should_dodge(
            self._tracker.n_updates, plan.miss_m, plan.tca_s,
            min_updates=int(self._p["min_track_updates"]),
            threat_radius_m=float(self._p["threat_radius_m"]),
            trigger_horizon_s=float(self._p["trigger_horizon_s"]),
        ):
            self._start_dodge(plan, q, msg.header.stamp)

    # ── Dodge lifecycle ──────────────────────────────────────────────────

    def _start_dodge(self, plan, q, stamp) -> None:
        R = quat_to_rot(q.x, q.y, q.z, q.w)          # base_link -> odom
        dir_body = R.T @ plan.direction               # odom -> body FLU
        self._dodge_cmd_body = dir_body * float(self._p["dodge_speed_mps"])

        now = self.get_clock().now()
        latency = now.nanoseconds * 1e-9 - self._stamp_to_sec(stamp)
        over = latency > float(self._p["latency_budget_s"])
        self.get_logger().warn(
            f"DODGE: miss={plan.miss_m:.2f} m tca={plan.tca_s:.2f} s "
            f"dir_body=({dir_body[0]:+.2f},{dir_body[1]:+.2f},{dir_body[2]:+.2f}) "
            f"latency={latency * 1000:.0f} ms"
            + ("  ** OVER BUDGET **" if over else "")
        )
        event = String()
        event.data = json.dumps({
            "t_trigger_s": now.nanoseconds * 1e-9,
            "latency_s": latency,
            "tca_s": plan.tca_s,
            "miss_m": plan.miss_m,
            "dodge_body": [float(v) for v in self._dodge_cmd_body],
            "dodge_enu": [float(v) for v in plan.direction],
            "over_budget": over,
        })
        self._event_pub.publish(event)
        self._alarm_pub.publish(Bool(data=True))
        self._set_patrol(False)
        self._state = EvadeState.EVADING
        self._phase_end = now + Duration(
            seconds=float(self._p["dodge_duration_s"]))

    def _evade_tick(self) -> None:
        if self._state is EvadeState.TRACKING:
            return
        now = self.get_clock().now()

        if self._state is EvadeState.EVADING:
            if now < self._phase_end:
                cmd = Twist()
                cmd.linear.x = float(self._dodge_cmd_body[0])
                cmd.linear.y = float(self._dodge_cmd_body[1])
                cmd.linear.z = float(self._dodge_cmd_body[2])
                self._evade_pub.publish(cmd)
            else:
                self._alarm_pub.publish(Bool(data=False))
                self._evade_pub.publish(Twist())  # zero: begin settle
                self._state = EvadeState.RECOVERING
                self._phase_end = now + Duration(
                    seconds=float(self._p["recover_hold_s"]))
        elif self._state is EvadeState.RECOVERING:
            self._evade_pub.publish(Twist())      # hold zero while settling
            if now >= self._phase_end:
                if bool(self._p["auto_resume_patrol"]):
                    self._set_patrol(True)
                self._tracker.reset()
                self._state = EvadeState.TRACKING
                self.get_logger().info("dodge complete -> TRACKING (patrol resumed)")

    def _set_patrol(self, run: bool) -> None:
        if not self._patrol_cli.service_is_ready():
            self.get_logger().warn(
                f"patrol service unavailable — continuing without "
                f"{'resuming' if run else 'pausing'} patrol",
                throttle_duration_sec=5.0)
            return
        req = SetBool.Request()
        req.data = run
        self._patrol_cli.call_async(req)

    # ── Intercept output ─────────────────────────────────────────────────

    def _publish_intercept(self, plan, pos_proj, vel_proj, T_odom_bl,
                           stamp) -> None:
        tca = plan.tca_s
        intercept_odom = (pos_proj + tca * vel_proj
                          + 0.5 * tca * tca * GRAVITY_ENU)
        T_bl_odom = np.linalg.inv(T_odom_bl)
        xyz = apply_transform(T_bl_odom, intercept_odom[None, :])[0]

        out = PointStamped()
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        out.point.x = float(xyz[0])
        out.point.y = float(xyz[1])
        out.point.z = float(xyz[2])
        self._intercept_pub.publish(out)

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = "base_link"
        m.ns = "intercept"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(xyz[0])
        m.pose.position.y = float(xyz[1])
        m.pose.position.z = float(xyz[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.20
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.0
        m.color.a = 0.9
        m.lifetime.nanosec = int(0.5e9)
        self._marker_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvasionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Register the entry point and bump the package**

In `src/huitzilin_perception/setup.py`: change `version="0.3.0"` → `version="0.4.0"`, change `description` to `"HuitzilinReflex Weeks 3-4: perception pipeline + Kalman evasion."`, and add to `console_scripts`:

```python
            "evasion = huitzilin_perception.evasion_node:main",
```

- [ ] **Step 4: Verify**

Run: `cd src/huitzilin_perception && python -m py_compile huitzilin_perception/evasion_node.py && python -m pytest test/ -q && echo OK`
Expected: all tests pass, `OK`

- [ ] **Step 5: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/evasion_node.py src/huitzilin_perception/params/evasion.yaml src/huitzilin_perception/setup.py
git commit -m "W4-06: evasion_node — odom-frame KF tracking, dodge trigger, patrol pause/resume, alarm mock"
```

---

### Task 7: Dodge battery + sweep harness

**Files:**
- Create: `src/huitzilin_perception/huitzilin_perception/dodge_battery.py`
- Create: `src/huitzilin_perception/config/week4_battery.yaml`
- Create: `src/huitzilin_perception/config/week4_sweep.yaml`
- Create: `scripts/run_dodge_battery.sh` (mark executable via `git update-index --chmod=+x`)
- Modify: `src/huitzilin_perception/setup.py` (entry point)

**Interfaces:**
- Consumes: `compute_spawn` (Task 1); `MIN_SPAWN_Z`, `gz_spawn`, `gz_remove` (Task 4); `/threat/evade_event` JSON (Task 6); `/gz/dynamic_poses` (`tf2_msgs/TFMessage`, bridged in Task 8).
- Produces: console entry point `dodge_battery`; report txt + CSV; exit code 0 unless harness errors.

- [ ] **Step 1: Write `config/week4_battery.yaml`**

```yaml
# week4_battery.yaml — Week 4 dodge battery (see docs/WEEK4_PLAN.md).
#
# Every run spawns one projectile at the patrolling drone and scores the
# outcome against Gazebo ground truth. compensate_gravity: true makes
# "direct hit" runs genuinely intercept the drone (flat Week 3 throws pass
# 0.7-2.8 m below it). expect_dodge: false rows are the false-dodge check.
#
# No hard success-rate gate (2026-07-16 decision): the battery reports
# measured rates; only harness errors fail the run.

defaults:
  offset_forward_m: 6.0
  offset_vertical_m: 0.0
  compensate_gravity: true
  repeats: 3

runs:
  - id: B01
    description: "Head-on slow (4 m/s), true hit"
    speed_mps: 4.0
    approach_angle_deg: 0.0
    miss_distance_m: 0.0
    expect_dodge: true

  - id: B02
    description: "Head-on medium (8 m/s), true hit — design point"
    speed_mps: 8.0
    approach_angle_deg: 0.0
    miss_distance_m: 0.0
    expect_dodge: true

  - id: B03
    description: "Head-on fast (14 m/s), true hit — hardest window"
    speed_mps: 14.0
    approach_angle_deg: 0.0
    miss_distance_m: 0.0
    expect_dodge: true

  - id: B04
    description: "Oblique +30deg medium, true hit"
    speed_mps: 8.0
    approach_angle_deg: 30.0
    miss_distance_m: 0.0
    expect_dodge: true

  - id: B05
    description: "Oblique -30deg medium, true hit"
    speed_mps: 8.0
    approach_angle_deg: -30.0
    miss_distance_m: 0.0
    expect_dodge: true

  - id: B06
    description: "Near miss 0.5 m medium — inside threat radius, must dodge"
    speed_mps: 8.0
    approach_angle_deg: 0.0
    miss_distance_m: 0.5
    expect_dodge: true

  - id: B07
    description: "Wide miss 1.5 m medium — outside threat radius, must NOT dodge"
    speed_mps: 8.0
    approach_angle_deg: 0.0
    miss_distance_m: 1.5
    expect_dodge: false
    repeats: 2
```

- [ ] **Step 2: Write `config/week4_sweep.yaml`**

```yaml
# week4_sweep.yaml — parameter sweep over the dodge trigger/maneuver.
#
# Grid kept deliberately small: the Dell renders depth at ~0.33 RTF, so each
# battery pass costs ~3x its sim duration in wall time.

parameters:
  dodge_speed_mps: [1.0, 1.5, 2.5]
  trigger_horizon_s: [1.0, 1.5]

runs: [B02, B03, B06]   # subset of week4_battery.yaml ids
repeats: 1
```

- [ ] **Step 3: Write `dodge_battery.py`**

```python
"""
dodge_battery.py — HuitzilinReflex Week 4: closed-loop dodge battery + sweep.

Spawns gravity-compensated projectiles at the patrolling drone and scores
each run against simulator ground truth:

  dodged      : did /threat/evade_event fire inside the run window?
  latency_s   : centroid-stamp -> dodge-command latency (from the event JSON)
  min_dist_m  : true minimum drone<->ball range, from the bridged Gazebo
                dynamic-pose stream (/gz/dynamic_poses, sim-time stamped)

Success per run:
  expect_dodge true  -> dodged AND min_dist_m > hit_radius_m
  expect_dodge false -> did NOT dodge (false-dodge check)

No hard success-rate gate (2026-07-16 decision): measured rates are
reported; the exit code is non-zero only for harness errors (no odom, no
ground-truth stream, spawn failure, missing config).

Sweep mode (-p sweep_config:=<yaml>) grids evasion-node parameters through
the /evasion set_parameters service between battery passes.

PREREQS (Dell only — live Gazebo depth): the full Week 4 stack is up and
the drone is patrolling. See docs/week4_dodge_runbook.md.

All run windows are timed in SIM time (use_sim_time:=true required).
"""

from __future__ import annotations

import csv
import itertools
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from huitzilin_perception.ballistics import compute_spawn
from huitzilin_perception.spawn_projectile import MIN_SPAWN_Z, gz_remove, gz_spawn

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

ODOM_WAIT_TIMEOUT_WALL_S = 60.0   # bring-up bound (wall clock; generous, exits early)
POSE_STREAM_TIMEOUT_WALL_S = 30.0


class DodgeBatteryNode(Node):
    """Battery orchestrator. run() executes in a worker thread while the
    main thread spins the executor (same pattern as score_bags.py)."""

    def __init__(self) -> None:
        super().__init__("dodge_battery")

        share = get_package_share_directory("huitzilin_perception")
        self.declare_parameter(
            "battery_config", str(Path(share) / "config" / "week4_battery.yaml"))
        self.declare_parameter("sweep_config", "")
        self.declare_parameter("output_file", "/tmp/week4_battery.txt")
        self.declare_parameter("csv_file", "/tmp/week4_battery.csv")
        self.declare_parameter("world_name", "huitzilin_runway")
        self.declare_parameter("drone_model", "iris_depth")
        self.declare_parameter("model_uri", "model://projectile")
        self.declare_parameter("hit_radius_m", 0.30)   # drone body + ball radius
        self.declare_parameter("run_window_s", 5.0)    # sim s to watch each throw
        self.declare_parameter("settle_s", 6.0)        # sim s between runs
        self.declare_parameter("latency_budget_s", 0.15)
        self.declare_parameter("evasion_node_name", "/evasion")

        self._battery_f = Path(self.get_parameter("battery_config").value)
        self._sweep_f = self.get_parameter("sweep_config").value
        self._out_f = Path(self.get_parameter("output_file").value)
        self._csv_f = Path(self.get_parameter("csv_file").value)
        self._world = self.get_parameter("world_name").value
        self._drone_model = self.get_parameter("drone_model").value
        self._model_uri = self.get_parameter("model_uri").value
        self._hit_radius = float(self.get_parameter("hit_radius_m").value)
        self._window_s = float(self.get_parameter("run_window_s").value)
        self._settle_s = float(self.get_parameter("settle_s").value)
        self._budget_s = float(self.get_parameter("latency_budget_s").value)
        self._evasion_name = self.get_parameter("evasion_node_name").value

        self._lock = threading.Lock()
        self._latest_odom = None
        self._pose_stream_seen = False
        self._active_ball = None
        self._min_dist = float("inf")
        self._events = []
        self._listening = False

        self.create_subscription(Odometry, "/huitzilin/odom",
                                 self._odom_cb, RELIABLE_QOS)
        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, RELIABLE_QOS)
        self.create_subscription(String, "/threat/evade_event",
                                 self._event_cb, RELIABLE_QOS)

        self.get_logger().info(
            f"dodge_battery ready | config={self._battery_f} "
            f"sweep={self._sweep_f or '-'} "
            f"world={self._world} drone_model={self._drone_model}"
        )

    # ── Subscriptions ────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _pose_cb(self, msg: TFMessage) -> None:
        self._pose_stream_seen = True
        with self._lock:
            ball_name = self._active_ball
            if ball_name is None:
                return
            drone = ball = None
            for tr in msg.transforms:
                if tr.child_frame_id == self._drone_model:
                    t = tr.transform.translation
                    drone = np.array([t.x, t.y, t.z])
                elif tr.child_frame_id == ball_name:
                    t = tr.transform.translation
                    ball = np.array([t.x, t.y, t.z])
            if drone is not None and ball is not None:
                d = float(np.linalg.norm(ball - drone))
                if d < self._min_dist:
                    self._min_dist = d

    def _event_cb(self, msg: String) -> None:
        with self._lock:
            if not self._listening:
                return
            try:
                self._events.append(json.loads(msg.data))
            except json.JSONDecodeError:
                self.get_logger().warn("unparseable /threat/evade_event payload")

    # ── Time helpers (SIM time via node clock; use_sim_time:=true) ───────

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait_sim(self, duration_s: float) -> None:
        t0 = self._sim_now()
        while rclpy.ok() and self._sim_now() - t0 < duration_s:
            time.sleep(0.02)  # wall-sleep; the WINDOW is judged in sim time

    def _wait_wall_for(self, predicate, timeout_s: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    # ── Main flow ────────────────────────────────────────────────────────

    def run(self) -> int:
        if not self._battery_f.exists():
            self.get_logger().error(f"battery config not found: {self._battery_f}")
            return 1
        with open(self._battery_f) as f:
            cfg = yaml.safe_load(f)
        defaults = cfg.get("defaults", {})
        runs = {r["id"]: {**defaults, **r} for r in cfg["runs"]}

        combos = [{}]
        selected = list(runs)
        repeats_override = None
        if self._sweep_f:
            sweep_path = Path(self._sweep_f)
            if not sweep_path.exists():
                self.get_logger().error(f"sweep config not found: {sweep_path}")
                return 1
            with open(sweep_path) as f:
                sweep = yaml.safe_load(f)
            keys = sorted(sweep["parameters"])
            combos = [dict(zip(keys, vals)) for vals in
                      itertools.product(*(sweep["parameters"][k] for k in keys))]
            selected = sweep.get("runs", selected)
            repeats_override = sweep.get("repeats")

        # Pre-flight: odom + ground-truth stream must be alive.
        if not self._wait_wall_for(lambda: self._latest_odom is not None,
                                   ODOM_WAIT_TIMEOUT_WALL_S):
            self.get_logger().error("no /huitzilin/odom — is the flight stack up?")
            return 1
        if not self._wait_wall_for(lambda: self._pose_stream_seen,
                                   POSE_STREAM_TIMEOUT_WALL_S):
            self.get_logger().error(
                "no /gz/dynamic_poses — launch week4_evasion.launch.py "
                "(it bridges /world/<world>/dynamic_pose/info)")
            return 1

        rows = []
        for combo in combos:
            if combo and not self._apply_params(combo):
                rows.append({"combo": self._combo_str(combo), "id": "-",
                             "rep": 0, "error": True, "success": False,
                             "expect_dodge": False,
                             "note": "set_parameters failed"})
                continue
            for rid in selected:
                if rid not in runs:
                    self.get_logger().warn(f"unknown run id {rid}; skipping")
                    continue
                scen = runs[rid]
                n_rep = int(repeats_override or scen.get("repeats", 1))
                for rep in range(n_rep):
                    row = self._one_run(scen, rep, combo)
                    rows.append(row)
                    mark = "✓" if row.get("success") else "✗"
                    self.get_logger().info(
                        f"  [{mark}] {row['combo']:<24} {row['id']:>4} r{rep} "
                        f"dodged={row.get('dodged')} "
                        f"min={row.get('min_dist_m', float('nan'))} m "
                        f"lat={row.get('latency_ms', '-')} ms | {row['note']}")

        return self._report(rows)

    @staticmethod
    def _combo_str(combo: dict) -> str:
        return ",".join(f"{k}={v}" for k, v in sorted(combo.items())) or "baseline"

    def _one_run(self, scen: dict, rep: int, combo: dict) -> dict:
        rid = scen["id"]
        base = {"combo": self._combo_str(combo), "id": rid, "rep": rep,
                "expect_dodge": bool(scen["expect_dodge"])}

        odom = self._latest_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        plan = compute_spawn(
            (p.x, p.y, p.z), yaw,
            speed_mps=float(scen["speed_mps"]),
            approach_angle_deg=float(scen.get("approach_angle_deg", 0.0)),
            miss_distance_m=float(scen.get("miss_distance_m", 0.0)),
            offset_forward_m=float(scen.get("offset_forward_m", 6.0)),
            offset_vertical_m=float(scen.get("offset_vertical_m", 0.0)),
            compensate_gravity=bool(scen.get("compensate_gravity", True)),
        )
        if plan.position[2] < MIN_SPAWN_Z:
            return {**base, "error": True, "success": False,
                    "note": f"spawn z={plan.position[2]:.2f} < {MIN_SPAWN_Z}"}

        name = f"ball_{rid}_r{rep}_{int(time.time())}"
        with self._lock:
            self._active_ball = name
            self._min_dist = float("inf")
            self._events = []
            self._listening = True

        ok, msg = gz_spawn(self._world, self._model_uri, name,
                           plan.position, plan.velocity)
        if not ok:
            with self._lock:
                self._listening = False
                self._active_ball = None
            return {**base, "error": True, "success": False, "note": msg}

        self._wait_sim(self._window_s)

        with self._lock:
            self._listening = False
            self._active_ball = None
            min_dist = self._min_dist
            events = list(self._events)

        gz_remove(self._world, name)
        self._wait_sim(self._settle_s)   # let patrol resume + bg model settle

        dodged = len(events) > 0
        latency_ms = round(events[0]["latency_s"] * 1000.0, 1) if dodged else None
        if min_dist == float("inf"):
            return {**base, "error": True, "success": False, "dodged": dodged,
                    "note": f"ball '{name}' never seen on /gz/dynamic_poses "
                            f"(check drone_model:={self._drone_model})"}

        if base["expect_dodge"]:
            success = dodged and min_dist > self._hit_radius
            note = ("clean dodge" if success else
                    "no dodge fired" if not dodged else
                    f"dodged but min_dist {min_dist:.2f} <= hit_radius")
        else:
            success = not dodged
            note = "correctly ignored" if success else "FALSE DODGE"

        return {**base, "error": False, "dodged": dodged,
                "latency_ms": latency_ms, "min_dist_m": round(min_dist, 3),
                "success": success, "note": note}

    def _apply_params(self, combo: dict) -> bool:
        cli = self.create_client(
            SetParameters, f"{self._evasion_name}/set_parameters")
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"{self._evasion_name}/set_parameters unavailable")
            return False
        req = SetParameters.Request()
        for k, v in combo.items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = v
            elif isinstance(v, int):
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = v
            else:
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = float(v)
            req.parameters.append(Parameter(name=k, value=pv))
        fut = cli.call_async(req)
        if not self._wait_wall_for(fut.done, 10.0):
            self.get_logger().error("set_parameters call timed out")
            return False
        ok = all(r.successful for r in fut.result().results)
        if not ok:
            self.get_logger().error(f"set_parameters rejected: {combo}")
        else:
            self.get_logger().info(f"sweep combo applied: {self._combo_str(combo)}")
        return ok

    # ── Reporting ────────────────────────────────────────────────────────

    def _report(self, rows: list) -> int:
        errors = [r for r in rows if r.get("error")]
        scored = [r for r in rows if not r.get("error")]

        lines = [
            "",
            "═" * 72,
            "  HuitzilinReflex Week 4 — Dodge Battery",
            f"  Battery: {self._battery_f.name}   "
            f"Sweep: {Path(self._sweep_f).name if self._sweep_f else '—'}   "
            f"hit_radius: {self._hit_radius} m   window: {self._window_s} s",
            "═" * 72,
            "",
            f"  {'Combo':<26} {'ID':>4} {'Rep':>3} {'Dodge':>5} "
            f"{'MinDist':>8} {'Lat ms':>7} {'OK':>3}  Note",
            "  " + "─" * 68,
        ]
        for r in rows:
            md = r.get("min_dist_m")
            lines.append(
                f"  {r['combo']:<26} {r['id']:>4} {r['rep']:>3} "
                f"{str(r.get('dodged', '-')):>5} "
                f"{md if md is not None else float('nan'):>8} "
                f"{str(r.get('latency_ms', '-')):>7} "
                f"{'✓' if r.get('success') else '✗':>3}  {r['note']}"
            )

        for combo in sorted({r["combo"] for r in scored}):
            sub = [r for r in scored if r["combo"] == combo]
            hits = [r for r in sub if r["expect_dodge"]]
            wides = [r for r in sub if not r["expect_dodge"]]
            n_dodge_ok = sum(1 for r in hits if r["success"])
            n_false = sum(1 for r in wides if not r["success"])
            lats = [r["latency_ms"] for r in sub if r.get("latency_ms") is not None]
            lines += [
                "",
                f"  [{combo}]",
                f"    dodge success: {n_dodge_ok}/{len(hits)}"
                + (f"  ({100.0 * n_dodge_ok / len(hits):.0f}%)" if hits else ""),
                f"    false dodges:  {n_false}/{len(wides)}",
            ]
            if lats:
                lines.append(
                    f"    latency: mean {np.mean(lats):.0f} ms, "
                    f"max {np.max(lats):.0f} ms "
                    f"(budget {self._budget_s * 1000:.0f} ms)")

        verdict = ("HARNESS ERRORS — see rows above" if errors
                   else "COMPLETE (no hard gate; judge rates above)")
        lines += ["", f"  {verdict}", "═" * 72, ""]

        report = "\n".join(lines)
        print(report)
        try:
            self._out_f.parent.mkdir(parents=True, exist_ok=True)
            self._out_f.write_text(report)
            with open(self._csv_f, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=[
                    "combo", "id", "rep", "expect_dodge", "dodged",
                    "latency_ms", "min_dist_m", "success", "error", "note"])
                w.writeheader()
                for r in rows:
                    w.writerow({k: r.get(k) for k in w.fieldnames})
            self.get_logger().info(
                f"Report → {self._out_f} | CSV → {self._csv_f}")
        except Exception as e:  # noqa: BLE001 — report I/O must not mask results
            self.get_logger().warn(f"could not write report: {e}")

        return 1 if errors else 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DodgeBatteryNode()
    exit_code = [1]

    def _run() -> None:
        try:
            exit_code[0] = node.run()
        finally:
            node._done = True

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    while rclpy.ok() and not getattr(node, "_done", False):
        rclpy.spin_once(node, timeout_sec=0.05)
    thread.join(timeout=5.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code[0])


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Write `scripts/run_dodge_battery.sh`**

```bash
#!/usr/bin/env bash
# Week 4 dodge battery / sweep — Dell only (live Gazebo depth required).
#
# Prereqs (docs/week4_dodge_runbook.md): depth world + SITL up, drone flying
# patrol under `ros2 launch huitzilin_perception week4_evasion.launch.py
# with_patrol:=true`.
#
# Usage:
#   ./scripts/run_dodge_battery.sh            # full battery
#   ./scripts/run_dodge_battery.sh sweep      # parameter sweep
#   EXTRA_ARGS="-p run_window_s:=6.0" ./scripts/run_dodge_battery.sh
set -euo pipefail

MODE="${1:-battery}"
PKG_SHARE="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception"

ARGS=(-p use_sim_time:=true)
if [[ "${MODE}" == "sweep" ]]; then
  ARGS+=(-p "sweep_config:=${PKG_SHARE}/config/week4_sweep.yaml"
         -p output_file:=/tmp/week4_sweep.txt
         -p csv_file:=/tmp/week4_sweep.csv)
fi
# shellcheck disable=SC2206
ARGS+=(${EXTRA_ARGS:-})

exec ros2 run huitzilin_perception dodge_battery --ros-args "${ARGS[@]}"
```

- [ ] **Step 5: Register the entry point**

In `src/huitzilin_perception/setup.py`, add to `console_scripts`:

```python
            "dodge_battery = huitzilin_perception.dodge_battery:main",
```

- [ ] **Step 6: Verify**

Run: `cd src/huitzilin_perception && python -m py_compile huitzilin_perception/dodge_battery.py && python -m pytest test/ -q && echo OK`
Expected: compiles, all tests pass, `OK`

- [ ] **Step 7: Commit**

```bash
git add src/huitzilin_perception/huitzilin_perception/dodge_battery.py src/huitzilin_perception/config/week4_battery.yaml src/huitzilin_perception/config/week4_sweep.yaml scripts/run_dodge_battery.sh src/huitzilin_perception/setup.py
git update-index --chmod=+x scripts/run_dodge_battery.sh
git commit -m "W4-07: dodge battery + sweep — ground-truth scoring via Gazebo dynamic poses, no hard gate"
```

---

### Task 8: `week4_evasion.launch.py`

**Files:**
- Create: `src/huitzilin_perception/launch/week4_evasion.launch.py`

**Interfaces:**
- Consumes: `week3_perception.launch.py` (live stack), `evasion` entry point (Task 6), `params/evasion.yaml` (Task 6).
- Produces: bridged ground-truth topic `/gz/dynamic_poses` (`tf2_msgs/TFMessage`) consumed by Task 7.

- [ ] **Step 1: Write the launch file**

```python
"""
week4_evasion.launch.py — HuitzilinReflex Week 4.

One-command bring-up of the Week 4 evasion stack:
  1. Week 3 live perception stack (bridges, TF, detector) — included
  2. evasion node (Kalman + dodge trigger)
  3. Gazebo dynamic-pose bridge -> /gz/dynamic_poses (battery ground truth)

USAGE (Dell, after world + SITL are up — docs/week4_dodge_runbook.md)
---------------------------------------------------------------------
  ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true

  # then, in another terminal:
  ./scripts/run_dodge_battery.sh          # battery
  ./scripts/run_dodge_battery.sh sweep    # parameter sweep
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_perception")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 flight stack"),
        DeclareLaunchArgument("world_name", default_value="huitzilin_runway"),
        DeclareLaunchArgument("gz_pose_bridge", default_value="true",
                              description="Bridge Gazebo dynamic poses (battery ground truth)"),
        DeclareLaunchArgument(
            "evasion_params",
            default_value=os.path.join(pkg, "params", "evasion.yaml"),
        ),
    ]

    week3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "week3_perception.launch.py")),
        launch_arguments={
            "mode": "live",
            "with_patrol": LaunchConfiguration("with_patrol"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    evasion = Node(
        package="huitzilin_perception",
        executable="evasion",
        name="evasion",
        output="screen",
        parameters=[
            LaunchConfiguration("evasion_params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(args + [week3, evasion,
                                     OpaqueFunction(function=_pose_bridge)])


def _pose_bridge(context):
    """Ground-truth pose bridge; the world name must be resolved to build
    the gz topic string, hence OpaqueFunction instead of a plain Node."""
    if context.launch_configurations.get("gz_pose_bridge", "true").lower() != "true":
        return []
    world = context.launch_configurations["world_name"]
    gz_topic = f"/world/{world}/dynamic_pose/info"
    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_pose_bridge",
        output="screen",
        arguments=[f"{gz_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[(gz_topic, "/gz/dynamic_poses")],
        parameters=[{"use_sim_time": True}],
    )]
```

- [ ] **Step 2: Syntax check**

Run: `cd src/huitzilin_perception && python -m py_compile launch/week4_evasion.launch.py && echo COMPILED`
Expected: `COMPILED`

- [ ] **Step 3: Commit**

```bash
git add src/huitzilin_perception/launch/week4_evasion.launch.py
git commit -m "W4-08: week4_evasion.launch.py — Week 3 stack + evasion + ground-truth pose bridge"
```

---

### Task 9: Docs — architecture promotion + Dell runbook

**Files:**
- Modify: `docs/architecture.md`
- Create: `docs/week4_dodge_runbook.md`

**Do NOT touch `CLAUDE.md`** (dirty from the hardware session).

- [ ] **Step 1: Promote the Week 4 contracts in `docs/architecture.md`**

1. Intro paragraph: replace `Weeks 1–3 contracts are **active**; Week 4–6 contracts are provisional until their nodes land. In Week 4 the evasion path (`/cmd/evade`) commands through the same `mav_bridge` velocity path.` with `Weeks 1–4 contracts are **active**; Week 5–6 contracts are provisional until their nodes land. The evasion path (`/cmd/evade`) commands through the same `mav_bridge` velocity path and preempts `/huitzilin/cmd_vel` while fresh.`
2. Nodes table, `mav_bridge` row: change the Inputs cell to `` `/huitzilin/cmd_vel`; `/cmd/evade` (priority); services `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` ``.
3. Nodes table, `evasion_node` row: replace with:
   `| evasion_node | Kalman filter + dodge trigger + patrol pause/resume | \`/threat/centroid\`, \`/huitzilin/odom\` | \`/threat/intercept\`, \`/cmd/evade\`, \`/payload/alarm\` (mock), \`/threat/evade_event\` | per centroid; 20 Hz while evading | **active, sim (Wk4)** | Pi / dev PC |`
4. Diagram: move the evasion lines into the `[ active ]` block:
   ```
   detector_node → /threat/centroid → evasion_node → /threat/intercept, /cmd/evade → mav_bridge
                                      evasion_node → /payload/alarm (mock until Wk6); /huitzilin/start_patrol (pause/resume)
   ```
   leaving only the payload_node line under `[ Wk5–6 future ]`.
5. Move `/threat/intercept` and `/cmd/evade` out of the Provisional table into a new section after the perception table:

   ```markdown
   ### Active — evasion (sim, promoted W4)

   | Interface | Type | QoS | Frame |
   |---|---|---|---|
   | `/threat/intercept` | `geometry_msgs/PointStamped` | Reliable | `base_link` |
   | `/cmd/evade` | `geometry_msgs/Twist` | Reliable | body **FLU** (bridge priority over `/huitzilin/cmd_vel`) |
   | `/threat/evade_event` | `std_msgs/String` (JSON) | Reliable | N/A |
   | `/payload/alarm` | `std_msgs/Bool` | Reliable | N/A (consumer arrives Wk6) |
   ```

   The Provisional (Wk5–6) table keeps only the `/payload/alarm` consumer row (payload_node).

- [ ] **Step 2: Write `docs/week4_dodge_runbook.md`**

````markdown
# Week 4 Dodge Battery Runbook (Dell)

Bring-up + scoring procedure for the Week 4 closed loop. Everything here
runs on the **native-Ubuntu Dell** (live Gazebo depth). World/SITL bring-up
is identical to Week 3 — see `docs/week3_capture_runbook.md` for the depth
world commands; only the launch + battery steps differ.

## 1. Bring-up (4 terminals)

```bash
# T1 — depth world (exact command per docs/week3_capture_runbook.md)
./scripts/week3_world.sh

# T2 — SITL (same fan-out as always)
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

# T3 — Week 4 stack (Week 3 stack + evasion + ground-truth pose bridge)
ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true

# T4 — arm, take off, start patrol (service types matter — see CLAUDE.md)
ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger
ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}'
```

Sanity checks before scoring:
- `ros2 topic hz /threat/centroid` shows traffic when something crosses the ROI.
- `ros2 topic echo /gz/dynamic_poses --once` lists the drone model. If its
  `child_frame_id` is not `iris_depth`, pass the real name to the battery:
  `EXTRA_ARGS="-p drone_model:=<name>" ./scripts/run_dodge_battery.sh`.
- `ros2 topic info /cmd/evade` shows 1 pub (evasion) + 1 sub (mav_bridge).

## 2. Single-shot smoke test

```bash
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=SMOKE -p speed_mps:=8.0 -p compensate_gravity:=true
```

Expected within ~1 s (sim): evasion logs `DODGE: miss=... tca=... latency=...`,
the drone visibly jinks sideways in Gazebo, patrol pauses and resumes
~1.5 s later, and `/payload/alarm` pulses true→false. If the drone does not
move, check `ros2 topic echo /cmd/evade` first — if commands stream but
nothing moves, the bridge priority path is at fault; if no commands, the
trigger never fired (check `/threat/evade_event`).

## 3. Full battery

```bash
./scripts/run_dodge_battery.sh
```

- ~20 spawns (B01–B07 × repeats), each: spawn → 5 s sim watch window →
  ball removed → 6 s sim settle. Budget ≥ 15 min wall at ~0.33 RTF.
- Output: table + per-combo aggregates → `/tmp/week4_battery.txt`,
  rows → `/tmp/week4_battery.csv`.
- **No hard success gate** (2026-07-16 decision): exit ≠ 0 only for harness
  errors (no odom / no pose stream / spawn failures).

## 4. Parameter sweep

```bash
./scripts/run_dodge_battery.sh sweep
```

Grids `dodge_speed_mps × trigger_horizon_s` (3×2) over B02/B03/B06 via
`ros2 param set` on the live evasion node — no restarts. Results →
`/tmp/week4_sweep.{txt,csv}`. Pick the winning combo, write it into
`params/evasion.yaml`, and re-run the full battery once to confirm.

## 5. What "done" looks like (Week 4 DoD)

- Battery report shows dodges firing on hit-intent runs with measured
  latency vs the 150 ms budget, and no false dodge on B07.
- `min_dist_m` on successful dodges exceeds `hit_radius_m` (0.30 m).
- Record the numbers + chosen params in `docs/JOURNAL.md` (Week 4 entry),
  then delete `docs/WEEK4_PLAN.md` (closed-plan convention).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Dodge fires, drone doesn't move | patrol still fighting the spike → check evasion log called `/huitzilin/start_patrol false`; verify bridge evade priority |
| No dodge on obvious hits | `/threat/centroid` silent (detector issue — Week 3 runbook) or `min_track_updates` unreached for fast balls at 15 Hz → try `trigger_horizon_s` 2.0 |
| `ball ... never seen on /gz/dynamic_poses` | wrong `world_name`/`drone_model` param, or pose bridge disabled (`gz_pose_bridge:=false`) |
| Balls pile up on the runway | `gz_remove` failing — check T3 log; remove manually: `gz service -s /world/huitzilin_runway/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "<ball>", type: MODEL'` |
| Everything slow / timing weird | judging by wall clock — all windows are sim-time; RTF ~0.33 is expected |
````

- [ ] **Step 3: Commit**

```bash
git add docs/architecture.md docs/week4_dodge_runbook.md
git commit -m "W4-09: docs — promote Week 4 evasion contracts, add Dell dodge-battery runbook"
```

---

## Post-plan (not subagent tasks)

- **Dell execution:** run the runbook (smoke test → battery → sweep → re-run battery), then write the `docs/JOURNAL.md` Week 4 entry with measured numbers and delete this plan file (closed-plan convention). Requires the physical Dell box — done by Jordan, not a subagent.
- Push `week4/kalman-dodge` to origin for the Dell to pull.
