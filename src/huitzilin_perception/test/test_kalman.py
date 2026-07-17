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
