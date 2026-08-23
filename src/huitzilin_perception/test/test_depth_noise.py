"""test_depth_noise.py — contracts for the rendered lane's noise stage.

Runs without ROS (numpy + scipy only), so it works on any dev box:

    python3 -m pytest src/huitzilin_perception/test/test_depth_noise.py

A Gazebo depth camera reports very nearly exact geometry: `iris_depth` declares
a 1 cm per-pixel gaussian and the optics probe world declares none at all.
Either way the error is ~30x too small at 26 m -- where the real figure is
0.30 m -- and independent per pixel where the real one is correlated. Scoring
the rendered lane on that makes the simulated sensor strictly better than the
part it stands for, so every save rate taken through it overstates. These tests
pin the noise stage hard enough that it cannot quietly degrade into "no noise".

The load-bearing property is in `test_small_patch_moves_coherently`: the ball
must receive a near-common-mode error, so the error lands on its centroid. That
has to emerge from the correlation length rather than a hand-set
`per_point_frac` -- a knob set to the answer proves nothing.

Two properties that are easy to assume the wrong way round:

  * The ball's spread is SENSITIVE to `correlation_px` -- it falls roughly as
    1/s -- so that parameter is a declared modelling assumption, not a free
    knob. (`test_spread_tightens_monotonically_with_correlation_length`)
  * The ball does NOT stay inside the shipped `cluster_max_extent_m: 0.35` at
    26 m. That gate was sized for a ball at <= 5 m, and roughly a fifth of ball
    frames exceed it once sigma reaches 0.30 m.
    (`test_the_shipped_extent_gate_rejects_the_ball_at_long_range`)
"""

import math

import numpy as np
import pytest

from huitzilin_perception.depth_noise import (
    BALL_PATCH_P2P_SIGMAS,
    DEFAULT_CORRELATION_PX,
    DEPTH_AXIS_BY_CONVENTION,
    apply_image_depth_noise,
    correlated_unit_field,
    required_cluster_max_extent_m,
)
from huitzilin_perception.synthetic_depth import depth_sigma_m


def _flat_wall(height, width, depth_m, fov_deg=27.0):
    """An organized cloud: a flat wall at `depth_m`, filling the frame."""
    f_px = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    us = np.arange(width) - (width - 1) / 2.0
    vs = np.arange(height) - (height - 1) / 2.0
    uu, vv = np.meshgrid(us, vs)
    z = np.full((height, width), float(depth_m))
    return np.stack([z * uu / f_px, z * vv / f_px, z], axis=-1)


# --- the unit field ----------------------------------------------------------

# A NOTE ON ESTIMATORS, because getting this wrong cost three false failures
# while this file was written. A correlated field holds far fewer independent
# samples than it has pixels: at correlation length 16 a 256x256 draw carries
# only about (256/16)^2 = 256 of them. So any single-draw statistic -- its
# mean, its sample std, the max over a handful of draws -- is dominated by
# sampling noise and measures the test loop rather than the model. Every
# distributional claim below is therefore averaged over seeds.

@pytest.mark.parametrize("corr_px", [0.0, 1.0, 4.0, 7.0, 16.0])
def test_field_is_unit_variance_at_every_correlation_length(corr_px):
    """The normalisation must be exact, or sigma silently depends on the knob.

    Smoothing white noise REDUCES its variance. If the rescale is wrong, then
    raising `correlation_px` to make the ball coherent would also quietly make
    the sensor more accurate -- the exact direction that flatters the result.
    """
    fields = [correlated_unit_field(256, 256, np.random.default_rng(7 + i),
                                    corr_px) for i in range(30)]
    assert fields[0].shape == (256, 256)
    # Mean of squares, not the sample std of one draw: the field is zero-mean
    # by construction, so E[x^2] is the unbiased estimate of its variance and
    # does not subtract a sample mean that is itself noisy.
    variance = float(np.mean([np.mean(f ** 2) for f in fields]))
    assert abs(math.sqrt(variance) - 1.0) < 0.05


@pytest.mark.parametrize("corr_px", [0.0, 1.0, 4.0, 7.0, 16.0])
def test_field_is_zero_mean_across_draws(corr_px):
    """Zero-mean is a property of the DISTRIBUTION, not of one draw.

    A single 256x256 field at correlation length 16 holds only ~256
    independent cells, so its own mean has a standard error near 0.25 and
    routinely lands far from zero. Demanding a small mean from one draw tests
    the sample size, not the model; averaging over seeds tests the model.
    """
    means = [float(np.mean(correlated_unit_field(
        256, 256, np.random.default_rng(1000 + i), corr_px)))
        for i in range(30)]
    stderr = float(np.std(means)) / math.sqrt(len(means))
    assert abs(float(np.mean(means))) < 4.0 * stderr + 0.01


def test_field_is_correlated_over_the_stated_length():
    """Neighbours must covary, and far-apart pixels must not."""
    field = correlated_unit_field(512, 512, np.random.default_rng(3), 8.0)
    near = float(np.mean(field[:, :-1] * field[:, 1:]))
    far = float(np.mean(field[:, :-64] * field[:, 64:]))
    assert near > 0.9, "adjacent pixels inside one correlation cell"
    assert abs(far) < 0.20, "64 px is 8 correlation lengths apart"


def test_zero_correlation_is_white():
    field = correlated_unit_field(128, 128, np.random.default_rng(1), 0.0)
    near = float(np.mean(field[:, :-1] * field[:, 1:]))
    assert abs(near) < 0.1


# --- the noise application ---------------------------------------------------

def test_zero_sigma_reproduces_the_input_exactly():
    """Same contract as `synthetic_depth.apply_depth_noise`: noiseless is
    genuinely noiseless, so a control arm is a true control."""
    cloud = _flat_wall(32, 32, 10.0)
    out = apply_image_depth_noise(
        cloud, np.random.default_rng(0), sigma_ref_m=0.0)
    assert np.array_equal(out, cloud)


def test_marginal_sigma_matches_the_calibrated_law():
    """Per-point depth error must have std sigma(z) -- the SAME law the
    synthetic lane was calibrated to (0.30 m at 26 m), imported not re-derived.
    """
    z = 26.0
    cloud = _flat_wall(200, 200, z)
    rng = np.random.default_rng(11)
    # Average over many draws: one draw of a correlated field has few
    # independent cells, so its sample std is a poor estimate of sigma.
    errs = [apply_image_depth_noise(cloud, rng)[..., 2] - z for _ in range(40)]
    got = float(np.std(np.concatenate([e.ravel() for e in errs])))
    assert got == pytest.approx(
        float(depth_sigma_m(z)), rel=0.10), "sigma(26 m) must be 0.30 m"


def test_sigma_grows_quadratically_with_range():
    rng = np.random.default_rng(5)
    spreads = {}
    for z in (13.0, 26.0):
        errs = [apply_image_depth_noise(_flat_wall(100, 100, z), rng)[..., 2] - z
                for _ in range(40)]
        spreads[z] = float(np.std(np.concatenate([e.ravel() for e in errs])))
    # z^2 => halving the range quarters sigma.
    assert spreads[26.0] / spreads[13.0] == pytest.approx(4.0, rel=0.15)


def test_bearing_is_preserved():
    """A disparity error scales the point ALONG ITS RAY. A stereo pair is
    wrong about range, not about which direction the ball lies in -- the same
    distinction `synthetic_depth` draws between a depth model and a position
    oracle with noise bolted on.
    """
    cloud = _flat_wall(24, 24, 20.0)
    out = apply_image_depth_noise(cloud, np.random.default_rng(2))
    for axis in (0, 1):
        before = cloud[..., axis] / cloud[..., 2]
        after = out[..., axis] / out[..., 2]
        np.testing.assert_allclose(after, before, rtol=1e-9)


# The ball sits inside a full frame, not alone in a 5x5 image, so the patch
# tests crop one out of a realistic frame. Sizing the array to the ball instead
# would change the boundary wrap and measure an artefact of the test rig.
_BALL_SPAN_PX = 5          # 80 mm at 26 m through the proposed AR0234 optics
_BALL_EXTENT_M = 0.080
_CLUSTER_MAX_EXTENT_M = 0.35   # detector.yaml, the gate this must not trip


def _ball_patch(out, span=_BALL_SPAN_PX):
    """The centre `span` x `span` returns -- the ball's footprint in-frame."""
    h, w, _ = out.shape
    r0, c0 = (h - span) // 2, (w - span) // 2
    return out[r0:r0 + span, c0:c0 + span, 2]


def test_small_patch_moves_coherently():
    """THE load-bearing contract.

    At 26 m the 80 mm ball spans ~5 px at the proposed optics -- well inside
    one correlation cell -- so it must receive essentially ONE disparity
    solution: its extent survives and the error lands on its centroid. This is
    what `synthetic_depth`'s fully common-mode default asserts by hand; here it
    has to fall out of the correlation length instead.

    The spread is summarised by its MEDIAN, not its maximum. The std of 25
    highly-correlated samples has only one or two effective degrees of freedom,
    so its sample maximum grows with the number of draws and describes the test
    loop rather than the model. What the ball's survival depends on is the
    typical frame.
    """
    z = 26.0
    frame = _flat_wall(128, 128, z)
    rng = np.random.default_rng(13)
    centroid_errs, spreads = [], []
    for _ in range(200):
        patch = _ball_patch(apply_image_depth_noise(frame, rng))
        centroid_errs.append(float(np.mean(patch)) - z)
        spreads.append(float(np.std(patch)))
    # A typical frame keeps the ball far tighter than its own 80 mm body.
    assert float(np.median(spreads)) < _BALL_EXTENT_M
    # And the error is not lost: it shows up on the centroid at full sigma.
    assert float(np.std(centroid_errs)) == pytest.approx(
        float(depth_sigma_m(z)), rel=0.20)


def test_correlation_is_what_keeps_the_ball_intact():
    """The direct demonstration that the correlation does the work.

    Against the fully-independent limit -- `correlation_px = 0`, the case
    `test_fully_independent_per_point_depth_noise_destroys_the_26_m_cluster`
    shows smears the 26 m ball into a ~1.5 m cigar -- the correlated field must
    be dramatically tighter. A model that scored the same either way would not
    be modelling a stereo matcher at all.
    """
    z = 26.0
    frame = _flat_wall(128, 128, z)

    def mean_spread(corr_px):
        rng = np.random.default_rng(19)
        return float(np.mean([
            float(np.std(_ball_patch(apply_image_depth_noise(
                frame, rng, correlation_px=corr_px)))) for _ in range(50)]))

    assert mean_spread(0.0) > 5.0 * mean_spread(DEFAULT_CORRELATION_PX)


def test_spread_tightens_monotonically_with_correlation_length():
    """`correlation_px` is a REAL parameter, and this records how much.

    An earlier version of this file asserted the ball was insensitive to it.
    That was wrong: the within-ball spread falls roughly as 1/s. The parameter
    is therefore set from the matcher's support window and declared as a
    modelling assumption -- it is not available for tuning toward a score, and
    any result taken through this lane has to quote it.
    """
    z = 26.0
    frame = _flat_wall(128, 128, z)

    def median_spread(corr_px):
        rng = np.random.default_rng(19)
        return float(np.median([
            float(np.std(_ball_patch(apply_image_depth_noise(
                frame, rng, correlation_px=corr_px)))) for _ in range(200)]))

    got = [median_spread(s) for s in (5.0, 7.0, 12.0, 20.0)]
    assert got == sorted(got, reverse=True), \
        f"longer correlation must hold the ball tighter, got {got}"
    # Roughly 1/s: quadrupling the correlation length shrinks the spread by
    # something close to 4x, not by nothing and not by orders of magnitude.
    assert 2.5 < got[0] / got[3] < 6.0


def _centroid_sigma(frame, z, corr_px, draws=400, seed=29):
    rng = np.random.default_rng(seed)
    errs = [float(np.mean(_ball_patch(apply_image_depth_noise(
        frame, rng, correlation_px=corr_px)))) - z for _ in range(draws)]
    return float(np.std(errs))


def test_centroid_carries_full_sigma_once_the_ball_fits_in_one_cell():
    """What the TRACKER sees, which is the only thing `kalman.py` consumes.

    Once the correlation length reaches the ball's ~5 px span the patch shares
    one disparity solution, so its centroid carries the FULL sigma(z) -- which
    is where RESULTS.md's 0.30 m spec and the oracle's own noise both live, and
    what the tca law was fitted against. Above that span the value is stable,
    so the modelling assumption in `correlation_px` does not leak into the
    measurement noise the filter is tuned for.
    """
    z = 26.0
    frame = _flat_wall(128, 128, z)
    for corr_px in (5.0, 7.0, 12.0, 20.0):
        assert _centroid_sigma(frame, z, corr_px) == pytest.approx(
            float(depth_sigma_m(z)), rel=0.25), \
            f"centroid sigma moved at correlation_px={corr_px}"


def test_independent_noise_averages_the_centroid_error_away():
    """The contrast that shows why the independent limit is not conservative.

    With uncorrelated returns the ball's 25 errors average down as 1/sqrt(N),
    so the centroid looks about five times MORE accurate than the 0.30 m the
    sensor is specified at. A lane built on white noise would therefore hand
    the tracker a better measurement than the real part delivers, while also
    smearing the cluster -- wrong in both directions at once.
    """
    z = 26.0
    frame = _flat_wall(128, 128, z)
    white = _centroid_sigma(frame, z, 0.0)
    assert white < 0.5 * float(depth_sigma_m(z))
    assert white < 0.5 * _centroid_sigma(frame, z, DEFAULT_CORRELATION_PX)


def test_large_surface_is_not_moved_as_one_rigid_sheet():
    """The counterpart to the ball contract.

    `apply_depth_noise` gives the WHOLE array one common draw, which is right
    for a ball-only cloud and wrong for a rendered scene: it would translate
    the ground plane coherently by ~sigma and blow up background differencing.
    A surface many correlation lengths across must instead break into
    independent cells.
    """
    cloud = _flat_wall(256, 256, 20.0)
    out = apply_image_depth_noise(cloud, np.random.default_rng(23))
    err = out[..., 2] - 20.0
    # Many cells across the frame => the frame-wide mean averages most of the
    # error away, while the per-pixel spread stays at full sigma.
    assert abs(float(np.mean(err))) < 0.5 * float(np.std(err))


# --- the cluster-extent gate -------------------------------------------------

def _apparent_extents(z, draws=1500, corr_px=DEFAULT_CORRELATION_PX):
    """Bounding-box side along the ray: the ball's body plus its depth noise.

    `cloud_geometry.cluster_extent` is the largest axis-aligned side of the
    cluster's bounding box, and at these ranges the noise runs along the ray,
    so the depth axis is the one that decides the gate.
    """
    # 64x64 is still many correlation lengths across, and this helper is
    # called seven times over; the larger frame only bought runtime.
    frame = _flat_wall(64, 64, z)
    rng = np.random.default_rng(202)
    out = []
    for _ in range(draws):
        patch = _ball_patch(apply_image_depth_noise(
            frame, rng, correlation_px=corr_px))
        out.append(float(patch.max() - patch.min()) + _BALL_EXTENT_M)
    return np.array(out)


def test_p2p_constant_matches_the_model():
    """`BALL_PATCH_P2P_SIGMAS` must stay a summary of the code, not a memory.

    It is range-independent by construction -- every term scales with
    sigma(z) -- so the same ratio has to come back at 13 m and at 26 m.
    """
    for z in (13.0, 26.0):
        p2p = _apparent_extents(z) - _BALL_EXTENT_M
        ratio = float(np.percentile(p2p, 99)) / float(depth_sigma_m(z))
        assert ratio == pytest.approx(BALL_PATCH_P2P_SIGMAS, rel=0.15)


def test_the_shipped_extent_gate_rejects_the_ball_at_long_range():
    """A FINDING, pinned so it cannot be rediscovered the expensive way.

    `cluster_max_extent_m: 0.35` was sized for a ball at <= 5 m, where sigma is
    a centimetre. At 26 m sigma is 0.30 m and the noise stretches the ball's
    bounding box along the ray, so a large minority of ball frames exceed the
    gate -- and `cluster_and_split` then re-clusters them at a tighter
    tolerance, fragmenting a 24-point ball below `cluster_min_points: 5`.

    Flying the long-range lane on the shipped gate would have produced a
    depressed save rate caused by a misconfigured threshold and read as a
    perception limit. Same failure class as `required_roi_max_range_m`.
    """
    tripped = float(np.mean(_apparent_extents(26.0) > _CLUSTER_MAX_EXTENT_M))
    assert tripped > 0.10, (
        "the shipped gate is expected to reject a large minority of ball "
        "frames at 26 m; if this no longer holds, the noise model or the "
        "shipped gate changed and the long-range lane must be re-derived")
    # And it is a LONG-RANGE problem only: at the reach the gate was sized for,
    # the same model sails through.
    assert float(np.mean(_apparent_extents(5.0) > _CLUSTER_MAX_EXTENT_M)) < 0.01


def test_required_gate_admits_the_ball_it_was_derived_for():
    """The fix, checked against the model it came from."""
    for z in (5.0, 13.0, 26.0):
        gate = required_cluster_max_extent_m(z)
        assert float(np.mean(_apparent_extents(z) > gate)) < 0.02
    # It must actually be a raise at long range, not a restatement of 0.35.
    assert required_cluster_max_extent_m(26.0) > _CLUSTER_MAX_EXTENT_M
    # ...and it must not balloon the gate at short range, where a loose gate
    # would start admitting background clutter the 0.35 m value keeps out.
    assert required_cluster_max_extent_m(5.0) < _CLUSTER_MAX_EXTENT_M


# --- which column is depth ---------------------------------------------------

def _gz_flu_ball_cloud(height, width, depth_m, span=_BALL_SPAN_PX,
                       fov_deg=27.0):
    """A REAL rendered cloud's shape: no-returns everywhere, a small boresight
    ball, all in Gazebo's sensor body frame (X-forward, Y-left, Z-up).

    The probe's actual cloud was 24 finite returns out of 520000, which is what
    makes the wrong-axis bug invisible: near boresight the body-frame Z is
    ~0, so `sigma(z)` collapses and the returns barely move. A full-frame wall
    does NOT reproduce this -- away from the centre row its body-frame Z is
    metres, and the bug does not show.
    """
    optical = _flat_wall(height, width, depth_m, fov_deg)
    body = np.stack([optical[..., 2], -optical[..., 0], -optical[..., 1]],
                    axis=-1)
    cloud = np.full((height, width, 3), np.inf)
    r0, c0 = (height - span) // 2, (width - span) // 2
    cloud[r0:r0 + span, c0:c0 + span] = body[r0:r0 + span, c0:c0 + span]
    return cloud


def _centroid_depth_sigma(cloud, axis, draws=300, seed=41):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(draws):
        noised = apply_image_depth_noise(cloud, rng, depth_axis=axis)
        finite = noised[np.isfinite(noised).all(axis=-1)]
        out.append(float(np.linalg.norm(finite.mean(axis=0))))
    return float(np.std(out))


def test_gz_flu_clouds_need_depth_axis_zero():
    """THE REGRESSION TEST. This bug shipped and no unit test above could see it.

    Gazebo's PointCloudPacked stays in the sensor body frame, where depth is X.
    Run with the optical default, a boresight ball's body-frame Z is ~0, so
    sigma(z) collapses to nothing and the returns barely move: the cloud comes
    back all but unchanged and the lane publishes a NOISELESS sensor.
    `run_noise_probe.sh` measured exactly that -- centroid range std 0.0000 m
    at sigma_ref_m 0.30, centroid at (+25.98, -0.00, +0.00), p2p 0.0002 m.

    Every test above passed throughout, because they all build their clouds
    with the same z-is-depth assumption the code had.
    """
    z = 26.0
    cloud = _gz_flu_ball_cloud(64, 64, z)
    expected = float(depth_sigma_m(z))

    wrong = _centroid_depth_sigma(cloud, axis=2)
    assert wrong < 0.01 * expected, (
        f"reading depth off the wrong axis must collapse the noise to "
        f"nothing; got {wrong:.5f} m against sigma {expected:.3f} m")

    right = _centroid_depth_sigma(cloud, axis=0)
    assert right == pytest.approx(expected, rel=0.25), (
        f"depth_axis=0 must deliver the calibrated sigma; got {right:.4f} m")


def test_the_convention_map_matches_the_detector_vocabulary():
    """Same two names `detector.yaml` already uses, so there is one concept."""
    assert DEPTH_AXIS_BY_CONVENTION == {"gz_flu": 0, "optical": 2}


def test_a_bad_depth_axis_is_refused():
    with pytest.raises(ValueError, match="depth_axis"):
        apply_image_depth_noise(
            _flat_wall(8, 8, 5.0), np.random.default_rng(0), depth_axis=3)


# --- no-return pixels --------------------------------------------------------

def test_non_finite_returns_pass_through_untouched():
    """A depth camera reports "no return" as NaN or +/-inf. Those must survive
    as non-finite: turning one into a number would fabricate a return, and
    `count_ball_points.py` already had to learn that isfinite is the only test
    that catches both.
    """
    cloud = _flat_wall(16, 16, 15.0)
    cloud[2, 3] = np.nan
    cloud[4, 5] = np.inf
    cloud[6, 7] = -np.inf
    out = apply_image_depth_noise(cloud, np.random.default_rng(31))
    assert not np.isfinite(out[2, 3]).any()
    assert not np.isfinite(out[4, 5]).any()
    assert not np.isfinite(out[6, 7]).any()
    finite = np.isfinite(cloud).all(axis=-1)
    assert np.isfinite(out[finite]).all(), "valid returns must stay valid"


def test_non_finite_returns_do_not_poison_their_neighbours():
    """NaN must not spread through the smoothing into the whole frame."""
    cloud = _flat_wall(64, 64, 15.0)
    cloud[32, 32] = np.nan
    out = apply_image_depth_noise(cloud, np.random.default_rng(37))
    neighbours = np.isfinite(out[28:37, 28:37]).all(axis=-1)
    assert neighbours.sum() >= 80, "only the one bad pixel may be non-finite"


# --- shape contract ----------------------------------------------------------

def test_unorganized_cloud_is_refused():
    """The correlation lives in IMAGE space. Handed a flat (N,3) list there is
    no neighbourhood to correlate over, and silently falling back to white
    noise would smear the ball while still reporting a plausible sigma -- a
    wrong instrument that still returns numbers. Refuse instead.
    """
    with pytest.raises(ValueError, match="organized"):
        apply_image_depth_noise(
            np.zeros((100, 3)), np.random.default_rng(0))


def test_default_correlation_length_is_documented_and_sane():
    assert 4.0 <= DEFAULT_CORRELATION_PX <= 16.0
