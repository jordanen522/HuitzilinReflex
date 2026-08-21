"""depth_noise.py — stereo depth error for a RENDERED cloud. Pure numpy/scipy.

WHAT THIS IS FOR. `scripts/optics_probe` established that a rendered Gazebo
depth camera at the proposed AR0234 optics resolves the 80 mm ball at 26 m
(24 returns at 800x650 / 27.0 deg, against `cluster_min_points: 5`). That
unblocks a lane whose GEOMETRY is genuinely rendered rather than fabricated by
`synthetic_depth.py`. But a rendered depth camera reports very nearly exact
geometry -- `iris_depth` declares a 1 cm per-pixel gaussian, the probe world
none at all -- where a real stereo pair is off by 0.30 m at 26 m, and
correlated rather than independent. Scoring the rendered lane as it comes out
of Gazebo would make the simulated sensor strictly better than the $89 part it
stands for, and every save rate taken through it would be an overstatement.

This module supplies the missing error, so the pipeline is:

    Gazebo depth camera  ->  ros_gz_bridge  ->  THIS  ->  detector_node
    (real geometry)                             (real error)   (unmodified)

WHY NOT JUST CALL `synthetic_depth.apply_depth_noise`. That function draws ONE
common-mode value for the whole array. That is right for the cloud it was built
for -- a fabricated ball and nothing else, where one disparity solution covers
the single patch. A rendered cloud is a whole scene: the same call would
translate the ground plane coherently by ~sigma every frame, which is not what
a stereo pair does and would wreck the detector's background differencing.

THE MODEL. Stereo disparity error is correlated over the matcher's support --
roughly its window, plus whatever the aggregation step spreads it across. So
the error field is generated in IMAGE space with a correlation length of a few
pixels. Both limits then fall out of one parameter instead of being chosen:

  * The ball spans ~5 px at 26 m, well inside one correlation cell, so it
    receives essentially ONE disparity solution: its 80 mm extent survives and
    the error lands on its centroid -- which is where RESULTS.md's 0.30 m spec
    and the oracle's own noise both live.
  * The ground plane spans hundreds of cells, so it decorrelates across the
    frame instead of moving as a rigid sheet.

`correlation_px` IS A REAL PARAMETER, NOT A FREE ONE. An earlier draft of this
docstring claimed the ball outcome was insensitive to it. That is false and the
measurement is in `test_spread_tightens_monotonically_with_correlation_length`:
median within-ball depth spread at 26 m runs 0.068 / 0.050 / 0.029 / 0.016 m
for correlation lengths of 5 / 7 / 12 / 20 px, i.e. roughly as 1/s. Raising it
makes the ball hold together better, so it cannot be tuned toward whatever
score is wanted. It is set from the matcher's support window and stated as a
modelling assumption, with that sensitivity on the record.

What IS stable is the quantity the tca law actually consumes, PROVIDED the
correlation length covers the ball: once one cell spans its ~5 px the patch
shares a single disparity solution and its CENTROID carries the full sigma(z),
unchanged from 5 px to 20 px. Below that it is not stable at all -- in the
white-noise limit the ball's returns average down as 1/sqrt(N) and the centroid
looks ~5x MORE accurate than the sensor is specified at, which is why the
independent limit is not the conservative choice it appears to be. Both are
pinned in `test_depth_noise.py`.

The sigma law itself is NOT re-derived here: `depth_sigma_m` is imported from
`synthetic_depth`, so both lanes are calibrated to the same measured 0.30 m at
26 m and cannot drift apart.

WHAT IS NOT MODELLED: matcher dropout on a textureless sphere, stereo
shadowing, reflectance, or the correlation length's dependence on texture. The
error here is unbiased and always present; a real matcher sometimes returns
nothing at all. That bounds what a result taken through this lane may claim,
and it is the same caveat `synthetic_depth` carries.
"""

from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from scipy.ndimage import gaussian_filter

from huitzilin_perception.synthetic_depth import (
    DEFAULT_DEPTH_SIGMA_M,
    DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    depth_sigma_m,
)

__all__ = [
    "BALL_PATCH_P2P_SIGMAS",
    "DEPTH_AXIS_BY_CONVENTION",
    "DEFAULT_BALL_EXTENT_M",
    "DEFAULT_CORRELATION_PX",
    "apply_image_depth_noise",
    "correlated_unit_field",
    "required_cluster_max_extent_m",
]

# Correlation length of the disparity error, in pixels. Sized to a stereo
# matcher's support window (OAK-D's census/SGBM family aggregates over roughly
# this scale). The precise value is deliberately not load-bearing: anything at
# or above the ball's ~5 px span at 26 m yields the same common-mode outcome,
# which `test_ball_sized_patch_is_insensitive_to_the_correlation_length` pins.
DEFAULT_CORRELATION_PX = 7.0

# scipy's Gaussian kernel is truncated at this many sigmas. Repeated here only
# so the normalisation probe below uses the same support as the real filter.
_TRUNCATE_SIGMAS = 4.0

# `mode="wrap"` makes the field statistically STATIONARY: with any other
# boundary rule the variance sags near the edges, so the sensor would be
# quietly more accurate at the frame border than at its centre.
_BOUNDARY_MODE = "wrap"


DEFAULT_BALL_EXTENT_M = 0.080   # the projectile model's diameter

# Peak-to-peak depth spread across the ball's returns, in units of sigma(z),
# at DEFAULT_CORRELATION_PX over a ~5 px patch. MEASURED from this model and
# range-INDEPENDENT by construction -- everything scales with sigma(z), so the
# same 0.592 / 1.215 / 1.522 (median / p95 / p99) come back at 13 m and at
# 26 m. `test_p2p_constant_matches_the_model` recomputes it, so it cannot
# silently drift away from the code it summarises. The p99 value is used, with
# rounding headroom, so roughly one ball frame in a hundred exceeds the gate.
BALL_PATCH_P2P_SIGMAS = 1.6


def required_cluster_max_extent_m(
    detection_range_m: float,
    *,
    ball_extent_m: float = DEFAULT_BALL_EXTENT_M,
    sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
    ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    n_sigma: float = BALL_PATCH_P2P_SIGMAS,
) -> float:
    """Smallest `cluster_max_extent_m` that does not reject the ball itself.

    The shipped 0.35 m was sized for a ball at <= 5 m, where sigma collapses to
    a centimetre. At 26 m sigma is 0.30 m and the depth noise stretches the
    ball's bounding box along the ray: apparent extent runs a median 0.26 m and
    a p95 of 0.45 m, so **20 % of ball frames trip the shipped gate**. Those
    are not discarded quietly either -- `cloud_geometry.cluster_and_split`
    re-clusters an oversized cluster at a tighter tolerance, which on a
    24-point ball fragments it below `cluster_min_points`.

    This is the same failure class as `required_roi_max_range_m`: a gate sized
    at short range silently delivers a worse sensor than the cell's label
    claims, and the result reads as a perception limit rather than a
    misconfiguration. 26 m needs 0.56 m; 5 m needs 0.10 m.
    """
    return float(ball_extent_m) + float(n_sigma) * float(
        depth_sigma_m(detection_range_m, sigma_ref_m, ref_range_m))


@lru_cache(maxsize=32)
def _unit_variance_scale(correlation_px: float, height: int, width: int) -> float:
    """Exact factor restoring unit variance after smoothing white noise.

    Measured from the filter itself rather than assumed from the continuum
    formula 2*sqrt(pi)*sigma. For circular convolution the output variance is
    the sum of the squared point-spread function, so filtering a delta and
    summing its square gives the answer exactly -- at any array size, including
    ones smaller than the kernel, where the continuum formula is badly wrong.

    Getting this wrong would be the worst kind of error here: raising
    `correlation_px` to make the ball coherent would ALSO shrink sigma, i.e.
    make the sensor more accurate, in the direction that flatters the result.
    """
    delta = np.zeros((height, width), dtype=np.float64)
    delta[0, 0] = 1.0
    psf = gaussian_filter(delta, sigma=correlation_px, mode=_BOUNDARY_MODE,
                          truncate=_TRUNCATE_SIGMAS)
    power = float(np.sum(psf ** 2))
    if power <= 0.0:  # pragma: no cover - only on a degenerate filter
        raise ValueError(
            "smoothing annihilated the field at correlation_px=%r; no scale "
            "restores unit variance" % (correlation_px,))
    return 1.0 / math.sqrt(power)


def correlated_unit_field(height: int,
                          width: int,
                          rng_gen: np.random.Generator,
                          correlation_px: float = DEFAULT_CORRELATION_PX,
                          ) -> np.ndarray:
    """A zero-mean, unit-variance Gaussian field correlated over `correlation_px`.

    `correlation_px <= 0` returns white noise, which is the fully-independent
    limit -- the one `synthetic_depth` shows smears the 26 m ball into a ~1.5 m
    cigar. It exists so that limit stays reachable and testable, not because it
    is a sensible sensor model.
    """
    if height <= 0 or width <= 0:
        raise ValueError("field needs a positive shape, got %rx%r"
                         % (height, width))
    raw = rng_gen.standard_normal((height, width))
    if correlation_px <= 0.0:
        return raw
    smoothed = gaussian_filter(raw, sigma=float(correlation_px),
                               mode=_BOUNDARY_MODE, truncate=_TRUNCATE_SIGMAS)
    return smoothed * _unit_variance_scale(
        float(correlation_px), int(height), int(width))


# Which column of an (H, W, 3) cloud holds depth, by the same vocabulary
# `detector.yaml` already uses for `cloud_convention`. Gazebo's
# PointCloudPacked stays in the sensor BODY frame, so the optical axis is X;
# a real OAK-D via DepthAI delivers Z-forward optical points.
DEPTH_AXIS_BY_CONVENTION = {"gz_flu": 0, "optical": 2}


def apply_image_depth_noise(points_hw3,
                            rng_gen: np.random.Generator,
                            *,
                            sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
                            ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
                            correlation_px: float = DEFAULT_CORRELATION_PX,
                            depth_axis: int = 2,
                            ) -> np.ndarray:
    """Perturb an ORGANIZED (H, W, 3) camera-frame cloud. Returns a new array.

    The perturbation is a DISPARITY error: depth moves by dz, and the other two
    axes follow proportionally, because x = z*(u-cx)/f. The point therefore
    slides along its own ray and its bearing is untouched -- a stereo pair is
    wrong about range, not about direction.

    `depth_axis` SAYS WHICH COLUMN IS DEPTH, and getting it wrong is silent.
    The default 2 is the optical convention (Z-forward). Gazebo's
    PointCloudPacked is NOT that: it stays in the sensor body frame (X-forward,
    Y-left, Z-up), where depth is column 0 -- `detector.yaml` handles the same
    thing with `cloud_convention: "gz_flu"`. Pointed at the wrong column, a
    boresight ball reads a depth of ~0, every return is skipped as unusable,
    and the cloud is republished untouched: a sensor with no error at all,
    reported as a clean run. That happened, and `run_noise_probe.sh` is what
    caught it -- no unit test on a synthetic array could, because the array
    was built with the same assumption as the code.

    Non-finite returns (a depth camera reports "no return" as NaN or as +/-inf,
    and both occur) are passed through EXACTLY. Turning one into a number would
    fabricate a return the sensor never made.

    Zero sigma reproduces the input exactly and consumes no randomness, so a
    noiseless control arm is genuinely noiseless -- the same contract as
    `synthetic_depth.apply_depth_noise` and `oracle.apply_noise`.
    """
    pts = np.asarray(points_hw3, dtype=np.float64)
    if pts.ndim != 3 or pts.shape[2] != 3:
        raise ValueError(
            "apply_image_depth_noise needs an organized (H, W, 3) cloud; got "
            "shape %r. The correlation lives in image space, so a flat (N, 3) "
            "list has no neighbourhood to correlate over -- falling back to "
            "white noise there would smear the ball while still reporting a "
            "plausible sigma." % (pts.shape,))
    if sigma_ref_m <= 0.0:
        return pts.copy()

    if depth_axis not in (0, 1, 2):
        raise ValueError("depth_axis must be 0, 1 or 2, got %r" % (depth_axis,))
    height, width, _ = pts.shape
    field = correlated_unit_field(height, width, rng_gen, correlation_px)

    depth = pts[:, :, depth_axis]
    # A point is usable only if all three coordinates are finite AND it has a
    # depth to scale: z == 0 has no ray, and dividing by it would manufacture
    # an infinity out of a legitimate return.
    with np.errstate(invalid="ignore"):
        usable = np.isfinite(pts).all(axis=2) & (depth != 0.0)

    out = pts.copy()
    if not usable.any():
        return out
    z = depth[usable]
    sigma = depth_sigma_m(np.abs(z), sigma_ref_m, ref_range_m)
    scale = 1.0 + (sigma * field[usable]) / z
    out[usable] = pts[usable] * scale[:, None]
    return out
