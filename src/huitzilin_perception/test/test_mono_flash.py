"""Mono-pair flash detection, on synthetic frames.

The whole approach rests on claims that can be checked without hardware: that
a small fast object survives differencing, that a sub-pixel centroid really is
sub-pixel, and that the triangulation inverts the projection it claims to
invert. If any of those is wrong, no amount of tuning on real bags will
rescue it -- and a wrong triangulation in particular produces a confident,
plausible, entirely fictional range.

The numbers that decide whether this reaches 20 m/s (delivered frame rate,
detection range against a real background) are hardware measurements. These
tests pin the maths those measurements will be interpreted with.
"""

import numpy as np
import pytest

from huitzilin_perception.mono_flash import (
    DEFAULT_BASELINE_M,
    DEFAULT_FX_PX,
    Blob,
    StereoMatch,
    extract_blobs,
    match_stereo,
    max_useful_range_m,
    temporal_difference,
    triangulate,
    triangulation_covariance,
)

CX, CY = 320.0, 240.0


def _frame(value=40, shape=(480, 640)):
    return np.full(shape, value, dtype=np.uint8)


def _disc(frame, cx, cy, radius=2.0, value=200):
    """Paint a filled disc; returns a copy so callers keep the original."""
    out = frame.copy()
    yy, xx = np.ogrid[:frame.shape[0], :frame.shape[1]]
    out[(xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2] = value
    return out


# --- temporal differencing ---------------------------------------------------

def test_a_moved_object_survives_differencing():
    prev = _disc(_frame(), 100, 240)
    cur = _disc(_frame(), 140, 240)
    mask = temporal_difference(prev, cur, threshold=30)
    assert mask.any()
    # Both the old and the new position light up: the object left one and
    # arrived at the other. Two blobs is the expected, correct output.
    assert len(extract_blobs(mask, max_area=500)) == 2


def test_a_static_scene_cancels_completely():
    """The property the whole method depends on. A background that does not
    cancel means every frame is full of candidates."""
    frame = _disc(_frame(), 300, 200)
    assert not temporal_difference(frame, frame, threshold=10).any()


def test_differencing_is_symmetric_in_contrast():
    """A dark ball on a bright sky must be as detectable as the reverse; which
    one you get is a property of the weather, not of the algorithm."""
    bright_on_dark = temporal_difference(_frame(20), _disc(_frame(20), 100, 100),
                                         threshold=30)
    dark_on_bright = temporal_difference(_frame(220),
                                         _disc(_frame(220), 100, 100, value=20),
                                         threshold=30)
    assert bright_on_dark.sum() == dark_on_bright.sum()


def test_sensor_noise_below_the_threshold_is_rejected():
    rng = np.random.default_rng(4)
    prev = _frame(120)
    cur = np.clip(prev + rng.normal(0, 3, prev.shape), 0, 255).astype(np.uint8)
    assert not temporal_difference(prev, cur, threshold=30).any()


def test_mismatched_frame_shapes_raise_rather_than_broadcast():
    """numpy would happily broadcast some shape pairs into a meaningless
    difference; a dropped or resized frame must be loud."""
    with pytest.raises(ValueError):
        temporal_difference(_frame(shape=(480, 640)), _frame(shape=(240, 320)),
                            threshold=10)


# --- blob extraction ---------------------------------------------------------

def test_the_centroid_is_sub_pixel():
    """Half a pixel of centroid accuracy is worth about as much as doubling
    the baseline, so 'roughly the right pixel' is not good enough."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[20, 20:22] = True          # two pixels: the centroid must be 20.5
    (blob,) = extract_blobs(mask, min_area=2)
    assert blob.x == pytest.approx(20.5)
    assert blob.y == pytest.approx(20.0)


def test_the_centroid_follows_the_energy_when_weighted():
    """Weighting moves the centroid to the middle of the blob's energy rather
    than the middle of its thresholded outline, which moves with threshold."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[20, 20:23] = True
    intensity = np.zeros((40, 40), dtype=np.float64)
    intensity[20, 20] = 100.0       # nearly all the energy at the left pixel
    intensity[20, 21] = 1.0
    intensity[20, 22] = 1.0
    (blob,) = extract_blobs(mask, min_area=3, intensity=intensity)
    assert blob.x < 20.5


def test_a_blob_larger_than_max_area_is_rejected():
    """The discriminator that does the most real-world work: a person, a
    shadow, or the whole frame shifting because the drone rolled are all
    large. A ball at range is a few pixels."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[5:35, 5:35] = True
    assert extract_blobs(mask, max_area=100) == []


def test_a_blob_smaller_than_min_area_is_rejected():
    mask = np.zeros((40, 40), dtype=bool)
    mask[10, 10] = True
    assert extract_blobs(mask, min_area=3) == []


def test_separate_objects_stay_separate():
    mask = np.zeros((40, 40), dtype=bool)
    mask[10, 10:13] = True
    mask[30, 30:33] = True
    assert len(extract_blobs(mask, min_area=3)) == 2


def test_an_empty_mask_yields_no_blobs():
    assert extract_blobs(np.zeros((40, 40), dtype=bool)) == []


def test_blob_order_is_deterministic():
    """A battery must replay identically; ties broken by iteration order would
    make the 'best candidate' vary run to run."""
    mask = np.zeros((40, 40), dtype=bool)
    mask[10, 10:13] = True
    mask[30, 30:33] = True
    assert extract_blobs(mask, min_area=3) == extract_blobs(mask, min_area=3)


# --- stereo matching ---------------------------------------------------------

def _blob(x, y, area=5, weight=5.0):
    return Blob(x=x, y=y, area=area, weight=weight)


def test_a_rectified_pair_matches_on_the_same_row():
    matches = match_stereo([_blob(300.0, 240.0)], [_blob(280.0, 240.2)])
    assert len(matches) == 1
    assert matches[0].disparity == pytest.approx(20.0)


def test_an_epipolar_violation_is_refused():
    """The check that catches an unrectified pair. Without it two unrelated
    objects match and triangulate to a confident, fictional range."""
    assert match_stereo([_blob(300.0, 100.0)], [_blob(280.0, 300.0)]) == []


def test_negative_disparity_is_refused():
    """A right-image blob to the RIGHT of its left-image partner is
    geometrically impossible for a forward-facing rectified pair."""
    assert match_stereo([_blob(280.0, 240.0)], [_blob(300.0, 240.0)]) == []


def test_a_vanishing_disparity_is_refused():
    """Disparity goes to zero at infinity, so accepting tiny values means
    accepting enormous ranges computed from rounding error."""
    assert match_stereo([_blob(300.0, 240.0)], [_blob(299.9, 240.0)],
                        min_disparity_px=0.5) == []


def test_each_blob_is_used_at_most_once():
    left = [_blob(300.0, 240.0), _blob(302.0, 240.0)]
    right = [_blob(280.0, 240.0)]
    assert len(match_stereo(left, right)) == 1


def test_the_closest_epipolar_agreement_wins():
    left = [_blob(300.0, 240.0)]
    right = [_blob(280.0, 241.5), _blob(281.0, 240.05)]
    (match,) = match_stereo(left, right)
    assert match.right.x == pytest.approx(281.0)


# --- triangulation -----------------------------------------------------------

def test_triangulation_round_trips_an_exactly_projected_point():
    """The load-bearing check. Project a known 3D point into both cameras by
    hand, then recover it."""
    truth = np.array([0.4, -0.2, 6.0])
    xl = CX + DEFAULT_FX_PX * truth[0] / truth[2]
    xr = CX + DEFAULT_FX_PX * (truth[0] - DEFAULT_BASELINE_M) / truth[2]
    y = CY + DEFAULT_FX_PX * truth[1] / truth[2]

    match = StereoMatch(left=_blob(xl, y), right=_blob(xr, y),
                        disparity=xl - xr)
    got = triangulate(match, cx=CX, cy=CY)
    np.testing.assert_allclose(got, truth, rtol=1e-9)


def test_a_larger_disparity_means_a_closer_object():
    near = triangulate(StereoMatch(_blob(320.0, 240.0), _blob(220.0, 240.0), 100.0),
                       cx=CX, cy=CY)
    far = triangulate(StereoMatch(_blob(320.0, 240.0), _blob(310.0, 240.0), 10.0),
                      cx=CX, cy=CY)
    assert near[2] < far[2]


def test_zero_disparity_returns_none_rather_than_a_point_at_infinity():
    """inf would propagate into the tracker and poison every state it
    touches; None is a normal 'no measurement' the caller already handles."""
    assert triangulate(StereoMatch(_blob(300.0, 240.0), _blob(300.0, 240.0), 0.0),
                       cx=CX, cy=CY) is None


# --- covariance --------------------------------------------------------------

def test_depth_uncertainty_grows_quadratically_and_lateral_linearly():
    """The asymmetry the tracker's per-measurement R exists to represent.
    Doubling the range doubles the lateral sigma and QUADRUPLES the depth
    sigma."""
    near = triangulation_covariance([0.0, 0.0, 5.0])
    far = triangulation_covariance([0.0, 0.0, 10.0])
    assert np.sqrt(far[0, 0] / near[0, 0]) == pytest.approx(2.0)
    assert np.sqrt(far[2, 2] / near[2, 2]) == pytest.approx(4.0)


def test_depth_is_far_less_certain_than_bearing_at_range():
    """At 10 m the same detection is centimetres across and metres deep. An
    isotropic R would either discard the good axis or trust the bad one."""
    cov = triangulation_covariance([0.0, 0.0, 10.0])
    assert cov[2, 2] > 100.0 * cov[0, 0]


def test_the_covariance_is_usable_as_a_kalman_measurement_matrix():
    cov = triangulation_covariance([0.1, 0.1, 8.0])
    assert cov.shape == (3, 3)
    assert np.linalg.det(cov) > 0.0
    np.testing.assert_allclose(cov, cov.T)


def test_a_wider_baseline_buys_depth_accuracy():
    """The one geometric lever available if range falls short: sigma_Z is
    inversely proportional to the baseline."""
    narrow = triangulation_covariance([0.0, 0.0, 10.0], baseline_m=0.075)
    wide = triangulation_covariance([0.0, 0.0, 10.0], baseline_m=0.150)
    assert wide[2, 2] == pytest.approx(narrow[2, 2] / 4.0)


# --- the range budget --------------------------------------------------------

def test_the_nominal_geometry_lands_near_the_range_the_budget_demands():
    """~9.6 m is what 20 m/s needs (0.08 s pipeline + ~0.40 s to clear the hit
    radius). This asserts the ORDER, not a promise: the real answer needs the
    device's calibration and its real centroid noise."""
    assert 8.0 < max_useful_range_m() < 13.0


def test_better_centroids_buy_range():
    assert max_useful_range_m(centroid_std_px=0.15) > max_useful_range_m(
        centroid_std_px=0.30)


def test_accepting_looser_depth_buys_range():
    assert max_useful_range_m(max_depth_std_m=2.0) > max_useful_range_m(
        max_depth_std_m=1.0)
