"""
mono_flash.py — find a fast small object in a stereo pair of mono frames.
Pure numpy/scipy, no ROS (the cloud_geometry.py pattern).

THE PROBLEM THIS EXISTS TO SOLVE. The envelope closes at 14 m/s because of
sensing, not thinking or moving: the depth pipeline first reports the 80 mm
ball at ~3.2-3.6 m, and at 20 m/s that is 0.17 s of warning -- less than
confirmation plus pipeline, before the airframe is asked to move at all.
Widening range gates cannot help; it was measured twice, and both times the
answer was that an 80 mm sphere does not project enough points at 6-8 m to
clear cluster_min_points. Dense stereo spends its whole budget computing depth
for the 99.99% of the frame that is not the ball, and quantises the ball's
disparity to whatever the block matcher can resolve.

The mono pair is a different instrument aimed at the same target. The OAK-D
Lite's two OV7251 sensors stream 640x480 grayscale far faster than the depth
node produces clouds, and a ball is trivially separable in the TIME domain: it
is the only thing in the frame that moves several pixels between consecutive
frames. So:

    difference two frames -> a handful of pixels -> centroid to sub-pixel
    -> match left against right on the epipolar line -> triangulate

Two properties matter for the reaction budget:
  * RATE. Confirmation costs min_track_updates frames. At 14.5 Hz that is
    0.21 s; at 90 fps it is 0.033 s. The most expensive term in the budget is
    divided by six without weakening the confirmation rule at all.
  * RANGE. A sub-pixel centroid needs a few bright pixels, not a resolvable
    disk. Detection range stops being set by "how many points survive
    clustering" and starts being set by contrast against the background.

WHAT THIS MODULE DOES NOT CLAIM. That the real device delivers those frame
rates over USB, or that a real ball is separable against a real cluttered
background at 10 m. Both are Week 6 measurements. Everything here is written
so those measurements can be made, and every default below is a NOMINAL value
to be replaced by the device's own calibration.

Frame convention: pixel coordinates are (x right, y down) as they come off
the sensor, and triangulate() returns a point in the CAMERA OPTICAL frame
(x right, y down, z forward), which is what REP-103 and the existing
camera_optical_frame TF already expect.
"""

from __future__ import annotations

from typing import NamedTuple, Optional, Sequence

import numpy as np
from scipy import ndimage

# --- NOMINAL OAK-D Lite mono geometry. REPLACE WITH THE DEVICE'S OWN --------
# calibration before trusting a range: fx here is derived from a datasheet FOV
# (~71.9 deg horizontal at 640x480), and a few percent of focal-length error is
# a few percent of every range this module reports. depthai exposes the factory
# calibration through CalibrationHandler; read it at bring-up.
DEFAULT_FX_PX = 441.7
DEFAULT_BASELINE_M = 0.075


class Blob(NamedTuple):
    x: float        # sub-pixel centroid, pixels right
    y: float        # sub-pixel centroid, pixels down
    area: int       # pixels above threshold
    weight: float   # summed intensity change: the "how sure" term


class StereoMatch(NamedTuple):
    left: Blob
    right: Blob
    disparity: float


def temporal_difference(prev, cur, *, threshold: float) -> np.ndarray:
    """Pixels that changed by more than `threshold` between two frames.

    Absolute difference, not signed: a dark ball against a bright background
    is exactly as detectable as the reverse, and which one you get depends on
    the sky that day.

    This is the cheapest possible detector, and that is the point -- one
    subtract and one compare over the frame, which is what makes a 90 fps
    budget plausible on a Pi 5. It also has an honest failure mode: it finds
    MOTION, not balls. Everything downstream (the size gates, epipolar
    matching, the tracker's ballistic model) exists to tell a ball from the
    other things that move.
    """
    prev = np.asarray(prev, dtype=np.int16)
    cur = np.asarray(cur, dtype=np.int16)
    if prev.shape != cur.shape:
        raise ValueError("frame shapes differ: %r vs %r" % (prev.shape, cur.shape))
    return np.abs(cur - prev) > float(threshold)


def extract_blobs(mask, *, min_area: int = 3, max_area: int = 400,
                  intensity=None) -> list[Blob]:
    """Connected components of `mask`, as sub-pixel centroids.

    Sub-pixel accuracy matters more than it looks. Range error is proportional
    to disparity error, so half a pixel of centroid accuracy is worth about as
    much as doubling the baseline. Weighting by intensity change (when
    supplied) puts the centroid at the middle of the blob's energy rather than
    the middle of its thresholded outline, which moves with the threshold.

    max_area is the discriminator that does the most work in the real world: a
    ball at range is a few pixels, while a person walking, a shadow, or the
    whole frame shifting because the drone rolled are all large. A blob the
    size of the frame is the drone's own motion, not a threat.
    """
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return []

    labels, n = ndimage.label(mask)
    if n == 0:
        return []

    idx = np.arange(1, n + 1)
    areas = ndimage.sum_labels(np.ones_like(labels, dtype=np.int32),
                               labels, index=idx)

    if intensity is None:
        weights = np.ones(mask.shape, dtype=np.float64)
    else:
        weights = np.abs(np.asarray(intensity, dtype=np.float64))
        if weights.shape != mask.shape:
            raise ValueError("intensity shape %r does not match mask %r"
                             % (weights.shape, mask.shape))
        # A component whose weights are all zero would give a 0/0 centroid.
        weights = weights + 1e-9

    centroids = ndimage.center_of_mass(weights, labels, idx)
    sums = ndimage.sum_labels(weights, labels, index=idx)

    out = []
    for (cy, cx), area, w in zip(centroids, areas, sums):
        if min_area <= area <= max_area:
            # center_of_mass returns (row, col) = (y, x).
            out.append(Blob(x=float(cx), y=float(cy), area=int(area),
                            weight=float(w)))
    # Brightest first: the caller usually wants the most confident candidate,
    # and ties must break deterministically for a battery to be replayable.
    out.sort(key=lambda b: (-b.weight, b.x, b.y))
    return out


def match_stereo(
    left: Sequence[Blob],
    right: Sequence[Blob],
    *,
    max_dy_px: float = 2.0,
    min_disparity_px: float = 0.5,
    max_disparity_px: float = 300.0,
) -> list[StereoMatch]:
    """Pair left blobs with right blobs across a RECTIFIED pair.

    Rectification is the assumption that makes this cheap: the same world
    point lands on the same image row in both cameras, so a candidate pair
    that disagrees vertically by more than max_dy_px is not the same object
    and needs no further thought. depthai rectifies on the Myriad X, but the
    assumption must be CHECKED on real frames -- an unrectified pair still
    produces matches, just wrong ones, and they look exactly like a ball at a
    plausible range.

    min_disparity_px is a range ceiling in disguise: disparity goes to zero at
    infinity, so accepting tiny disparities means accepting enormous ranges
    computed from rounding error.

    Greedy by vertical agreement, one use per blob. There are only ever a
    handful of candidates, so nothing more clever is warranted.
    """
    pairs = []
    for i, lb in enumerate(left):
        for j, rb in enumerate(right):
            disparity = lb.x - rb.x
            if not (min_disparity_px <= disparity <= max_disparity_px):
                continue
            dy = abs(lb.y - rb.y)
            if dy > max_dy_px:
                continue
            pairs.append((dy, i, j, disparity))

    pairs.sort()
    used_l: set[int] = set()
    used_r: set[int] = set()
    out = []
    for _dy, i, j, disparity in pairs:
        if i in used_l or j in used_r:
            continue
        used_l.add(i)
        used_r.add(j)
        out.append(StereoMatch(left=left[i], right=right[j],
                               disparity=float(disparity)))
    return out


def triangulate(
    match: StereoMatch,
    *,
    cx: float,
    cy: float,
    fx: float = DEFAULT_FX_PX,
    baseline_m: float = DEFAULT_BASELINE_M,
) -> Optional[np.ndarray]:
    """3D point in the camera optical frame, or None if degenerate.

        Z = fx * B / d,  X = (xl - cx) * Z / fx,  Y = (yl - cy) * Z / fx

    None rather than an exception for zero disparity: it means "at infinity",
    which is a normal thing for a mismatched pair to imply, not an error.
    """
    d = float(match.disparity)
    if d <= 0.0:
        return None
    z = fx * baseline_m / d
    x = (match.left.x - cx) * z / fx
    y = (match.left.y - cy) * z / fx
    return np.array([x, y, z], dtype=np.float64)


def triangulation_covariance(
    point,
    *,
    fx: float = DEFAULT_FX_PX,
    baseline_m: float = DEFAULT_BASELINE_M,
    centroid_std_px: float = 0.3,
) -> np.ndarray:
    """3x3 measurement covariance in the camera optical frame.

    This is why the tracker grew a per-measurement R. Stereo error is
    profoundly anisotropic, and the asymmetry grows with range:

        sigma_X = sigma_Y = Z * sigma_px / fx          (linear in Z)
        sigma_Z = Z^2 * sigma_disp / (fx * B)          (QUADRATIC in Z)

    At 10 m with these nominal numbers that is ~7 mm across and ~1.0 m deep --
    a factor of about 140. An isotropic R either throws away a bearing
    measurement good to a centimetre, or believes a range estimate good only
    to a metre.

    sigma_disp is sqrt(2) * centroid_std_px: disparity is a difference of two
    independently estimated centroids, so their errors add in quadrature.
    """
    p = np.asarray(point, dtype=np.float64).reshape(3)
    z = float(p[2])
    sigma_lat = z * centroid_std_px / fx
    sigma_disp = float(np.sqrt(2.0)) * centroid_std_px
    sigma_depth = z * z * sigma_disp / (fx * baseline_m)
    return np.diag([sigma_lat ** 2, sigma_lat ** 2, sigma_depth ** 2])


def max_useful_range_m(
    *,
    fx: float = DEFAULT_FX_PX,
    baseline_m: float = DEFAULT_BASELINE_M,
    centroid_std_px: float = 0.3,
    max_depth_std_m: float = 1.0,
) -> float:
    """Range at which depth uncertainty reaches max_depth_std_m.

    Inverting the sigma_Z expression above. This is the number that decides
    whether the mono-pair approach can reach 20 m/s at all, so it is worth
    being able to compute rather than guess: with the nominal geometry and a
    1 m depth tolerance it lands near 10 m, the same order as the ~9.6 m the
    timing budget demands. That closeness is exactly why the real calibration
    and the real centroid noise must be measured rather than assumed -- the
    margin is thin enough that a 30% error in either decides the answer.
    """
    sigma_disp = float(np.sqrt(2.0)) * centroid_std_px
    return float(np.sqrt(max_depth_std_m * fx * baseline_m / sigma_disp))
