"""synthetic_depth.py — a synthetic depth cloud from Gazebo truth. Pure numpy.

WHAT THIS IS, EXACTLY. It fabricates the point cloud a depth sensor of a stated
optics would return for the ball, from ground truth, so that the REAL,
unmodified `detector_node.py` runs its own pipeline (ROI gate -> voxel
downsample -> background differencing -> Euclidean clustering -> centroid) over
it. It is a synthetic cloud through a real pipeline. **It is not a rendered
depth sensor**, and nothing it produces is evidence that a real sensor achieves
the modelled reach.

WHY A TRUTH SOURCE IS UNAVOIDABLE. An 80 mm ball at 26 m subtends 0.176°, which
is sub-pixel at the sim camera's 640x480 over 67.3°. No rendered depth camera in
this world can ever produce that return; that is why `oracle.py` exists at all.
What changes here is only *who computes the detection*: the oracle asserts a
centroid on `/threat/centroid`, while this module hands the real clustering code
points and lets it find the centroid itself.

WHAT IS MODELLED, and what each choice costs:

  * OPTICS, not a cone. The 10 mm M12 lens that buys 26 m gives a RECTANGULAR
    ±13.5° x ±11.0° frustum (docs/RESULTS.md §5). `oracle.in_view` is a single
    cone half-angle, which is a different sector, so the gate here is per-axis.
    Everything else the oracle already models -- ball selection, the body
    rotation, the pipeline delay -- is imported from it, never re-derived.

  * POINT COUNT derived from angular size against angular resolution. The ball
    is resolved into as many returns as the sensor's IFOV allows and no more.
    A constant would make every long-range detection an artefact of the number
    chosen rather than of the optics claimed.

  * DEPTH error, along the ray, growing as z^2. That is the difference between
    a depth-sensor model and a position oracle with noise bolted on: a stereo
    pair is wrong about RANGE, not about bearing. Calibrated to the measured
    0.30 m at 26 m in docs/RESULTS.md §4.1.

  * The depth error is split into a COMMON-MODE part shared by every return on
    the ball and an INDEPENDENT per-return part, with the marginal per-return
    sigma equal to sigma_depth(z) either way. The default is fully common-mode,
    i.e. the ball's patch gets ONE disparity solution -- which is what the
    0.30 m figure in RESULTS.md is a spec for, and what the oracle applies to
    its single centroid. Making it fully independent instead smears the 80 mm
    ball into a ~1.5 m cigar at 26 m, which detector.yaml's
    `cluster_max_extent_m: 0.35` (sized for a ball at <= 5 m) rejects outright.
    See test_synthetic_depth.py, which pins both behaviours.

WHAT IS NOT MODELLED: reflectance, stereo shadowing, matcher dropout on a
textureless sphere, or any background return at all. The cloud contains the
ball and nothing else, so the detector's background-differencing stage runs but
has nothing to reject. That is stated again in the node docstring, because it
bounds what a result taken through this lane may claim.

Gates are evaluated on the TRUE geometry and noise is applied afterwards --
visibility is a property of where the ball is, not of where the sensor
mistakenly reports it. Same rule as oracle.py.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

# base_link and the gz camera body frame are both FLU (x forward, y left,
# z up), so the boresight is +x. Imported semantics, not a second convention:
# oracle.BORESIGHT is the same vector for the same reason.
from huitzilin_perception.oracle import BORESIGHT  # noqa: F401  (re-export)

# ── The modelled sensor: AR0234 class + 10 mm M12 (docs/RESULTS.md §4.1/§5) ───
DEFAULT_FOV_HALF_H_DEG = 13.5
DEFAULT_FOV_HALF_V_DEG = 11.0
DEFAULT_IMAGE_WIDTH_PX = 1600
DEFAULT_IMAGE_HEIGHT_PX = 1300
# 27.0 deg / 1600 px = 0.295 mrad/px, against the 0.300 mrad/px RESULTS.md
# quotes for a 3.0 um pitch on 10 mm. The 1.7% difference is the doc rounding
# ">=1600x1300 active" to a round pixel count; it is not a separate model.

# The projectile: 80 mm diameter, from models/projectile/model.sdf.
DEFAULT_BALL_RADIUS_M = 0.04

# Depth accuracy of the specified stereo pair, at the range it is specified at.
DEFAULT_DEPTH_SIGMA_M = 0.30
DEFAULT_DEPTH_SIGMA_REF_RANGE_M = 26.0

# Gazebo's SceneBroadcaster publishes dynamic poses at 60 Hz, so no synthetic
# sensor driven off that stream can emit faster, and a requested rate is
# delivered as 60/ceil(60/requested) (docs/RESULTS.md §3). Dead time is
# rate-dependent -- 0.178 s at 60 Hz, 0.270 s at 12 Hz -- so a node that
# believes the requested rate quotes the wrong constant beside every number.
SCENE_BROADCASTER_HZ = 60.0

# Upper bound on returns per frame, for cost only. It binds below ~3.4 m, where
# the ball already saturates its voxel neighbourhood, so it can never change a
# detection outcome; it exists so a ball at 0.3 m does not ask for a 650k-point
# message at 60 Hz. It may only ever REMOVE points -- see the unit test.
DEFAULT_MAX_POINTS_PER_BALL = 5000

# Vogel/sunflower spiral: equal-area coverage of the projected disc with an
# exact point count and no RNG, so the same geometry always yields the same
# returns and a seeded run is replayable end to end.
_GOLDEN_ANGLE_RAD = math.pi * (3.0 - math.sqrt(5.0))

# A boundary has to be decidable. Without this, "exactly on the frustum edge"
# lands on whichever side the last ulp of atan2 fell, which is not a testable
# contract. One picoradian is ~26 nm of cross-range at 26 m.
_ANGLE_EPS_RAD = 1e-12

# How far a detector's ROI ceiling must clear the modelled reach. The ROI gate
# runs on the NOISED cloud, so a ceiling at the reach itself discards every
# frame whose depth draw went long — about one in six at 1 sigma, and those are
# the FIRST detections of a throw, the ones that buy tca. 4 sigma leaves that
# below one frame in 15000.
ROI_HEADROOM_SIGMAS = 4.0


# ── the two configuration floors, as arithmetic ──────────────────────────────

def required_roi_max_range_m(
    detection_range_m: float,
    *,
    sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
    ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    n_sigma: float = ROI_HEADROOM_SIGMAS,
) -> float:
    """Smallest `roi_max_range_m` that does not clip this modelled reach.

    Grows FASTER than the reach, because sigma goes as z^2: 26 m needs 27.2 m
    of ceiling, 30 m needs 31.6 m. A cell that raises detection_range_m without
    raising the detector's ceiling delivers a shorter sensor than its label,
    which CLAUDE.md names as the failure class that has invalidated whole
    result sets. `week6_synthetic_depth.launch.py` refuses to start on it.
    """
    return float(detection_range_m) + n_sigma * float(
        depth_sigma_m(detection_range_m, sigma_ref_m, ref_range_m))


def min_detectable_closing_speed_mps(diff_threshold_m: float,
                                     delivered_rate_hz: float) -> float:
    """Slowest ball frame differencing can separate from its own last frame.

    A ball advances `v / rate` metres between frames. Frame differencing marks
    a return as foreground only if it is further than `diff_threshold_m` from
    every background return, so once `v / rate <= diff_threshold_m` the current
    ball sits inside its own previous blob, the whole cloud reads as
    background, and NOTHING is detected — at any range, with every stage
    upstream looking healthy.

        v_min = diff_threshold_m * delivered_rate_hz

    This is a property of `detector_node`'s differencing meeting a fast frame
    rate, not of this model: at the shipped 0.10 m and 60 Hz it is 6.0 m/s. It
    does not bite at long range, where the common-mode depth jitter (0.30 m at
    26 m) dwarfs the threshold, only at short range where sigma collapses to
    0.005 m — which is exactly where a slow scenario like B01 lives. The
    rendered lane never met it because it runs at 15 Hz (floor 1.5 m/s).
    """
    if delivered_rate_hz <= 0.0:
        return 0.0
    return float(diff_threshold_m) * float(delivered_rate_hz)


# ── rate ──────────────────────────────────────────────────────────────────────

def quantize_rate_hz(requested_hz: float,
                     base_hz: float = SCENE_BROADCASTER_HZ) -> float:
    """The rate actually delivered when pacing off a `base_hz` pose stream.

    A requested 14.5 Hz is delivered as 12 Hz, which costs 0.09 s of dead time
    and 1.8 m of required reach at 20 m/s. Always quote the delivered rate.
    """
    if requested_hz <= 0.0 or requested_hz >= base_hz:
        return float(base_hz)
    return float(base_hz) / math.ceil(base_hz / requested_hz)


def emit_period_s(requested_hz: float,
                  base_hz: float = SCENE_BROADCASTER_HZ) -> float:
    """Minimum sim-time gap between emissions, in seconds.

    Deliberately half a pose period SHORT of the decimation point rather than
    1/quantised. Gazebo steps at 1 ms and broadcasts at 60 Hz, so stamps
    advance 16 ms and 17 ms alternately; a threshold of exactly 1/60 s rejects
    every 16 ms gap and turns a requested 60 Hz into a delivered ~30 Hz with
    nothing in the log to say so. Half a period of slack decides the same way
    for every stamp on the grid.
    """
    decimation = round(base_hz / quantize_rate_hz(requested_hz, base_hz))
    return (decimation - 0.5) / float(base_hz)


# ── visibility ────────────────────────────────────────────────────────────────

def in_frustum(
    rel_cam,
    *,
    detection_range_m: float,
    min_range_m: float = 0.30,
    fov_half_h_deg: float = DEFAULT_FOV_HALF_H_DEG,
    fov_half_v_deg: float = DEFAULT_FOV_HALF_V_DEG,
) -> bool:
    """Would this optics see a ball at `rel_cam` (camera FLU) at all?

    RECTANGULAR, per-axis, because an image sensor is. `oracle.in_view` gates
    on one cone half-angle, which admits corners this frustum rejects and
    rejects corners this one admits -- approximating one with the other would
    defend a sector nobody has. Range gating matches the oracle exactly:
    inclusive at both ends, judged on the true geometry.
    """
    rel = np.asarray(rel_cam, dtype=np.float64).reshape(3)
    rng_m = float(np.linalg.norm(rel))
    if rng_m <= 0.0 or rng_m < min_range_m or rng_m > detection_range_m:
        return False
    forward = float(rel[0])
    if forward <= 0.0:
        return False
    azimuth = abs(math.atan2(float(rel[1]), forward))
    elevation = abs(math.atan2(float(rel[2]), math.hypot(forward, float(rel[1]))))
    return (azimuth <= math.radians(fov_half_h_deg) + _ANGLE_EPS_RAD
            and elevation <= math.radians(fov_half_v_deg) + _ANGLE_EPS_RAD)


# ── how many returns the optics can put on the ball (Ruling P4) ──────────────

def ball_point_count(
    ball_radius_m: float,
    range_m: float,
    *,
    hfov_rad: float,
    vfov_rad: float,
    width_px: int,
    height_px: int,
    max_points: int = DEFAULT_MAX_POINTS_PER_BALL,
) -> int:
    """Returns the sensor can place on the ball's silhouette at this range.

    Derived, never chosen: the sphere's silhouette subtends 2*asin(r/z); the
    camera resolves hfov/width per pixel horizontally and vfov/height
    vertically; the ellipse those two spans bound has area pi/4 * a * b pixels,
    and one pixel is one return.

    Floors to an integer, so a ball too small to fill a pixel returns 0 -- the
    honest answer for a sub-pixel target, and the reason the caller must treat
    0 as "no return this frame" rather than as an error.

    `hfov_rad`/`vfov_rad` are FULL angles, twice the frustum half-angles.
    """
    if width_px <= 0 or height_px <= 0 or hfov_rad <= 0.0 or vfov_rad <= 0.0:
        raise ValueError(
            "degenerate optics: hfov=%r vfov=%r %rx%r px"
            % (hfov_rad, vfov_rad, width_px, height_px))
    if ball_radius_m <= 0.0 or range_m <= 0.0:
        return 0
    ratio = ball_radius_m / range_m
    subtense_rad = math.pi if ratio >= 1.0 else 2.0 * math.asin(ratio)
    px_h = subtense_rad / (hfov_rad / width_px)
    px_v = subtense_rad / (vfov_rad / height_px)
    return min(int(math.floor(0.25 * math.pi * px_h * px_v)), int(max_points))


# ── the ball's camera-facing surface ─────────────────────────────────────────

def _perpendicular_basis(unit: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors spanning the plane normal to `unit`."""
    seed = np.zeros(3)
    seed[int(np.argmin(np.abs(unit)))] = 1.0
    u = np.cross(unit, seed)
    u /= np.linalg.norm(u)
    return u, np.cross(unit, u)


def ball_surface_points(center_cam, ball_radius_m: float,
                        n_points: int) -> np.ndarray:
    """`n_points` returns on the camera-FACING surface of the ball.

    A depth camera samples the front surface on a pixel grid, so the returns
    are laid out on the projected disc and then lifted onto the near side of
    the sphere -- not scattered over the whole sphere, and not collapsed onto
    the centre. Deterministic (a Vogel spiral, no RNG), so the only stochastic
    part of a cloud is its depth noise.
    """
    centre = np.asarray(center_cam, dtype=np.float64).reshape(3)
    n = int(n_points)
    if n <= 0:
        return np.zeros((0, 3), dtype=np.float64)
    rng_m = float(np.linalg.norm(centre))
    if rng_m <= 0.0:
        raise ValueError("ball is on the camera origin; no bearing exists")
    bearing = centre / rng_m
    u_axis, v_axis = _perpendicular_basis(bearing)

    idx = np.arange(n, dtype=np.float64)
    radius = ball_radius_m * np.sqrt((idx + 0.5) / n)
    angle = idx * _GOLDEN_ANGLE_RAD
    toward_camera = np.sqrt(np.maximum(ball_radius_m ** 2 - radius ** 2, 0.0))
    lateral = (radius * np.cos(angle))[:, None] * u_axis \
        + (radius * np.sin(angle))[:, None] * v_axis
    return centre + lateral - toward_camera[:, None] * bearing


# ── depth noise ──────────────────────────────────────────────────────────────

def depth_sigma_m(range_m,
                  sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
                  ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M):
    """Stereo depth sigma at `range_m`, quadratic in range.

    z = f*b/d, so dz/dd goes as z^2 and a fixed disparity error becomes a range
    error growing as the square of range (oracle.py's docstring says the same).
    Pinned by sigma(ref_range_m) = sigma_ref_m, i.e. the measured 0.30 m at
    26 m. Scalar in, scalar out; array in, array out.
    """
    if ref_range_m <= 0.0:
        raise ValueError("ref_range_m must be > 0, got %r" % (ref_range_m,))
    return sigma_ref_m * (np.asarray(range_m, dtype=np.float64)
                          / ref_range_m) ** 2


def apply_depth_noise(
    points_cam,
    rng_gen: np.random.Generator,
    *,
    sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
    ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    per_point_frac: float = 0.0,
) -> np.ndarray:
    """Perturb every return ALONG ITS OWN BEARING RAY. Returns a new array.

    `per_point_frac` splits the error into a common-mode draw shared by the
    whole ball and an independent per-return draw, weighted so the marginal
    per-return sigma is sigma_depth(z) for any split:

        dz_i = sigma(z_i) * ( sqrt(1-f^2) * g_common + f * g_i )

    f = 0 (the default) means the ball's patch has ONE disparity solution --
    the returns keep the ball's physical 80 mm extent and the error lands on
    the centroid, which is where RESULTS.md's 0.30 m spec and the oracle's own
    noise both live. f = 1 makes every return independent, which at 26 m smears
    the ball across ~1.5 m and is rejected by the shipped
    `cluster_max_extent_m`. Both are pinned in test_synthetic_depth.py.

    Zero sigma reproduces the input exactly and consumes no randomness, so a
    noiseless run is genuinely noiseless -- same contract as
    `oracle.apply_noise`.
    """
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if not 0.0 <= per_point_frac <= 1.0:
        raise ValueError(
            "per_point_frac must be in [0, 1], got %r" % (per_point_frac,))
    if pts.shape[0] == 0 or sigma_ref_m <= 0.0:
        return pts.copy()
    ranges = np.linalg.norm(pts, axis=1)
    if not np.all(ranges > 0.0):
        raise ValueError("a return sits on the camera origin; no ray exists")

    sigma = depth_sigma_m(ranges, sigma_ref_m, ref_range_m)
    common = float(rng_gen.standard_normal())
    independent = rng_gen.standard_normal(pts.shape[0])
    unit_gauss = (math.sqrt(max(1.0 - per_point_frac ** 2, 0.0)) * common
                  + per_point_frac * independent)
    return pts + (pts / ranges[:, None]) * (sigma * unit_gauss)[:, None]


# ── one frame ────────────────────────────────────────────────────────────────

def synthesize_ball_cloud(
    rel_cam,
    *,
    detection_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    min_range_m: float = 0.30,
    fov_half_h_deg: float = DEFAULT_FOV_HALF_H_DEG,
    fov_half_v_deg: float = DEFAULT_FOV_HALF_V_DEG,
    width_px: int = DEFAULT_IMAGE_WIDTH_PX,
    height_px: int = DEFAULT_IMAGE_HEIGHT_PX,
    ball_radius_m: float = DEFAULT_BALL_RADIUS_M,
    max_points: int = DEFAULT_MAX_POINTS_PER_BALL,
    sigma_ref_m: float = DEFAULT_DEPTH_SIGMA_M,
    ref_range_m: float = DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    per_point_frac: float = 0.0,
    rng_gen: Optional[np.random.Generator] = None,
) -> Optional[np.ndarray]:
    """The (N, 3) camera-FLU returns for one ball, or None if there are none.

    None covers "outside the modelled frustum" and "too small to fill a pixel"
    alike; the caller cannot tell them apart, exactly as it could not with a
    real sensor. Gates run on the true geometry, noise afterwards.
    """
    rel = np.asarray(rel_cam, dtype=np.float64).reshape(3)
    if not in_frustum(rel, detection_range_m=detection_range_m,
                      min_range_m=min_range_m,
                      fov_half_h_deg=fov_half_h_deg,
                      fov_half_v_deg=fov_half_v_deg):
        return None

    n_points = ball_point_count(
        ball_radius_m, float(np.linalg.norm(rel)),
        hfov_rad=math.radians(2.0 * fov_half_h_deg),
        vfov_rad=math.radians(2.0 * fov_half_v_deg),
        width_px=width_px, height_px=height_px, max_points=max_points)
    if n_points <= 0:
        return None

    surface = ball_surface_points(rel, ball_radius_m, n_points)
    rng_gen = np.random.default_rng() if rng_gen is None else rng_gen
    return apply_depth_noise(surface, rng_gen, sigma_ref_m=sigma_ref_m,
                             ref_range_m=ref_range_m,
                             per_point_frac=per_point_frac)


def camera_relative(rel_body, camera_offset_xyz_m: Sequence[float]) -> np.ndarray:
    """base_link -> camera vector to ball. Returns a new array.

    The static TF chain the launch file publishes puts the camera at
    `camera_offset_xyz_m` in base_link, and detector_node transforms its
    centroid back through that same chain. Subtracting it here makes the round
    trip exact instead of leaving a fixed 0.10 m forward bias the oracle lane
    does not have.
    """
    body = np.asarray(rel_body, dtype=np.float64).reshape(3)
    offset = np.asarray(camera_offset_xyz_m, dtype=np.float64).reshape(3)
    return body - offset
