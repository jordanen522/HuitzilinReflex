"""Honesty checks for the synthetic depth cloud.

This module fabricates a `sensor_msgs/PointCloud2` from Gazebo truth so that
the REAL, unmodified detector_node.py pipeline (ROI gate -> voxel downsample ->
background differencing -> Euclidean clustering -> centroid) computes the
20 m/s centroids instead of them being asserted by the oracle. It is a
synthetic cloud through a real pipeline. It is NOT a rendered sensor, and no
test here should ever be read as evidence that one exists.

That makes it exactly the kind of tool that can prove whatever you wanted, so
these tests pin the properties that stop it:

  * it must refuse to report what the modelled optics could not see,
  * its point count must fall out of the ball's angular size against the
    camera's angular resolution, never out of a number chosen to clear
    `cluster_min_points`,
  * its error must be a DEPTH error -- along the ray, growing as z^2 -- and
    calibrated to the 0.30 m at 26 m in docs/RESULTS.md §4.1,
  * and the cloud it produces must actually survive the shipped detector's
    own geometry stages, which is checked here against the real
    cloud_geometry functions rather than assumed.

Pure numpy/scipy: no ROS import anywhere, so this runs in CI.
"""

import math

import numpy as np
import pytest

from huitzilin_perception.cloud_geometry import cluster_and_split, voxel_downsample
from huitzilin_perception.synthetic_depth import (
    DEFAULT_BALL_RADIUS_M,
    DEFAULT_DEPTH_SIGMA_M,
    DEFAULT_DEPTH_SIGMA_REF_RANGE_M,
    DEFAULT_FOV_HALF_H_DEG,
    DEFAULT_FOV_HALF_V_DEG,
    DEFAULT_IMAGE_HEIGHT_PX,
    DEFAULT_IMAGE_WIDTH_PX,
    SCENE_BROADCASTER_HZ,
    apply_depth_noise,
    ball_point_count,
    ball_surface_points,
    depth_sigma_m,
    emit_period_s,
    in_frustum,
    quantize_rate_hz,
    synthesize_ball_cloud,
)

# The shipped detector.yaml values the cloud has to survive. Mirrored here on
# purpose rather than parsed: if one of them changes, this file must be re-read
# and re-justified, not silently re-pointed at a new number.
DETECTOR_VOXEL_LEAF_M = 0.02
DETECTOR_CLUSTER_TOL_M = 0.20
DETECTOR_CLUSTER_MIN_POINTS = 5
DETECTOR_CLUSTER_MAX_EXTENT_M = 0.35
DETECTOR_CLUSTER_SPLIT_TOL_M = 0.10
DETECTOR_CLUSTER_SPLIT_MAX_POINTS = 5000
# synthetic_depth_detector.yaml's one forced widening (detector.yaml ships 5.00).
# 28.00, not 26.0x: the ROI gate runs on the NOISED cloud, so it has to clear
# detection_range_m by several depth sigmas or it silently eats the far-edge
# detections — the earliest and most valuable ones. See the test below.
OVERLAY_ROI_MAX_RANGE_M = 28.00
OVERLAY_ROI_MIN_RANGE_M = 0.30

OPTICS = dict(
    hfov_rad=math.radians(2.0 * DEFAULT_FOV_HALF_H_DEG),
    vfov_rad=math.radians(2.0 * DEFAULT_FOV_HALF_V_DEG),
    width_px=DEFAULT_IMAGE_WIDTH_PX,
    height_px=DEFAULT_IMAGE_HEIGHT_PX,
)


# ── rate quantization ────────────────────────────────────────────────────────
#
# Gazebo's SceneBroadcaster runs at 60 Hz, so a requested rate is delivered as
# 60/ceil(60/requested) (docs/RESULTS.md §3). Rate is not cosmetic: dead time is
# 0.178 s at 60 Hz and 0.270 s at 12 Hz, which is 1.8 m of required reach at
# 20 m/s. A node that believes it runs at 14.5 Hz while delivering 12 quotes the
# wrong constant beside every number it produces.

@pytest.mark.parametrize("requested,delivered", [
    (60.0, 60.0),
    (90.0, 60.0),      # cannot outrun the pose stream
    (30.0, 30.0),
    (20.0, 20.0),
    (15.0, 15.0),
    (14.5, 12.0),      # the oracle's shipped default — RESULTS.md names this one
    (12.0, 12.0),
    (10.0, 10.0),
    (7.0, 60.0 / 9.0),
])
def test_the_delivered_rate_is_the_quantised_one(requested, delivered):
    assert quantize_rate_hz(requested) == pytest.approx(delivered)


def test_a_non_positive_rate_means_every_pose():
    """0 disables rate limiting, exactly as the oracle's rate_hz does."""
    assert quantize_rate_hz(0.0) == pytest.approx(SCENE_BROADCASTER_HZ)


def test_the_emit_period_sits_half_a_pose_short_of_the_decimation():
    """The period must tolerate the pose grid's own jitter.

    Gazebo steps at 1 ms and broadcasts at 60 Hz, so stamps advance 16 ms and
    17 ms alternately. A period of exactly 1/60 s therefore rejects every 16 ms
    gap and a requested 60 Hz would be delivered as ~30 Hz — silently, and in
    the one direction that flatters nothing but confuses everything. Placing
    the threshold half a pose period below the decimation point makes the
    decision immune to that jitter.
    """
    assert emit_period_s(60.0) == pytest.approx(0.5 / SCENE_BROADCASTER_HZ)
    assert emit_period_s(30.0) == pytest.approx(1.5 / SCENE_BROADCASTER_HZ)
    assert emit_period_s(14.5) == pytest.approx(4.5 / SCENE_BROADCASTER_HZ)


def test_the_emit_period_accepts_exactly_the_quantised_cadence():
    """Walk the real 60 Hz pose grid and count what gets through."""
    for requested in (60.0, 30.0, 20.0, 15.0, 14.5, 12.0):
        period = emit_period_s(requested)
        last = None
        emitted = 0
        for k in range(601):                      # 10 s of pose stamps
            t = k / SCENE_BROADCASTER_HZ
            if last is None or t - last >= period:
                last = t
                emitted += 1
        # 601 stamps span 10 s; the first is free, so compare the rest.
        assert (emitted - 1) / 10.0 == pytest.approx(
            quantize_rate_hz(requested), rel=0.02)


# ── the modelled frustum ─────────────────────────────────────────────────────
#
# The 10 mm M12 lens that buys 26 m narrows the cone to ±13.5° H / ±11.0° V
# (docs/RESULTS.md §5), and that loss is the whole cost of the reach. It is
# RECTANGULAR — a real image sensor is — so approximating it with the oracle's
# single cone half-angle would defend a sector nobody has.

def test_a_ball_dead_ahead_is_seen():
    assert in_frustum([20.0, 0.0, 0.0], detection_range_m=26.0)


def test_a_ball_behind_the_camera_is_never_seen():
    """The most important refusal: warning from outside the frustum is warning
    no sensor could have given."""
    assert not in_frustum([-20.0, 0.0, 0.0], detection_range_m=26.0)


def test_the_horizontal_edge_is_where_the_lens_puts_it():
    r = 20.0
    for deg, seen in ((13.4, True), (13.6, False)):
        a = math.radians(deg)
        assert in_frustum([r * math.cos(a), r * math.sin(a), 0.0],
                          detection_range_m=26.0) is seen


def test_the_vertical_edge_is_where_the_lens_puts_it():
    r = 20.0
    for deg, seen in ((10.9, True), (11.1, False)):
        e = math.radians(deg)
        assert in_frustum([r * math.cos(e), 0.0, r * math.sin(e)],
                          detection_range_m=26.0) is seen


def test_the_frustum_is_rectangular_not_a_cone():
    """A cone of half-angle 13.5° would reject this; a rectangular frustum
    accepts it. This is the whole reason in_view() could not be reused."""
    r = 20.0
    az, el = math.radians(13.0), math.radians(10.5)     # 16.7° off boresight
    rel = [r * math.cos(el) * math.cos(az),
           r * math.cos(el) * math.sin(az),
           r * math.sin(el)]
    assert math.degrees(math.acos(rel[0] / r)) > DEFAULT_FOV_HALF_H_DEG
    assert in_frustum(rel, detection_range_m=26.0)


def test_a_ball_exactly_on_each_edge_is_still_reported():
    """A boundary has to be decidable, so the comparison is inclusive."""
    r = 20.0
    a = math.radians(DEFAULT_FOV_HALF_H_DEG)
    assert in_frustum([r * math.cos(a), r * math.sin(a), 0.0],
                      detection_range_m=26.0)
    e = math.radians(DEFAULT_FOV_HALF_V_DEG)
    assert in_frustum([r * math.cos(e), 0.0, r * math.sin(e)],
                      detection_range_m=26.0)


def test_range_gating_matches_the_oracle_semantics():
    """Both ends inclusive, judged on the TRUE range."""
    assert in_frustum([26.0, 0.0, 0.0], detection_range_m=26.0)
    assert not in_frustum([26.01, 0.0, 0.0], detection_range_m=26.0)
    assert in_frustum([0.30, 0.0, 0.0], detection_range_m=26.0, min_range_m=0.30)
    assert not in_frustum([0.29, 0.0, 0.0], detection_range_m=26.0,
                          min_range_m=0.30)


def test_a_ball_on_top_of_the_camera_is_not_reported():
    """Guards the divide-by-zero in the bearing test."""
    assert not in_frustum([0.0, 0.0, 0.0], detection_range_m=26.0)


# ── the derived point count (Ruling P4) ──────────────────────────────────────
#
# The count must fall out of the ball's angular size against the camera's
# angular resolution. A constant picked to clear cluster_min_points would make
# every detection at range an artefact of the number chosen, which is precisely
# the thing this whole exercise exists to stop doing.

def test_the_count_falls_with_range():
    counts = [ball_point_count(DEFAULT_BALL_RADIUS_M, z, max_points=10 ** 9,
                               **OPTICS)
              for z in (3.0, 5.0, 8.0, 13.0, 21.0, 26.0, 40.0, 80.0)]
    assert counts == sorted(counts, reverse=True)
    assert len(set(counts)) == len(counts)          # strictly decreasing


def test_the_count_at_the_26_m_design_point():
    """26 m is the derived sensor requirement (docs/RESULTS.md §4). The 80 mm
    ball spans ~10.4 px x ~10.4 px there, so the disc carries 85 returns."""
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0, **OPTICS) == 85


def test_the_26_m_count_clears_the_shipped_clustering_floor():
    """The finding this test exists to record, either way it lands."""
    assert (ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0, **OPTICS)
            >= DETECTOR_CLUSTER_MIN_POINTS)


def test_the_count_at_the_oak_d_lites_3_4_m_reach():
    """The reach the aircraft as built actually has, for comparison."""
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 3.4, **OPTICS) == 4998


def test_the_count_eventually_falls_below_the_clustering_floor():
    """Not something to tune around: past ~107 m this optics cannot deliver a
    clusterable ball at all, and the node must say so by returning nothing."""
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 107.0, **OPTICS) >= 5
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 108.0, **OPTICS) < 5


def test_a_sub_pixel_ball_returns_nothing_at_all():
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 400.0, **OPTICS) == 0


def test_more_pixels_on_the_same_cone_buy_more_returns():
    """`reach x half-angle` is fixed by pixel count (RESULTS.md §5) — the count
    must respond to resolution, not only to range."""
    coarse = ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0, **OPTICS)
    fine = ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0,
                            **{**OPTICS, "width_px": 3200, "height_px": 2600})
    assert fine > 3 * coarse


def test_the_ceiling_only_ever_removes_points():
    """A cap is an upper bound for cost, never a floor that manufactures
    returns: it may only bind where the physical count is already larger."""
    physical = ball_point_count(DEFAULT_BALL_RADIUS_M, 1.0, max_points=10 ** 9,
                                **OPTICS)
    capped = ball_point_count(DEFAULT_BALL_RADIUS_M, 1.0, max_points=5000,
                              **OPTICS)
    assert capped == 5000 < physical
    assert ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0, max_points=5000,
                            **OPTICS) == 85


def test_degenerate_optics_are_refused_rather_than_guessed():
    for bad in ({"width_px": 0}, {"height_px": 0}, {"hfov_rad": 0.0},
                {"vfov_rad": -1.0}):
        with pytest.raises(ValueError):
            ball_point_count(DEFAULT_BALL_RADIUS_M, 26.0, **{**OPTICS, **bad})


# ── the ball's surface ───────────────────────────────────────────────────────

def test_the_returns_sit_on_the_camera_facing_surface():
    """A depth camera sees the front of the ball, not its centre and not its
    far side: every return must be on the sphere and nearer than the centre."""
    centre = np.array([26.0, 0.0, 0.0])
    pts = ball_surface_points(centre, DEFAULT_BALL_RADIUS_M, 85)
    assert pts.shape == (85, 3)
    radii = np.linalg.norm(pts - centre, axis=1)
    np.testing.assert_allclose(radii, DEFAULT_BALL_RADIUS_M, atol=1e-9)
    assert np.all(np.linalg.norm(pts, axis=1) <= 26.0 + 1e-9)


def test_the_surface_is_deterministic():
    a = ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M, 85)
    b = ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M, 85)
    np.testing.assert_array_equal(a, b)


def test_the_surface_follows_the_ball_off_boresight():
    """The visible cap must face the CAMERA, not a fixed axis."""
    centre = np.array([18.0, 4.0, 3.0])
    pts = ball_surface_points(centre, DEFAULT_BALL_RADIUS_M, 60)
    assert np.all(np.linalg.norm(pts, axis=1) <= np.linalg.norm(centre) + 1e-9)


def test_no_points_asked_for_is_an_empty_cloud_not_an_error():
    assert ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M,
                               0).shape == (0, 3)


# ── depth noise ──────────────────────────────────────────────────────────────
#
# This is the structural point of the whole node. A position oracle with an
# arbitrary 3D offset bolted on is not a depth sensor; a depth sensor is wrong
# ALONG THE RAY, and wrong as z^2 (oracle.py's own docstring says so).
# Calibrated to the measured 0.30 m at 26 m in docs/RESULTS.md §4.1.

def test_the_sigma_law_is_quadratic_and_hits_the_measured_point():
    assert depth_sigma_m(26.0) == pytest.approx(DEFAULT_DEPTH_SIGMA_M)
    assert depth_sigma_m(52.0) == pytest.approx(4.0 * DEFAULT_DEPTH_SIGMA_M)
    assert depth_sigma_m(13.0) == pytest.approx(DEFAULT_DEPTH_SIGMA_M / 4.0)
    assert depth_sigma_m(3.4) == pytest.approx(
        DEFAULT_DEPTH_SIGMA_M * (3.4 / DEFAULT_DEPTH_SIGMA_REF_RANGE_M) ** 2)


def test_the_sigma_law_is_vectorised():
    got = depth_sigma_m(np.array([13.0, 26.0, 52.0]))
    np.testing.assert_allclose(got, [0.075, 0.30, 1.20])


def test_zero_sigma_is_exactly_noiseless():
    pts = ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M, 20)
    out = apply_depth_noise(pts, np.random.default_rng(1), sigma_ref_m=0.0)
    np.testing.assert_array_equal(out, pts)


def test_the_perturbation_stays_on_the_bearing_ray():
    """A noised return must keep its bearing exactly. Anything else is a
    position error wearing a depth error's name."""
    rng = np.random.default_rng(4)
    pts = ball_surface_points([20.0, 3.0, 2.0], DEFAULT_BALL_RADIUS_M, 40)
    out = apply_depth_noise(pts, rng, per_point_frac=1.0)
    unit_in = pts / np.linalg.norm(pts, axis=1)[:, None]
    unit_out = out / np.linalg.norm(out, axis=1)[:, None]
    np.testing.assert_allclose(unit_out, unit_in, atol=1e-9)


def test_the_depth_error_at_26_m_is_the_measured_0_30_m():
    """Per-point marginal sigma, whatever the common/independent split."""
    for frac in (0.0, 0.5, 1.0):
        rng = np.random.default_rng(9)
        base = np.array([[26.0, 0.0, 0.0]])
        err = [float(np.linalg.norm(
                   apply_depth_noise(base, rng, per_point_frac=frac)[0]) - 26.0)
               for _ in range(6000)]
        assert np.std(err) == pytest.approx(DEFAULT_DEPTH_SIGMA_M, rel=0.05)
        assert abs(np.mean(err)) < 0.02


def test_the_error_shrinks_quadratically_at_short_range():
    rng = np.random.default_rng(13)
    base = np.array([[3.4, 0.0, 0.0]])
    err = [float(np.linalg.norm(
               apply_depth_noise(base, rng, per_point_frac=1.0)[0]) - 3.4)
           for _ in range(6000)]
    assert np.std(err) == pytest.approx(depth_sigma_m(3.4), rel=0.06)


def test_the_default_split_moves_the_whole_ball_together():
    """per_point_depth_frac 0.0 = the disparity solution for the ball's patch
    is ONE estimate: the returns keep the ball's physical 80 mm extent and the
    error lands on the centroid, which is where docs/RESULTS.md's 0.30 m spec
    and the oracle's noise_std_xyz_m both live."""
    pts = ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M, 85)
    out = apply_depth_noise(pts, np.random.default_rng(2), per_point_frac=0.0)
    shift = np.linalg.norm(out, axis=1) - np.linalg.norm(pts, axis=1)
    # One shared draw. Not bit-identical across returns: sigma is a function of
    # each return's own range and the ball is 80 mm deep, so the shifts differ
    # by ~4e-5 m at 26 m — five hundred times under the 0.02 m voxel leaf, and
    # three orders under the 0.35 m extent gate.
    assert np.std(shift) < 1e-3
    spread = out.max(axis=0) - out.min(axis=0)
    assert spread.max() < 3.0 * DEFAULT_BALL_RADIUS_M


def test_a_frac_outside_zero_to_one_is_refused():
    pts = ball_surface_points([26.0, 0.0, 0.0], DEFAULT_BALL_RADIUS_M, 5)
    for bad in (-0.1, 1.1):
        with pytest.raises(ValueError):
            apply_depth_noise(pts, np.random.default_rng(0),
                              per_point_frac=bad)


# ── the whole cloud ──────────────────────────────────────────────────────────

def _cloud(rel_cam, **kw):
    return synthesize_ball_cloud(rel_cam, rng_gen=np.random.default_rng(
        kw.pop("seed", 5)), **kw)


def test_a_ball_outside_the_frustum_produces_no_cloud():
    assert _cloud([26.0, 8.0, 0.0]) is None          # 17° off in azimuth
    assert _cloud([30.0, 0.0, 0.0]) is None          # past detection_range_m
    assert _cloud([0.1, 0.0, 0.0]) is None           # inside min_range_m


def test_gates_are_judged_on_truth_not_on_the_noisy_return():
    """Visibility is a property of where the ball IS. Noising first would let
    a ball just outside the gate be jittered into view, and the node would then
    report detections its own configuration says are impossible."""
    for _ in range(200):
        assert _cloud([26.6, 0.0, 0.0], seed=0) is None


def test_the_same_seed_replays_the_same_cloud():
    a = _cloud([26.0, 0.0, 0.0], seed=11, per_point_frac=1.0)
    b = _cloud([26.0, 0.0, 0.0], seed=11, per_point_frac=1.0)
    np.testing.assert_array_equal(a, b)


def test_different_seeds_give_different_clouds():
    a = _cloud([26.0, 0.0, 0.0], seed=11, per_point_frac=1.0)
    b = _cloud([26.0, 0.0, 0.0], seed=12, per_point_frac=1.0)
    assert not np.array_equal(a, b)


def test_the_cloud_is_a_float_n_by_3():
    cloud = _cloud([26.0, 0.0, 0.0])
    assert cloud.shape == (85, 3)
    assert cloud.dtype == np.float64


# ── survival through the shipped detector geometry ───────────────────────────
#
# The point of the exercise is that the REAL pipeline computes the centroid, so
# "the cloud is plausible" is not enough — it has to survive detector_node's own
# stages. These run the actual cloud_geometry functions the node calls, with
# synthetic_depth_detector.yaml's values, in the same order.

def _through_the_detector(cloud, max_extent=DETECTOR_CLUSTER_MAX_EXTENT_M):
    """ROI gate -> voxel -> cluster/split, exactly as _cloud_cb_impl does.

    Background differencing is omitted here because a ball-only cloud makes it
    a no-op (see the node docstring) — every point is novel, so the foreground
    mask is all-True and the next stage sees the whole cloud.
    """
    # detector_node's cloud_convention: "gz_flu" remap, verbatim.
    optical = np.stack((-cloud[:, 1], -cloud[:, 2], cloud[:, 0]),
                       axis=1).astype(np.float32)
    depth = optical[:, 2]
    roi = optical[(depth >= OVERLAY_ROI_MIN_RANGE_M)
                  & (depth <= OVERLAY_ROI_MAX_RANGE_M)]
    vox = voxel_downsample(roi, DETECTOR_VOXEL_LEAF_M)
    return vox, cluster_and_split(
        vox,
        tol=DETECTOR_CLUSTER_TOL_M,
        min_pts=DETECTOR_CLUSTER_MIN_POINTS,
        max_extent=max_extent,
        split_tol=DETECTOR_CLUSTER_SPLIT_TOL_M,
        max_split_points=DETECTOR_CLUSTER_SPLIT_MAX_POINTS,
    )


def test_a_26_m_ball_survives_the_real_detector_geometry():
    """The measurement this node exists to make possible. If this fails, the
    Dell run cannot produce a centroid at 26 m and no amount of flying will
    change that."""
    vox, clusters = _through_the_detector(_cloud([26.0, 0.0, 0.0]))
    assert vox.shape[0] >= DETECTOR_CLUSTER_MIN_POINTS
    assert len(clusters) == 1
    assert clusters[0].shape[0] >= DETECTOR_CLUSTER_MIN_POINTS


def test_the_surviving_cluster_is_ball_sized_not_a_smear():
    from huitzilin_perception.cloud_geometry import cluster_extent
    _, clusters = _through_the_detector(_cloud([26.0, 0.0, 0.0]))
    assert cluster_extent(clusters[0]) <= DETECTOR_CLUSTER_MAX_EXTENT_M


def test_the_recovered_centroid_lands_on_the_ball():
    """Within the depth error the model itself claims, and no further off."""
    _, clusters = _through_the_detector(_cloud([26.0, 0.0, 0.0], seed=3))
    centroid = clusters[0].mean(axis=0)
    # optical (X right, Y down, Z fwd) back to the camera FLU truth.
    flu = np.array([centroid[2], -centroid[0], -centroid[1]])
    assert abs(np.linalg.norm(flu) - 26.0) < 4.0 * depth_sigma_m(26.0)
    assert abs(flu[1]) < 0.10 and abs(flu[2]) < 0.10


def test_the_overlays_roi_ceiling_clears_the_depth_noise_at_full_reach():
    """Why synthetic_depth_detector.yaml says 28.00 and not 26.50.

    The ROI gate is applied to the NOISED cloud, so a ceiling only just above
    detection_range_m throws away every frame whose common-mode depth draw went
    long — roughly one in six at 1 sigma. Those are the FIRST detections of the
    throw, the ones that buy tca, so losing them costs exactly the quantity the
    26 m requirement was derived from, and it would look like a marginal sensor
    rather than a misconfigured gate. Requirement: the ceiling must clear
    detection_range_m by >= 4 * depth_sigma(detection_range_m).
    """
    assert (OVERLAY_ROI_MAX_RANGE_M - 26.0) >= 4.0 * depth_sigma_m(26.0)
    lost = sum(_through_the_detector(_cloud([26.0, 0.0, 0.0], seed=s))[1] == []
               for s in range(60))
    assert lost == 0


def test_the_ball_survives_across_the_whole_modelled_reach():
    for z in (1.0, 3.4, 8.0, 15.0, 21.0, 26.0):
        _, clusters = _through_the_detector(_cloud([float(z), 0.0, 0.0]))
        assert len(clusters) >= 1, f"no cluster at {z} m"
        assert clusters[0].shape[0] >= DETECTOR_CLUSTER_MIN_POINTS


def test_fully_independent_per_point_depth_noise_destroys_the_26_m_cluster():
    """The finding behind per_point_depth_frac's 0.0 default, pinned.

    Drawing the 0.30 m depth error INDEPENDENTLY per return smears the 80 mm
    ball into a ~1.5 m cigar along the ray. detector.yaml's cluster_max_extent_m
    is 0.35 m — sized for a ball at <= 5 m, where the same law gives 0.011 m —
    so the blob is rejected outright and the run scores zero detections at
    26 m while every stage upstream looks healthy. That is a property of the
    shipped detector config meeting a long-range depth error, not a tuning
    accident, and it is why the default treats the ball's disparity as ONE
    estimate.
    """
    cloud = _cloud([26.0, 0.0, 0.0], per_point_frac=1.0, seed=7)
    vox, clusters = _through_the_detector(cloud)
    assert vox.shape[0] > 40                       # the returns are all there
    assert clusters == []                          # and every one is discarded
