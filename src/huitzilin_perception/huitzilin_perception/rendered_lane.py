"""rendered_lane.py — configuration invariants for the rendered long-range lane.

Pure python: no ROS, no rclpy, no launch. That is the whole reason it exists
apart from week7_rendered.launch.py, which is where this logic naturally wants
to live and where it cannot be tested. A launch file imports `launch` and
`launch_ros`, so CI's ROS-free subset cannot import it at all, and a guard that
no test can reach is not a guard -- it is a comment that raises.

WHAT IT GUARDS. Two detector gates are expressed in METRES against a RANGE, and
both were sized when the ball was never further than 5 m and the depth error was
a centimetre:

    roi_max_range_m       the far edge of the region of interest
    cluster_max_extent_m  the largest bounding box still considered ball-sized

Neither fails loudly when it is too small for the range being flown. They
quietly deliver a shorter, worse sensor than the run's label claims, and the
result reads as a perception limit rather than as a misconfiguration. CLAUDE.md
names that failure class as one that has invalidated whole result sets, and
every unit test stays green through it because the gate and the range it is
sized against live in different files.

The measured instance that motivated this: at 26 m the 0.30 m along-ray depth
error stretches the 80 mm ball's bounding box to a median 0.26 m and a p95 of
0.45 m, so the shipped 0.35 m gate rejects about 20 % of ball frames -- and
`cloud_geometry.cluster_and_split` does not discard them quietly, it re-clusters
the oversized cluster at `cluster_split_tol_m`, which on a 24-point ball
fragments it below `cluster_min_points`. Flying that would have produced a
depressed save rate indistinguishable from a real sensor limit.
"""

from __future__ import annotations

from huitzilin_perception.depth_noise import required_cluster_max_extent_m
from huitzilin_perception.synthetic_depth import (
    ROI_HEADROOM_SIGMAS,
    required_roi_max_range_m,
)

# The camera's far clip, from models/iris_ar0234/model.sdf. The longest range
# the lane can produce a return at, so an ROI ceiling past it can never bind
# and only overstates the sensor. test_rendered_lane.py holds this in step with
# the SDF rather than trusting it to stay in step.
AR0234_FAR_CLIP_M = 35.0

# The reach the long-range arm is BUILT to serve, from docs/RESULTS.md: 21.1 m
# buys P(save)=0.90 at 20 m/s, and 26 m is the sensor that scored 28/29 head-on
# in the oracle lane. It is NOT an input to the rendered lane -- nothing there
# clamps the cloud to a range -- it is only the range the detector's gates must
# not reject a ball at.
DESIGN_REACH_M = 26.0


def assert_gates_clear_the_rendered_range(detector: dict) -> None:
    """Refuse a detector config whose gates reject the ball it is aimed at.

    `detector` is the `ros__parameters` mapping of a shipped detector params
    file. Raises RuntimeError with the arithmetic in the message; returns None
    on a config that is self-consistent.

    THIS IS DELIBERATELY NOT AN ERROR FOR A SHORT-RANGE CONFIG. The OAK-D
    fidelity gate runs the same launch file on params/detector.yaml at
    5.0 m / 0.35 m, and that pairing is correct for its own range. The extent
    check is therefore sized against the ROI ceiling THE CONFIG ITSELF declares,
    never against DESIGN_REACH_M, so a self-consistent short-range config passes
    untouched. What it catches is the config that opens the ROI to 28 m and
    leaves the extent gate where a 5 m sensor left it.
    """
    ceiling = float(detector["roi_max_range_m"])
    needed_extent = required_cluster_max_extent_m(ceiling)
    have_extent = float(detector["cluster_max_extent_m"])
    if have_extent < needed_extent:
        raise RuntimeError(
            "roi_max_range_m %.2f m admits a ball whose noise-stretched extent "
            "needs cluster_max_extent_m >= %.2f m, but the launched detector "
            "params allow only %.2f m. cluster_and_split would re-cluster the "
            "ball at cluster_split_tol_m and fragment it below "
            "cluster_min_points on a large share of frames, and the run would "
            "report that as a perception limit. Raise cluster_max_extent_m, or "
            "lower roi_max_range_m."
            % (ceiling, needed_extent, have_extent))

    # The other direction is only worth checking once a config is clearly aiming
    # long. A 5 m fidelity-gate config is not aiming long and must not be nagged
    # for failing to clear a reach it never claimed.
    if ceiling < DESIGN_REACH_M:
        return

    needed_roi = required_roi_max_range_m(DESIGN_REACH_M)
    if ceiling < needed_roi:
        raise RuntimeError(
            "roi_max_range_m %.2f m does not clear the %.1f m design reach by "
            "%g sigma of depth noise (needs %.2f m). The ROI gate runs on the "
            "NOISED cloud, so the long draws it would clip are a throw's FIRST "
            "detections -- the ones that buy tca. Raise roi_max_range_m; it "
            "must stay under the camera's %.1f m far clip."
            % (ceiling, DESIGN_REACH_M, ROI_HEADROOM_SIGMAS, needed_roi,
               AR0234_FAR_CLIP_M))

    if ceiling > AR0234_FAR_CLIP_M:
        raise RuntimeError(
            "roi_max_range_m %.2f m is past the camera's own far clip of "
            "%.1f m -- the gate can never bind, and a config that names a "
            "range the optics cannot reach overstates the sensor. Lower it, or "
            "raise <far> in models/iris_ar0234/model.sdf."
            % (ceiling, AR0234_FAR_CLIP_M))
