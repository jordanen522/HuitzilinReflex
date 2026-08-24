"""Attribute detections to the projectile using ground truth.

Scores detections from a bag against /gz/dynamic_poses rather than against how
they moved. The companion heuristic, score_bags_logic.attribute_closing_ball,
only answers "something closed at a ball-like rate", which does not separate the
ball from newly-explored terrain along a patrol leg; ground truth does. Bags
captured before scripts/capture_scenario.sh started recording /gz/dynamic_poses
cannot be scored here.

Design rules:
  * Pure python, no ROS, so the ROS-free CI subset covers it.
  * `match_radius_m` is a matching tolerance, not an accuracy figure —
    `score_scenario` also returns the per-match error distribution, and a pass is
    not meaningful without it.
  * Detections and both truth tracks must already be in one common fixed frame
    (the detector publishes in `fixed_frame`, "odom"). No frame conversion
    happens here; see docs/frames.md.
"""

from __future__ import annotations

import math

from huitzilin_perception.synthetic_depth import depth_sigma_m

__all__ = [
    "DEFAULT_MIN_MATCHED",
    "MATCH_FLOOR_M",
    "MATCH_SIGMAS",
    "STAMP_SKEW_S",
    "interpolate_track",
    "match_radius_m",
    "score_scenario",
    "void_reason",
]

# Sigmas of modelled depth error admitted as "the same object". 3 sigma admits
# ~99.7 % of true detections, so a correct detection at 26 m (sigma 0.30 m) is
# not called a miss for being 0.6 m out.
MATCH_SIGMAS = 3.0

# Floor on the match radius. Below ~9 m the modelled error falls under a voxel
# and 3 sigma would be tighter than the discretisation the detector's own
# pipeline imposes (voxel_leaf_m 0.02, the ball's 0.08 m extent, clustering
# quantisation), so correct short-range detections would read as unmatched.
MATCH_FLOOR_M = 0.50

# Allowance for stamp skew between the cloud and the pose stream. At 20 m/s the
# ball covers 0.40 m in this time.
STAMP_SKEW_S = 0.020

# Matched detections needed inside the window to count as recalled. Tied to
# evasion.yaml's min_track_updates of 3 — fewer confirmations is a detection
# the aircraft cannot act on.
DEFAULT_MIN_MATCHED = 3


def match_radius_m(range_m: float, ball_speed_mps: float) -> float:
    """How far from truth a detection may sit and still be the same object.

    R(z) = max(MATCH_FLOOR_M, MATCH_SIGMAS * sigma(z)) + v * STAMP_SKEW_S,
    with sigma from the lane's own error model (0.30 m at 26 m, growing as z^2).

    Grows with range because the sensor's error does. A fixed tolerance would be
    too tight at 26 m (correct detections read as misses) or too loose at 5 m
    (a detection matches a ball a metre away).
    """
    sigma = float(depth_sigma_m(float(range_m)))
    return (max(MATCH_FLOOR_M, MATCH_SIGMAS * sigma)
            + float(ball_speed_mps) * STAMP_SKEW_S)


def interpolate_track(track, t: float):
    """Truth position at sim time `t`, linearly between the two nearest samples.

    Returns None outside the track's span, deliberately. gz's dynamic_pose/info
    carries only MOVING entities, so a settled ball stops appearing; holding the
    last sample would match late detections to a position nothing is at any more.
    A detection with no truth behind it is unmatched.

    `track` is [(t, (x, y, z)), ...]; it need not be sorted.
    """
    if not track:
        return None
    ordered = sorted(track, key=lambda s: s[0])
    if t < ordered[0][0] or t > ordered[-1][0]:
        return None

    lo, hi = 0, len(ordered) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ordered[mid][0] <= t:
            lo = mid
        else:
            hi = mid

    t0, p0 = ordered[lo]
    t1, p1 = ordered[hi]
    if t1 == t0:
        return tuple(float(c) for c in p0)
    frac = (t - t0) / (t1 - t0)
    return tuple(float(a) + frac * (float(b) - float(a))
                 for a, b in zip(p0, p1))


def void_reason(label: str, ball_track, drone_track):
    """Why this scenario cannot be scored at all, or None if it can.

    A void is neither a failure nor a pass: the bag cannot answer the question
    and is re-captured. It stays in the denominator meanwhile, so voiding can
    never shrink a set.

    Two conditions:
      - a POSITIVE with no projectile truth. There is nothing to match against,
        and scoring it would silently record a legitimate detection as a miss.
        Negatives are exempt: most of them have no projectile on purpose, and
        voiding them would delete the false-positive measurement.
      - ANY scenario with no drone truth. The match radius needs each
        detection's range from the camera, which comes from the drone's own
        transform, so without it there is no gate to apply.
    """
    if not drone_track:
        return ("no drone transform in /gz/dynamic_poses — the match radius "
                "needs each detection's range from the camera, and gz "
                "publishes only MOVING entities, so a still airframe drops out")
    if label == "positive" and not ball_track:
        return ("no projectile transform in /gz/dynamic_poses for a positive "
                "scenario — nothing to attribute detections to")
    return None


def score_scenario(detections, ball_track, drone_track,
                   ball_speed_mps: float, window,
                   min_matched: int = DEFAULT_MIN_MATCHED) -> dict:
    """Score one bag's detections against ground truth, matched by radius and window.

    `detections` and both tracks are [(t_sim, (x, y, z)), ...] in one common
    fixed frame. `window` is the (lower, upper) strict detection window from
    score_bags_logic.strict_window_bounds, applied first: a detection outside it
    belongs to a different measurement, so it is neither a match nor a false
    positive here. The window exists because every replay opens with a cold
    background map and a false-positive burst.

    Returns:
      in_window   int    detections considered
      matched     int    attributed to the projectile
      unmatched   int    everything else in the window — terrain, clutter, and
                         the airframe itself
      recalled    bool   matched >= min_matched
      fired       bool   unmatched >= min_matched
      errors_m    list   || detected - truth || for the matched ones, in order
      radii_m     list   the gate each matched detection cleared, so a run can
                         be re-checked against a different tolerance without
                         re-scoring the bag
      min_matched int    the K actually applied

    `recalled` and `fired` are independent and neither cancels the other: a
    scenario can be correctly recalled while also emitting spurious centroids.
    """
    lower, upper = float(window[0]), float(window[1])
    in_window = [(float(t), p) for t, p in detections if lower <= t <= upper]

    matched, unmatched = 0, 0
    errors_m: list[float] = []
    radii_m: list[float] = []

    for t, point in in_window:
        truth = interpolate_track(ball_track, t)
        drone = interpolate_track(drone_track, t)
        if truth is None or drone is None:
            # No truth behind this detection at this instant, so it is not
            # attributable -- which is exactly what unmatched means.
            unmatched += 1
            continue
        error = math.dist(point, truth)
        radius = match_radius_m(math.dist(point, drone), ball_speed_mps)
        if error <= radius:
            matched += 1
            errors_m.append(error)
            radii_m.append(radius)
        else:
            unmatched += 1

    return {
        "in_window": len(in_window),
        "matched": matched,
        "unmatched": unmatched,
        "recalled": matched >= min_matched,
        "fired": unmatched >= min_matched,
        "errors_m": errors_m,
        "radii_m": radii_m,
        "min_matched": int(min_matched),
    }
