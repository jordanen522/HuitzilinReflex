#!/usr/bin/env python3
"""
hz_funnel_attribute.py — attribute a detection MISS to a specific gate.

Offline join of two recordings made during one throw:
  * the detector's per-frame dump   (detector.yaml: debug_dump_dir)
  * ground truth                    (scripts/hz_truth_probe.py)

The open Week-4 question is why ~1.7 m of range is detected and then discarded:
the ball is first seen at ~4.5 m, but the track that fires a dodge always begins
at ~2.8 m and is always exactly three consecutive frames old. Something drops
the ball on the frames in between, and the funnel log cannot say what — it
reports survivor counts, and a frame with no ball-sized cluster reads the same
whether the background map absorbed the ball or the ball fell under
cluster_min_points.

This resolves that by asking, per frame, where the ball's own points went:

    absent      the cloud never contained a point near the ball
                (occlusion, range/angle gate, or nothing rendered)
    background  points were there, and the background map called them background
    unclustered points survived the diff but formed no cluster
                (cluster_min_points / cluster_tolerance_m)
    outsized    a cluster sat on the ball but exceeded cluster_max_points
    outvoted    a ball cluster existed and a LARGER cluster won the pick
    lowscore    the ball won the pick and min_publish_score rejected it
    flood       frame skipped wholesale on fg_max_points
    OK          published

Frames are keyed by the cloud's sim stamp, which is the only clock that joins
the two recordings; the callback blocks the executor for longer than a frame
period, so a wall-clock join would be ambiguous by a whole frame.

Usage:
    python3 scripts/hz_funnel_attribute.py \
        --dump /tmp/hz_dump --truth /tmp/truth.csv
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict

import numpy as np

# The dumped cloud is voxelised at 0.05 m and the camera sees only the ball's
# near hemisphere, so a point belonging to an 80 mm ball can sit ~0.1 m from the
# ball's true CENTRE. 0.20 m is generous enough not to miss the ball's own
# points and tight enough that a wall 1 m behind it cannot be mistaken for them.
NEAR_POINT_M = 0.20
# A cluster centroid this far from the ball is a different object. Wider than
# NEAR_POINT_M because a cluster that merges the ball with a neighbouring
# surface has a centroid pulled off the ball — and that is still "the ball was
# clustered", just badly.
NEAR_CLUSTER_M = 0.40


def load_truth(path: str) -> tuple[dict, dict]:
    """Group truth rows by (kind, name) -> (times, Nx3 positions), time-sorted.

    Returns (positions, recv) where recv maps the same key to the ROS sim clock
    at arrival, needed to bracket the two clocks (see fit_gz_offset).
    """
    rows: dict[tuple[str, str], list] = defaultdict(list)
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows[(r["kind"], r["name"])].append(
                (float(r["stamp_s"]), float(r["recv_s"]),
                 float(r["x"]), float(r["y"]), float(r["z"])))
    out, recv = {}, {}
    for key, vals in rows.items():
        a = np.array(sorted(vals), dtype=float)
        out[key] = (a[:, 0], a[:, 2:5])
        recv[key] = a[:, 1]
    return out, recv


def fit_odom_clock_offset(truth: dict, recv: dict, drone_model: str):
    """Seconds to ADD to a /huitzilin/odom stamp to reach the Gazebo sim clock.

    Two clocks are in play, measured live 2026-07-27:

        /oak/points        631.951 s          Gazebo sim clock
        /gz/dynamic_poses  631.9xx s          Gazebo sim clock
        /huitzilin/odom    1785183152.876 s   WALL clock

    mav_bridge, patrol and telemetry_logger run with use_sim_time=False —
    week2_sitl.launch.py never sets it — while every Gazebo-sourced node runs on
    sim time. So the dumped frames and the ball's true pose share a clock and
    join directly; only odom needs shifting, and it is needed only to establish
    the constant world->odom translation.

    Bracketed by arrival time (recv_s is on the sim clock), then refined by the
    only available check: the drone appears in both streams, so the right offset
    is the one that makes its world track and its odom track differ by a pure
    constant. The residual achieved is reported, so a bad fit cannot pass
    silently.
    """
    key = ("pose", drone_model)
    if key not in truth:
        return None, None, "no pose rows for the drone model"
    if ("odom", "drone") not in truth:
        return None, None, "no odom rows"

    wt, wp = truth[key]
    ot, op = truth[("odom", "drone")]
    # recv - stamp is (clock constant - transport delay); the max over thousands
    # of samples is the least-delayed one, bracketing the constant to a few ms.
    coarse = float(np.max(recv[("odom", "drone")] - ot))

    def spread(c: float) -> float:
        t = ot + c
        sel = (t >= wt[0]) & (t <= wt[-1])
        if sel.sum() < 50:
            return np.inf
        resid = np.stack([op[sel, i] - np.interp(t[sel], wt, wp[:, i])
                          for i in range(3)], axis=1)
        return float(np.linalg.norm(resid.std(axis=0)))

    grid = coarse + np.arange(-0.500, 0.500, 0.001)
    scores = np.array([spread(c) for c in grid])
    best = int(np.argmin(scores))
    if not np.isfinite(scores[best]):
        return None, None, "no overlap between the pose and odom streams"
    return float(grid[best]), float(scores[best]), ""


def interp_xyz(t_query: float, times: np.ndarray, pos: np.ndarray):
    """Linear interpolation, refusing to extrapolate.

    Returns None outside the recorded span. A ball's position guessed past the
    end of its own track is exactly the kind of silently-wrong number this
    project has already been burned by.
    """
    if t_query < times[0] or t_query > times[-1]:
        return None
    return np.array([np.interp(t_query, times, pos[:, i]) for i in range(3)])


def world_to_odom_offset(truth: dict, drone_model: str):
    """Constant translation from Gazebo world into the odom frame.

    Both are ENU and odom's origin is the drone's spawn point, so the frames
    differ by a translation only. Measured rather than assumed: the spread of
    the per-sample offset is reported, and a large spread means this assumption
    is wrong and every range below it is wrong too.
    """
    if ("pose", drone_model) not in truth or ("odom", "drone") not in truth:
        return None, None
    wt, wp = truth[("pose", drone_model)]
    ot, op = truth[("odom", "drone")]
    offs = []
    for t, p in zip(ot, op):
        w = interp_xyz(t, wt, wp)
        if w is not None:
            offs.append(p - w)
    if not offs:
        return None, None
    offs = np.array(offs)
    return offs.mean(axis=0), offs.std(axis=0)


def classify(d: dict, ball_odom: np.ndarray) -> tuple[str, str]:
    """Return (verdict, detail) for one dumped frame."""
    if "pts" not in d:
        # Died before the background stage; the counts say where.
        for key, label in (("n_raw", "raw=0"), ("n_range", "range=0"),
                           ("n_angle", "angle=0"), ("n_voxel", "voxel=0")):
            if int(d.get(key, -1)) == 0:
                return "absent", label
        return "absent", "no cloud recorded"

    pts = d["pts"]
    fg = d["fg"]
    dist = np.linalg.norm(pts - ball_odom, axis=1)
    near = dist < NEAR_POINT_M
    n_near = int(near.sum())
    if n_near == 0:
        return "absent", f"closest cloud point {dist.min():.2f} m from ball"

    n_near_fg = int((near & fg).sum())
    if n_near_fg == 0:
        return "background", f"{n_near} pts on ball, all called background"

    if bool(d.get("fg_flood", False)):
        return "flood", f"fg={int(d['n_fg'])} > fg_max_points"

    detail_fg = f"{n_near_fg}/{n_near} ball pts foreground"

    sizes = d.get("cl_size", np.zeros(0, int))
    if sizes.size == 0:
        return "unclustered", f"{detail_fg}, no clusters at all"
    cents = d["cl_centroid"]
    cdist = np.linalg.norm(cents - ball_odom, axis=1)
    hit = int(np.argmin(cdist))
    if cdist[hit] > NEAR_CLUSTER_M:
        return "unclustered", (f"{detail_fg}, nearest cluster "
                               f"{cdist[hit]:.2f} m away")

    size = int(sizes[hit])
    ext = float(d["cl_extent"][hit])
    detail_cl = f"{detail_fg}, cluster {size} pts extent {ext:.2f} m"

    if "best_size" not in d:
        # Every split cluster failed cluster_max_points.
        return "outsized", detail_cl
    best_c = d["best_centroid"]
    best_off = float(np.linalg.norm(best_c - ball_odom))
    if best_off > NEAR_CLUSTER_M:
        return "outvoted", (f"{detail_cl}, but best was {int(d['best_size'])} pts "
                            f"at {best_off:.2f} m")
    if not bool(d.get("published", False)):
        return "lowscore", f"{detail_cl}, score {float(d['score']):.3f}"
    return "OK", detail_cl


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True, help="detector debug_dump_dir")
    ap.add_argument("--truth", required=True, help="hz_truth_probe.py CSV")
    ap.add_argument("--drone-model", default="iris_depth")
    ap.add_argument("--ball", default="", help="ball model name (default: each)")
    ap.add_argument("--max-range", type=float, default=6.0,
                    help="only report frames with the ball inside this range")
    args = ap.parse_args()

    truth, recv = load_truth(args.truth)
    k, fit_resid, why = fit_odom_clock_offset(truth, recv, args.drone_model)
    if k is None:
        print(f"cannot align clocks ({why}); "
              f"names seen: {sorted({n for _, n in truth})}", file=sys.stderr)
        return 1
    print(f"odom->gz clock offset {k:.3f} s "
          f"(drone track residual {fit_resid:.3f} m at the fit; "
          f"a large residual here means the fit failed and every range below is "
          f"measured at the wrong instant of the flight)")
    # Only odom is on the wall clock; pose rows and the dumped frames are both
    # already on the Gazebo clock and need no shift.
    truth = {key: ((t + k, p) if key[0] == "odom" else (t, p))
             for key, (t, p) in truth.items()}

    offset, spread = world_to_odom_offset(truth, args.drone_model)
    if offset is None:
        print(f"no truth for drone model '{args.drone_model}' — "
              f"names seen: {sorted({n for _, n in truth})}", file=sys.stderr)
        return 1
    print(f"world->odom offset {np.round(offset, 3)} "
          f"(per-sample spread {np.round(spread, 3)} m; a large spread here "
          f"invalidates every range below)")

    files = sorted(glob.glob(os.path.join(args.dump, "f_*.npz")),
                   key=lambda p: int(os.path.basename(p)[2:-4]))
    if not files:
        print(f"no dumps in {args.dump}", file=sys.stderr)
        return 1
    stamps = np.array([int(os.path.basename(p)[2:-4]) * 1e-9 for p in files])
    print(f"{len(files)} dumped frames, "
          f"sim span {stamps[0]:.3f}..{stamps[-1]:.3f} s")

    balls = ([args.ball] if args.ball
             else sorted(n for k, n in truth if k == "pose" and n.startswith("ball")))
    if not balls:
        print("no ball_* model in the truth log — was a throw made?",
              file=sys.stderr)
        return 1

    drone_t, drone_p = truth[("pose", args.drone_model)]
    for ball in balls:
        bt, bp = truth[("pose", ball)]
        print(f"\n=== {ball} — flight {bt[0]:.3f}..{bt[-1]:.3f} s "
              f"({len(bt)} truth samples) ===")
        print(f"{'sim_t':>10} {'range':>6} {'gap':>7}  verdict      detail")

        tally: dict[str, int] = defaultdict(int)
        prev_t = None
        in_window = np.where((stamps >= bt[0]) & (stamps <= bt[-1]))[0]
        for i in in_window:
            t = float(stamps[i])
            b_w = interp_xyz(t, bt, bp)
            d_w = interp_xyz(t, drone_t, drone_p)
            if b_w is None or d_w is None:
                continue
            rng = float(np.linalg.norm(b_w - d_w))
            if rng > args.max_range:
                continue
            d = dict(np.load(files[i], allow_pickle=False))
            verdict, detail = classify(d, b_w + offset)
            gap = "" if prev_t is None else f"{(t - prev_t) * 1000:.0f}ms"
            prev_t = t
            tally[verdict] += 1
            print(f"{t:10.3f} {rng:6.2f} {gap:>7}  {verdict:<11}  {detail}")

        # Frames the camera published that the detector never processed: a
        # dropped cloud breaks a consecutive run exactly like a failed gate, and
        # from outside the two are indistinguishable. The period is taken from
        # the run's own median gap rather than assumed to be 15 Hz — the dump
        # itself costs latency, so the achieved rate is the honest baseline.
        if len(in_window) > 1:
            gaps = np.diff(stamps[in_window])
            period = float(np.median(gaps))
            missed = int(np.clip(np.round(gaps / period) - 1, 0, None).sum())
            print(f"\n         frame period (median) {period * 1000:.0f} ms "
                  f"= {1 / period:.1f} Hz")
            print(f"\nsummary: {dict(sorted(tally.items()))}")
            print(f"         ~{missed} camera frames never reached the detector "
                  f"during this flight (max gap {gaps.max() * 1000:.0f} ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
