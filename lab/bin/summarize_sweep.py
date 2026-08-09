#!/usr/bin/env python3
"""summarize_sweep.py — the sweep, scored on the counterfactual.

WHY THIS REPLACES THE OLD SCORE. dodge_battery's `success` is
`dodged and min_dist_m > hit_radius`, which cannot tell a dodge that saved the
aircraft from a throw that was never on target. Measured at 3.4 m: two of the
three 8 m/s "successes" had a counterfactual closest approach of 0.31 and
0.57 m -- already outside the 0.30 m hit radius before the vehicle moved. The
authoritative classification is therefore:

    WOULD_HAVE_MISSED  the throw missed on its own; the dodge proved nothing
    DODGE_SAVED        would have hit, did not -- the ONLY row that counts
    DODGE_FAILED       would have hit, did hit

NO_DODGE rows are folded into the same scheme rather than reported separately:
with no dodge the counterfactual IS the actual, so such a run is
WOULD_HAVE_MISSED when the ball missed anyway and DODGE_FAILED when it did not.
A trigger that never fired against a lethal throw is a failure, and hiding it
in its own bucket is how "0/17" once looked like a tuning problem.

Rows with expect_dodge=False are held out of that accounting entirely and
reported as the false-dodge count, which must be zero.

detection_range_m is an INPUT and is printed on every row.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import re
import sys
from collections import defaultdict

HIT_RADIUS_M = 0.30
CELL_RE = re.compile(r"_r(?P<range>[0-9.]+)\.csv$")


def _f(row, key):
    v = (row.get(key) or "").strip()
    if v in ("", "None", "NA"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _b(row, key):
    return (row.get(key) or "").strip().lower() == "true"


def load_cell(battery_csv, hit_radius):
    """Join one cell's battery rows to its counterfactual verdicts."""
    cf_path = battery_csv[:-4] + "_cf.csv"
    cf = {}
    if os.path.exists(cf_path):
        with open(cf_path, newline="") as fh:
            for r in csv.DictReader(fh):
                key = (r["scenario"], r["rep"])
                if key != ("", ""):
                    cf[key] = r

    runs = []
    with open(battery_csv, newline="") as fh:
        for r in csv.DictReader(fh):
            if _b(r, "error") or _b(r, "skipped"):
                # Harness errors are counted, never scored. A half-speed launch
                # or a spawn below the floor tests nothing.
                runs.append({"scenario": r["id"], "rep": r["rep"],
                             "harness_error": True,
                             "note": r.get("note", "")})
                continue

            c = cf.get((r["id"], r["rep"]))
            actual = _f(r, "min_dist_m")
            if c is not None:
                a = _f(c, "actual_min_m")
                actual = a if a is not None else actual
                cfm = _f(c, "counterfactual_min_m")
                # NO_DODGE: no dodge happened, so the counterfactual IS the
                # actual. Substituting it here is what lets a never-fired
                # trigger be scored against a lethal throw.
                if cfm is None:
                    cfm = actual
                delta = _f(c, "delta_m")
                verdict_src = c["verdict"]
            else:
                cfm, delta, verdict_src = None, None, "NO_CF"

            would_hit = None if cfm is None else (cfm <= hit_radius)
            hit = None if actual is None else (actual <= hit_radius)
            if would_hit is None or hit is None:
                verdict = "UNSCORED"
            elif not would_hit:
                verdict = "WOULD_HAVE_MISSED"
            elif hit:
                verdict = "DODGE_FAILED"
            else:
                verdict = "DODGE_SAVED"

            runs.append({
                "scenario": r["id"], "rep": r["rep"], "harness_error": False,
                "expect_dodge": _b(r, "expect_dodge"),
                "fired": _b(r, "dodged"),
                "speed": _f(r, "ball_speed_mps"),
                "actual": actual, "cf": cfm, "delta": delta,
                "tca": _f(r, "tca_s"), "lat": _f(r, "latency_ms"),
                "det": _f(r, "first_det_range_m"),
                # The counterfactual's own straight-line fit error. delta is
                # only believable when it clears this: at 3.4 m the measured
                # deltas (0.007-0.032 m) sat inside a 0.009-0.014 m residual,
                # so "the dodge moved it 2 cm" and "the fit was 1 cm off" are
                # the same number. Carried per run so that is checkable rather
                # than assumed either way.
                "resid": None if c is None else _f(c, "fit_resid_m"),
                "verdict": verdict, "cf_verdict": verdict_src,
            })
    return runs


def agg(vals):
    vals = [v for v in vals if v is not None]
    return (sum(vals) / len(vals)) if vals else None


def fmt(v, nd=3):
    return "-" if v is None else f"{v:.{nd}f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results-dir", default=os.path.expanduser("~/hz_lab/results"))
    ap.add_argument("--tag", default="sweep")
    ap.add_argument("--hit-radius", type=float, default=HIT_RADIUS_M)
    # ~3 m/s^2 is the airframe's measured capability (CLAUDE.md). The tilt
    # ceiling ATC_ANGLE_MAX=30 deg would permit g*tan(30) = 5.66, so this is
    # the conservative end of the real envelope, not a soft target.
    ap.add_argument("--airframe-accel", type=float, default=3.0)
    args = ap.parse_args()

    R = args.hit_radius
    cells = sorted(
        (p for p in glob.glob(os.path.join(args.results_dir, f"{args.tag}_r*.csv"))
         if CELL_RE.search(p)),
        key=lambda p: float(CELL_RE.search(p).group("range")))
    if not cells:
        print(f"no cells matching {args.tag}_r*.csv in {args.results_dir}")
        return 1

    print("=" * 112)
    print(f"SWEEP '{args.tag}' — scored on the COUNTERFACTUAL (hit radius {R} m)")
    print("  det_rng is an INPUT: what the oracle was TOLD it could see, never a measurement.")
    print("=" * 112)
    # FAILED is split, because the two halves indict different subsystems:
    #   noFire = the trigger never commanded anything  -> sensing / prediction
    #   hit    = it commanded, and the ball arrived anyway -> time / authority
    hdr = (f"{'det_rng':>7} {'scen':>5} {'speed':>6} {'n':>3} {'fired':>5} "
           f"{'wldHit':>6} {'SAVED':>5} {'F:noFire':>8} {'F:hit':>5} "
           f"{'missAny':>7} {'act_avg':>7} {'act_min':>7} {'cf_avg':>7} "
           f"{'delta':>7} {'resid':>6} {'tca':>6} {'lat_ms':>6} {'err':>3}")
    print(hdr)
    print("-" * 112)

    everything = []
    for path in cells:
        rng = float(CELL_RE.search(path).group("range"))
        runs = load_cell(path, R)
        by_scen = defaultdict(list)
        for r in runs:
            by_scen[r["scenario"]].append(r)

        for scen in sorted(by_scen):
            rs = by_scen[scen]
            errs = [r for r in rs if r["harness_error"]]
            ok = [r for r in rs if not r["harness_error"]]
            if not ok:
                print(f"{rng:>7} {scen:>5} {'-':>6} {0:>3} "
                      f"{'-':>5} {'-':>6} {'-':>5} {'-':>8} {'-':>5} {'-':>7} "
                      f"{'-':>7} {'-':>7} {'-':>7} {'-':>7} {'-':>6} {'-':>6} "
                      f"{'-':>6} {len(errs):>3}")
                continue

            expect = ok[0]["expect_dodge"]
            fired = sum(1 for r in ok if r["fired"])
            if expect:
                saved = sum(1 for r in ok if r["verdict"] == "DODGE_SAVED")
                fail_nofire = sum(1 for r in ok if r["verdict"] == "DODGE_FAILED"
                                  and not r["fired"])
                fail_hit = sum(1 for r in ok if r["verdict"] == "DODGE_FAILED"
                               and r["fired"])
                wld = saved + fail_nofire + fail_hit
                missed = sum(1 for r in ok if r["verdict"] == "WOULD_HAVE_MISSED")
            else:
                wld = saved = fail_nofire = fail_hit = missed = 0

            acts = [r["actual"] for r in ok]
            print(f"{rng:>7} {scen:>5} {fmt(agg([r['speed'] for r in ok]),1):>6} "
                  f"{len(ok):>3} {fired:>5} {wld:>6} {saved:>5} {fail_nofire:>8} "
                  f"{fail_hit:>5} {missed:>7} {fmt(agg(acts)):>7} "
                  f"{fmt(min([a for a in acts if a is not None], default=None)):>7} "
                  f"{fmt(agg([r['cf'] for r in ok])):>7} "
                  f"{fmt(agg([r['delta'] for r in ok])):>7} "
                  f"{fmt(agg([r['resid'] for r in ok])):>6} "
                  f"{fmt(agg([r['tca'] for r in ok])):>6} "
                  f"{fmt(agg([r['lat'] for r in ok]),1):>6} {len(errs):>3}")
            for r in ok:
                everything.append((rng, scen, r))
        print("-" * 112)

    # ── false dodges ────────────────────────────────────────────────────────
    print("\nFALSE DODGES (expect_dodge=false rows; anything but 0 is a regression)")
    fd = defaultdict(lambda: [0, 0])
    for rng, scen, r in everything:
        if not r["expect_dodge"]:
            fd[(rng, scen)][1] += 1
            if r["fired"]:
                fd[(rng, scen)][0] += 1
    if fd:
        for (rng, scen) in sorted(fd):
            f, n = fd[(rng, scen)]
            flag = "" if f == 0 else "   <<< FALSE DODGE"
            print(f"   det_rng {rng:>5}  {scen}: fired {f}/{n}{flag}")
    else:
        print("   (none in this sweep)")

    # ── the headline, split by speed ────────────────────────────────────────
    print("\nBY BALL SPEED (never blended — CLAUDE.md)")
    print(f"{'speed':>6} {'det_rng':>7} {'n':>3} {'wouldHit':>8} {'SAVED':>5} "
          f"{'FAILED':>6} {'save_rate_of_lethal':>19}")
    buckets = defaultdict(list)
    for rng, scen, r in everything:
        if r["expect_dodge"] and r["speed"] is not None:
            buckets[(round(r["speed"] / 2) * 2, rng)].append(r)
    for (spd, rng) in sorted(buckets):
        rs = buckets[(spd, rng)]
        saved = sum(1 for r in rs if r["verdict"] == "DODGE_SAVED")
        failed = sum(1 for r in rs if r["verdict"] == "DODGE_FAILED")
        lethal = saved + failed
        rate = f"{saved}/{lethal}" if lethal else "0/0 (none lethal)"
        print(f"{spd:>6} {rng:>7} {len(rs):>3} {lethal:>8} {saved:>5} "
              f"{failed:>6} {rate:>19}")

    # ── bottleneck arithmetic ───────────────────────────────────────────────
    print("\n" + "=" * 112)
    print("BOTTLENECK ARITHMETIC — per lethal throw (would have hit)")
    print("  d_req = hit_radius - counterfactual closest approach — how much further it had to move")
    print("  a_req = 2*d_req/tca^2 — the acceleration that would have been needed in the time it HAD")
    print(f"  a_req is then compared against the airframe's measured capability, {args.airframe_accel} m/s^2.")
    print("  delta is NOT divided by tca^2 here. At small tca that amplifies the counterfactual's own")
    print("  fit residual into three-digit accelerations; 'ok?' marks delta > 2*resid, i.e. the runs")
    print("  where the escape is even distinguishable from the straight-line fit error.")
    print("=" * 112)
    print(f"{'det_rng':>7} {'scen':>5} {'rep':>3} {'speed':>6} {'cf':>6} "
          f"{'d_req':>6} {'tca':>6} {'delta':>7} {'resid':>6} {'ok?':>4} "
          f"{'a_req':>7} {'verdict':>16} {'why':>8}")
    a_reqs = []
    real_deltas = []
    for rng, scen, r in everything:
        if not r["expect_dodge"] or r["verdict"] not in ("DODGE_FAILED", "DODGE_SAVED"):
            continue
        cf, tca, delta, resid = r["cf"], r["tca"], r["delta"], r["resid"]
        d_req = None if cf is None else max(R - cf, 0.0)
        a_req = None
        if d_req is not None and tca and tca > 0:
            a_req = 2.0 * d_req / (tca * tca)
            a_reqs.append((a_req, rng, r["speed"]))
        credible = (delta is not None and resid is not None
                    and abs(delta) > 2.0 * resid)
        if credible:
            real_deltas.append(delta)
        why = "noFire" if not r["fired"] else ("hit" if r["verdict"] == "DODGE_FAILED" else "-")
        print(f"{rng:>7} {scen:>5} {r['rep']:>3} {fmt(r['speed'],1):>6} "
              f"{fmt(cf,2):>6} {fmt(d_req,2):>6} {fmt(tca):>6} {fmt(delta):>7} "
              f"{fmt(resid):>6} {('yes' if credible else 'NO'):>4} "
              f"{fmt(a_req,1):>7} {r['verdict']:>16} {why:>8}")

    A = args.airframe_accel
    print(f"\n  AIRFRAME REFERENCE: {A} m/s^2 measured capability.")
    if a_reqs:
        over = [x for x in a_reqs if x[0] > A]
        print(f"  a_req exceeded it on {len(over)}/{len(a_reqs)} lethal throws that fired "
              f"(median a_req {sorted(x[0] for x in a_reqs)[len(a_reqs)//2]:.1f} m/s^2).")
    print(f"  Escape distinguishable from fit noise on {len(real_deltas)} throws"
          + (f" (mean {sum(real_deltas)/len(real_deltas):.3f} m)." if real_deltas else "."))

    t_needed = math.sqrt(2.0 * R / A)
    print(f"\n  REQUIREMENT AT {A} m/s^2, dead-centre throw (d_req = {R} m):")
    print(f"    tca needed after commit                    {t_needed:.2f} s")
    print("    closing distance that tca costs, and the detection range it implies once")
    print("    confirmation + pipeline are added (both measured per cell, printed above):")
    for v in (8.0, 14.0, 20.0):
        print(f"      {v:>4.0f} m/s : {v * t_needed:>5.2f} m after commit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
