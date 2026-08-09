#!/usr/bin/env python3
"""chain.py — does detection range actually buy tca, then displacement, then saves?

summarize_sweep.py already scores the sweep and prints per-cell tca/delta and a
by-speed save table; it is the authority on verdicts and this script does not
re-derive them. Two things it deliberately does not produce, and this adds:

  1. The causal chain as ONE monotonicity table per speed:
         det_rng -> mean tca -> mean delta -> saved/lethal
     If range buys tca but tca does not buy delta, the limit is the maneuver.
     If range does not buy tca, the limit is sensing/confirmation.

  2. a_eff = 2*delta/tca^2, RESIDUAL-GATED. summarize_sweep.py refuses a_eff
     outright because at small tca it turns the counterfactual's own fit error
     into a three-digit acceleration (its 9.0/S20/r0 row: tca 0.02 s, delta
     -0.004 m, |delta| < resid -> a_req 281 m/s^2, pure noise). Gating on
     |delta| >= 2*fit_resid_m keeps exactly the rows where the escape is
     distinguishable from the straight-line fit, and those are the only rows an
     acceleration may be quoted from. Ungated rows are counted, never averaged.

Speeds are never blended. S20N is the false-dodge control: it has no lethal
throw, so it has no save rate and is excluded here entirely (summarize_sweep.py
reports it). detection_range_m is an INPUT -- what the oracle was TOLD it could
see -- and is printed on every row. Read-only.
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import statistics

RESID_FACTOR = 2.0          # |delta| below this x fit_resid_m is not interpretable


def _f(row, key):
    v = (row.get(key) or "").strip()
    if v in ("", "None", "NA", "nan"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def load_cells(tag, results_dir):
    """{range: [(scenario, rep, cf_row, battery_row)]} for ONE tag only."""
    cells = {}
    for cf_path in sorted(glob.glob(os.path.join(results_dir, f"{tag}_r*_cf.csv"))):
        m = re.search(re.escape(tag) + r"_r([0-9.]+)_cf\.csv$", cf_path)
        if not m:
            continue
        rng = float(m.group(1))

        bat = {}
        bat_path = cf_path[:-len("_cf.csv")] + ".csv"
        if os.path.exists(bat_path):
            with open(bat_path) as f:
                for r in csv.DictReader(f):
                    # The scenario name is in whichever of id/combo carries it.
                    for key in ("id", "combo"):
                        v = (r.get(key) or "").strip()
                        if re.fullmatch(r"S\d+N?", v):
                            bat[(v, (r.get("rep") or "").strip())] = r
                            break

        rows = []
        with open(cf_path) as f:
            for c in csv.DictReader(f):
                sc, rep = c["scenario"].strip(), c["rep"].strip()
                rows.append((sc, rep, c, bat.get((sc, rep), {})))
        cells[rng] = rows
    return cells


def speed_of(scenario):
    m = re.match(r"S(\d+)", scenario)
    return int(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--results-dir", default=os.path.expanduser("~/hz_lab/results"))
    ap.add_argument("--hit-radius", type=float, default=0.30)
    args = ap.parse_args()

    cells = load_cells(args.tag, args.results_dir)
    if not cells:
        raise SystemExit(f"no {args.tag}_r*_cf.csv under {args.results_dir}")

    ranges = sorted(cells)
    # Real speeds only; S20N is the control and carries no lethal throw.
    scenarios = sorted({sc for rows in cells.values() for sc, *_ in rows
                        if not sc.endswith("N")}, key=speed_of)

    print("=" * 100)
    print(f"CHAIN '{args.tag}' — det_rng -> tca -> displacement -> DODGE_SAVED")
    print("  det_rng is an INPUT: what the oracle was TOLD it could see, never a measurement.")
    print(f"  a_eff averaged ONLY over throws with |delta| >= {RESID_FACTOR:g}x fit_resid_m.")
    print("=" * 100)

    for sc in scenarios:
        print(f"\n  {sc}  —  ball speed {speed_of(sc)} m/s")
        print("    det_rng  n  mean_tca  mean_delta  mean_a_eff  n_interp  mean_d_req  "
              "mean_a_req  SAVED/lethal")
        for rng in ranges:
            rows = [r for r in cells[rng] if r[0] == sc]
            if not rows:
                continue
            tcas, deltas, aeffs, dreqs, areqs = [], [], [], [], []
            saved = lethal = 0
            for sc_, rep, c, b in rows:
                verdict = c["verdict"].strip()
                cf = _f(c, "counterfactual_min_m")
                act = _f(c, "actual_min_m")
                tca = _f(b, "tca_s")
                delta = _f(c, "delta_m")
                resid = _f(c, "fit_resid_m")

                # No dodge -> the actual path IS the counterfactual path.
                ref = cf if cf is not None else act
                is_lethal = ref is not None and ref < args.hit_radius
                lethal += is_lethal
                saved += (verdict == "DODGE_SAVED")

                if tca:
                    tcas.append(tca)
                if delta is not None:
                    deltas.append(delta)
                if is_lethal and ref is not None:
                    d_req = args.hit_radius - ref
                    dreqs.append(d_req)
                    if tca:
                        areqs.append(2.0 * d_req / (tca * tca))
                # Residual gate: only these rows may be turned into an accel.
                if (delta is not None and resid and tca
                        and abs(delta) >= RESID_FACTOR * resid):
                    aeffs.append(2.0 * delta / (tca * tca))

            def m(a, p=4, w=8):
                return f"{statistics.mean(a):{w}.{p}f}" if a else " " * (w - 1) + "-"
            print(f"    {rng:7g} {len(rows):2d}  {m(tcas,3)}  {m(deltas,4,10)}  "
                  f"{m(aeffs,2,10)}  {len(aeffs):8d}  {m(dreqs,3,10)}  {m(areqs,2,10)}  "
                  f"{saved:>6}/{lethal}")

    print("\n" + "=" * 100)
    print("READING IT")
    print("  range buys tca, tca buys delta, delta buys saves   -> sensing-limited;")
    print("     the minimum range for reliable 20 m/s is where SAVED/lethal saturates.")
    print("  range buys tca but delta does not follow           -> maneuver/authority-limited;")
    print("     more range cannot fix it and a_eff will sit well under the airframe figure.")
    print("  range does not buy tca                             -> confirmation/pipeline-limited.")
    print("=" * 100)


if __name__ == "__main__":
    main()
