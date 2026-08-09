#!/usr/bin/env python3
"""Re-aggregate a dodge-battery CSV, split by scenario, with failures ATTRIBUTED.

Two corrections this exists to enforce:

1. `dodged` is NOT the outcome. dodge_battery.py line 799 defines
   success = dodged and min_dist_m > hit_radius_m, while dodged is merely
   len(events) > 0 -- a dodge COMMAND fired. Reading `dodged` as the result
   scores a vehicle that jinked and was hit anyway as a win.

2. dodge_battery.py's own report blends across ball speeds, which
   docs/dodge_battery_runbook.md section 3 says not to quote.

The failure split is the point. For expect_dodge rows a failure is either:
    NO-FIRE  -- no dodge command at all -> a SENSING/TRIGGER bound
    HIT      -- fired, but min_dist <= hit_radius -> an AUTHORITY bound
Those two have completely different fixes, and a single success rate hides
which one is binding.

off_target rows (the harness's own throw missed by more than off_target_tol_m)
are reported separately and excluded from the denominator: a ball that was
never going to hit is not a dodge the vehicle failed.

  summarize_battery.py FILE.csv --label "oracle range 9 m"
"""

from __future__ import annotations

import argparse
import collections
import csv
import statistics
import sys


def truthy(v) -> bool:
    return (v or "").strip().lower() in ("true", "1", "yes")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def mean_or_dash(vals, fmt="%.2f"):
    vals = [v for v in vals if v is not None]
    return (fmt % statistics.mean(vals)) if vals else "-"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file")
    ap.add_argument("--label", default="")
    ap.add_argument("--hit-radius", type=float, default=0.30)
    args = ap.parse_args()

    with open(args.csv_file) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("EMPTY: %s" % args.csv_file)
        return 1

    def blank():
        return {"n": 0, "fired": 0, "ok": 0, "nofire": 0, "hit": 0,
                "skip": 0, "offtgt": 0, "err": 0, "expect": True,
                "speeds": [], "det": [], "tca": [], "lat": [], "mind": []}

    scen = collections.defaultdict(blank)

    for r in rows:
        sid = r.get("id", "?")
        a = scen[sid]
        a["expect"] = truthy(r.get("expect_dodge"))
        if truthy(r.get("skipped")):
            a["skip"] += 1
            continue
        if truthy(r.get("error")):
            a["err"] += 1
            continue
        if truthy(r.get("off_target")):
            a["offtgt"] += 1
            continue

        a["n"] += 1
        a["speeds"].append(fnum(r.get("ball_speed_mps")))
        fired = truthy(r.get("dodged"))
        if fired:
            a["fired"] += 1
        if truthy(r.get("success")):
            a["ok"] += 1
        elif a["expect"]:
            if not fired:
                a["nofire"] += 1
            else:
                a["hit"] += 1
        for key, col in (("det", "first_det_range_m"), ("tca", "tca_s"),
                         ("lat", "latency_ms"), ("mind", "min_dist_m")):
            a[key].append(fnum(r.get(col)))

    if args.label:
        print("== %s" % args.label)
    print("file: %s   (success = fired AND min_dist > %.2f m)"
          % (args.csv_file, args.hit_radius))
    print("")
    hdr = ("%-5s %-7s %-7s %-8s %-8s %-8s %-7s %-8s %-8s %-8s"
           % ("id", "speed", "expect", "SUCCESS", "fired",
              "fail:why", "det_rng", "tca_s", "lat_ms", "min_dist"))
    print(hdr)
    print("-" * len(hdr))
    for sid in sorted(scen):
        a = scen[sid]
        why = []
        if a["nofire"]:
            why.append("%dxNO-FIRE" % a["nofire"])
        if a["hit"]:
            why.append("%dxHIT" % a["hit"])
        extra = []
        if a["skip"]:
            extra.append("%d skipped" % a["skip"])
        if a["offtgt"]:
            extra.append("%d off-target" % a["offtgt"])
        if a["err"]:
            extra.append("%d error" % a["err"])
        print("%-5s %-7s %-7s %-8s %-8s %-8s %-7s %-8s %-8s %-8s%s"
              % (sid, mean_or_dash(a["speeds"], "%.1f"),
                 "dodge" if a["expect"] else "NO-dodge",
                 "%d/%d" % (a["ok"], a["n"]),
                 "%d/%d" % (a["fired"], a["n"]),
                 ",".join(why) or "-",
                 mean_or_dash(a["det"]), mean_or_dash(a["tca"], "%.3f"),
                 mean_or_dash(a["lat"], "%.0f"), mean_or_dash(a["mind"]),
                 ("   [" + "; ".join(extra) + "]") if extra else ""))

    print("")
    print("ATTRIBUTION (expect_dodge rows only):")
    tot_n = sum(a["n"] for a in scen.values() if a["expect"])
    tot_ok = sum(a["ok"] for a in scen.values() if a["expect"])
    tot_nf = sum(a["nofire"] for a in scen.values() if a["expect"])
    tot_hit = sum(a["hit"] for a in scen.values() if a["expect"])
    print("  scored throws      %d" % tot_n)
    print("  succeeded          %d" % tot_ok)
    print("  failed NO-FIRE     %d   <- sensing/trigger bound" % tot_nf)
    print("  failed HIT         %d   <- authority/dynamics bound" % tot_hit)
    fd = [(s, a) for s, a in scen.items() if not a["expect"]]
    for sid, a in sorted(fd):
        print("  false-dodge %s: fired %d/%d (want 0)" % (sid, a["fired"], a["n"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
