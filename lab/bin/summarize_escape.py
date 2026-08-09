#!/usr/bin/env python3
"""Compare dodge escape curves across arms of an experiment.

Input: one or more hz_dodge_response.py CSVs
(dodge_idx,t_since_cmd_s,escape_m,ideal_m,tca_s,miss_m).

The headline number is T30: the time after the dodge command at which escape
displacement reaches the 0.30 m hit radius, linearly interpolated between
samples. That is the quantity a longer detection range has to buy, so it
converts "the vehicle is sluggish" into "the vehicle needs N seconds", which
can be compared directly against tca at commit.

Dodges whose escape goes NEGATIVE (the vehicle moved opposite the commanded
escape direction) are counted and reported separately rather than averaged in:
they are a different failure, and burying them in a mean hides both.

  summarize_escape.py a.csv b.csv --labels "jerk 5" "jerk 20"
"""

from __future__ import annotations

import argparse
import collections
import csv
import statistics
import sys

HIT_RADIUS_M = 0.30


def load(path):
    per = collections.defaultdict(dict)
    tca = {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            i = int(r["dodge_idx"])
            per[i][float(r["t_since_cmd_s"])] = float(r["escape_m"])
            tca[i] = float(r["tca_s"])
    return per, tca


def t30(curve):
    """Interpolated time to reach HIT_RADIUS_M, or None if never reached."""
    ts = sorted(curve)
    prev_t, prev_e = 0.0, 0.0
    for t in ts:
        e = curve[t]
        if e >= HIT_RADIUS_M:
            if e == prev_e:
                return t
            return prev_t + (HIT_RADIUS_M - prev_e) * (t - prev_t) / (e - prev_e)
        prev_t, prev_e = t, e
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--labels", nargs="*", default=[])
    args = ap.parse_args()

    labels = list(args.labels) + list(args.files[len(args.labels):])

    print("escape displacement vs time since dodge command")
    print("(mean over dodges with positive escape; hit radius %.2f m)"
          % HIT_RADIUS_M)
    print("")
    times = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 1.00)
    hdr = "%-14s %4s " % ("arm", "n") + " ".join("%7.2fs" % t for t in times) + "   %8s" % "T30_s"
    print(hdr)
    print("-" * len(hdr))

    for path, label in zip(args.files, labels):
        try:
            per, tca = load(path)
        except (OSError, ValueError) as e:
            print("%-14s  could not read: %s" % (label, e))
            continue
        good, bad = {}, []
        for i, curve in per.items():
            if curve.get(1.00, 0.0) <= 0.0:
                bad.append(i)
            else:
                good[i] = curve
        row = "%-14s %4d " % (label, len(good))
        for t in times:
            vals = [c[t] for c in good.values() if t in c]
            row += " %7s" % ("%.4f" % statistics.mean(vals) if vals else "-")
        t30s = [v for v in (t30(c) for c in good.values()) if v is not None]
        row += "   %8s" % ("%.3f" % statistics.mean(t30s) if t30s else ">1.0")
        print(row)
        if bad:
            tcas = ", ".join("%.2f" % tca[i] for i in bad)
            print("%-14s  %d dodge(s) with NON-POSITIVE escape, excluded "
                  "(tca at commit: %s)" % ("", len(bad), tcas))

    print("")
    print("T30 = interpolated seconds for the escape to clear the hit radius.")
    print("Compare it against tca at commit: if tca < T30 the ball arrives")
    print("before the vehicle is clear, whatever the trigger did.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
