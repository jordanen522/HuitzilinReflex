#!/usr/bin/env python3
"""Segment a dataflash log by WP_ACC PARM writes and report what the position
controller actually DEMANDED during each dodge in each arm.

WHY THIS EXISTS. The escape-displacement metric from hz_dodge_edges.py /
hz_maneuver_analyze.py is too noisy to settle the WP_ACC question: session
drift between two identically-configured 2.5 arms (0.0458 vs 0.0723 m at
t=0.30) exceeded the entire 2.5->5.0 difference. The controller's own
input-shaped feedforward acceleration is logged at full rate and is not noisy.

THE QUESTION. In the WP_ACC=5.0 arm, does |Dacc| ever exceed 2.5 m/s^2?
  yes -> the clamp genuinely lifted and bought no escape, so acceleration is
         not the binding term above ~2.5 and the limit is downstream.
  no  -> the write never reached the controller. ModeGuided::pva_control_start()
         latches the limit via NE_set_max_speed_accel_m(), so a mid-flight
         param change only takes effect when that runs again -- in which case
         the "5.0 arm" was never a 5.0 arm and the whole sweep is void.

Dodge windows come from GUIP.Type: 2 = position target (patrol), 4 = velocity
target (dodge), so the 2->4 transition marks each dodge exactly.

Reads a .BIN; writes nothing.
"""
import sys
import math
from collections import defaultdict

from pymavlink import mavutil

WINDOW_S = 1.0          # dodge window after the velocity target appears
GUIP_TYPE_VEL = 4       # velocity target == dodge
PIN_FRAC = 0.98         # within 2% of the clamp counts as pinned
PAIR_TOL_S = 0.010      # PSCN/PSCE are written back-to-back, us apart


def pair_ne(north, east):
    """Two-pointer nearest merge of the N and E position-controller streams.

    PSCN and PSCE are logged by separate calls that each stamp their own
    micros64(), so the timestamps are close but not identical. An exact dict
    join silently drops nearly every sample.
    """
    out = []
    j = 0
    for tn, dn, tan_ in north:
        while j + 1 < len(east) and abs(east[j + 1][0] - tn) <= abs(east[j][0] - tn):
            j += 1
        if not east:
            break
        te, de, tae_ = east[j]
        if abs(te - tn) <= PAIR_TOL_S:
            out.append((tn, math.hypot(dn, de), math.hypot(tan_, tae_)))
    return out


def main():
    path = sys.argv[1]
    mlog = mavutil.mavlink_connection(path)

    wpacc_writes = []
    guip = []
    north = []      # (t, DAN, TAN)
    east = []       # (t, DAE, TAE)
    att = []        # (t, roll_rad, pitch_rad)

    n = 0
    while True:
        m = mlog.recv_match(type=['PARM', 'GUIP', 'PSCN', 'PSCE', 'ATT'])
        if m is None:
            break
        n += 1
        t = getattr(m, 'TimeUS', None)
        if t is None:
            continue
        t = t / 1e6
        typ = m.get_type()
        if typ == 'PARM':
            if getattr(m, 'Name', '') == 'WP_ACC':
                wpacc_writes.append((t, float(m.Value)))
        elif typ == 'GUIP':
            guip.append((t, int(m.Type)))
        elif typ == 'PSCN':
            north.append((t, float(m.DAN), float(m.TAN)))
        elif typ == 'PSCE':
            east.append((t, float(m.DAE), float(m.TAE)))
        elif typ == 'ATT':
            att.append((t, float(m.Roll), float(m.Pitch)))

    print("log %s" % path)
    print("messages scanned: %d" % n)
    print("WP_ACC PARM writes: %d" % len(wpacc_writes))
    for t, v in wpacc_writes:
        print("   t=%10.2f  WP_ACC=%.3f" % (t, v))
    if not wpacc_writes:
        print("!! no WP_ACC writes in this log -- wrong log file")
        return

    north.sort()
    east.sort()
    dmag = pair_ne(north, east)
    print("PSCN %d  PSCE %d  paired %d" % (len(north), len(east), len(dmag)))

    guip.sort()
    dodges = []
    prev = None
    for t, ty in guip:
        if ty == GUIP_TYPE_VEL and prev != GUIP_TYPE_VEL:
            dodges.append(t)
        prev = ty
    print("dodge windows (GUIP.Type ->4 edges): %d" % len(dodges))

    def arm_of(t):
        val = None
        for tw, v in wpacc_writes:
            if tw <= t:
                val = v
            else:
                break
        return val

    att.sort()
    per_arm = defaultdict(list)
    for d0 in dodges:
        arm = arm_of(d0)
        if arm is None:
            continue
        seg = [(t, dm, tm) for (t, dm, tm) in dmag if d0 <= t <= d0 + WINDOW_S]
        if len(seg) < 5:
            continue
        peak_d = max(s[1] for s in seg)
        peak_t = max(s[2] for s in seg)
        pinned = sum(1 for s in seg if s[1] >= PIN_FRAC * arm) / len(seg)
        tilt = [math.hypot(r, p) * 180.0 / math.pi
                for (t, r, p) in att if d0 <= t <= d0 + WINDOW_S]
        per_arm[arm].append((peak_d, peak_t, pinned, max(tilt) if tilt else 0.0))

    print("")
    print("=" * 86)
    print("%8s %7s %10s %10s %7s %9s %14s %9s"
          % ("WP_ACC", "dodges", "max|Dacc|", "meanpeak", ">2.5", "pinned%",
             "meanpeak|Tacc|", "maxtilt"))
    print("=" * 86)
    for arm in sorted(per_arm):
        rows = per_arm[arm]
        peaks = [r[0] for r in rows]
        tpeaks = [r[1] for r in rows]
        pins = [r[2] for r in rows]
        tilts = [r[3] for r in rows]
        over = sum(1 for p in peaks if p > 2.5)
        print("%8.2f %7d %10.3f %10.3f %7d %9.1f %14.3f %9.1f"
              % (arm, len(rows), max(peaks), sum(peaks) / len(peaks), over,
                 100 * sum(pins) / len(pins), sum(tpeaks) / len(tpeaks),
                 max(tilts)))
    print("=" * 86)
    print("")
    print("per-dodge peak |Dacc| by arm:")
    for arm in sorted(per_arm):
        print("  WP_ACC=%.2f: %s"
              % (arm, ", ".join("%.3f" % r[0] for r in per_arm[arm])))


if __name__ == '__main__':
    main()
