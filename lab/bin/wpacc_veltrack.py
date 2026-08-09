#!/usr/bin/env python3
"""Does the vehicle ACHIEVE the commanded velocity step during a dodge?

WHY. wpacc_dataflash.py showed the acceleration clamp is not binding at or
above the 2.5 default (peak demand 1.43 m/s^2 against a 5.0 clamp), while
commanded total acceleration and achieved tilt both ROSE with the clamp
(1.55 -> 3.09 -> 3.81 m/s^2; 21 -> 22 -> 28 deg against a 30 deg
ATC_ANGLE_MAX) and escape displacement stayed flat. More acceleration is
being asked for and more tilt achieved without more displacement, so the
deficit is not in the acceleration limit.

Escape at t=1.0 s averages 0.47 m against an ideal of 1.5 m, and against
~1.05 m for a ramp limited only by WP_ACC=2.5. The vehicle delivers under
half of what its own clamp permits.

THE DISCRIMINATOR. PSCN/PSCE log the whole chain per axis:
    DVN = desired velocity  (the shaped input, what the dodge asked for)
    TVN = target velocity   (after the position controller)
    VN  = achieved velocity (what the vehicle did)
If VN tracks DVN, the controller is doing its job and the COMMAND is small --
the escape never asked for much. If VN lags DVN badly, the vehicle is being
asked and failing to deliver, and the limit is in the airframe or the
attitude loop.

Also reports how long each GUIDED velocity target (GUIP.Type == 4) actually
persisted, to confirm the 1.0 s dodge_duration_s stream is not truncated.

Reads a .BIN; writes nothing.
"""
import sys
import math
from collections import defaultdict

from pymavlink import mavutil

GUIP_TYPE_VEL = 4
WINDOW_S = 1.0
PAIR_TOL_S = 0.010


def pair_streams(north, east):
    """Two-pointer nearest merge; PSCN/PSCE stamp their own micros64()."""
    out = []
    j = 0
    if not east:
        return out
    for rec_n in north:
        tn = rec_n[0]
        while j + 1 < len(east) and abs(east[j + 1][0] - tn) <= abs(east[j][0] - tn):
            j += 1
        if abs(east[j][0] - tn) <= PAIR_TOL_S:
            out.append((tn, rec_n[1:], east[j][1:]))
    return out


def main():
    path = sys.argv[1]
    mlog = mavutil.mavlink_connection(path)

    wpacc_writes = []
    guip = []       # (t, type, vx, vy)
    north = []      # (t, DVN, TVN, VN)
    east = []       # (t, DVE, TVE, VE)

    while True:
        m = mlog.recv_match(type=['PARM', 'GUIP', 'PSCN', 'PSCE'])
        if m is None:
            break
        t = getattr(m, 'TimeUS', None)
        if t is None:
            continue
        t = t / 1e6
        typ = m.get_type()
        if typ == 'PARM':
            if getattr(m, 'Name', '') == 'WP_ACC':
                wpacc_writes.append((t, float(m.Value)))
        elif typ == 'GUIP':
            guip.append((t, int(m.Type), float(m.vX), float(m.vY)))
        elif typ == 'PSCN':
            north.append((t, float(m.DVN), float(m.TVN), float(m.VN)))
        elif typ == 'PSCE':
            east.append((t, float(m.DVE), float(m.TVE), float(m.VE)))

    north.sort()
    east.sort()
    guip.sort()
    merged = pair_streams(north, east)
    print("paired PSCN/PSCE velocity samples: %d" % len(merged))

    # dodge windows + how long the velocity target persisted
    dodges = []
    prev = None
    start = None
    for t, ty, vx, vy in guip:
        if ty == GUIP_TYPE_VEL and prev != GUIP_TYPE_VEL:
            start = (t, vx, vy)
        elif ty != GUIP_TYPE_VEL and prev == GUIP_TYPE_VEL and start is not None:
            dodges.append((start[0], t - start[0], start[1], start[2]))
            start = None
        prev = ty
    if start is not None:
        dodges.append((start[0], float('nan'), start[1], start[2]))
    print("dodge windows: %d" % len(dodges))

    def arm_of(t):
        val = None
        for tw, v in wpacc_writes:
            if tw <= t:
                val = v
            else:
                break
        return val

    per_arm = defaultdict(list)
    for d0, dur, cvx, cvy in dodges:
        arm = arm_of(d0)
        if arm is None:
            continue
        seg = [s for s in merged if d0 <= s[0] <= d0 + WINDOW_S]
        if len(seg) < 5:
            continue
        v0n, v0e = seg[0][1][2], seg[0][2][2]
        v0 = math.hypot(v0n, v0e)
        d_step = 0.0
        a_step = 0.0
        max_lag = 0.0
        for t, (dvn, tvn, vn), (dve, tve, ve) in seg:
            d_step = max(d_step, math.hypot(dvn - v0n, dve - v0e))
            a_step = max(a_step, math.hypot(vn - v0n, ve - v0e))
            max_lag = max(max_lag, math.hypot(dvn - vn, dve - ve))
        cmd = math.hypot(cvx, cvy)
        per_arm[arm].append((dur, cmd, v0, d_step, a_step, max_lag))

    print("")
    print("=" * 92)
    print("%7s %6s %8s %9s %10s %11s %11s %9s"
          % ("WP_ACC", "n", "dur_s", "cmd|v|", "v_at_cmd", "desired_dv",
             "achieved_dv", "max_lag"))
    print("=" * 92)
    for arm in sorted(per_arm):
        rows = per_arm[arm]
        n = len(rows)

        def mean(i):
            vals = [r[i] for r in rows if r[i] == r[i]]
            return sum(vals) / len(vals) if vals else float('nan')

        print("%7.2f %6d %8.2f %9.2f %10.2f %11.3f %11.3f %9.3f"
              % (arm, n, mean(0), mean(1), mean(2), mean(3), mean(4), mean(5)))
    print("=" * 92)
    print("")
    print("legend: desired_dv = peak |shaped desired velocity - velocity at command|")
    print("        achieved_dv = peak |actual velocity - velocity at command|")
    print("        max_lag = peak |desired - achieved| within the window")
    print("")
    for arm in sorted(per_arm):
        print("  WP_ACC=%.2f achieved_dv per dodge: %s"
              % (arm, ", ".join("%.2f" % r[4] for r in per_arm[arm])))
        print("  WP_ACC=%.2f desired_dv per dodge: %s"
              % (arm, ", ".join("%.2f" % r[3] for r in per_arm[arm])))
        print("  WP_ACC=%.2f duration_s per dodge: %s"
              % (arm, ", ".join("%.2f" % r[0] for r in per_arm[arm])))


if __name__ == '__main__':
    main()
