#!/usr/bin/env python3
"""Did the dodge setpoints actually reach the vehicle?

Counts rising edges of GUIDED desired velocity (DVN/DVE) in a dataflash log
and, for each one, measures ACTUAL displacement from SIM2 ground truth over
the following second -- measured against the extrapolated pre-command track,
not against a fixed point, because during a battery the vehicle is PATROLLING.
hover_reference fits a constant-velocity model to the pre-command window and
ref_pos extrapolates it, so what is reported is escape relative to where the
vehicle would have been had nothing been commanded.

This distinguishes "the trigger fired" from "the vehicle moved". Pre-fix, a
zero Accel latched alongside every dodge selected MASK_VEL_ACCEL on
MAV_FRAME_BODY_OFFSET_NED, which ArduCopter discards whole: the trigger fired,
the setpoint never landed, DV never stepped. Log 00000013.BIN showed 5 DV
edges against 33 fired dodges.

Deliberately does NOT use analyse_trial: that path carries the edge-to-event
order-matching bug and the unreliable dir_err_deg. Displacement MAGNITUDE
needs neither.
"""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hz_maneuver_analyze import (load_bin, find_edges, hover_reference,
                                 ref_pos, slice_between)

WIN_S = 1.0

def main():
    path = sys.argv[1]
    sim2, pscn, psce, guip, att = load_bin(path)
    edges = find_edges(pscn, psce)
    print("log             %s" % path)
    print("SIM2 %d   PSCN %d   PSCE %d" % (len(sim2), len(pscn), len(psce)))
    print("DV rising edges %d" % len(edges))
    if not edges:
        print("  (no commanded-velocity edges -- nothing reached GUIDED)")
        return
    print("")
    print("  %-9s %-10s %-10s %-10s %-6s" % ("t_edge", "cmd|DV|", "disp_max", "dvel_max", "n_pre"))
    disps = []
    for (t, dvn, dve, i) in edges:
        cmd = math.hypot(dvn, dve)
        ref = hover_reference(sim2, t)
        if ref is None:
            print("  %-9.2f %-10.3f  (no pre-command window)" % (t, cmd))
            continue
        seg = slice_between(sim2, t, t + WIN_S)
        if not seg:
            continue
        vrn, vre = ref["n"][1], ref["e"][1]
        dmax, dvmax = 0.0, 0.0
        for s in seg:
            pn, pe, pd = ref_pos(ref, s[0])
            dmax = max(dmax, math.hypot(s[1] - pn, s[2] - pe))
            dvmax = max(dvmax, math.hypot(s[4] - vrn, s[5] - vre))
        disps.append(dmax)
        print("  %-9.2f %-10.3f %-10.3f %-10.3f %-6d"
              % (t, cmd, dmax, dvmax, ref["n_pre"]))
    if disps:
        disps.sort()
        n = len(disps)
        print("")
        print("  displacement vs extrapolated track, %.1f s window, n=%d" % (WIN_S, n))
        print("    min %.3f   median %.3f   max %.3f   mean %.3f"
              % (disps[0], disps[n // 2], disps[-1], sum(disps) / n))

main()
