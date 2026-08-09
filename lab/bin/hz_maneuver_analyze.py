#!/usr/bin/env python3
"""hz_maneuver_analyze.py — the response curve, from the FC's own record.

Reads the dataflash written while hz_maneuver_probe.py flew, and answers one
question: after a dodge is commanded, what does the vehicle actually do?

WHY THE DATAFLASH AND NOT ROS ODOM
  odom is 30 Hz -- 1.5 samples per 50 ms bin, which cannot carry the grid this
  is asked for -- and it arrives over MAVLink, so its timestamps carry
  transport delay. SIM2 is the SIMULATOR'S OWN state at 400 Hz (2.5 ms): truth,
  not an estimate, with no telemetry lag and no EKF lag. Everything below is
  measured inside the flight controller's clock, so no cross-clock alignment
  is needed or attempted.

THE FOUR SIGNALS, and what each one rules in or out:
  DVN/DVE  desired velocity  — what GUIDED was asked for. If this never equals
                               the commanded 1.5 m/s, the SETPOINT INTERFACE is
                               dropping or reinterpreting the command.
  TVN/TVE  target velocity   — what the position controller shaped that into.
                               DV steps; if TV RAMPS toward it over hundreds of
                               ms, the limit is COMMAND SHAPING, and the ramp
                               slope is the acceleration limit in m/s^2.
  VN/VE    actual velocity   — what the controller achieved against its target.
                               TV-vs-V is the controller's tracking; DV-vs-TV is
                               the shaping. They are different failures.
  TAN/TAE  target accel      — the shaped acceleration demand. If this saturates
  AN/AE    actual accel        at a ceiling, that ceiling is the limit.

T = 0 IS STATED TWICE, ON PURPOSE
  PSC* and GUIP log at 10 Hz, so the moment the command reached the controller
  is only known to within one 100 ms sample. Zeroing the curve there would put
  a +/-100 ms smear on a +50 ms grid point, which is worse than useless. So:
    t_edge   the first PSC sample carrying the new desired velocity. The
             command arrived in (t_edge - 0.1, t_edge]. Bracket, not a value.
    t_onset  the first SIM2 sample where horizontal velocity leaves the hover
             noise floor by ONSET_SIGMA and stays out. 2.5 ms, unambiguous.
  The physics curve is zeroed on t_onset. Latency is reported as the bracket
  between them, with its width, and never quietly folded into the curve.

Deliberately does NOT score a throw, compute a miss distance, or read a battery
CSV. That is the whole point of this phase: the sweep's a_eff is confounded by
per-throw aim (d_req varied 0.00-0.19 m) and by a counterfactual extrapolation
horizon that grows with detection range. Neither confound can reach this file.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys

# Grid the response curve is reported on, seconds after t_onset.
GRID = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50)
# TCA values the envelope is quoted at (the user-facing question).
TCA_GRID = (0.10, 0.20, 0.30, 0.40, 0.50)

PRE_LO, PRE_HI = 0.60, 0.15   # hover reference window, seconds before t_edge
ONSET_SIGMA = 6.0             # departures below this are hover noise
ONSET_HOLD_S = 0.02           # must stay out this long to count as onset
CMD_ON = 0.5                  # m/s: |desired velocity| above this = commanded
CMD_OFF = 0.1                 # m/s: and below this = quiet
MIN_EDGE_GAP_S = 3.0          # two edges closer than this are one event
MATCH_TOL_S = 1.5             # edge<->event pairing tolerance, after de-biasing
DIR_MIN_V = 0.25              # m/s: below this an angle is noise -- report none
DIR_MIN_D = 0.05              # m:   same floor, for the displacement angle


def load_bin(path):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    sim2, pscn, psce, guip, att = [], [], [], [], []
    want = ["SIM2", "PSCN", "PSCE", "GUIP", "ATT"]
    while True:
        msg = m.recv_match(type=want)
        if msg is None:
            break
        d = msg.to_dict()
        t = d["TimeUS"] * 1e-6
        k = msg.get_type()
        if k == "SIM2":
            sim2.append((t, d["PN"], d["PE"], d["PD"], d["VN"], d["VE"], d["VD"]))
        elif k == "PSCN":
            pscn.append((t, d["DVN"], d["TVN"], d["VN"], d["DAN"], d["TAN"], d["AN"]))
        elif k == "PSCE":
            psce.append((t, d["DVE"], d["TVE"], d["VE"], d["DAE"], d["TAE"], d["AE"]))
        elif k == "GUIP":
            guip.append((t, d.get("vX"), d.get("vY"), d.get("vZ")))
        elif k == "ATT":
            att.append((t, d["Yaw"], d["DesRoll"], d["Roll"], d["DesPitch"], d["Pitch"]))
    for name, arr in (("SIM2", sim2), ("PSCN", pscn), ("PSCE", psce)):
        if not arr:
            sys.exit(f"{path}: no {name} messages — wrong log, or logging is off")
    return sim2, pscn, psce, guip, att


def find_edges(pscn, psce):
    """Rising edges of commanded horizontal speed, from DVN/DVE.

    Paired by index: PSCN and PSCE are logged from the same 10 Hz slot, so
    sample i of each is the same controller update. Checked, not assumed.
    """
    n = min(len(pscn), len(psce))
    edges = []
    quiet = True
    for i in range(n):
        t = pscn[i][0]
        if abs(psce[i][0] - t) > 0.05:
            continue                      # slots drifted apart; skip this pair
        spd = math.hypot(pscn[i][1], psce[i][1])   # |desired velocity|, NE
        if quiet and spd > CMD_ON:
            if not edges or t - edges[-1][0] > MIN_EDGE_GAP_S:
                edges.append((t, pscn[i][1], psce[i][1], i))
            quiet = False
        elif not quiet and spd < CMD_OFF:
            quiet = True
    return edges


def match_edges_to_events(edges, events, tol_s=MATCH_TOL_S):
    """Pair dataflash edges with probe events by TIME, never by ordinal.

    The old code did `events[k]` for edge k. That is right only when the two
    lists correspond exactly, and they routinely do not: find_edges emits an
    extra edge when a hold dips below CMD_OFF and re-rises, and drops one when
    two commands land inside MIN_EDGE_GAP_S. A single extra edge early shifts
    every later pairing by one, so each trial is then labelled and
    direction-checked against a DIFFERENT command -- silently, because the
    numbers still look perfectly plausible. Observed in 00000015.BIN: edges
    3.4 s apart against a probe cadence of 8.35 s.

    The probe clock and the dataflash clock may sit at a constant offset, so
    the offset is measured rather than assumed: every (edge, event) pair is
    tried as an anchor and the offset pairing the most edges wins, ties broken
    by total residual. A 1-D RANSAC; at tens of trials the cost is trivial.

    Returns (pairs, delta, unmatched_event_indices) where pairs[i] is the
    event matched to edges[i], or None.
    """
    ev_t = []
    for ev in events:
        try:
            ev_t.append(float(ev["t_commit_s"]))
        except (KeyError, TypeError, ValueError):
            ev_t.append(None)
    usable = [(j, t) for j, t in enumerate(ev_t) if t is not None]
    if not edges or not usable:
        return [None] * len(edges), None, [j for j, _ in usable]

    def greedy(delta):
        """Nearest-neighbour, one-to-one, within tolerance."""
        used, chosen, resid = set(), [], 0.0
        for e in edges:
            bd, bj = None, None
            for j, t in usable:
                if j in used:
                    continue
                d = abs(e[0] - (t + delta))
                if d <= tol_s and (bd is None or d < bd):
                    bd, bj = d, j
            if bj is None:
                chosen.append(None)
            else:
                used.add(bj)
                chosen.append(bj)
                resid += bd
        return chosen, sum(1 for c in chosen if c is not None), resid

    best = None
    for e in edges:
        for _, t in usable:
            delta = e[0] - t
            _, n, resid = greedy(delta)
            if best is None or (n, -resid) > (best[0], -best[1]):
                best = (n, resid, delta)
    delta = best[2]

    chosen, _, _ = greedy(delta)
    pairs = [events[j] if j is not None else None for j in chosen]
    matched = {j for j in chosen if j is not None}
    unmatched = [j for j, _ in usable if j not in matched]
    return pairs, delta, unmatched


def slice_between(arr, t0, t1):
    return [r for r in arr if t0 <= r[0] <= t1]


def hover_reference(sim2, t_edge):
    """Constant-velocity fit over the pre-command window.

    From a settled hover this is very nearly (position, 0), but it is fitted
    rather than assumed so any residual drift is subtracted rather than
    counted as escape -- the same discipline the sweep's counterfactual used.
    """
    pre = slice_between(sim2, t_edge - PRE_LO, t_edge - PRE_HI)
    if len(pre) < 20:
        return None
    tm = statistics.fmean(r[0] for r in pre)
    out = {}
    for idx, key in ((1, "n"), (2, "e"), (3, "d")):
        ts = [r[0] - tm for r in pre]
        vs = [r[idx] for r in pre]
        sxx = sum(t * t for t in ts)
        sxy = sum(t * v for t, v in zip(ts, vs))
        slope = sxy / sxx if sxx > 0 else 0.0
        out[key] = (statistics.fmean(vs), slope, tm)
    vh = [math.hypot(r[4], r[5]) for r in pre]
    out["sigma_vh"] = statistics.pstdev(vh) if len(vh) > 1 else 0.0
    out["mean_vh"] = statistics.fmean(vh)
    out["n_pre"] = len(pre)
    return out


def ref_pos(ref, t):
    """Where the hover fit says the vehicle would have been at time t."""
    return tuple(ref[k][0] + ref[k][1] * (t - ref[k][2]) for k in ("n", "e", "d"))


def find_onset(sim2, ref, t_edge):
    """First sustained departure of horizontal speed from the hover floor."""
    thresh = ref["mean_vh"] + ONSET_SIGMA * max(ref["sigma_vh"], 1e-4)
    win = slice_between(sim2, t_edge - 0.15, t_edge + 1.0)
    for i, r in enumerate(win):
        if math.hypot(r[4], r[5]) <= thresh:
            continue
        held = [q for q in win[i:] if q[0] <= r[0] + ONSET_HOLD_S]
        if held and all(math.hypot(q[4], q[5]) > thresh for q in held):
            return r[0], thresh
    return None, thresh


def sample_at(sim2, t):
    """Linear interpolation onto an exact time (SIM2 is 2.5 ms, so this is tiny)."""
    lo, hi = 0, len(sim2) - 1
    if t <= sim2[0][0] or t >= sim2[-1][0]:
        return None
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if sim2[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = sim2[lo], sim2[hi]
    span = b[0] - a[0]
    f = 0.0 if span <= 0 else (t - a[0]) / span
    return tuple(a[j] + f * (b[j] - a[j]) for j in range(7))


def yaw_at(att, t):
    if not att:
        return None
    best = min(att, key=lambda r: abs(r[0] - t))
    return math.radians(best[1])


def analyse_trial(sim2, pscn, psce, att, edge, cmd_flu):
    t_edge = edge[0]
    ref = hover_reference(sim2, t_edge)
    if ref is None:
        return None
    t_onset, thresh = find_onset(sim2, ref, t_edge)
    if t_onset is None:
        return None

    out = {"t_edge": t_edge, "t_onset": t_onset,
           "lat_lo": max(t_onset - t_edge, 0.0),
           "lat_hi": t_onset - t_edge + 0.1,
           "sigma_vh": ref["sigma_vh"], "n_pre": ref["n_pre"],
           "dv_n": edge[1], "dv_e": edge[2]}

    # Commanded direction in earth NED: body FLU -> body NED -> earth NED.
    # (the FLU->NED flip is mav_bridge._flu_to_ned; the yaw rotation is what
    # MAV_FRAME_BODY_OFFSET_NED means)
    psi = yaw_at(att, t_edge)
    if psi is not None and cmd_flu is not None:
        bx, by = cmd_flu[0], -cmd_flu[1]
        out["cmd_n"] = bx * math.cos(psi) - by * math.sin(psi)
        out["cmd_e"] = bx * math.sin(psi) + by * math.cos(psi)

    # Response curve, zeroed on the measured onset.
    for dt in GRID:
        s = sample_at(sim2, t_onset + dt)
        if s is None:
            out[f"d{dt}"] = None
            out[f"v{dt}"] = None
            continue
        rn, re, _ = ref_pos(ref, s[0])
        out[f"d{dt}"] = math.hypot(s[1] - rn, s[2] - re)
        out[f"v{dt}"] = math.hypot(s[4], s[5])
        if dt == GRID[-1]:
            out["dn"], out["de"] = s[1] - rn, s[2] - re
            # Velocity with the fitted hover drift removed, so the direction
            # check subtracts exactly the reference the displacement does.
            out["vn"] = s[4] - ref["n"][1]
            out["ve"] = s[5] - ref["e"][1]

    # Peaks over the commanded hold.
    win = slice_between(sim2, t_onset, t_onset + 1.2)
    if win:
        out["v_peak"] = max(math.hypot(r[4], r[5]) for r in win)
        out["t_vpeak"] = min((r for r in win),
                             key=lambda r: -math.hypot(r[4], r[5]))[0] - t_onset
        # Acceleration by centred difference over 25 ms of 400 Hz ground truth.
        acc, k = [], 5
        for i in range(k, len(win) - k):
            dt2 = win[i + k][0] - win[i - k][0]
            if dt2 <= 0:
                continue
            an = (win[i + k][4] - win[i - k][4]) / dt2
            ae = (win[i + k][5] - win[i - k][5]) / dt2
            acc.append((win[i][0] - t_onset, math.hypot(an, ae)))
        if acc:
            out["a_peak"] = max(a for _, a in acc)
            out["t_apeak"] = max(acc, key=lambda p: p[1])[0]

    # Direction fidelity at GRID[-1] s after onset.
    #
    # The reference is the DATAFLASH desired velocity (DVN/DVE) -- what the
    # controller was actually asked for. The probe's cmd_flu is deliberately
    # NOT the reference: using it requires both a correct edge<->event match
    # and a correct FLU->NED reconstruction, so an error in either arrives
    # disguised as an airframe direction error. It is cross-checked separately
    # below instead, which is the honest way to spend it.
    #
    # Compared against achieved VELOCITY, not displacement: a velocity
    # setpoint commands velocity, and displacement at +0.5 s of a jerk-limited
    # ramp is small enough for residual hover drift to dominate the angle. The
    # old code compared against displacement and gated at 1e-6 m, so a
    # millimetre of drift yielded a confident, meaningless number -- which is
    # why dir_err_deg never agreed with anything.
    def _angle(an, ae, bn, be, floor_b):
        na, nb = math.hypot(an, ae), math.hypot(bn, be)
        if na <= 1e-6 or nb < floor_b:
            return None          # too small to have a direction; say so
        cosang = max(-1.0, min(1.0, (an * bn + ae * be) / (na * nb)))
        return math.degrees(math.acos(cosang))

    dvn, dve = edge[1], edge[2]
    if out.get("vn") is not None:
        out["dir_err_deg"] = _angle(dvn, dve, out["vn"], out["ve"], DIR_MIN_V)
    if out.get("dn") is not None:
        out["dir_err_pos_deg"] = _angle(dvn, dve, out["dn"], out["de"],
                                        DIR_MIN_D)
    # Cross-check: did the probe's own command reach the controller as sent?
    # A large value means the pairing or the frame convention is wrong -- NOT
    # that the airframe flew badly. Keep the two readings separate.
    if "cmd_n" in out:
        out["cmd_vs_dv_deg"] = _angle(dvn, dve, out["cmd_n"], out["cmd_e"],
                                      1e-6)

    # Shaping: desired vs shaped-target vs actual, through the hold.
    prof = []
    for i in range(len(pscn)):
        t = pscn[i][0]
        if not (t_edge - 0.15 <= t <= t_edge + 1.05) or i >= len(psce):
            continue
        prof.append((round(t - t_edge, 2),
                     math.hypot(pscn[i][1], psce[i][1]),   # DV desired
                     math.hypot(pscn[i][2], psce[i][2]),   # TV shaped target
                     math.hypot(pscn[i][3], psce[i][3]),   # V  actual
                     math.hypot(pscn[i][5], psce[i][5]),   # TA target accel
                     math.hypot(pscn[i][6], psce[i][6])))  # A  actual accel
    out["profile"] = prof
    return out


def agg(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    if not vals:
        return None
    vals.sort()
    return {"n": len(vals), "min": vals[0], "med": statistics.median(vals),
            "max": vals[-1], "mean": statistics.fmean(vals),
            "p25": vals[max(0, int(0.25 * (len(vals) - 1)))]}


def fmt(v, p=3, w=8):
    return " " * (w - 1) + "-" if v is None else f"{v:{w}.{p}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--events", required=True)
    ap.add_argument("--out", default=None, help="per-trial CSV")
    a = ap.parse_args()

    sim2, pscn, psce, guip, att = load_bin(a.bin)
    print(f"loaded {os.path.basename(a.bin)}: SIM2 {len(sim2)} @"
          f"{len(sim2)/max(sim2[-1][0]-sim2[0][0],1e-9):.0f} Hz, "
          f"PSCN {len(pscn)}, PSCE {len(psce)}, GUIP {len(guip)}, ATT {len(att)}")

    events = []
    with open(a.events) as f:
        for r in csv.DictReader(f):
            events.append(r)
    edges = find_edges(pscn, psce)
    print(f"probe logged {len(events)} commits; dataflash shows {len(edges)} "
          f"desired-velocity rising edges")
    if not edges:
        sys.exit("no commanded edges in the log — did the probe reach the FC?")
    pairs, delta, unmatched = match_edges_to_events(edges, events)
    n_matched = sum(1 for p in pairs if p is not None)
    if delta is None:
        print("  !! no usable t_commit_s in the events CSV — trials stay "
              "unlabelled and the probe cross-check is skipped")
    else:
        print(f"  matched {n_matched}/{len(edges)} edges to events by time "
              f"(clock offset {delta:+.2f} s); "
              f"{len(edges) - n_matched} edges and {len(unmatched)} events "
              f"unmatched")
        if n_matched < len(edges) or unmatched:
            print("     unmatched entries are REPORTED, not shifted onto their "
                  "neighbours: an unmatched edge keeps label '?' and is left "
                  "out of the probe cross-check. Displacement and direction "
                  "still stand — they come from the dataflash alone.")
        # A WRONG offset still pairs edges by luck: with events every T
        # seconds and a +/-tol window, a random edge lands inside one about
        # 2*tol/T of the time. Quote that, so a chance fit is never read as a
        # successful alignment just because the count looks respectable.
        ev_ts = sorted(float(e["t_commit_s"]) for e in events
                       if e.get("t_commit_s") not in (None, ""))
        if len(ev_ts) > 1:
            gaps = [b - a for a, b in zip(ev_ts, ev_ts[1:]) if b > a]
            spacing = statistics.median(gaps) if gaps else 0.0
            p = min(1.0, 2 * MATCH_TOL_S / spacing) if spacing > 0 else 1.0
            exp = p * len(edges)
            print(f"     events are ~{spacing:.2f} s apart, so ~{exp:.1f} of "
                  f"{len(edges)} edges would pair with a WRONG offset by "
                  f"chance alone")
            if n_matched <= 1.5 * exp:
                print("     !! MATCH COUNT IS AT OR NEAR CHANCE — treat every "
                      "label as unverified. dir_err_deg is unaffected (it is "
                      "dataflash-only), but cmd~dv is meaningless here, and a "
                      "large cmd~dv is itself evidence the pairing is wrong.")

    rows = []
    for k, edge in enumerate(edges):
        ev = pairs[k]
        cmd_flu = None
        label = "?"
        if ev:
            label = ev["dir_label"]
            cmd_flu = (float(ev["cmd_flu_x"]), float(ev["cmd_flu_y"]),
                       float(ev["cmd_flu_z"]))
        r = analyse_trial(sim2, pscn, psce, att, edge, cmd_flu)
        if r is None:
            print(f"  trial {k} ({label}): no usable hover reference or onset — dropped")
            continue
        r["trial"], r["label"] = k, label
        r["matched"] = 1 if ev else 0
        rows.append(r)

    if not rows:
        sys.exit("no analysable trials")

    print("\n" + "=" * 108)
    print("PER-TRIAL RESPONSE  (t = 0 is the measured onset; latency bracket is "
          "onset minus the 10 Hz command edge)")
    print("=" * 108)
    hdr = (f"{'trl':>3} {'dir':>4} {'lat_lo':>7} {'lat_hi':>7} {'sig_vh':>7} "
           + " ".join(f"{'d'+str(int(g*1000)):>7}" for g in GRID)
           + f" {'v_pk':>6} {'a_pk':>6} {'t_apk':>6} {'dir_err':>7}"
           + f" {'cmd~dv':>7}")
    print(hdr)
    for r in rows:
        print(f"{r['trial']:>3} {r['label']:>4} {fmt(r['lat_lo'],3,7)} "
              f"{fmt(r['lat_hi'],3,7)} {fmt(r['sigma_vh'],4,7)} "
              + " ".join(fmt(r.get(f'd{g}'), 4, 7) for g in GRID)
              + f" {fmt(r.get('v_peak'),2,6)} {fmt(r.get('a_peak'),2,6)} "
              f"{fmt(r.get('t_apeak'),2,6)} {fmt(r.get('dir_err_deg'),1,7)}"
              f" {fmt(r.get('cmd_vs_dv_deg'),1,7)}")

    for label in sorted({r["label"] for r in rows}) + ["ALL"]:
        sub = rows if label == "ALL" else [r for r in rows if r["label"] == label]
        if len(sub) < 2:
            continue
        print(f"\n  {label}  (n={len(sub)})   displacement from onset, metres")
        print(f"    {'dt_s':>6} {'n':>3} {'min':>8} {'p25':>8} {'median':>8} "
              f"{'max':>8}   {'a_eff_med':>9}")
        for g in GRID:
            s = agg(sub, f"d{g}")
            if not s:
                continue
            # a_eff from the curve itself: d = 0.5*a*t^2 with t measured, not
            # inferred from a counterfactual. Quoted per grid point because a
            # ramping response has no single acceleration.
            aeff = 2.0 * s["med"] / (g * g)
            print(f"    {g:>6.2f} {s['n']:>3} {s['min']:>8.4f} {s['p25']:>8.4f} "
                  f"{s['med']:>8.4f} {s['max']:>8.4f}   {aeff:>9.2f}")
        for key, lab in (("v_peak", "peak lateral velocity m/s"),
                         ("a_peak", "peak lateral accel m/s^2"),
                         ("t_apeak", "time to peak accel s"),
                         ("lat_lo", "onset - cmd edge, low s"),
                         ("dir_err_deg", "DV vs achieved velocity deg"),
                         ("dir_err_pos_deg", "DV vs displacement deg"),
                         ("cmd_vs_dv_deg", "probe cmd vs DV deg")):
            s = agg(sub, key)
            if s:
                print(f"    {lab:<30} min {s['min']:.3f}  med {s['med']:.3f}  "
                      f"max {s['max']:.3f}")

    print("\n" + "=" * 108)
    print("TCA -> MAXIMUM RELIABLE DISPLACEMENT  (this is the maneuver envelope)")
    print("  'reliable' = the WORST trial, not the best: an evasion that only "
          "works sometimes has not worked.")
    print("  hit_radius is 0.30 m, so a dead-centre throw needs d = 0.30.")
    print("=" * 108)
    print(f"  {'tca_s':>6} {'n':>3} {'worst':>8} {'p25':>8} {'median':>8} "
          f"{'best':>8}   {'clears 0.30?':>13}")
    for g in TCA_GRID:
        s = agg(rows, f"d{g}")
        if not s:
            continue
        verdict = ("ALL" if s["min"] >= 0.30 else
                   "some" if s["max"] >= 0.30 else "none")
        print(f"  {g:>6.2f} {s['n']:>3} {s['min']:>8.4f} {s['p25']:>8.4f} "
              f"{s['med']:>8.4f} {s['max']:>8.4f}   {verdict:>13}")

    print("\n" + "=" * 108)
    print("SHAPING — median across trials, relative to the command edge")
    print("  DV = desired (what GUIDED was asked for)   TV = target after the "
          "controller's shaping   V = actual")
    print("  DV stepping while TV RAMPS is command shaping, and the TV slope IS "
          "the acceleration limit.")
    print("=" * 108)
    print(f"  {'t_s':>6} {'DV':>7} {'TV':>7} {'V':>7} {'TA':>7} {'A':>7}  n")
    bins = {}
    for r in rows:
        for t, dv, tv, v, ta, aa in r.get("profile", []):
            bins.setdefault(round(t, 1), []).append((dv, tv, v, ta, aa))
    for t in sorted(bins):
        vals = bins[t]
        med = [statistics.median([v[i] for v in vals]) for i in range(5)]
        print(f"  {t:>6.1f} {med[0]:>7.3f} {med[1]:>7.3f} {med[2]:>7.3f} "
              f"{med[3]:>7.3f} {med[4]:>7.3f}  {len(vals)}")

    if a.out:
        keys = (["trial", "label", "t_edge", "t_onset", "lat_lo", "lat_hi",
                 "sigma_vh", "v_peak", "a_peak", "t_apeak", "dir_err_deg",
                 "dir_err_pos_deg", "cmd_vs_dv_deg", "matched"]
                + [f"d{g}" for g in GRID] + [f"v{g}" for g in GRID])
        with open(a.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print(f"\nper-trial CSV -> {a.out}")


if __name__ == "__main__":
    main()
