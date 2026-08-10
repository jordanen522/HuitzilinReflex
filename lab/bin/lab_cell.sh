#!/usr/bin/env bash
# lab_cell.sh — ONE sweep cell: bring up at a range, fly, score a battery.
#
# Split out of lab_sweep.sh so the fidelity gate is literally the same code
# path as a sweep cell. A gate that runs through a different script is not a
# gate on the sweep.
#
# Three instruments run alongside the battery:
#   * hz_counterfactual.py — was the miss the DODGE or the THROW? Without it
#     `success` cannot distinguish a save from a throw that was never on
#     target, and dodge_battery only checks aim error on runs that did NOT
#     dodge (dodge_battery.py:811).
#   * an altitude sampler — three runs of the last sweep died on
#     "spawn z=0.03 < 0.3", which means the AIRCRAFT WAS ON THE GROUND, not
#     that the scenario was bad. Spawn z is the drone's own altitude
#     (ballistics.py:156), so a descent silently converts the tail of every
#     battery into harness errors. Sampled so it is visible, not inferred.
#   * the oracle's announced range, re-read from its own startup log.
#
# detection_range_m is an INPUT. Every number out of here is conditional on a
# sensor that can see that far, and is never evidence that one exists.

source "$HOME/hz_tools/hz_env.sh"          # before `set -u` (CLAUDE.md)
set -u

LAB="$HOME/hz_lab"
BIN="$LAB/bin"
RESULTS="$LAB/results"
source "$BIN/hz_numeric.sh"   # first_numeric(): skips the DDS lost-message token
RANGE="${RANGE:-3.4}"
# Extra `-p` args for dodge_battery, word-split on purpose. Empty by
# default so every previously measured cell is byte-identical.
BATTERY_ARGS="${BATTERY_ARGS:-}"
TAG="${TAG:-cell}"
CONFIG="${CONFIG:-$LAB/config/sweep_speeds.yaml}"

mkdir -p "$RESULTS"
STEM="$RESULTS/${TAG}_r${RANGE}"

# pkill patterns are bracketed so they cannot match this script's own command
# line, or the ssh invocation that started it. Un-bracketed `pkill -f
# lab_sweep` killed the controlling session once already (exit 255).
stop_probes() {
  pkill -INT -f "hz_counterfactua[l]" 2>/dev/null
  pkill -INT -f "hz_dodge_respons[e]" 2>/dev/null
  sleep 2
  pkill -KILL -f "hz_counterfactua[l]" 2>/dev/null
  pkill -KILL -f "hz_dodge_respons[e]" 2>/dev/null
  [ -n "${ALT_PID:-}" ] && kill "$ALT_PID" 2>/dev/null
  return 0
}
trap stop_probes EXIT

echo "################ oracle detection_range_m = $RANGE ################"

STACK=oracle RANGE="$RANGE" bash "$BIN/lab_up.sh" 2>&1 | tail -6
# PIPESTATUS, not $?: a pipeline reports tail's status, so `if ! cmd | tail`
# is ALWAYS true and this guard never fired. A cell that never armed then ran
# a full battery and wrote a CSV indistinguishable from real data.
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
  echo "!! bring-up failed at range $RANGE"; exit 1
fi

# Trust what the oracle says it started with, never what was requested: it
# reads detection_range_m once in __init__ and installs no parameter callback.
ACTUAL="$(grep -aoE "range [0-9.]+ m \(an INPUT" /tmp/lab_stack.log | tail -1 \
          | grep -oE "[0-9.]+" | head -1)"
if [ -z "$ACTUAL" ]; then
  echo "!! oracle never announced a range at $RANGE — is it running?"
  grep -aiE "oracle|error|died|Exception" /tmp/lab_stack.log | tail -8
  exit 1
fi
if ! awk -v a="$ACTUAL" -v r="$RANGE" 'BEGIN{exit !(sqrt((a-r)^2) < 0.01)}'; then
  echo "!! oracle started at $ACTUAL m but $RANGE m requested — ABORT"; exit 1
fi
echo "[ok] oracle confirmed at $ACTUAL m"

# The prefix bug published zero centroids while logging nothing wrong. Confirm
# the oracle agrees with the battery about what a ball is called BEFORE
# spending 12 minutes finding out it does not.
#
# The same banner carries every OTHER launch-time input, and range is the only
# one checked above. noise_std_xyz_m, seed and the FOV half-angle arrive
# through `oracle_params:=`, which reaches lab_up.sh only by environment
# inheritance of EXTRA_LAUNCH — a cell that loses it silently re-flies the
# SHIPPED defaults at the requested range and looks entirely normal. Two cells
# were lost that way. Assert on what the node says it started with.
BANNER="$(grep -a "oracle_detector up" /tmp/lab_stack.log | tail -1)"
if [ -z "$BANNER" ]; then
  echo "!! oracle printed no startup banner — is it running?"; exit 1
fi
echo "[cfg] $BANNER"

# Exactly one of EACH stack singleton must be alive. A node that outlived its
# own `ros2 launch` survives lab_up.sh's teardown (which kills by process
# GROUP) and keeps running alongside the new one.
#
# This check used to cover the oracle only, and the gap cost a whole 6-cell
# queue. lab_queue.sh's kill list named `evasion_nod[e]` / `patrol_nod[e]`
# while the installed executables are `evasion` / `patrol`, so seven complete
# stacks accumulated. The oracle check passed every time — the oracle was one
# of the processes being killed correctly — while underneath it:
#   * `ros2 param set /evasion` and `ros2 param get /evasion` addressed
#     DIFFERENT nodes, so a dodge_speed override read back as the shipped 1.5;
#   * seven gz_pose_bridge instances republished the same dynamic-pose stream,
#     so hz_counterfactual scored NO_FIT and reported actual=54 m;
#   * ball speed derived from those poses read 8-32 m/s for one 20 m/s throw,
#     which the battery reported as "impulse wrench dropped".
# None of that raises an error. Match on INSTALLED PATH, which cannot drift
# from the entry-point name the way a guessed `_node` suffix did.
SINGLETONS='oracle_detecto[r]
huitzilin_perception/evasio[n]
huitzilin_perception/gz_pose_bridg[e]
huitzilin_sim/mav_bridg[e]
huitzilin_sim/patro[l]'
DIRTY=0
while IFS= read -r pat; do
  n="$(pgrep -fc "$pat" 2>/dev/null || true)"
  [ "${n:-0}" = "1" ] && continue
  echo "!! ${n:-0} processes match '$pat', expected exactly 1"
  pgrep -af "$pat"
  DIRTY=1
done <<< "$SINGLETONS"
if [ "$DIRTY" != "0" ]; then
  echo "!! stack is NOT clean — ABORT. Every number from a duplicated stack is void,"
  echo "   and it fails silently: see the comment above this check."
  exit 1
fi

# INERT when unset, so every cell measured before this hook existed ran an
# identical code path. Set it to an ERE matched against the banner line.
EXPECT_BANNER="${EXPECT_BANNER:-}"
if [ -n "$EXPECT_BANNER" ]; then
  if printf '%s' "$BANNER" | grep -qE "$EXPECT_BANNER"; then
    echo "[ok] banner matches EXPECT_BANNER='$EXPECT_BANNER'"
  else
    echo "!! banner does NOT match EXPECT_BANNER='$EXPECT_BANNER' — ABORT"
    echo "   EXTRA_LAUNCH was: '${EXTRA_LAUNCH:-<unset>}'"
    exit 1
  fi
fi

bash "$BIN/lab_fly.sh" 2>&1 | tail -4
if [ "${PIPESTATUS[0]}" -ne 0 ]; then
  echo "!! never got airborne at range $RANGE"; exit 1
fi

# Optional per-cell override of a RUNTIME-SWEEPABLE evasion parameter (the set
# evasion.yaml lists as such: dodge_speed_mps, threat_radius_m,
# trigger_horizon_s, dodge_duration_s, min_track_updates — the Kalman keys are
# fixed at node start and must go through evasion_params instead).
#
# INERT when the variable is unset, so every result measured before a hook
# existed ran an identical code path. Must come AFTER lab_up.sh: that restarts
# /evasion, which reloads evasion.yaml and resets these to shipped values.
#
# The readback is not a formality. A silently ignored set would fly a whole
# sweep at the shipped value, and when the stack was duplicated `set` and `get`
# reached DIFFERENT nodes — this check is what caught that.
set_evasion_param() {
  local key="$1" want="$2" rb
  [ -n "$want" ] || return 0
  ros2 param set /evasion "$key" "$want" 2>&1
  rb="$(ros2 param get /evasion "$key" 2>&1)"
  echo "[$key] requested $want | readback: $rb"
  case "$rb" in
    *"$want"*) return 0 ;;
    *) echo "!! $key did not read back as $want - ABORT"; return 1 ;;
  esac
}

DODGE_SPEED="${DODGE_SPEED:-}"
DODGE_DURATION="${DODGE_DURATION:-}"
# threat_radius_m is the gate that actually holds the dodge. min_track_updates
# is 3 and every cell commits at 4.6-17.3 updates, so confirmations are not
# what costs the time -- waiting for the predicted miss to fall inside this
# radius is, and track_age is 70-94% of the whole dead time. Unset by default,
# so a cell that does not name it flies the shipped 0.75.
THREAT_RADIUS="${THREAT_RADIUS:-}"
set_evasion_param dodge_speed_mps  "$DODGE_SPEED"    || exit 1
set_evasion_param dodge_duration_s "$DODGE_DURATION" || exit 1
set_evasion_param threat_radius_m  "$THREAT_RADIUS"  || exit 1

# Assert the evasion node actually received the measurement covariance this
# cell asked for. EXPECT_BANNER covers oracle_params ONLY; evasion_params
# travels the same fragile path (EXTRA_LAUNCH -> lab_up.sh -> launch file), and
# a cell that loses it silently flies the shipped isotropic belief — which is
# exactly the arm a covariance A/B is trying to distinguish itself from, so the
# failure reads as a clean null rather than as an error.
#
# This is the ONLY chance to check it: meas_std_xyz_m is in FIXED_AT_START, so
# the node read it once at construction and a later `ros2 param set` would be
# accepted and ignored. INERT when unset.
EXPECT_MEAS_STD="${EXPECT_MEAS_STD:-}"
if [ -n "$EXPECT_MEAS_STD" ]; then
  MRB="$(ros2 param get /evasion meas_std_xyz_m 2>&1)"
  echo "[meas_std_xyz_m] expect '$EXPECT_MEAS_STD' | readback: $MRB"
  case "$MRB" in
    *"$EXPECT_MEAS_STD"*) echo "[ok] measurement covariance confirmed" ;;
    *) echo "!! meas_std_xyz_m did not read back as '$EXPECT_MEAS_STD' — ABORT"
       echo "   EXTRA_LAUNCH was: '${EXTRA_LAUNCH:-<unset>}'"
       exit 1 ;;
  esac
fi

echo "== counterfactual probe -> ${STEM}_cf.csv"
nohup setsid python3 "$BIN/hz_counterfactual.py" --out "${STEM}_cf.csv" \
  > "${STEM}_cf.log" 2>&1 < /dev/null & disown
sleep 3
if ! pgrep -f "hz_counterfactua[l]" > /dev/null; then
  echo "!! counterfactual probe failed to start"; tail -20 "${STEM}_cf.log"; exit 1
fi

# Optional escape-response probe. INERT when RESP_OUT is unset. Passive
# subscriber only - it commands nothing, so it cannot perturb the battery.
# Its value is on a NULL: if saves fail to appear, this separates "the
# escape curve did not reproduce" from "the trigger never fired at the
# right tca".
RESP_BIN="${RESP_BIN:-$HOME/huitzilin_ws/scripts/hz_dodge_response.py}"
if [ "${RESP_OUT:-}" = "1" ]; then
  echo "== escape response probe -> ${STEM}_resp.csv"
  # --dodge-speed scales the probe's ideal_m reference column. Without it the
  # probe assumes the shipped 1.5, so every fraction-of-ideal figure from a
  # 3.0 m/s cell read 2x optimistic.
  nohup setsid python3 "$RESP_BIN" --out "${STEM}_resp.csv" \
    --dodge-speed "${DODGE_SPEED:-1.5}" \
    > "${STEM}_resp.log" 2>&1 < /dev/null & disown
  sleep 5
  if ! pgrep -f "hz_dodge_respons[e]" > /dev/null; then
    echo "!! response probe failed to start - CONTINUING (diagnostic only, not the measurement)"
    tail -5 "${STEM}_resp.log"
  fi
fi

echo "== altitude sampler -> ${STEM}_alt.csv"
: > "${STEM}_alt.csv"
(
  while true; do
    # `ros2 topic echo` prints "A message was lost!!!" on STDOUT, so the
    # 2>/dev/null below never suppressed it and head -1 used to take it.
    # first_numeric skips any number of diagnostic lines (hz_numeric.sh).
    z="$(timeout 8 ros2 topic echo /huitzilin/odom --once \
         --field pose.pose.position.z 2>/dev/null | first_numeric)"
    echo "$(date +%s),${z:-NA}" >> "${STEM}_alt.csv"
    sleep 10
  done
) & ALT_PID=$!

echo "== battery: $CONFIG"
ros2 run huitzilin_perception dodge_battery --ros-args \
  -p use_sim_time:=true \
  -p "battery_config:=$CONFIG" \
  -p det_match_tol_m:=1.5 \
  -p min_launch_speed_frac:=0.85 \
  -p "output_file:=${STEM}.txt" \
  -p "csv_file:=${STEM}.csv" $BATTERY_ARGS 2>&1 | tail -14
# min_launch_speed_frac 0.85, not the 0.25 default. The gate caught an S20 run
# that launched at 10.7 m/s against a 20.0 spec (53%) and was scored as a
# 20 m/s NO-FIRE. At 3.4 m that changed nothing -- every S20 row was NO-FIRE
# anyway -- but at a range where 20 m/s can actually fire, a half-speed ball
# scored as the target case is exactly the wrong error. Measured speeds sit
# within 1.5% of spec (7.92-8.00, 14.07-14.15, 19.68-20.11), so 0.85 rejects a
# dropped impulse with a wide margin and never a good throw.

stop_probes

echo ""
python3 "$BIN/summarize_battery.py" "${STEM}.csv" \
  --label "oracle detection_range_m = $RANGE (an INPUT)"

echo ""
echo "== altitude over the battery (spawn z IS drone z) =="
# NUM gates every arithmetic use of $2. Without it awk coerced the
# Amessagewaslost!!! token to 0 and reported a 0.00 m sample no aircraft
# flew. Rejects are COUNTED, never silently dropped: a sampler that starts
# failing must be visible in the cell output rather than invisible.
awk -F, -v NUM="^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$" \
    '$2=="NA"||$2==""{next}
     $2 !~ NUM {bad++; next}
     {n++; v=$2+0; s+=v; if(n==1||v<min)min=v; if(n==1||v>max)max=v}
     END{if(n)printf "   n=%d  min=%.2f  max=%.2f  mean=%.2f m\n",n,min,max,s/n;
         else print "   no altitude samples";
         if(bad)printf "   !! %d NON-NUMERIC sampler tokens rejected - instrumentation fault\n",bad}' \
    "${STEM}_alt.csv"
awk -F, -v NUM="^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$" \
    '$2!="NA" && $2!="" && $2 ~ NUM && $2+0 < 0.30 {n++} END{
  if(n) printf "   !! %d samples BELOW 0.30 m — spawn-z harness errors expected\n", n}' \
  "${STEM}_alt.csv"

echo ""
echo "== counterfactual verdicts =="
if [ -s "${STEM}_cf.csv" ]; then
  awk -F, 'NR>1{v[$9]++; if($3!="")sv[$3"/"$9]++}
           END{for(k in v) printf "   %-20s %d\n", k, v[k];
               print "   -- by scenario --";
               for(k in sv) printf "   %-24s %d\n", k, sv[k]}' "${STEM}_cf.csv"
else
  echo "   (empty — probe saw no episodes)"
fi

# ── contamination screen ─────────────────────────────────────────────────────
#
# A duplicated stack does not raise an error. It returns numbers that look
# entirely normal, and one six-cell queue was scored, reported and only then
# discovered to be void. These three checks are the cheapest reliable
# detectors, calibrated against those quarantined cells:
#
#   implied rate  = (track_updates-1)/track_age_s. Cannot exceed the rate the
#                   oracle was launched with. Void cells ran 1.4-2.9x over;
#                   every clean cell sits at or below 1.12x (Gazebo catch-up
#                   bursts explain the small overshoot). Flags above 1.20x.
#   n_post        = pose samples hz_counterfactual saw after the dodge. One
#                   60 Hz pose bridge gives 230-354. Void cells gave
#                   1179-6317 — one band per duplicated bridge.
#   resp dodges   = rows in the response probe. One /evasion node fires once
#                   per throw; void cells fitted ZERO.
#
# Advisory, not fatal: a cell that has already flown is worth keeping and
# judging. The abort for this lives in lab_up/lab_queue teardown and in the
# singleton assertion above, which run BEFORE the 25 minutes are spent.
echo ""
echo "== contamination screen =="
ORACLE_HZ="${ORACLE_RATE:-14.5}"
awk -F, -v launched="$ORACLE_HZ" '
  NR>1 && $31+0 > 1 && $32+0 > 0 {
    r = ($31 - 1) / $32; n++; s += r; if (n == 1 || r > mx) mx = r
  }
  END {
    if (!n) { print "   no track rows to screen"; exit }
    # 0.0 means "limiter disabled" -> Gazebos full 60 Hz pose grid.
    lim = (launched + 0 == 0.0) ? 60.0 : launched + 0
    printf "   implied detection rate: mean %.1f Hz, max %.1f Hz (launched %.1f)\n",
           s/n, mx, lim
    if (mx > lim * 1.20)
      printf "   !! max is %.2fx the launched rate — SUSPECT duplicate publishers\n",
             mx / lim
  }' "${STEM}.csv"

if [ -s "${STEM}_cf.csv" ]; then
  awk -F, 'NR>1 && $12+0 > 0 {n++; s+=$12; if(n==1||$12+0>mx)mx=$12+0}
           END{ if(!n){print "   no counterfactual rows to screen"; exit}
                printf "   n_post: mean %.0f, max %.0f (one 60 Hz bridge = 230-354)\n", s/n, mx;
                if (mx > 600) print "   !! n_post far above one bridge — SUSPECT duplicate pose bridges" }' \
    "${STEM}_cf.csv"
fi

# Two one-sided gaps in the gates upstream of here, both of which have already
# produced a wrong number that nothing complained about:
#
#   ball speed   min_launch_speed_frac rejects a throw that leaves too SLOW and
#                says nothing about one that leaves too FAST. A corrupted
#                derived speed reads high — one run reported 60.98 m/s against
#                a 20.0 spec and was scored. The spec speed is not in the CSV,
#                so compare each throw against the MEDIAN of its own scenario:
#                all reps of one id share a spec, and good throws sit within
#                1.5% of it. A 1.15x spread is far outside that and far inside
#                the 2-3x a corruption produces.
#                THIS FLAGS A READOUT, NOT NECESSARILY A BAD THROW. Before
#                discarding anything, check the row against the three
#                independent witnesses: tca_s and first_det_range_m in this
#                same CSV, and n_pre/n_post/fit_resid_m in the _cf.csv. The
#                60.98 m/s throw above had tca 0.75 s and first detection
#                22.71 m — both in family with its nine 20 m/s siblings — and
#                a cf fit residual of 0.0011 m, so the BALL was fine and only
#                the derived speed was garbage. That cell's 8/10 stands.
#   first det    offset_forward_m must keep the ball outside the sensor at
#                spawn, or the cell measures a shorter sensor wearing the
#                launched label. Every flown "12 m" cell once delivered first
#                detection at 9.9-11.3 m. The runbook says to verify this by
#                hand on every run; doing it by hand is how it was missed.
#                Applied retroactively this fires on exactly the pre-e1e77d3
#                12 m cells (2.11-3.62 m short) and on nothing clean.
#                EXPECT IT TO FIRE ON A DELIBERATE-MISS CELL. Off-axis throws
#                cross the range sphere outside the +-45 deg cone and are only
#                seen once they enter it, which is nearer: the false-dodge cell
#                reads 5.14 m short by geometry, not by fault.
awk -F, -v want="$RANGE" '
  NR>1 && $25+0 > 0 { sp[$2] = sp[$2] " " $25 }
  NR>1 && $30+0 > 0 { dn++; ds += $30; if (dn==1 || $30+0 < dmn) dmn = $30+0 }
  END {
    for (id in sp) {
      n = split(sp[id], v, " ")
      # median of this scenario, insertion sort (n is <= repeats, tiny)
      for (i=2; i<=n; i++) { x=v[i]+0; j=i-1; while (j>0 && v[j]+0 > x) { v[j+1]=v[j]; j-- } v[j+1]=x }
      med = (n % 2) ? v[(n+1)/2]+0 : (v[n/2] + v[n/2+1]) / 2.0
      mx = v[n]+0
      if (med > 0 && mx > med * 1.15)
        printf "   !! %s: ball speed max %.2f is %.2fx its own median %.2f — SUSPECT a corrupted derived speed\n",
               id, mx, mx/med, med
    }
    if (!dn) { print "   no first_det_range_m rows to screen"; exit }
    printf "   first detection: mean %.2f m, min %.2f m (launched %.1f)\n", ds/dn, dmn, want+0
    # 1.0 m, not the ~0.2 m the runbook asks for by hand. The gate can only
    # fire on a pose tick, and at 60 Hz / 20 m/s the ball covers 0.333 m
    # between ticks, so first detection legitimately lands one to two ticks
    # inside the range: the clean cells here read 0.11-0.59 m short. The
    # failure this catches is 2-3 m short and up (a "12 m" cell delivering
    # 9.9 m), so 1.0 m separates them with room on both sides.
    if (want+0 > 0 && dmn < want+0 - 1.0)
      printf "   !! first detection is %.2f m short of the launched range — the ball entered the gate from INSIDE it; this cell measured a shorter sensor\n",
             want+0 - dmn
  }' "${STEM}.csv"

if [ "${RESP_OUT:-}" = "1" ] && [ -s "${STEM}_resp.csv" ]; then
  NRESP="$(awk -F, 'NR>1 && $1!="" {seen[$1]=1} END{print length(seen)}' "${STEM}_resp.csv")"
  NFIRED="$(awk -F, 'NR>1 && tolower($5)=="true" {n++} END{print n+0}' "${STEM}.csv")"
  echo "   response probe fitted $NRESP dodges vs $NFIRED fired in the battery"
  [ "${NRESP:-0}" = "0" ] && [ "${NFIRED:-0}" != "0" ] && \
    echo "   !! probe fitted NOTHING while the battery fired — SUSPECT a second /evasion"
fi

echo "== cell $RANGE done"
