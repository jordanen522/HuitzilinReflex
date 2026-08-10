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

# Optional per-cell dodge_speed_mps override. INERT when DODGE_SPEED is unset,
# so every result measured before this hook existed ran an identical code path.
# Must come AFTER lab_up.sh: that restarts /evasion, which reloads evasion.yaml
# and resets dodge_speed_mps to 1.5. Readback is checked because a silently
# ignored set would produce a whole sweep secretly flown at the shipped value.
DODGE_SPEED="${DODGE_SPEED:-}"
if [ -n "$DODGE_SPEED" ]; then
  ros2 param set /evasion dodge_speed_mps "$DODGE_SPEED" 2>&1
  RB="$(ros2 param get /evasion dodge_speed_mps 2>&1)"
  echo "[dodge_speed] requested $DODGE_SPEED | readback: $RB"
  case "$RB" in
    *"$DODGE_SPEED"*) ;;
    *) echo "!! dodge_speed_mps did not read back as $DODGE_SPEED - ABORT"; exit 1 ;;
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
  nohup setsid python3 "$RESP_BIN" --out "${STEM}_resp.csv" \
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
echo "== cell $RANGE done"
