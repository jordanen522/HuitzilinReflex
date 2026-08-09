#!/usr/bin/env bash
# lab_valrange.sh - fly the escape-curve prediction. 14 m/s, dead-centre,
# dodge_speed_mps 3.0, oracle range as the lever that buys tca.
#
# THE PREDICTION. lab_dodgespeed.sh measured escape displacement vs time
# since trigger on PROBE throws; at dodge_speed_mps 3.0 it crosses the
# 0.30 m hit radius at t ~= 0.60 s. If that curve is real, a BATTERY cell
# whose measured "tca at dodge commit" exceeds ~0.60 s must convert
# on-course throws into misses, and a cell below it must not.
#
# WHY THIS RUN EXISTS. The previous offline model predicted 78% at 14 m/s
# with a 12 m sensor and flew 0/6. A second unflown model is not a
# deliverable. This is the falsification attempt, and a null here means
# the range table (9.4 / 14.1 / 18.8 m) does not get reported.
#
# READ tca, NOT RANGE. Range is only the lever; closing speed is estimated
# from an odom cruise that dodge_battery itself measures at 2.09-3.49 m/s.
# Score each cell on the tca it actually achieved.
#
# SCORE ON THE COUNTERFACTUAL. saved = counterfactual <= 0.30 m AND actual
# > 0.30 m. Never `dodged`, never `success`.
#
# One stack restart per range: oracle_detector reads detection_range_m once
# in __init__ and has no parameter callback, so a live set would silently
# produce a sweep in which every cell ran at the same range. That restart
# also reloads evasion.yaml, which is why DODGE_SPEED is re-applied per
# cell inside lab_cell.sh rather than once here.
#
# detection_range_m is an INPUT. Nothing here is evidence such a sensor
# exists.

source "$HOME/hz_tools/hz_env.sh"          # before `set -u` (CLAUDE.md)
set -u

LAB="$HOME/hz_lab"
BIN="$LAB/bin"
RESULTS="$LAB/results"
TAG="${TAG:-val14}"
CELLS="${CELLS:-10.0 14.0 18.0}"
DSPEED="${DSPEED:-3.0}"
BATTERY_ARGS="${BATTERY_ARGS:-}"
GUARD="$HOME/hz_baseline_guard/eeprom.bin.sha256"

mkdir -p "$RESULTS"
LOG="$RESULTS/${TAG}_log.txt"
: > "$LOG"

check_baseline() {
  if [ -f "$GUARD" ]; then
    if sha256sum -c "$GUARD" --status 2>/dev/null; then
      echo "[guard] baseline eeprom intact ($1)" | tee -a "$LOG"
    else
      echo "!! BASELINE EEPROM CHANGED ($1)" | tee -a "$LOG"; exit 1
    fi
  else
    # Fail CLOSED. A missing guard file means the eeprom check cannot run at
    # all, which is strictly worse than a failing check -- not a reason to
    # proceed. Every other script in bin/ already fails closed here.
    echo "!! GUARD FILE MISSING: $GUARD ($1) -- refusing to run" | tee -a "$LOG"
    exit 1
  fi
}

# This sweep sets no MAVLink parameter - dodge_speed_mps is a ROS param on
# /evasion. The guard runs anyway: it is cheap, and a drift detected here
# would mean something else in the stack is writing eeprom.bin.
check_baseline entry

restore() {
  echo "== restoring dodge_speed_mps=1.5" | tee -a "$LOG"
  ros2 param set /evasion dodge_speed_mps 1.5 2>&1 | tee -a "$LOG" || true
  check_baseline exit
}
trap restore EXIT

for R in $CELLS; do
  case "$R" in
    *.*) ;;
    *) echo "!! range $R is not a float - oracle_detector dies on an INTEGER parameter. Write it as ${R}.0" | tee -a "$LOG"; continue ;;
  esac
  CFG="$LAB/config/${TAG}_r${R}.yaml"
  if [ ! -f "$CFG" ]; then
    echo "!! missing config $CFG - skipping cell $R" | tee -a "$LOG"; continue
  fi
  echo "" | tee -a "$LOG"
  # A failed cell must not abort the sweep: the other ranges are
  # independent measurements and are still worth having.
  RANGE="$R" TAG="$TAG" CONFIG="$CFG" DODGE_SPEED="$DSPEED" RESP_OUT=1 BATTERY_ARGS="$BATTERY_ARGS" \
    bash "$BIN/lab_cell.sh" 2>&1 | tee -a "$LOG" \
    || echo "!! cell $R failed - continuing" | tee -a "$LOG"
done

echo "" | tee -a "$LOG"
echo "== validation sweep complete" | tee -a "$LOG"
echo "   battery CSVs        $RESULTS/${TAG}_r*.csv" | tee -a "$LOG"
echo "   counterfactual CSVs $RESULTS/${TAG}_r*_cf.csv" | tee -a "$LOG"
echo "   escape response     $RESULTS/${TAG}_r*_resp.csv" | tee -a "$LOG"
