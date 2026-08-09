#!/usr/bin/env bash
# lab_sweep.sh — success rate vs (oracle detection range) x (ball speed).
#
# The per-cell work lives in lab_cell.sh, so the fidelity gate and a sweep cell
# are the same code. A gate that runs through different code is not a gate.
#
# RANGES MUST BE WRITTEN AS FLOATS. `detection_range_m:=5` is inferred as
# INTEGER against the node's DOUBLE declaration, and oracle_detector dies at
# startup with InvalidParameterTypeException. That is now also defended in
# week6_oracle.launch.py (ParameterValue value_type=float), but the launch file
# is not the only way in, and a sweep that silently skips four of five cells is
# expensive: the whole of "3.4 5 7 9 12" ran exactly one cell.
#
# One full stack restart per range, NOT `ros2 param set`: oracle_detector reads
# detection_range_m once in __init__ and installs no parameter callback, so a
# live set would report success, change nothing, and silently produce a sweep
# in which every row was measured at the same range.
#
# detection_range_m is an INPUT. Every number out of here is conditional on a
# sensor that can see that far, and is never evidence that one exists.

source "$HOME/hz_tools/hz_env.sh"          # before `set -u` (CLAUDE.md)
set -u

LAB="$HOME/hz_lab"
RESULTS="$LAB/results"
BIN="$LAB/bin"
CONFIG="${CONFIG:-$LAB/config/sweep_speeds.yaml}"
RANGES="${RANGES:-3.4 5.0 7.0 9.0 12.0}"
TAG="${TAG:-sweep}"

mkdir -p "$RESULTS"
LOG="$RESULTS/${TAG}_log.txt"
: > "$LOG"

for R in $RANGES; do
  case "$R" in
    *.*) ;;
    *) echo "!! range '$R' is not a float — oracle_detector would die on an INTEGER parameter. Write it as ${R}.0" | tee -a "$LOG"; continue ;;
  esac

  echo "" | tee -a "$LOG"
  # A failed cell must not abort the sweep: the later ranges are independent
  # measurements and are still worth having.
  RANGE="$R" TAG="$TAG" CONFIG="$CONFIG" bash "$BIN/lab_cell.sh" 2>&1 | tee -a "$LOG" \
    || echo "!! cell $R failed — continuing" | tee -a "$LOG"
done

echo "" | tee -a "$LOG"
echo "== sweep complete" | tee -a "$LOG"
echo "   battery CSVs        $RESULTS/${TAG}_r*.csv" | tee -a "$LOG"
echo "   counterfactual CSVs $RESULTS/${TAG}_r*_cf.csv" | tee -a "$LOG"
echo "   altitude traces     $RESULTS/${TAG}_r*_alt.csv" | tee -a "$LOG"
