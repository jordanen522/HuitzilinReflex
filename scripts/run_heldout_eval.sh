#!/usr/bin/env bash
# run_heldout_eval.sh — the OFFICIAL held-out perception scoring pass.
#
# Supersedes the ad-hoc score_heldout_truth.sh that ran the 2026-08-22
# 04:03 PDT pass (2/18 recalled). That script launched ONE detector process
# for the whole 24-scenario split. params/rendered_detector.yaml (frozen,
# unchanged by this script) sets use_persistent_bg:true, bg_map_ttl_s:20.0 --
# by design, for the detector's own real operation. Run across a whole split
# in one process, that same design makes later scenarios inherit background
# state from earlier scenarios that share corridor geometry
# (offset_forward_m), which is a harness isolation bug: each scenario must be
# scored as an independent bag, and independence requires each replay to see
# the cold-start background the detector actually evaluates against in real
# operation.
#
# Diagnosed 2026-08-22 (background agent a63ca933ed20696a4): H12 scored
# matched=0 after 11 prior scenarios shared its process, matched=4 after 2,
# matched=5 (same near-perfect 0.02-0.08m errors as its other matches) run
# alone. H02 and H05 reproduced matched=0 even from a cold process --
# real misses, not contamination.
#
# THE FIX: one detector process PER SCENARIO. Same detector binary, same
# frozen rendered_detector.yaml, same bags -- restarted, not reconfigured.
#
# Usage:
#   ./scripts/run_heldout_eval.sh
#
# Requires: built workspace (source install/setup.bash), Dell box (needs the
# captured heldout_* bags under /data/huitzilin_bags), python3 with pyyaml
# (already a rosdep of huitzilin_perception).
set -o pipefail

source /opt/ros/jazzy/setup.bash
source ~/huitzilin_ws/install/setup.bash
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"

BAG_DIR="/data/huitzilin_bags"
SPLIT="heldout"
BAG_PREFIX="heldout_"
STAMP="$(date +%Y%m%d_%H%M%S)"
RESULTS_JSONL="/tmp/heldout_eval_${STAMP}.jsonl"
FINAL_OUTPUT="/tmp/heldout_eval_${STAMP}_final.txt"
MATRIX="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/config/scenario_matrix.yaml"
DETECTOR_PARAMS="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/params/rendered_detector.yaml"

export RCUTILS_LOGGING_BUFFERED_STREAM=0

echo "=== HuitzilinReflex Held-out Perception Scoring (per-scenario detector restart) ==="
echo "    Bag dir      : $BAG_DIR"
echo "    Split        : $SPLIT"
echo "    Detector     : $DETECTOR_PARAMS"
echo "    Results jsonl: $RESULTS_JSONL"
echo ""

pkill -f "huitzilin_perception/detector" 2>/dev/null || true
pkill -f "ros2 run huitzilin_perception detector" 2>/dev/null || true
pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true
pkill -f "bag play .*--topics /clock" 2>/dev/null || true
rm -f "$RESULTS_JSONL"

TF1_PID=""
TF2_PID=""
DETECTOR_PID=""
WARMUP_PID=""

cleanup() {
  for pid in "$DETECTOR_PID" "$WARMUP_PID" "$TF1_PID" "$TF2_PID"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done
  pkill -f "huitzilin_perception/detector" 2>/dev/null || true
  pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true
  pkill -f "bag play .*--topics /clock" 2>/dev/null || true
  return 0
}
trap cleanup EXIT

# TF is static and stateless -- no contamination risk -- so it stays up for
# the whole run rather than restarting per scenario.
ros2 run tf2_ros static_transform_publisher \
  --x 0.10 --y 0 --z 0.02 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id camera_link \
  --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
TF1_PID=$!
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll -1.5707963 --pitch 0 --yaw -1.5707963 \
  --frame-id camera_link --child-frame-id camera_optical_frame \
  --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
TF2_PID=$!

SCENARIO_IDS=$(python3 -c "
import yaml
with open('$MATRIX') as f:
    m = yaml.safe_load(f)
print(' '.join(m['split']['$SPLIT']))
")
echo "    Scenarios: $SCENARIO_IDS"
echo ""

for SID in $SCENARIO_IDS; do
  echo "--- $SID ---"

  WARMUP_BAG=""
  for cand in "$BAG_DIR"/${BAG_PREFIX}${SID}*; do
    if [ -e "$cand" ]; then WARMUP_BAG="$cand"; break; fi
  done
  if [ -z "$WARMUP_BAG" ]; then
    echo "ERROR: no ${BAG_PREFIX}${SID}* bag in $BAG_DIR"
    exit 1
  fi

  DETECTOR_LOG="/tmp/heldout_eval_${STAMP}_${SID}_detector.log"
  WARMUP_LOG="/tmp/heldout_eval_${STAMP}_${SID}_warmup.log"

  ros2 bag play "$WARMUP_BAG" --topics /clock --loop >"$WARMUP_LOG" 2>&1 &
  WARMUP_PID=$!

  printf "    Waiting for /clock ..."
  CLOCK_UP=0
  for _ in $(seq 1 12); do
    if timeout 3 ros2 topic echo /clock --once >/dev/null 2>&1; then CLOCK_UP=1; break; fi
  done
  if [ "$CLOCK_UP" -ne 1 ]; then
    echo " FAILED"
    echo "ERROR: warm-up player produced no /clock for $SID. See $WARMUP_LOG"
    exit 1
  fi
  echo " up"

  ros2 run huitzilin_perception detector \
    --ros-args --params-file "$DETECTOR_PARAMS" \
    -p use_sim_time:=true \
    -p debug_funnel:=false \
    > >(tee "$DETECTOR_LOG") 2>&1 &
  DETECTOR_PID=$!

  printf "    Waiting for detector clock guard ..."
  GUARD=0
  for _ in $(seq 1 60); do
    if grep -q "simulated time is publishing and advancing" "$DETECTOR_LOG" 2>/dev/null; then
      GUARD=1; break
    fi
    if grep -qE "frozen at 0|nothing publishes /clock" "$DETECTOR_LOG" 2>/dev/null; then
      GUARD=2; break
    fi
    if ! kill -0 "$DETECTOR_PID" 2>/dev/null; then
      GUARD=3; break
    fi
    sleep 0.5
  done
  case "$GUARD" in
    1) echo " passed" ;;
    2) echo " FAILED"; echo "ERROR: clock guard rejected for $SID. See $DETECTOR_LOG"; exit 1 ;;
    3) echo " FAILED"; echo "ERROR: detector exited during startup for $SID. See $DETECTOR_LOG"; exit 1 ;;
    *) echo " TIMEOUT"; echo "ERROR: no clock-guard verdict for $SID. See $DETECTOR_LOG"; exit 1 ;;
  esac

  kill "$WARMUP_PID" 2>/dev/null || true
  pkill -f "bag play .*--topics /clock" 2>/dev/null || true
  WARMUP_PID=""
  sleep 2

  SCORE_EXIT=0
  ros2 run huitzilin_perception truth_score_heldout \
    --ros-args \
    -p bag_dir:="$BAG_DIR" \
    -p scenario_matrix:="$MATRIX" \
    -p split:="$SPLIT" \
    -p bag_prefix:="$BAG_PREFIX" \
    -p drone_model:=iris_ar0234 \
    -p scenario_id:="$SID" \
    -p results_jsonl:="$RESULTS_JSONL" \
    -p output_file:="/tmp/heldout_eval_${STAMP}_${SID}_unused.txt" \
    -p use_sim_time:=true || SCORE_EXIT=$?
  if [ "$SCORE_EXIT" -ne 0 ]; then
    echo "ERROR: truth_score_heldout exited $SCORE_EXIT for $SID"
    exit 1
  fi

  # kill only sends the signal -- it does not block until the process is
  # actually gone. A python/rclpy node can take real wall-clock time to tear
  # down its DDS publishers, and the next scenario's warmup+detector start
  # immediately after this returns. If the old detector is still alive when
  # the next one starts, both briefly publish to the same reliable
  # /threat/centroid topic, and the new scorer's subscriber (queue depth 10)
  # can pick up a few of the dying process's late messages as if they were
  # the new scenario's own detections -- the exact cross-scenario
  # contamination this per-scenario-restart harness exists to prevent, just
  # at the restart boundary instead of within one long-lived process.
  # Escalate to SIGKILL and confirm death before moving on.
  kill "$DETECTOR_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$DETECTOR_PID" 2>/dev/null || break
    sleep 0.5
  done
  kill -9 "$DETECTOR_PID" 2>/dev/null || true
  pkill -9 -f "huitzilin_perception/detector" 2>/dev/null || true
  sleep 1
  DETECTOR_PID=""
  echo ""
done

kill "$TF1_PID" "$TF2_PID" 2>/dev/null || true
TF1_PID=""
TF2_PID=""

python3 "$HERE/finalize_heldout_report.py" \
  --results-jsonl "$RESULTS_JSONL" \
  --scenario-matrix "$MATRIX" \
  --split "$SPLIT" \
  --bag-dir "$BAG_DIR" \
  --min-matched 3 \
  --output-file "$FINAL_OUTPUT"
