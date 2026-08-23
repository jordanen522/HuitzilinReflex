#!/usr/bin/env bash
# run_regression.sh — Week 3, W3-18
# One-command regression: replays the bag library through the detector and
# exits non-zero if recall drops below the 95% floor.
#
# USAGE:
#   ./scripts/run_regression.sh [bag_dir] [split]
#   ./scripts/run_regression.sh /data/huitzilin_bags test
#
# Requires: ROS 2 Jazzy sourced, huitzilin_perception built.

set -eo pipefail

# ROS/ament setup.bash reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so enable -u only AFTER sourcing them.
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../install/setup.bash" 2>/dev/null || {
  echo "ERROR: workspace not built — run 'colcon build' first"; exit 1; }
set -u

BAG_DIR="${1:-/data/huitzilin_bags}"
SPLIT="${2:-test}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="/tmp/week3_regression_${STAMP}.txt"
DETECTOR_LOG="/tmp/week3_regression_detector_${STAMP}.log"
WARMUP_LOG="/tmp/week3_regression_warmup_${STAMP}.log"
MATRIX="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/config/scenario_matrix.yaml"

# rcutils buffers its log stream when stderr is not a terminal, and the startup
# gate below reads the detector's log to decide whether the clock guard passed.
# Buffered, that verdict can sit unflushed for longer than the gate waits.
export RCUTILS_LOGGING_BUFFERED_STREAM=0

echo "=== HuitzilinReflex Week 3 Regression ==="
echo "    Bag dir  : $BAG_DIR"
echo "    Split    : $SPLIT"
echo "    Output   : $OUTPUT"
echo "    Detector : $DETECTOR_LOG"
echo ""

# Reap orphans from previous runs. 'kill $DETECTOR_PID' only kills the
# 'ros2 run' wrapper; its python child survives, keeps subscribing, and spams
# stale-code logs / competing /threat/centroid publishes into the next run.
pkill -f "huitzilin_perception/detector" 2>/dev/null || true
pkill -f "ros2 run huitzilin_perception detector" 2>/dev/null || true
pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true
pkill -f "bag play .*--topics /clock" 2>/dev/null || true

# Every background PID this script owns, declared before anything starts so the
# EXIT trap can be armed first. The warm-up player below runs BEFORE the
# detector, so a trap installed after the detector (as it was) would leak it on
# any failure in between.
DETECTOR_PID=""
WARMUP_PID=""
TF1_PID=""
TF2_PID=""

# Kill wrappers AND their orphan-prone python/TF children on exit.
cleanup() {
  # '|| true' per iteration: under 'set -e' an unset PID makes the AND-list
  # fail, the loop inherits that status, and cleanup would return before ever
  # reaching the pkill lines below — leaking the children it exists to reap.
  for pid in "$DETECTOR_PID" "$WARMUP_PID" "$TF1_PID" "$TF2_PID"; do
    [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
  done
  pkill -f "huitzilin_perception/detector" 2>/dev/null || true
  pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true
  pkill -f "bag play .*--topics /clock" 2>/dev/null || true
  return 0   # never let cleanup's last command decide the script's exit status
}
trap cleanup EXIT

# Static TF chain base_link -> camera_link -> camera_optical_frame.
# The bags do NOT record /tf_static, and this harness bypasses
# week3_perception.launch.py (which normally publishes these), so without them
# the detector's final transform-to-base_link fails on EVERY frame and no
# centroid is ever published -> 0% recall no matter what the thresholds are.
# Offsets must match the launch-file defaults (camera_link_x/y/z, optical rot).
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

# --- Clock warm-up -----------------------------------------------------------
# detector_node installs clock_guard, a ONE-SHOT startup check: it polls at 2 Hz
# and destroys its own timer on the first verdict that is not WAIT. Started with
# use_sim_time:=true it must see a /clock publisher AND a ROS time past zero
# within DEFAULT_GRACE_S (5 s), or it logs FATAL and takes the node down.
#
# The only /clock publisher in this harness is the `ros2 bag play --clock` that
# score_bags spawns per scenario — which does not exist until score_bags has
# started, read the scenario matrix and reached the first bag, comfortably past
# 5 s. So the detector died during startup and every bag afterwards replayed
# into a dead process: TP=0, recall 0.0%, from a detector that was never asked
# to detect anything. Broken since ae02696 (the commit that added the guard) and
# reproduced byte for byte at 9e9595c. That 0% is a DEAD NODE — it is never a
# detection regression, and never a reason to touch detector.yaml.
#
# The fix is to hand the guard a clock to resolve against before score_bags
# runs, rather than widening the grace window: that window guards every other
# node in the stack and should not be stretched to fit one harness.
#
# The warm-up player is inert by construction. --topics /clock replays ONLY the
# recorded clock track, so /oak/points and /huitzilin/odom stay silent and the
# detector's persistent background map cannot be seeded with frames from outside
# the scenario being scored — verified on the Dell: /clock advancing
# with one publisher, both sensor topics confirmed silent. --loop keeps time
# moving if startup drags. It is killed the moment the guard passes; the guard
# never runs again, and score_bags drives /clock from that point on.
WARMUP_BAG=""
for cand in "$BAG_DIR"/week3_*; do
  if [ -d "$cand" ]; then WARMUP_BAG="$cand"; break; fi
  case "$cand" in
    *.mcap) if [ -f "$cand" ]; then WARMUP_BAG="$cand"; break; fi ;;
  esac
done
if [ -z "$WARMUP_BAG" ]; then
  echo "ERROR: no week3_* bag in $BAG_DIR — nothing can publish /clock, and the"
  echo "       detector's clock guard would take the node down 5 s after start."
  exit 1
fi
echo "    Clock warm-up: $(basename "$WARMUP_BAG") (--topics /clock — time only)"
ros2 bag play "$WARMUP_BAG" --topics /clock --loop >"$WARMUP_LOG" 2>&1 &
WARMUP_PID=$!

# Gate on a /clock message ARRIVING, not on a publisher existing: "advertised
# but frozen at 0" is itself one of the guard's two FAIL verdicts, so a
# publisher-count check would happily wave through the exact state we are
# trying to avoid handing the detector.
printf "    Waiting for /clock to advance ..."
CLOCK_UP=0
for _ in $(seq 1 12); do
  if timeout 3 ros2 topic echo /clock --once >/dev/null 2>&1; then CLOCK_UP=1; break; fi
done
if [ "$CLOCK_UP" -ne 1 ]; then
  echo " FAILED"
  echo "ERROR: warm-up player produced no /clock message. See $WARMUP_LOG"
  exit 1
fi
echo " up"

# Launch detector in background (needs use_sim_time so bags drive the clock).
# DEBUG_FUNNEL=true ./scripts/run_regression.sh ... enables per-stage funnel
# logging (raw/range/angle/voxel/fg/cluster counts + early-return reason).
# Output is tee'd, not redirected: the startup gate below has to read the clock
# guard's verdict, and DEBUG_FUNNEL runs still need it on the terminal.
ros2 run huitzilin_perception detector \
  --ros-args --params-file \
  "$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/params/detector.yaml" \
  -p use_sim_time:=true \
  -p debug_funnel:="${DEBUG_FUNNEL:-false}" \
  > >(tee "$DETECTOR_LOG") 2>&1 &
DETECTOR_PID=$!

# Wait for the guard to return a verdict rather than sleeping a fixed interval.
# A dead detector must abort the run here, with the reason, instead of letting
# 15 bags replay into nothing and reporting the result as 0% recall.
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
  2) echo " FAILED"
     echo "ERROR: clock guard rejected the clock. See $DETECTOR_LOG"; exit 1 ;;
  3) echo " FAILED"
     echo "ERROR: detector exited during startup. See $DETECTOR_LOG"; exit 1 ;;
  *) echo " TIMEOUT"
     echo "ERROR: no clock-guard verdict in 30 s. See $DETECTOR_LOG"; exit 1 ;;
esac

# The guard has passed and destroyed its own timer, so it will not re-check.
# Drop the warm-up clock: from here score_bags' own `bag play --clock` must be
# the only time source, exactly as it is in a normal scoring run.
kill "$WARMUP_PID" 2>/dev/null || true
pkill -f "bag play .*--topics /clock" 2>/dev/null || true
WARMUP_PID=""

sleep 2  # let the detector's subscriptions match before the first bag replays

# Run scorer (replays bags, scores, exits with code).
# '|| SCORE_EXIT=$?' is required: under 'set -e' a failing score_bags ends the
# script here, so the assignment below it only ever ran on the pass path and
# SCORE_EXIT was always 0 — and the report path never printed for the run you
# actually need it for.
SCORE_EXIT=0
ros2 run huitzilin_perception score_bags \
  --ros-args \
  -p bag_dir:="$BAG_DIR" \
  -p scenario_matrix:="$MATRIX" \
  -p split:="$SPLIT" \
  -p recall_floor:=0.95 \
  -p output_file:="$OUTPUT" \
  -p use_sim_time:=true || SCORE_EXIT=$?
echo ""
echo "Report saved to: $OUTPUT"
exit $SCORE_EXIT
