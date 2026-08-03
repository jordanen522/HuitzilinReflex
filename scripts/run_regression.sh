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
OUTPUT="/tmp/week3_regression_$(date +%Y%m%d_%H%M%S).txt"
MATRIX="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/config/scenario_matrix.yaml"

echo "=== HuitzilinReflex Week 3 Regression ==="
echo "    Bag dir : $BAG_DIR"
echo "    Split   : $SPLIT"
echo "    Output  : $OUTPUT"
echo ""

# Reap orphans from previous runs. 'kill $DETECTOR_PID' only kills the
# 'ros2 run' wrapper; its python child survives, keeps subscribing, and spams
# stale-code logs / competing /threat/centroid publishes into the next run.
pkill -f "huitzilin_perception/detector" 2>/dev/null || true
pkill -f "ros2 run huitzilin_perception detector" 2>/dev/null || true
pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true

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

# Launch detector in background (needs use_sim_time so bags drive the clock).
# DEBUG_FUNNEL=true ./scripts/run_regression.sh ... enables per-stage funnel
# logging (raw/range/angle/voxel/fg/cluster counts + early-return reason).
ros2 run huitzilin_perception detector \
  --ros-args --params-file \
  "$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/params/detector.yaml" \
  -p use_sim_time:=true \
  -p debug_funnel:="${DEBUG_FUNNEL:-false}" &
DETECTOR_PID=$!
# Kill wrappers AND their orphan-prone python/TF children on exit.
cleanup() {
  kill "$DETECTOR_PID" "$TF1_PID" "$TF2_PID" 2>/dev/null || true
  pkill -f "huitzilin_perception/detector" 2>/dev/null || true
  pkill -f "static_transform_publisher.*child-frame-id camera" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2  # give detector + TF publishers time to start

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
