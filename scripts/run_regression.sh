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

set -euo pipefail

source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../install/setup.bash" 2>/dev/null || \
  (echo "ERROR: workspace not built — run 'colcon build' first" && exit 1)

BAG_DIR="${1:-/data/huitzilin_bags}"
SPLIT="${2:-test}"
OUTPUT="/tmp/week3_regression_$(date +%Y%m%d_%H%M%S).txt"
MATRIX="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/config/scenario_matrix.yaml"

echo "=== HuitzilinReflex Week 3 Regression ==="
echo "    Bag dir : $BAG_DIR"
echo "    Split   : $SPLIT"
echo "    Output  : $OUTPUT"
echo ""

# Launch detector in background (needs use_sim_time so bags drive the clock)
ros2 run huitzilin_perception detector \
  --ros-args --params-file \
  "$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/params/detector.yaml" \
  -p use_sim_time:=true &
DETECTOR_PID=$!
trap "kill $DETECTOR_PID 2>/dev/null" EXIT

sleep 2  # give detector time to start

# Run scorer (replays bags, scores, exits with code)
ros2 run huitzilin_perception score_bags \
  --ros-args \
  -p bag_dir:="$BAG_DIR" \
  -p scenario_matrix:="$MATRIX" \
  -p split:="$SPLIT" \
  -p recall_floor:=0.95 \
  -p output_file:="$OUTPUT" \
  -p use_sim_time:=true

SCORE_EXIT=$?
echo ""
echo "Report saved to: $OUTPUT"
exit $SCORE_EXIT
