#!/usr/bin/env bash
# run_tests.sh — the whole unit-test suite, both packages.
#
# USAGE:
#   ./scripts/run_tests.sh              # everything
#   ./scripts/run_tests.sh -k clock     # any pytest args are forwarded
#
# Requires a built workspace: several modules import rclpy, and the tests
# import huitzilin_sim / huitzilin_perception by package name.
#
# NOTE the source-then-set-u ordering below. ROS/ament setup.bash reference
# unbound variables (AMENT_TRACE_SETUP_FILES and friends), so a script that
# enables 'set -u' first exits at the source line with no useful message.
# run_regression.sh gets this right; preflight_hw.sh sidesteps it by not using
# -u at all.

set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash" 2>/dev/null || {
  echo "ERROR: workspace not built — run 'colcon build --symlink-install' first"
  exit 1; }
set -u

cd "$WS"
exec python3 -m pytest \
  src/huitzilin_sim/test \
  src/huitzilin_perception/test \
  -q "$@"
