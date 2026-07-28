#!/usr/bin/env bash
# Week 4 dodge battery / sweep — Dell only (live Gazebo depth required).
#
# Prereqs (docs/dodge_battery_runbook.md): depth world + SITL up, drone flying
# patrol under `ros2 launch huitzilin_perception week4_evasion.launch.py
# with_patrol:=true`.
#
# Usage:
#   ./scripts/run_dodge_battery.sh            # full battery
#   ./scripts/run_dodge_battery.sh sweep      # parameter sweep
#   EXTRA_ARGS="-p run_window_s:=6.0" ./scripts/run_dodge_battery.sh
set -euo pipefail

MODE="${1:-battery}"
PKG_SHARE="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception"

ARGS=(-p use_sim_time:=true)
if [[ "${MODE}" == "sweep" ]]; then
  ARGS+=(-p "sweep_config:=${PKG_SHARE}/config/week4_sweep.yaml"
         -p output_file:=/tmp/week4_sweep.txt
         -p csv_file:=/tmp/week4_sweep.csv)
fi
# shellcheck disable=SC2206
ARGS+=(${EXTRA_ARGS:-})

exec ros2 run huitzilin_perception dodge_battery --ros-args "${ARGS[@]}"
