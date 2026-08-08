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
#   ./scripts/run_dodge_battery.sh week6      # 20 m/s synthetic-oracle battery
#   EXTRA_ARGS="-p run_window_s:=6.0" ./scripts/run_dodge_battery.sh
#
# week6 mode requires week6_oracle.launch.py, NOT week4_evasion: its rows
# assume /threat/centroid comes from oracle_detector at a configured range.
# Run the fidelity gate first (detection_range_m:=3.4) — see the header of
# config/week6_synthetic_battery.yaml.
set -euo pipefail

MODE="${1:-battery}"
PKG_SHARE="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception"

ARGS=(-p use_sim_time:=true)
if [[ "${MODE}" == "sweep" ]]; then
  ARGS+=(-p "sweep_config:=${PKG_SHARE}/config/week4_sweep.yaml"
         -p output_file:=/tmp/week4_sweep.txt
         -p csv_file:=/tmp/week4_sweep.csv)
elif [[ "${MODE}" == "week6" ]]; then
  # det_match_tol_m is raised from its 0.75 m default because it is a DISTANCE
  # tolerance standing in for a TIME one: a 20 m/s ball moves 1.0 m per 50 ms
  # of centroid-vs-pose skew, so the shipped value would silently reject every
  # true detection and leave first_det_range_m blank at exactly the speed being
  # studied. It affects reporting only, never the dodge.
  ARGS+=(-p "battery_config:=${PKG_SHARE}/config/week6_synthetic_battery.yaml"
         -p det_match_tol_m:=1.5
         -p output_file:=/tmp/week6_battery.txt
         -p csv_file:=/tmp/week6_battery.csv)
fi
# shellcheck disable=SC2206
ARGS+=(${EXTRA_ARGS:-})

exec ros2 run huitzilin_perception dodge_battery --ros-args "${ARGS[@]}"
