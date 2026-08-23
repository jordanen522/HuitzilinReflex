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
#   ./scripts/run_dodge_battery.sh week6depth # 20 m/s through the REAL detector
#   ./scripts/run_dodge_battery.sh week7      # 20 m/s through the RENDERED lane
#   DRONE_MODEL=iris_ar0234 ./scripts/run_dodge_battery.sh week7
#   EXTRA_ARGS="-p run_window_s:=6.0" ./scripts/run_dodge_battery.sh
#
# week6 mode requires week6_oracle.launch.py, NOT week4_evasion: its rows
# assume /threat/centroid comes from oracle_detector at a configured range.
# Run the fidelity gate first — see the header of
# config/week6_synthetic_battery.yaml.
#
# A FIDELITY GATE MUST PIN THE SENSOR, NOT ONLY ITS REACH. Quoting
# detection_range_m alone describes the wrong instrument: "the sensor" is reach
# AND sector AND rate, and a gate that matches one axis while leaving the other
# two at some other sensor's values does not reproduce the reference. Measured
# on the depth lane — at reach 3.4 m the shipped sector and rate gave
# D11 4/6 and D12 6/6, the shipped sector at 15 Hz gave 1/6 and 0/6, and only
# the fully matched sensor returned 2/6 and 6/6 against a Week 4 reference of
# 0/17 and 78/78. For week6depth the three axes are detection_range_m,
# sensor_params (params/synthetic_depth_oakd_gate.yaml) and sensor_rate_hz; the
# full command is in that file's header and in week6_synthetic_depth.launch.py.
# For week6 they are detection_range_m, oracle_rate_hz, and fov_half_angle_deg
# via oracle_params — but note the oracle models sector as a SINGLE CONE
# half-angle, so it cannot express the rendered camera's rectangular frustum at
# all; the depth lane's gate is the one that can.
#
# week6depth mode requires week6_synthetic_depth.launch.py, and NEITHER of the
# other two: its rows assume /threat/centroid is COMPUTED by the real,
# unmodified detector from a synthetic cloud on /oak/points. The two week6
# lanes are mutually exclusive — both ends reach /threat/centroid. It is a
# separate mode rather than an EXTRA_ARGS override of battery_config because an
# override would pass battery_config TWICE, so which config the run actually
# used would depend on argument-parsing order rather than being visible in this
# script. A named mode keeps it explicit, matching the week6 precedent above.
# Fidelity gate first, then hover:
#   EXTRA_ARGS="-p hover_mode:=true" ./scripts/run_dodge_battery.sh week6depth
# week7 mode requires week7_rendered.launch.py and NEITHER week6 lane: its rows
# assume /threat/centroid is computed by the real detector from a cloud the
# renderer actually produced. It is the only lane whose reach is an OUTPUT
# rather than an input, so there is no detection_range_m to pin here — the
# sensor is the WORLD FILE, and which world is running is not visible from this
# script. Quote it with every number. Fidelity gate (G01/G02, baseline world,
# noise off, patrol) first, then hover:
#   DRONE_MODEL=iris_ar0234 EXTRA_ARGS="-p hover_mode:=true" \
#       ./scripts/run_dodge_battery.sh week7
#
# DRONE_MODEL exists because huitzilin_runway_ar0234.sdf names its drone entity
# iris_ar0234 while dodge_battery defaults to iris_depth. It is a named variable
# rather than something to bury in EXTRA_ARGS so that the one axis which differs
# between the two week7 arms is visible in the command line. It defaults to
# iris_depth, which is correct for every other mode in this script and for
# week7's own fidelity-gate arm.
set -euo pipefail

MODE="${1:-battery}"
PKG_SHARE="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception"

ARGS=(-p use_sim_time:=true -p "drone_model:=${DRONE_MODEL:-iris_depth}")
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
elif [[ "${MODE}" == "week6depth" ]]; then
  # det_match_tol_m 1.5 for the same reason as week6 above, and it is not
  # optional here either: it is a DISTANCE tolerance standing in for a TIME
  # one, a 20 m/s ball moves 1.0 m per 50 ms of centroid-vs-pose skew, and the
  # shipped 0.75 m would silently leave first_det_range_m blank at exactly the
  # speed under study — which is the one column that proves the cell delivered
  # the reach it was launched with. It affects reporting only, never the dodge.
  ARGS+=(-p "battery_config:=${PKG_SHARE}/config/week6_depth_battery.yaml"
         -p det_match_tol_m:=1.5
         -p output_file:=/tmp/week6_depth_battery.txt
         -p csv_file:=/tmp/week6_depth_battery.csv)
elif [[ "${MODE}" == "week7" ]]; then
  # det_match_tol_m 1.5 for the same reason as the two week6 modes: it is a
  # DISTANCE tolerance standing in for a TIME one, a 20 m/s ball moves 1.0 m per
  # 50 ms of centroid-vs-pose skew, and the shipped 0.75 m would silently leave
  # first_det_range_m blank at exactly the speed under study. It matters more
  # here than anywhere else: in this lane first_det_range_m is not a check on a
  # launched reach, it IS the reach measurement, and a blank column would
  # discard the headline output. It affects reporting only, never the dodge.
  ARGS+=(-p "battery_config:=${PKG_SHARE}/config/week7_rendered_battery.yaml"
         -p det_match_tol_m:=1.5
         -p output_file:=/tmp/week7_rendered_battery.txt
         -p csv_file:=/tmp/week7_rendered_battery.csv)
fi
# shellcheck disable=SC2206
ARGS+=(${EXTRA_ARGS:-})

exec ros2 run huitzilin_perception dodge_battery --ros-args "${ARGS[@]}"
