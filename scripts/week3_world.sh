#!/usr/bin/env bash
# week3_world.sh — bring up the Week 3 perception world in Gazebo (Terminal 1).
#
# Sets GZ_SIM_RESOURCE_PATH to the *installed* huitzilin_perception models/worlds
# (the single fact that most often breaks a fresh launch — without it Gazebo
# cannot find iris_depth or huitzilin_runway.sdf), then launches the world
# headless (-s) at real-time (-r).
#
# Prereq: ROS 2 Jazzy + workspace overlay sourced in THIS shell:
#   source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
#
# Verified: with SITL up, this world flies (Frame QUAD/X, EKF+GPS,
# no "No JSON sensor message") and /oak/points streams 15 Hz sim, metronome-stable.
set -euo pipefail

PKG_SHARE="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception"
export GZ_SIM_RESOURCE_PATH="$PKG_SHARE/models:$PKG_SHARE/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "[world] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "[world] launching $PKG_SHARE/worlds/huitzilin_runway.sdf ..."
exec gz sim -s -r "$PKG_SHARE/worlds/huitzilin_runway.sdf"
