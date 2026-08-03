#!/usr/bin/env bash
# HuitzilinReflex — Week 2 Day 1 preflight checklist
# Run this before any Week 2 work to confirm the Week 1 baseline is still green.
set -e

echo "=== HuitzilinReflex Week 2 Preflight ==="
echo ""

echo "[1/4] ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash
# NOT 'ros2 --version' — that is not a flag in Jazzy, it exits non-zero, and
# under 'set -e' it killed this script at step 1 so checks 2-4 never ran.
echo "ROS_DISTRO=$(printenv ROS_DISTRO)"
if [ "$(printenv ROS_DISTRO)" != "jazzy" ]; then
    echo "WARNING: expected jazzy"
fi
echo ""

echo "[2/4] Gazebo Harmonic..."
gz sim --version 2>&1 | head -2
echo "GZ_VERSION=${GZ_VERSION:-NOT SET (export GZ_VERSION=harmonic)}"
echo ""

echo "[3/4] ardupilot_gz workspace..."
if [ -f "$HOME/ardu_ws/install/setup.bash" ]; then
    source "$HOME/ardu_ws/install/setup.bash"
    ros2 pkg list | grep ardupilot || echo "WARNING: ardupilot packages not found"
else
    echo "WARNING: ~/ardu_ws/install/setup.bash not found"
fi
echo ""

echo "[4/4] pymavlink heartbeat check..."
echo "      (run this while SITL is active in another terminal)"
echo ""
echo "      python3 -c \\"
echo "        \"from pymavlink import mavutil; \\"
echo "          m=mavutil.mavlink_connection('udp:127.0.0.1:14550'); \\"
echo "          m.wait_heartbeat(); \\"
echo "          print('HEARTBEAT sys', m.target_system, 'comp', m.target_component)\""
echo ""

echo "=== Preflight checks 1-3 done. Run check 4 manually with SITL running. ==="
echo ""
echo "To start the sim baseline (Week 1 DoD):"
echo "  ros2 launch ardupilot_gz_bringup iris_runway.launch.py"
echo ""
echo "To build and run our Week 2 stack:"
echo "  cd ~/huitzilin_ws && colcon build --symlink-install && source install/setup.bash"
echo "  ros2 launch huitzilin_sim week2_sitl.launch.py"
