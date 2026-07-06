#!/usr/bin/env bash
# week3_flyup.sh — arm, take off, and start patrol (Terminal 4 helper).
#
# Chains arm+takeoff so ArduCopter's DISARM_DELAY auto-disarm never fires
# (arming then sitting idle on the ground ~10 s disarms — you must take off
# immediately), polls odom until the climb completes, then starts the patrol
# loop. Run AFTER the perception stack (T3, week3_perception.launch.py) is up.
#
# Prereq: ROS 2 Jazzy + workspace overlay sourced in THIS shell.
#
# NOTE: takes off to the bridge.yaml default (2 m). N04 needs the drone flying
# >= ~10.5 m — set takeoff_alt_m: 12.0 in bridge.yaml before capturing N04, then
# restore 2 m for the rest.
set -euo pipefail

CLIMB_TARGET_M=1.8   # consider takeoff complete once odom z clears this
CLIMB_TIMEOUT_S=30

echo "[flyup] arming + taking off (chained to beat auto-disarm)..."
ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger

echo "[flyup] waiting for climb to >= ${CLIMB_TARGET_M} m (sim, via odom)..."
CLIMBED=0
for _ in $(seq 1 "$CLIMB_TIMEOUT_S"); do
  Z=$(timeout 3 ros2 topic echo --once --field pose.pose.position.z /huitzilin/odom 2>/dev/null | head -1 | tr -d '[:space:]' || true)
  if [ -n "$Z" ] && awk "BEGIN{exit !($Z >= $CLIMB_TARGET_M)}"; then
    echo "[flyup] reached z=${Z} m"; CLIMBED=1; break
  fi
  sleep 1
done

if [ "$CLIMBED" -ne 1 ]; then
  echo "[flyup] ERROR: never reached ${CLIMB_TARGET_M} m in ${CLIMB_TIMEOUT_S}s."
  echo "[flyup]        Check SITL/bridge; do NOT start patrol on the ground."
  exit 2
fi

echo "[flyup] starting patrol..."
ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}'
echo "[flyup] airborne + patrolling. Ready for: ./scripts/capture_scenario.sh <ID>"
