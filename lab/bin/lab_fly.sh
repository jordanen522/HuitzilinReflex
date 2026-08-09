#!/usr/bin/env bash
# lab_fly.sh — arm, take off, and start patrol on the lab vehicle.
#
# Verifies altitude from /huitzilin/odom rather than sleeping a guessed
# interval: at RTF ~0.33 a wall-clock sleep is not a statement about the
# vehicle. Exits non-zero if the drone never reaches altitude, so a caller
# never runs a battery against a grounded aircraft.

# ROS is sourced BEFORE `set -u`: /opt/ros/jazzy/setup.bash reads
# AMENT_TRACE_SETUP_FILES unset and exits under -u (CLAUDE.md sharp edge).
source "$HOME/hz_tools/hz_env.sh"
set -u
source "$HOME/hz_lab/bin/hz_numeric.sh"   # first_numeric(): skips the DDS lost-message token

TARGET_ALT="${TARGET_ALT:-1.5}"
ALT_TIMEOUT="${ALT_TIMEOUT:-180}"   # wall seconds
WITH_PATROL="${WITH_PATROL:-1}"

alt_now() {
  timeout 12 ros2 topic echo /huitzilin/odom --once --field pose.pose.position.z 2>/dev/null \
    | first_numeric   # NOT head -1: ros2 topic echo prints the DDS
                     # "A message was lost!!!" diagnostic on STDOUT, and head
                     # took it, so a live aircraft read as altitude 0.
}

echo "== arm"
timeout 40 ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}' || exit 1
sleep 4

echo "== takeoff"
timeout 40 ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger || exit 1

echo "== waiting for altitude >= ${TARGET_ALT} m"
deadline=$(( $(date +%s) + ALT_TIMEOUT ))
reached=0
while [ "$(date +%s)" -lt "$deadline" ]; do
  a="$(alt_now)"
  if [ -n "$a" ]; then
    echo "   alt=$a"
    if awk -v a="$a" -v t="$TARGET_ALT" 'BEGIN{exit !(a+0 >= t+0)}'; then
      reached=1; break
    fi
  fi
  sleep 5
done

if [ "$reached" -ne 1 ]; then
  echo "!! never reached ${TARGET_ALT} m — not starting patrol" >&2
  exit 1
fi
echo "== airborne"

if [ "$WITH_PATROL" = "1" ]; then
  echo "== start patrol"
  timeout 40 ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}' || exit 1
  sleep 10
  echo "== patrol started"
fi
