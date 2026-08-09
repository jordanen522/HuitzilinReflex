#!/usr/bin/env bash
# lab_jerk.sh — A/B the dodge escape curve across PSC_JERK_NE values.
#
# THE HYPOTHESIS UNDER TEST. PSC_JERK_NE = 5 m/s^3 means the position
# controller needs 5.66/5 = 1.13 s to reach the acceleration a 30 deg tilt
# already permits -- four times the 0.18-0.29 s tca window. If that is the
# binding term, raising it should raise escape displacement at fixed t. If
# escape is unchanged across a 12x change in the limit, jerk is NOT the
# constraint and the hypothesis is dead.
#
# WHAT IS MEASURED: escape displacement vs time since the dodge command
# (hz_dodge_response.py), NOT dodge success. At 8 m/s success is saturated at
# 78/78 and could not show an improvement if one existed.
#
# ISOLATION: a PARAM_SET persists to eeprom.bin. This is safe ONLY because
# SITL here runs with CWD=~/hz_lab. The baseline sha256 is checked at entry
# and at exit, and PSC_JERK_NE is restored on ANY exit path via trap.

# ROS before `set -u` -- setup.bash reads unset vars (CLAUDE.md sharp edge).
source "$HOME/hz_tools/hz_env.sh"
source "$HOME/venv-ardupilot/bin/activate"
set -u

LAB="$HOME/hz_lab"
RESULTS="$LAB/results"
CONFIG="$LAB/config/jerk_probe.yaml"
PSET="python3 $LAB/bin/lab_param_set.py"
GUARD="$HOME/hz_baseline_guard/eeprom.bin.sha256"
JERKS="${JERKS:-5 20 60}"
BASELINE_JERK="${BASELINE_JERK:-5}"

mkdir -p "$RESULTS"

check_baseline() {
  if sha256sum -c "$GUARD" --status 2>/dev/null; then
    echo "[guard] baseline eeprom intact ($1)"
  else
    echo "!! BASELINE EEPROM CHANGED at: $1" >&2
    exit 1
  fi
}

restore() {
  echo "== restoring PSC_JERK_NE=$BASELINE_JERK"
  $PSET --set "PSC_JERK_NE=$BASELINE_JERK" || \
    echo "!! RESTORE FAILED — lab vehicle left at a non-baseline jerk" >&2
  check_baseline "exit"
}
trap restore EXIT

check_baseline "entry"

alt_now() {
  timeout 12 ros2 topic echo /huitzilin/odom --once --field pose.pose.position.z 2>/dev/null \
    | head -1 | tr -d ' \r'
}

for J in $JERKS; do
  echo ""
  echo "################ PSC_JERK_NE = $J ################"

  a="$(alt_now)"
  if [ -z "$a" ] || ! awk -v a="$a" 'BEGIN{exit !(a+0 >= 1.0)}'; then
    echo "!! drone is not airborne (alt=${a:-none}) — aborting before arm $J" >&2
    exit 1
  fi
  echo "[pre] altitude $a m"

  if ! $PSET --set "PSC_JERK_NE=$J"; then
    echo "!! could not set PSC_JERK_NE=$J (clamped or absent) — skipping arm" >&2
    continue
  fi
  $PSET --dump "$RESULTS/params_jerk_${J}.parm" >/dev/null

  RESP="$RESULTS/dodge_resp_jerk${J}.csv"
  rm -f "$RESP"
  nohup setsid python3 "$HOME/huitzilin_ws/scripts/hz_dodge_response.py" \
    --out "$RESP" > "/tmp/lab_resp_${J}.log" 2>&1 < /dev/null &
  RESP_PID=$!
  sleep 8
  echo "[probe] recording -> $RESP"

  ros2 run huitzilin_perception dodge_battery --ros-args \
    -p use_sim_time:=true \
    -p "battery_config:=$CONFIG" \
    -p "output_file:=$RESULTS/jerk_${J}_battery.txt" \
    -p "csv_file:=$RESULTS/jerk_${J}_battery.csv" \
    2>&1 | tail -25

  sleep 4
  kill -TERM -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null
  sleep 3
  kill -KILL -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null

  echo "[arm $J] dodges recorded: $(( $(wc -l < "$RESP" 2>/dev/null || echo 1) - 1 )) rows"
done

echo ""
echo "== all arms complete"
