#!/usr/bin/env bash
# lab_dodgespeed.sh — A/B the dodge escape across dodge_speed_mps.
#
# WHY THIS RE-OPENS A DOCUMENTED NULL. CLAUDE.md lists "dodge_speed_mps
# 1.5->4.0" under "Measured nulls — do not re-run". It is re-run here for a
# specific, stated reason, not a hunch:
#
#   1. The null was scored on escape DISPLACEMENT. The WP_ACC sweep
#      (2026-08-08) flew the identical 2.5 configuration twice and got 0.0458
#      vs 0.0723 m at t=0.30 -- session drift LARGER than the effect that
#      metric was asked to detect. A null from an instrument that coarse is
#      not a refutation.
#   2. Dataflash now says the command is the binding term. Achieved velocity
#      change EXCEEDS the shaped desired change in every arm (0.93 vs 0.81,
#      1.26 vs 1.17, 1.47 vs 0.97 m/s), so the vehicle over-delivers on what
#      it is asked for. It is only ever asked for ~1 m/s.
#   3. dodge_velocity_command() returns v_drone + dodge_speed_mps * d_hat,
#      capping only the CRUISE term. The commanded velocity step therefore
#      scales 1:1 with dodge_speed_mps by construction. If escape does not
#      scale, something between the node and the controller is eating it and
#      this measurement names which.
#
# THE INSTRUMENT IS DIFFERENT THIS TIME. Escape displacement is still
# recorded, but the primary read is the dataflash velocity step
# (PSCN/PSCE DVN/DVE vs VN/VE) via wpacc_veltrack.py, which has no session
# drift because it compares command against achievement inside one dodge.
#
# FALSIFICATION. desired_dv must scale with dodge_speed_mps. If commanding
# 4.5 m/s still produces a ~1 m/s velocity step, the escape is being eaten
# downstream of the node -- look at the dodge_floor_m re-spend and the
# MASK/frame handling in mav_bridge, not at any ArduPilot parameter.
#
# ARMS stay UNDER dodge_max_speed_mps (6.0). At escape >= the cap,
# dodge_velocity_command takes its esc_norm >= cap branch and drops the cruise
# term entirely, which is a different regime and would confound the sweep.
# 4.5 escape against a ~3.5 m/s cruise asks for 5.7 m/s, still inside 6.0.
#
# The 1.5 control is flown FIRST and LAST so session drift is measured.
#
# ISOLATION: this sets a ROS parameter on /evasion, not a MAVLink parameter,
# so eeprom.bin is never touched. The baseline guard is still checked at entry
# and exit as hygiene, and dodge_speed_mps is restored on ANY exit via trap.

# ROS before `set -u` -- setup.bash reads unset vars (CLAUDE.md sharp edge).
source "$HOME/hz_tools/hz_env.sh"
source "$HOME/venv-ardupilot/bin/activate"
set -u

LAB="$HOME/hz_lab"
RESULTS="$LAB/results"
CONFIG="$LAB/config/wpacc_probe.yaml"
PSET="python3 $LAB/bin/lab_param_set.py"
GUARD="$HOME/hz_baseline_guard/eeprom.bin.sha256"
ARMS="${ARMS:-1.5 3.0 4.5 1.5}"
BASELINE_DSPEED="${BASELINE_DSPEED:-1.5}"

mkdir -p "$RESULTS"

check_baseline() {
  if sha256sum -c "$GUARD" --status 2>/dev/null; then
    echo "[guard] baseline eeprom intact ($1)"
  else
    echo "!! BASELINE EEPROM CHANGED at: $1" >&2
    exit 1
  fi
}

set_dspeed() {
  ros2 param set /evasion dodge_speed_mps "$1" 2>&1 | tr -d '\r'
}

restore() {
  echo "== restoring dodge_speed_mps=$BASELINE_DSPEED"
  set_dspeed "$BASELINE_DSPEED" || \
    echo "!! RESTORE FAILED - /evasion left at a non-baseline dodge_speed_mps" >&2
  check_baseline "exit"
}
trap restore EXIT

check_baseline "entry"

alt_now() {
  timeout 12 ros2 topic echo /huitzilin/odom --once --field pose.pose.position.z 2>/dev/null \
    | head -1 | tr -d ' \r'
}

i=0
for A in $ARMS; do
  i=$((i+1))
  TAG="$(printf '%02d_%s' "$i" "$A")"
  echo ""
  echo "################ arm $i : dodge_speed_mps = $A ################"

  a="$(alt_now)"
  if [ -z "$a" ] || ! awk -v a="$a" 'BEGIN{exit !(a+0 >= 1.0)}'; then
    echo "!! drone is not airborne (alt=${a:-none}) - aborting before arm $A" >&2
    exit 1
  fi
  echo "[pre] altitude $a m"

  echo "[set] dodge_speed_mps -> $A"
  set_dspeed "$A"
  # Read it back: FIXED_AT_START keys are refused wholesale by the node's
  # _on_param_set, and a refused set must not be flown as if it took.
  RB="$(timeout 12 ros2 param get /evasion dodge_speed_mps 2>/dev/null | tr -d '\r')"
  echo "[readback] $RB"
  if ! printf '%s' "$RB" | grep -q "$A"; then
    echo "!! readback does not show $A - skipping arm" >&2
    continue
  fi
  # MAVLink params are NOT swept here, but dump them so each arm's row can be
  # proved to have flown at the same WP_ACC / jerk / tilt limits.
  $PSET --dump "$RESULTS/params_dspeed_${TAG}.parm" >/dev/null

  RESP="$RESULTS/dodge_resp_dspeed_${TAG}.csv"
  rm -f "$RESP"
  nohup setsid python3 "$HOME/huitzilin_ws/scripts/hz_dodge_response.py" \
    --out "$RESP" > "/tmp/lab_resp_dspeed_${TAG}.log" 2>&1 < /dev/null &
  RESP_PID=$!
  sleep 8
  echo "[probe] recording -> $RESP"

  ros2 run huitzilin_perception dodge_battery --ros-args \
    -p use_sim_time:=true \
    -p "battery_config:=$CONFIG" \
    -p "output_file:=$RESULTS/dspeed_${TAG}_battery.txt" \
    -p "csv_file:=$RESULTS/dspeed_${TAG}_battery.csv" \
    2>&1 | tail -25

  sleep 4
  kill -TERM -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null
  sleep 3
  kill -KILL -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null

  echo "[arm $TAG] dodges recorded: $(( $(wc -l < "$RESP" 2>/dev/null || echo 1) - 1 )) rows"
done

echo ""
echo "== all arms complete"
