#!/usr/bin/env bash
# lab_wpacc.sh — A/B the dodge escape curve across WP_ACC values.
#
# THE HYPOTHESIS UNDER TEST. Dataflash PSCN.DAN / PSCE.DAE never exceed 2.500
# m/s^2 in any of 40 logged dodges, and sit pinned at exactly that value for a
# mean 39% of the dodge second (35/40 dodges reach it). 2.500 is WP_ACC, the
# default. ModeGuided::pva_control_start() passes wp_nav->get_wp_acceleration_mss()
# to the horizontal position controller, so every GUIDED velocity setpoint --
# i.e. every dodge -- is acceleration-limited by it. Measured escape rises at
# ~2.17 m/s^2, just under the clamp.
#
# FALSIFICATION. If WP_ACC binds, escape displacement at fixed t must ORDER
# with it: 1.0 < 2.5 < 5.0. If escape is flat across a 5x span, WP_ACC is not
# the constraint and this hypothesis dies like the jerk one.
#
# Why this design differs from lab_jerk.sh: that sweep went one direction only
# and ended with n = 4/4/2 after turn-artifact exclusion, so its arms were not
# separable. Here the span brackets the baseline in BOTH directions (a lever
# must lower escape as well as raise it), repeats are 8 not 6, and the 2.5
# control is flown TWICE -- first and last -- so drift across the session is
# measured rather than assumed.
#
# ISOLATION: a PARAM_SET persists to eeprom.bin. Safe ONLY because SITL here
# runs with CWD=~/hz_lab. Baseline sha256 checked at entry and exit, WP_ACC
# restored on ANY exit path via trap.

# ROS before `set -u` -- setup.bash reads unset vars (CLAUDE.md sharp edge).
source "$HOME/hz_tools/hz_env.sh"
source "$HOME/venv-ardupilot/bin/activate"
set -u

LAB="$HOME/hz_lab"
RESULTS="$LAB/results"
CONFIG="$LAB/config/wpacc_probe.yaml"
PSET="python3 $LAB/bin/lab_param_set.py"
GUARD="$HOME/hz_baseline_guard/eeprom.bin.sha256"
ARMS="${ARMS:-2.5 1.0 5.0 2.5}"
BASELINE_WPACC="${BASELINE_WPACC:-2.5}"

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
  echo "== restoring WP_ACC=$BASELINE_WPACC"
  $PSET --set "WP_ACC=$BASELINE_WPACC" || \
    echo "!! RESTORE FAILED - lab vehicle left at a non-baseline WP_ACC" >&2
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
  echo "################ arm $i : WP_ACC = $A ################"

  a="$(alt_now)"
  if [ -z "$a" ] || ! awk -v a="$a" 'BEGIN{exit !(a+0 >= 1.0)}'; then
    echo "!! drone is not airborne (alt=${a:-none}) - aborting before arm $A" >&2
    exit 1
  fi
  echo "[pre] altitude $a m"

  if ! $PSET --set "WP_ACC=$A"; then
    echo "!! could not set WP_ACC=$A (clamped or absent) - skipping arm" >&2
    continue
  fi
  $PSET --dump "$RESULTS/params_wpacc_${TAG}.parm" >/dev/null

  RESP="$RESULTS/dodge_resp_wpacc_${TAG}.csv"
  rm -f "$RESP"
  nohup setsid python3 "$HOME/huitzilin_ws/scripts/hz_dodge_response.py" \
    --out "$RESP" > "/tmp/lab_resp_wpacc_${TAG}.log" 2>&1 < /dev/null &
  RESP_PID=$!
  sleep 8
  echo "[probe] recording -> $RESP"

  ros2 run huitzilin_perception dodge_battery --ros-args \
    -p use_sim_time:=true \
    -p "battery_config:=$CONFIG" \
    -p "output_file:=$RESULTS/wpacc_${TAG}_battery.txt" \
    -p "csv_file:=$RESULTS/wpacc_${TAG}_battery.csv" \
    2>&1 | tail -25

  sleep 4
  kill -TERM -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null
  sleep 3
  kill -KILL -"$(ps -o pgid= -p "$RESP_PID" | tr -d ' ')" 2>/dev/null

  echo "[arm $TAG] dodges recorded: $(( $(wc -l < "$RESP" 2>/dev/null || echo 1) - 1 )) rows"
done

echo ""
echo "== all arms complete"
