#!/usr/bin/env bash
# lab_up.sh — bring up an ISOLATED SITL lab (world + SITL + MAVProxy + stack).
#
# ISOLATION CONTRACT
#   ArduPilot SITL writes eeprom.bin into its CWD. sim_vehicle.py is started
#   here with CWD=$LAB_DIR, so the baseline vehicle state at
#   ~/huitzilin_ws/eeprom.bin is never opened by anything this script starts.
#   That is CHECKED, not assumed: the baseline sha256 is verified before and
#   after bring-up, and the script fails loudly if it moved.
#
#   This is why hz_tools/hz_restart.sh cannot be reused: its line 8 is
#   `cd "$HOME/huitzilin_ws"`, which is exactly the directory whose eeprom.bin
#   must not be written.
#
# Usage:
#   STACK=oracle RANGE=3.4 ./lab_up.sh
#   STACK=week4 ./lab_up.sh          # real depth detector
#   STACK=bridge ./lab_up.sh         # bridge only (maneuver probe; no ball)
#   STACK=none  ./lab_up.sh          # world + SITL only (param work)

set -u

LAB_DIR="${LAB_DIR:-$HOME/hz_lab}"
STACK="${STACK:-oracle}"
RANGE="${RANGE:-12.0}"
ORACLE_RATE="${ORACLE_RATE:-14.5}"
EXTRA_LAUNCH="${EXTRA_LAUNCH:-}"
WORLD_WAIT="${WORLD_WAIT:-25}"
SITL_WAIT="${SITL_WAIT:-40}"
MAVP_WAIT="${MAVP_WAIT:-25}"
STACK_WAIT="${STACK_WAIT:-35}"

TOOLS="$HOME/hz_tools"
GUARD="$HOME/hz_baseline_guard/eeprom.bin.sha256"

check_baseline() {
  if sha256sum -c "$GUARD" --status 2>/dev/null; then
    echo "[guard] baseline eeprom intact ($1)"
    return 0
  fi
  echo "!! BASELINE EEPROM CHANGED at stage: $1" >&2
  echo "!! restore from ~/hz_baseline_guard/eeprom.bin.baseline before continuing" >&2
  return 1
}

check_baseline "pre-bringup" || exit 1

mkdir -p "$LAB_DIR"
if [ ! -f "$LAB_DIR/eeprom.bin" ]; then
  echo "[lab] seeding isolated eeprom from baseline copy"
  cp -p "$HOME/hz_baseline_guard/eeprom.bin.baseline" "$LAB_DIR/eeprom.bin"
fi

echo "== teardown"
# Killed by INSTALLED PATH, not by executable name. `evasion_nod[e]` matched
# nothing (colcon installs the entry point as `lib/huitzilin_perception/
# evasion`), which let node processes outlive the `ros2 launch` process group
# this loop targets. lab_cell.sh's singleton check is the backstop; this is
# the fix. See the long note in lab_queue.sh for what the survivors did.
for pat in "[h]uitzilin_ws/install/" "[r]os2 launch huitzilin" "[p]arameter_bridge" "gz-transport-topi[c]" "[m]avproxy.py" "[s]im_vehicle.py" "[a]rducopter" "[g]z sim"; do
  for p in $(pgrep -f "$pat"); do
    kill -TERM -"$(ps -o pgid= -p "$p" | tr -d ' ')" 2>/dev/null
  done
done
sleep 6
for pat in "[h]uitzilin_ws/install/" "[r]os2 launch huitzilin" "[p]arameter_bridge" "gz-transport-topi[c]" "[m]avproxy.py" "[s]im_vehicle.py" "[a]rducopter" "[g]z sim"; do
  for p in $(pgrep -f "$pat"); do
    kill -KILL -"$(ps -o pgid= -p "$p" | tr -d ' ')" 2>/dev/null
  done
done
sleep 2

echo "== world"
nohup setsid bash -c "source $TOOLS/hz_env.sh && exec $HOME/huitzilin_ws/scripts/week3_world.sh" \
  > /tmp/lab_world.log 2>&1 < /dev/null & disown
sleep "$WORLD_WAIT"

echo "== SITL (CWD=$LAB_DIR — isolated eeprom)"
nohup setsid bash -c "source $TOOLS/hz_env_sitl.sh && cd '$LAB_DIR' && exec sim_vehicle.py -v ArduCopter \
  -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --no-mavproxy" > /tmp/lab_sitl.log 2>&1 < /dev/null & disown
sleep "$SITL_WAIT"

echo "== MAVProxy"
nohup setsid bash -c "source $TOOLS/hz_env_sitl.sh && cd '$LAB_DIR' && exec mavproxy.py --daemon \
  --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 \
  --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553" \
  > /tmp/lab_mavproxy.log 2>&1 < /dev/null & disown
sleep "$MAVP_WAIT"

case "$STACK" in
  oracle)
    LAUNCH="ros2 launch huitzilin_perception week6_oracle.launch.py with_patrol:=true detection_range_m:=$RANGE oracle_rate_hz:=$ORACLE_RATE $EXTRA_LAUNCH"
    ;;
  week4)
    LAUNCH="ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true $EXTRA_LAUNCH"
    ;;
  bridge)
    # mav_bridge + patrol(autostart false) + telemetry. No oracle and no
    # depth detector: the maneuver probe drives /cmd/evade itself and there
    # is no ball, so anything that renders or detects only costs RTF. Same
    # bridge, same cmd_router, same MAVLink as a scored dodge.
    # week2_sitl.launch.py carries NO clock bridge -- week6_oracle.launch.py
    # says so in its own header and ships one for exactly this reason. Without
    # /clock every node here dies on the clock guard after its 5 s grace, which
    # is precisely what happened the first time this stack was brought up.
    NEED_CLOCK_BRIDGE=1
    LAUNCH="ros2 launch huitzilin_sim week2_sitl.launch.py $EXTRA_LAUNCH"
    ;;
  none)
    LAUNCH=""
    ;;
  *)
    echo "unknown STACK=$STACK" >&2; exit 2 ;;
esac

if [ "${NEED_CLOCK_BRIDGE:-0}" = "1" ]; then
  echo "== clock bridge (gz /clock -> ROS /clock)"
  # A clock SOURCE must run on WALL time: a bridge started with use_sim_time
  # would sit waiting for the very clock it is supposed to be publishing.
  nohup setsid bash -c "source $TOOLS/hz_env.sh && exec ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" > /tmp/lab_clock.log 2>&1 < /dev/null & disown
  sleep 6
  if ! pgrep -f "parameter_bridg[e]" > /dev/null; then
    echo "!! clock bridge failed to start"; tail -5 /tmp/lab_clock.log; exit 1
  fi
fi

if [ -n "$LAUNCH" ]; then
  echo "== ROS stack: $LAUNCH"
  nohup setsid bash -c "source $TOOLS/hz_env.sh && exec $LAUNCH" \
    > /tmp/lab_stack.log 2>&1 < /dev/null & disown
  sleep "$STACK_WAIT"
fi

echo "== eeprom check"
ls -la "$LAB_DIR/eeprom.bin" 2>/dev/null || echo "(lab eeprom missing!)"
check_baseline "post-bringup" || exit 1

echo "== stack log tail"
[ -n "$LAUNCH" ] && tail -5 /tmp/lab_stack.log
echo "== done"
