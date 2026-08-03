#!/usr/bin/env bash
# preflight_hw.sh - hardware bench checklist, Week 5 S5-6.
#
# preflight_check.sh is the SITL equivalent (Gazebo, ardu_ws, a SITL
# heartbeat). This is its hardware counterpart: real FC on serial, real
# OAK-D, real Pi.
#
# USAGE:  ./scripts/preflight_hw.sh [connection]
#         ./scripts/preflight_hw.sh /dev/ttyACM0
#
# THIS IS A CHECKLIST, NOT A GATE. It always exits 0 and counts warnings.
# Deliberately no `set -e`: a partly-built aircraft fails most of these, and
# aborting at the first would hide the other eight. The point is to see all
# nine at once.
#
# PROPS OFF for everything here (SAFETY_CASE.md section 4).

# No `set -e`: a partly-built aircraft fails most of these checks and aborting
# at the first would hide the other eight.
# No `set -u` either: /opt/ros/jazzy/setup.bash references unbound variables
# (AMENT_TRACE_SETUP_FILES and friends) and exits the shell under -u. That is
# why run_regression.sh only enables -u after sourcing; this script sources
# inside a numbered stage, so it stays off throughout. Unset variables are
# handled explicitly with ${VAR:-} instead.
set -o pipefail

WARN=0
warn() { echo "    WARNING: $*"; WARN=$((WARN + 1)); }
ok()   { echo "    OK: $*"; }

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
PARM="$WS/src/huitzilin_sim/params/hw_frame.parm"
HW_BRIDGE="$WS/src/huitzilin_sim/params/hw_bridge.yaml"

# Default to the by-id path in hw_bridge.yaml rather than a hardcoded ttyACM0:
# ttyACM numbering shifts when anything else enumerates first.
DEFAULT_CONN="$(sed -n 's/^ *connection: *"\(.*\)"/\1/p' "$HW_BRIDGE" 2>/dev/null | head -1)"
CONN="${1:-${DEFAULT_CONN:-/dev/ttyACM0}}"

echo "=== HuitzilinReflex hardware preflight ==="
echo "    Workspace  : $WS"
echo "    Connection : $CONN"
echo "    PROPS OFF, kill-switch in hand."
echo ""

echo "[1/9] ROS 2 + workspace"
if source /opt/ros/jazzy/setup.bash 2>/dev/null; then
  # `ros2 --version` is not a flag in Jazzy; it prints the usage block.
  ok "ROS 2 ${ROS_DISTRO:-unknown} ($(command -v ros2))"
else
  warn "could not source /opt/ros/jazzy/setup.bash"
fi
if source "$WS/install/setup.bash" 2>/dev/null; then
  ok "workspace overlay sourced"
else
  warn "workspace not built - run: colcon build --symlink-install"
fi
echo ""

echo "[2/9] Flight controller serial device"
if [ -e "$CONN" ]; then
  ok "$CONN present"
  case "$CONN" in
    /dev/serial/by-id/*) ok "stable by-id path" ;;
    *) warn "$CONN is not a /dev/serial/by-id path; it renumbers on re-enumeration" ;;
  esac
else
  warn "$CONN not present - FC not connected, or hw_bridge.yaml still has the placeholder id"
fi
echo ""

echo "[3/9] Heartbeat from the flight controller"
if [ -e "$CONN" ]; then
  python3 - "$CONN" <<'PY'
import sys
try:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(sys.argv[1], baud=115200)
    hb = m.wait_heartbeat(timeout=5)
    print("    OK: heartbeat sys=%d comp=%d" % (m.target_system, m.target_component)
          if hb else "    WARNING: no heartbeat in 5 s")
except Exception as e:
    print("    WARNING: %s" % e)
PY
else
  warn "skipped - no serial device"
fi
echo ""

echo "[4/9] Parameter readback vs hw_frame.parm"
if [ -e "$CONN" ]; then
  # Exit 1 and 2 mean different things (mismatch vs unreachable FC) and a bare
  # '||' reported both as "parameter mismatch", which sends you tuning
  # parameters when the real problem is a dead serial link.
  RB=0
  python3 "$HERE/hw_param_readback.py" --connection "$CONN" || RB=$?
  case "$RB" in
    0) ;;
    1) warn "parameter mismatch (see above) - load $PARM" ;;
    2) warn "could NOT reach the flight controller on $CONN - not a parameter problem" ;;
    *) warn "hw_param_readback.py exited $RB" ;;
  esac
else
  warn "skipped - no serial device. Expected set: $(grep -cvE '^\s*(#|$)' "$PARM") parameters"
fi
echo ""

echo "[5/9] OAK-D at SuperSpeed"
if command -v lsusb >/dev/null 2>&1; then
  if lsusb 2>/dev/null | grep -qi "03e7"; then
    if lsusb -t 2>/dev/null | grep -q "5000M"; then
      ok "Movidius device present, a 5000M port is in use"
    else
      warn "OAK-D present but no 5000M link - USB-2 negotiation, re-seat the cable/port"
    fi
  else
    warn "no Movidius (03e7) device - OAK-D not enumerated"
  fi
else
  warn "lsusb not available"
fi
echo ""

echo "[6/9] Pi throttling"
if command -v vcgencmd >/dev/null 2>&1; then
  T="$(vcgencmd get_throttled 2>/dev/null)"
  if [ "$T" = "throttled=0x0" ]; then
    ok "$T"
  else
    warn "$T - undervoltage or thermal limiting; check the BEC before flying"
  fi
else
  warn "vcgencmd not available (not a Pi) - skipping throttle check"
fi
echo ""

echo "[7/9] Disk space for logs and bags"
AVAIL_MB="$(df -Pm "$WS" 2>/dev/null | awk 'NR==2 {print $4}')"
if [ -n "${AVAIL_MB:-}" ] && [ "$AVAIL_MB" -ge 5000 ]; then
  ok "${AVAIL_MB} MB free"
else
  warn "only ${AVAIL_MB:-?} MB free - a depth bag fills this fast"
fi
echo ""

echo "[8/9] Kill-switch channel"
echo "    MANUAL: toggle the kill-switch and confirm the PWM moves."
echo "      mavproxy.py --master=$CONN"
echo "      status RC_CHANNELS      (watch chan7_raw)"
echo "    RC7_OPTION must read 31 (motor emergency stop), on its own dedicated"
echo "    momentary switch - never a shared or mode switch."
echo ""

echo "[9/9] Payload backends"
python3 -c "import rpi_ws281x" 2>/dev/null && ok "rpi_ws281x importable" \
  || warn "rpi_ws281x missing - payload_node degrades to a no-op (fine off-Pi)"
python3 -c "import gpiod" 2>/dev/null && ok "gpiod importable" \
  || warn "gpiod missing - siren degrades to a no-op (fine off-Pi)"
echo ""

echo "=== $WARN warning(s) - this is a checklist, not a gate ==="
echo ""
echo "Next, in order (hardware_bringup.md section 6), PROPS OFF throughout:"
echo "  1. Arm via GCS; confirm IMU/EKF healthy and every failsafe fires."
echo "  2. Confirm the Pi stays up on BEC power alone, USB-C PSU unplugged."
echo "  3. Confirm the OAK-D enumerates SuperSpeed (check 5 above)."
echo "  4. Only then: props on, airframe held down, brief static throttle-up."
echo ""
echo "Flight stack against the real FC:"
echo "  ros2 run huitzilin_sim mav_bridge --ros-args \\"
echo "    --params-file $HW_BRIDGE -p use_sim_time:=false"
exit 0
