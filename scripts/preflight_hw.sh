#!/usr/bin/env bash
# preflight_hw.sh - hardware bench checklist, run on the Pi.
#
# preflight_check.sh is the SITL equivalent. This one checks the real Pi, the
# router in front of the flight controller, the OAK-D and the payload wiring.
#
# USAGE:  ./scripts/preflight_hw.sh
#
# THIS IS A CHECKLIST, NOT A GATE. It always exits 0 and counts warnings.
# No `set -e`: a partly-built aircraft fails most of these, and aborting at the
# first would hide the rest. No `set -u` either: /opt/ros/jazzy/setup.bash
# references unbound variables.
#
# PROPS OFF for everything here (SAFETY_CASE.md section 4). docs/HARDWARE.md
# says what to do about each warning.
set -o pipefail

WARN=0
warn() { echo "    WARNING: $*"; WARN=$((WARN + 1)); }
ok()   { echo "    OK: $*"; }

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
ROUTER_CONF=/etc/mavlink-router/main.conf
BOOT_CONFIG=/boot/firmware/config.txt
TOOLS=udpin:0.0.0.0:14554          # router [UdpEndpoint tools]

echo "=== HuitzilinReflex hardware preflight ==="
echo "    PROPS OFF, kill-switch in hand."
echo ""

echo "[1/9] ROS 2 + workspace"
if source /opt/ros/jazzy/setup.bash 2>/dev/null; then
  ok "ROS 2 ${ROS_DISTRO:-unknown}"
else
  warn "could not source /opt/ros/jazzy/setup.bash"
fi
if source "$WS/install/setup.bash" 2>/dev/null; then
  ok "workspace overlay sourced"
else
  warn "workspace not built - run: colcon build --symlink-install"
fi
echo ""

echo "[2/9] Flight controller USB device + router config"
FC_IDS="$(ls /dev/serial/by-id/ 2>/dev/null | grep -i ardupilot || true)"
if [ -n "$FC_IDS" ]; then
  ok "FC enumerated: $FC_IDS"
else
  warn "no ArduPilot device in /dev/serial/by-id - FC unplugged or not flashed"
fi
DEVICE="$(sed -n 's/^ *Device *= *//p' "$ROUTER_CONF" 2>/dev/null | head -1)"
if [ -z "$DEVICE" ]; then
  warn "$ROUTER_CONF missing - install hardware/mavlink-router.conf there"
elif [ "${DEVICE#*CHANGE-ME}" != "$DEVICE" ]; then
  warn "$ROUTER_CONF still has the CHANGE-ME placeholder - set Device to /dev/serial/by-id/$FC_IDS"
elif [ -e "$DEVICE" ]; then
  ok "router Device $DEVICE present"
else
  warn "router Device $DEVICE does not exist"
fi
echo ""

echo "[3/9] mavlink-router service + heartbeat through it"
if systemctl is-active --quiet mavlink-router 2>/dev/null; then
  ok "mavlink-router active"
else
  warn "mavlink-router not running - sudo systemctl enable --now mavlink-router"
fi
python3 - "$TOOLS" <<'PY'
import sys
try:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(sys.argv[1])
    hb = m.wait_heartbeat(timeout=5)
    print("    OK: heartbeat sys=%d comp=%d" % (m.target_system, m.target_component)
          if hb else "    WARNING: no heartbeat on %s in 5 s" % sys.argv[1])
    m.close()
except Exception as e:
    print("    WARNING: %s" % e)
PY
echo ""

echo "[4/9] Parameter readback vs hw_frame.parm"
RB=0
python3 "$HERE/hw_param_readback.py" --connection "$TOOLS" --timeout 20 || RB=$?
case "$RB" in
  0) ;;
  1) warn "parameter mismatch (see above) - load src/huitzilin_sim/params/hw_frame.parm" ;;
  2) warn "could NOT reach the flight controller - not a parameter problem" ;;
  *) warn "hw_param_readback.py exited $RB" ;;
esac
echo ""

echo "[5/9] OAK-D at SuperSpeed"
if lsusb 2>/dev/null | grep -qi "03e7"; then
  if lsusb -t 2>/dev/null | grep -q "5000M"; then
    ok "Movidius device present, a 5000M port is in use"
  else
    warn "OAK-D present but no 5000M link - USB-2 negotiation, use a blue USB-3 port"
  fi
else
  warn "no Movidius (03e7) device - OAK-D not enumerated"
fi
echo ""

echo "[6/9] Pi power and boot config"
T="$(vcgencmd get_throttled 2>/dev/null || true)"
if [ "$T" = "throttled=0x0" ]; then
  ok "$T"
elif [ -n "$T" ]; then
  warn "$T - undervoltage or thermal limiting; check the BEC before flying"
else
  warn "vcgencmd not available - install libraspberrypi-bin"
fi
for line in usb_max_current_enable=1 dtparam=spi=on; do
  if grep -q "^$line" "$BOOT_CONFIG" 2>/dev/null; then
    ok "$line in $BOOT_CONFIG"
  else
    warn "$line missing from $BOOT_CONFIG"
  fi
done
echo ""

echo "[7/9] Disk space for logs and bags"
AVAIL_MB="$(df -Pm "$WS" 2>/dev/null | awk 'NR==2 {print $4}')"
if [ -n "${AVAIL_MB:-}" ] && [ "$AVAIL_MB" -ge 5000 ]; then
  ok "${AVAIL_MB} MB free"
else
  warn "only ${AVAIL_MB:-?} MB free - a depth bag fills this fast"
fi
echo ""

echo "[8/9] Payload: SPI LED + siren GPIO"
python3 -c "import spidev" 2>/dev/null && ok "spidev importable" \
  || warn "spidev missing - sudo apt install python3-spidev"
[ -e /dev/spidev0.0 ] && ok "/dev/spidev0.0 present" \
  || warn "/dev/spidev0.0 missing - dtparam=spi=on, then reboot"
python3 -c "import gpiod" 2>/dev/null && ok "gpiod importable" \
  || warn "gpiod missing - sudo apt install python3-libgpiod"
RP1="$(gpiodetect 2>/dev/null | grep pinctrl-rp1 || true)"
[ -n "$RP1" ] && ok "header chip: $RP1" \
  || warn "no pinctrl-rp1 chip in gpiodetect - siren_chip auto will not find the header"
for g in dialout spi gpio; do
  id -nG | grep -qw "$g" && ok "user in group $g" \
    || warn "user not in group $g - sudo usermod -aG $g $USER, then log in again"
done
echo ""

echo "[9/9] Kill-switch channel"
echo "    MANUAL: in QGroundControl (UDP 14550 to this Pi), open the radio page,"
echo "    flip the kill switch and watch channel 7 move. RC7_OPTION must read 31"
echo "    (motor emergency stop), on its own switch - never a shared or mode switch."
echo ""

echo "=== $WARN warning(s) - this is a checklist, not a gate ==="
echo "Next: docs/HARDWARE.md, bench stages 1-6, PROPS OFF throughout."
exit 0
