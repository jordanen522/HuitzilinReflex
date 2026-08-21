#!/usr/bin/env bash
# Optics probe driver. One camera configuration in, one result row out.
#
# Usage: run_probe.sh NAME HFOV_DEG WIDTH HEIGHT RATE_HZ FAR_M BALL_X_M [FRAMES]
#
# Source ROS BEFORE running this: `set -u` here would break setup.bash.
set -euo pipefail
set -m   # job control, so signals reach the children (see bag_capture_runbook)

NAME="${1:?name}"; HFOV_DEG="${2:?hfov_deg}"; W="${3:?width}"; H="${4:?height}"
RATE="${5:?rate_hz}"; FAR="${6:?far_m}"; BALLX="${7:?ball_x_m}"; FRAMES="${8:-30}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${PROBE_OUT:-$HERE/../../lab/probe_out}"; mkdir -p "$OUT"
WORLD="$OUT/${NAME}.sdf"

export HFOV_DEG
HFOV_RAD="$(python3 -c "import math,os;print(format(math.radians(float(os.environ['HFOV_DEG'])),'.6f'))")"

sed -e "s|@HFOV_RAD@|$HFOV_RAD|g" -e "s|@WIDTH@|$W|g" -e "s|@HEIGHT@|$H|g" \
    -e "s|@RATE@|$RATE|g" -e "s|@FAR@|$FAR|g" -e "s|@BALLX@|$BALLX|g" \
    "$HERE/probe_world.sdf.in" > "$WORLD"

cleanup() {
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  [ -n "${GZ_PID:-}" ] && kill "$GZ_PID" 2>/dev/null || true
  sleep 2
  pkill -f "parameter_bridge.*probe" 2>/dev/null || true
  pkill -f "${NAME}.sdf" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "=== $NAME: hfov=${HFOV_DEG}deg ${W}x${H} @${RATE}Hz far=${FAR}m ball@${BALLX}m ==="

gz sim -s -r --headless-rendering "$WORLD" > "$OUT/${NAME}.gz.log" 2>&1 &
GZ_PID=$!
sleep 8

ros2 run ros_gz_bridge parameter_bridge \
  "/gz/probe/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args -r /gz/probe/depth/points:=/probe/points \
  > "$OUT/${NAME}.bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 5

python3 "$HERE/count_ball_points.py" --topic /probe/points --frames "$FRAMES" \
  | tee "$OUT/${NAME}.result.txt"
