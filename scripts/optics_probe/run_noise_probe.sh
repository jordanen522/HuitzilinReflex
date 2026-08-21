#!/usr/bin/env bash
# Noise-stage probe driver. Same rig as run_probe.sh, with depth_noise_node
# spliced between the bridge and the measurement:
#
#   gz depth camera -> ros_gz_bridge -> depth_noise_node -> measure_ball_noise
#
# The point is the A/B. Run it once at the modelled sigma and once at
# SIGMA=0.0; the noised arm must scatter the ball centroid's range at ~sigma
# while the control collapses to ~0. A noised arm that also reads ~0 means the
# cloud is passing through untouched.
#
# Usage: run_noise_probe.sh NAME SIGMA_REF_M BALL_X_M [FRAMES] [CORRELATION_PX]
#
# Optics are pinned to the iris_ar0234 configuration (800x650 @ 27.0 deg,
# far 35 m) because that is the lane this stage exists to serve. A sensor is
# reach AND sector AND rate: all three are fixed here on purpose, so the only
# thing varying between arms is the noise.
#
# Source ROS BEFORE running this: `set -u` here would break setup.bash.
set -euo pipefail
set -m   # job control, so signals reach the children (see bag_capture_runbook)

NAME="${1:?name}"; SIGMA="${2:?sigma_ref_m}"; BALLX="${3:?ball_x_m}"
FRAMES="${4:-120}"; CORR="${5:-7.0}"

# Pinned iris_ar0234 optics. Keep in step with models/iris_ar0234/model.sdf.
HFOV_DEG=27.0; W=800; H=650; RATE=30; FAR=35.0

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${PROBE_OUT:-$HERE/../../lab/probe_out}"; mkdir -p "$OUT"
WORLD="$OUT/${NAME}.sdf"

export HFOV_DEG
HFOV_RAD="$(python3 -c "import math,os;print(format(math.radians(float(os.environ['HFOV_DEG'])),'.6f'))")"

sed -e "s|@HFOV_RAD@|$HFOV_RAD|g" -e "s|@WIDTH@|$W|g" -e "s|@HEIGHT@|$H|g" \
    -e "s|@RATE@|$RATE|g" -e "s|@FAR@|$FAR|g" -e "s|@BALLX@|$BALLX|g" \
    "$HERE/probe_world.sdf.in" > "$WORLD"

cleanup() {
  [ -n "${NOISE_PID:-}" ] && kill "$NOISE_PID" 2>/dev/null || true
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  [ -n "${GZ_PID:-}" ] && kill "$GZ_PID" 2>/dev/null || true
  sleep 2
  # Kill by INSTALLED PATH, never a guessed _node name: CLAUDE.md records
  # seven stacks accumulating behind a pattern that matched nothing.
  pkill -f "parameter_bridge.*probe" 2>/dev/null || true
  pkill -f "huitzilin_perception/depth_noise" 2>/dev/null || true
  pkill -f "${NAME}.sdf" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "=== $NAME: ${W}x${H} @${HFOV_DEG}deg ball@${BALLX}m | sigma_ref=${SIGMA} m corr=${CORR} px ==="

gz sim -s -r --headless-rendering "$WORLD" > "$OUT/${NAME}.gz.log" 2>&1 &
GZ_PID=$!
sleep 8

ros2 run ros_gz_bridge parameter_bridge \
  "/gz/probe/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args -r /gz/probe/depth/points:=/oak/points_rendered \
  > "$OUT/${NAME}.bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 5

ros2 run huitzilin_perception depth_noise --ros-args \
  -p cloud_in_topic:=/oak/points_rendered \
  -p cloud_out_topic:=/oak/points \
  -p sigma_ref_m:="$SIGMA" \
  -p ref_range_m:=26.0 \
  -p correlation_px:="$CORR" \
  > "$OUT/${NAME}.noise.log" 2>&1 &
NOISE_PID=$!
sleep 5

{
  echo "config              ${W}x${H} @${HFOV_DEG}deg far=${FAR}m rate=${RATE}Hz"
  echo "ball_x_m            ${BALLX}"
  echo "sigma_ref_m         ${SIGMA}   (at ref_range_m 26.0)"
  echo "correlation_px      ${CORR}"
  python3 "$HERE/measure_ball_noise.py" --topic /oak/points --frames "$FRAMES"
} | tee "$OUT/${NAME}.result.txt"

echo "--- depth_noise node log ---"
grep -E "depth_noise:" "$OUT/${NAME}.noise.log" || true
