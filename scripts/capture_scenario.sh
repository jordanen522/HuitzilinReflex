#!/usr/bin/env bash
# capture_scenario.sh <SCENARIO_ID> — record one labeled Week 3 rosbag (W3-10).
#
# Prereqs (all already running, drone AIRBORNE and patrolling):
#   T1 gz sim  ·  T2 sim_vehicle.py  ·  T3 week3_perception.launch.py with_patrol:=true
#   and THIS shell sourced:  source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
#
# Reads the scenario's params from scenario_matrix.yaml, records the 5 topics,
# spawns the projectile (scenarios with speed>0 only), stops, writes the label.
#
# Env overrides:  RECORD_SECONDS (default 10; use 60 for N05)
#
# NOTE — two scenarios need manual handling (see docs/week3_capture_runbook.md):
#   N02  negative "patrol turn" — no spawn; just record while it flies a turn (this
#        script records a normal 10 s patrol window, which is usually fine).
#   N04  negative "far below glide path" — needs a VERTICAL offset spawn_projectile
#        can't do. This script will refuse N04; capture it by hand per the runbook.
set -euo pipefail

ID="${1:?usage: capture_scenario.sh <SCENARIO_ID>  (e.g. S02)}"
BAG_DIR="/data/huitzilin_bags"
MATRIX="$HOME/huitzilin_ws/src/huitzilin_perception/config/scenario_matrix.yaml"
RECORD_SECONDS="${RECORD_SECONDS:-10}"

if [ "$ID" = "N04" ]; then
  echo "N04 needs a vertical-offset spawn this script can't do — capture it by hand (runbook §2)."; exit 2
fi
mkdir -p "$BAG_DIR"

# Pull this scenario's fields from the matrix.
eval "$(python3 - "$MATRIX" "$ID" <<'PY'
import sys, yaml
matrix, sid = sys.argv[1], sys.argv[2]
d = yaml.safe_load(open(matrix))
try:
    r = next(s for s in d["scenarios"] if s["id"] == sid)
except StopIteration:
    sys.exit(f"scenario {sid} not in matrix")
speed = float(r.get("speed_mps", 0) or 0)
print(f'LABEL={r["label"]}')
print(f'SPEED={speed}')
print(f'ANGLE={r.get("approach_angle_deg",0.0)}')
print(f'MISS={r.get("miss_distance_m",0.0)}')
print(f'OFFSET={r.get("offset_forward_m",6.0)}')
print(f'CLOSEST={r.get("closest_approach_m",0.0)}')
print(f'TTC={r.get("time_to_closest_s",0.0)}')
print(f'SPAWN={"yes" if speed > 0 else "no"}')
PY
)"

echo "[$ID] label=$LABEL spawn=$SPAWN speed=$SPEED angle=$ANGLE miss=$MISS offset=$OFFSET record=${RECORD_SECONDS}s"
BAG="$BAG_DIR/week3_${ID}"
rm -rf "$BAG"

ros2 bag record -s mcap -o "$BAG" \
  --topics /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
REC_PID=$!
sleep 3   # let the recorder subscribe (incl. /clock)

if [ "$SPAWN" = "yes" ]; then
  echo "[$ID] spawning projectile..."
  ros2 run huitzilin_perception spawn_projectile --ros-args \
    -p scenario_id:="$ID" -p speed_mps:="$SPEED" -p approach_angle_deg:="$ANGLE" \
    -p miss_distance_m:="$MISS" -p offset_forward_m:="$OFFSET" || \
    echo "[$ID] (spawn_projectile returned nonzero — often just the gz confirm timeout; continuing)"
else
  echo "[$ID] negative scenario — no spawn; recording a clean patrol window."
fi

sleep "$RECORD_SECONDS"
echo "[$ID] stopping recorder..."
kill -INT "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true

# Label sidecar (bag_start_sim_t omitted on purpose: score_bags then counts any
# detection in the bag as the signal — correct for one-threat-per-bag capture).
cat > "$BAG_DIR/week3_${ID}.label.yaml" <<YAML
scenario_id: $ID
label: $LABEL
closest_approach_m: $CLOSEST
time_to_closest_s: $TTC
detection_window_s: 4.0
YAML

echo "[$ID] done."
ros2 bag info "$BAG" | sed -n '1,6p'
grep -c . "$BAG_DIR/week3_${ID}.label.yaml" >/dev/null && echo "[$ID] label written: $BAG_DIR/week3_${ID}.label.yaml"
