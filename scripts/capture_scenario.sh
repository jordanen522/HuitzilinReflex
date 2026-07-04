#!/usr/bin/env bash
# capture_scenario.sh <SCENARIO_ID> — record one labeled Week 3 rosbag (W3-10).
#
# Prereqs (all already running, drone AIRBORNE and patrolling):
#   T1 gz sim  ·  T2 sim_vehicle.py  ·  T3 week3_perception.launch.py with_patrol:=true
#   and THIS shell sourced:  source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
#
# Records the 5 topics for RECORD_SECONDS (default 10), spawning the projectile
# ~3 s in for scenarios with speed>0, then stops the recorder via `timeout
# --signal=INT` (robust; cannot hang like a manual kill/wait) and writes the label.
#
# Env overrides:  RECORD_SECONDS (default 10; use 60 for N05)
#
# Manual cases (see docs/week3_capture_runbook.md):
#   N04 — needs a vertical-offset spawn this script can't do; it refuses N04.
#   N02 — negative "patrol turn"; recorded here as a normal patrol window.
set -euo pipefail

ID="${1:?usage: capture_scenario.sh <SCENARIO_ID>  (e.g. S02)}"
BAG_DIR="/data/huitzilin_bags"
MATRIX="$HOME/huitzilin_ws/src/huitzilin_perception/config/scenario_matrix.yaml"
RECORD_SECONDS="${RECORD_SECONDS:-10}"
SPAWN_LEAD=3   # seconds into the recording before spawning

if [ "$ID" = "N04" ]; then
  echo "N04 needs a vertical-offset spawn this script can't do — capture it by hand (runbook)."; exit 2
fi
mkdir -p "$BAG_DIR"

eval "$(python3 - "$MATRIX" "$ID" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1])); sid = sys.argv[2]
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

# Spawn (if any) fires in the background ~SPAWN_LEAD s into the recording.
SPAWN_BG=""
if [ "$SPAWN" = "yes" ]; then
  ( sleep "$SPAWN_LEAD"
    echo "[$ID] spawning projectile..."
    ros2 run huitzilin_perception spawn_projectile --ros-args \
      -p scenario_id:="$ID" -p speed_mps:="$SPEED" -p approach_angle_deg:="$ANGLE" \
      -p miss_distance_m:="$MISS" -p offset_forward_m:="$OFFSET" \
      || echo "[$ID] (spawn returned nonzero — usually just the gz confirm timeout; continuing)"
  ) &
  SPAWN_BG=$!
else
  echo "[$ID] negative scenario — no spawn; recording a clean patrol window."
fi

# Record in the FOREGROUND; timeout sends SIGINT after the window so rosbag2
# flushes cleanly. --kill-after force-kills if it ever refuses to stop.
TOTAL=$(( RECORD_SECONDS + SPAWN_LEAD ))
echo "[$ID] recording ${TOTAL}s (spawn at +${SPAWN_LEAD}s)..."
timeout --signal=INT --kill-after=20 "$TOTAL" \
  ros2 bag record -s mcap -o "$BAG" \
  --topics /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid \
  || true   # timeout exits 124, SIGINT-stopped rosbag exits nonzero — both expected

[ -n "$SPAWN_BG" ] && { wait "$SPAWN_BG" 2>/dev/null || true; }

cat > "$BAG_DIR/week3_${ID}.label.yaml" <<YAML
scenario_id: $ID
label: $LABEL
closest_approach_m: $CLOSEST
time_to_closest_s: $TTC
detection_window_s: 4.0
YAML

echo "[$ID] done — bag: $BAG   label: $BAG_DIR/week3_${ID}.label.yaml"
ros2 bag info "$BAG" | grep -E 'Duration|Count:' || true
