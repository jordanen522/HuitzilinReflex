#!/usr/bin/env bash
# capture_scenario.sh <SCENARIO_ID> — record one labeled Week 3 rosbag (W3-10).
#
# Prereqs (all running, drone AIRBORNE and patrolling):
#   T1 gz sim · T2 sim_vehicle.py · T3 week3_perception.launch.py with_patrol:=true
#   and THIS shell sourced (ros2 jazzy + workspace overlay).
#
# Writes the label, fires the projectile ~SPAWN_LEAD s in, then hands the
# recorder to YOUR terminal in the foreground. rosbag2 only flushes cleanly on
# an interactive Ctrl-C, so:  >>> press Ctrl-C ~8-10 s after the spawn <<<  to
# stop and save. The label sidecar is already on disk before recording starts.
#
# Env: RECORD_SECONDS is only advisory here (you stop with Ctrl-C). N05 = long run.
set -euo pipefail

ID="${1:?usage: capture_scenario.sh <SCENARIO_ID>  (e.g. S02)}"
BAG_DIR="/data/huitzilin_bags"
MATRIX="$HOME/huitzilin_ws/src/huitzilin_perception/config/scenario_matrix.yaml"
SPAWN_LEAD=3

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

BAG="$BAG_DIR/week3_${ID}"
rm -rf "$BAG"

# Label first — exists regardless of how you stop the recorder.
cat > "$BAG_DIR/week3_${ID}.label.yaml" <<YAML
scenario_id: $ID
label: $LABEL
closest_approach_m: $CLOSEST
time_to_closest_s: $TTC
detection_window_s: 4.0
YAML

echo "[$ID] label=$LABEL spawn=$SPAWN speed=$SPEED angle=$ANGLE miss=$MISS offset=$OFFSET"
echo "[$ID] label written -> $BAG_DIR/week3_${ID}.label.yaml"

# Spawn fires SPAWN_LEAD s into the recording (background; survives the exec).
if [ "$SPAWN" = "yes" ]; then
  ( sleep "$SPAWN_LEAD"
    echo "[$ID] >>> spawning projectile now <<<"
    ros2 run huitzilin_perception spawn_projectile --ros-args \
      -p scenario_id:="$ID" -p speed_mps:="$SPEED" -p approach_angle_deg:="$ANGLE" \
      -p miss_distance_m:="$MISS" -p offset_forward_m:="$OFFSET" \
      || echo "[$ID] (spawn returned nonzero — usually just the gz confirm timeout)"
  ) &
  disown
  echo "[$ID] recording... projectile spawns at +${SPAWN_LEAD}s."
else
  echo "[$ID] negative scenario — no spawn; recording a clean patrol window."
fi

echo "[$ID] >>> PRESS Ctrl-C ~8-10 s after the spawn to stop & save the bag. <<<"
# exec so Ctrl-C goes straight to rosbag2 as an interactive SIGINT (clean flush).
exec ros2 bag record -s mcap -o "$BAG" \
  --topics /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid
