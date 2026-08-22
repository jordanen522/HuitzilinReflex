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
# Resolve through the ament index like every sibling script, rather than
# assuming the workspace is ~/huitzilin_ws.
MATRIX="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/config/scenario_matrix.yaml"
# Bag filename prefix. "week3_" is every bag captured before 2026-08-21;
# score_bags takes the same default, so nothing existing changes. The held-out
# set is captured with BAG_PREFIX=heldout_ -- those bags are not Week 3 data
# and must not be named as though they were:
#   BAG_PREFIX=heldout_ ./scripts/capture_scenario.sh H01
BAG_PREFIX="${BAG_PREFIX:-week3_}"
SPAWN_LEAD=3

mkdir -p "$BAG_DIR"

# Capture first, then eval. 'eval "$(python3 ...)"' cannot fail the script:
# a command substitution used as an argument is not checked by 'set -e', so an
# unknown scenario id produced an empty eval and the script died 20 lines later
# on '$OFFSET_V: unbound variable' instead of saying the id was wrong.
if ! SCENARIO_ENV="$(python3 - "$MATRIX" "$ID" <<'PY'
import shlex, sys, yaml
d = yaml.safe_load(open(sys.argv[1])); sid = sys.argv[2]
try:
    r = next(s for s in d["scenarios"] if s["id"] == sid)
except StopIteration:
    sys.exit(f"scenario {sid} not in matrix")
speed = float(r.get("speed_mps", 0) or 0)
# shlex.quote: labels are free text, and an unquoted space or metacharacter
# would be re-parsed by eval.
print(f'LABEL={shlex.quote(str(r["label"]))}')
print(f'SPEED={speed}')
print(f'ANGLE={r.get("approach_angle_deg",0.0)}')
print(f'MISS={r.get("miss_distance_m",0.0)}')
print(f'OFFSET={r.get("offset_forward_m",6.0)}')
print(f'OFFSET_V={r.get("offset_vertical_m",0.0) or 0.0}')
print(f'CLOSEST={r.get("closest_approach_m",0.0)}')
print(f'TTC={r.get("time_to_closest_s",0.0)}')
print(f'SPAWN={"yes" if speed > 0 else "no"}')
# compensate_gravity defaults False, matching spawn_projectile's own default,
# so every S/N/T entry keeps the flat throw it was captured with. The held-out
# H entries set it true: they spawn ~36 m out, and a flat throw from 2 m
# altitude is on the ground after 0.639 s and ~3 m of travel, so it would never
# reach the drone at all -- a geometry failure that would score as a perception
# failure on every single positive.
print(f'GRAVCOMP={"true" if r.get("compensate_gravity", False) else "false"}')
PY
)"; then
  echo "[$ID] ERROR: could not read scenario $ID from $MATRIX"; exit 2
fi
eval "$SCENARIO_ENV"

# Vertical-offset scenarios (N04): the spawn point must stay above ground,
# so the drone has to be flying high enough BEFORE we start recording.
# N04 = -10 m offset -> needs altitude >= ~10.5 m (e.g. takeoff_alt_m: 12.0).
if awk "BEGIN{exit !($OFFSET_V < 0)}"; then
  # '|| true': set -e + pipefail must not kill the script before our error msg
  DZ=$(timeout 10 ros2 topic echo --once --field pose.pose.position.z /huitzilin/odom 2>/dev/null | head -1 | tr -d '[:space:]' || true)
  if [ -z "$DZ" ]; then
    echo "[$ID] ERROR: no /huitzilin/odom — is the sim/bridge stack up?"; exit 3
  fi
  SPAWN_Z=$(awk "BEGIN{printf \"%.2f\", $DZ + $OFFSET_V}")
  if awk "BEGIN{exit !($SPAWN_Z < 0.5)}"; then
    echo "[$ID] ERROR: spawn would be at z=${SPAWN_Z} m (drone z=${DZ} m, offset ${OFFSET_V} m)."
    echo "[$ID]        Fly higher first — need altitude >= $(awk "BEGIN{printf \"%.1f\", 0.5 - ($OFFSET_V)}") m"
    echo "[$ID]        (e.g. takeoff_alt_m: 12.0 in bridge.yaml, or a GUIDED climb), then rerun."
    exit 3
  fi
  echo "[$ID] vertical offset ${OFFSET_V} m OK (drone z=${DZ} m -> spawn z=${SPAWN_Z} m)"
fi

BAG="$BAG_DIR/${BAG_PREFIX}${ID}"
rm -rf "$BAG"

# Label first — exists regardless of how you stop the recorder.
cat > "$BAG_DIR/${BAG_PREFIX}${ID}.label.yaml" <<YAML
scenario_id: $ID
label: $LABEL
closest_approach_m: $CLOSEST
time_to_closest_s: $TTC
detection_window_s: 4.0
YAML

echo "[$ID] label=$LABEL spawn=$SPAWN speed=$SPEED angle=$ANGLE miss=$MISS offset=$OFFSET gravcomp=$GRAVCOMP"
echo "[$ID] label written -> $BAG_DIR/${BAG_PREFIX}${ID}.label.yaml"

# Spawn fires SPAWN_LEAD s into the recording (background; survives the exec).
if [ "$SPAWN" = "yes" ]; then
  ( sleep "$SPAWN_LEAD"
    echo "[$ID] >>> spawning projectile now <<<"
    ros2 run huitzilin_perception spawn_projectile --ros-args \
      -p scenario_id:="$ID" -p speed_mps:="$SPEED" -p approach_angle_deg:="$ANGLE" \
      -p miss_distance_m:="$MISS" -p offset_forward_m:="$OFFSET" \
      -p offset_vertical_m:="$OFFSET_V" \
      -p compensate_gravity:="$GRAVCOMP" \
      || echo "[$ID] (spawn returned nonzero — usually just the gz confirm timeout)"
  ) &
  disown
  echo "[$ID] recording... projectile spawns at +${SPAWN_LEAD}s."
else
  echo "[$ID] negative scenario — no spawn; recording a clean patrol window."
fi

echo "[$ID] >>> PRESS Ctrl-C ~8-10 s after the spawn to stop & save the bag. <<<"
# exec so Ctrl-C goes straight to rosbag2 as an interactive SIGINT (clean flush).
# /gz/dynamic_poses IS GROUND TRUTH AND IS NOT OPTIONAL.
# Without it a bag can say THAT a centroid was published and never whether it
# was OF THE BALL. Every bag captured before this line was added lacks it,
# which is why none of them can carry the held-out recall claim -- see
# docs/perception_eval.md section 3.1, which VOIDS any positive bag with no
# projectile transform in this stream.
#
# IT REQUIRES gz_pose_bridge TO BE RUNNING. Nothing else publishes it, and a
# topic nobody publishes records as zero messages with no error: `ros2 bag
# record --topics` on an absent topic waits quietly rather than failing, so a
# whole capture session can complete and be void. Check BEFORE the session:
#   ros2 topic hz /gz/dynamic_poses
exec ros2 bag record -s mcap -o "$BAG" \
  --topics /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid \
           /gz/dynamic_poses
