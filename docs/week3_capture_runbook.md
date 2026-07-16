# Bag capture & regression runbook (Dell box)

How to (re)capture the labeled scenario library and run the detector regression.
Week 3's original capture is done — this is kept for re-captures (e.g. after another
odom/bridge change invalidates the bags) and re-tuning. Native-Ubuntu Dell only; all
timing in **sim time** (`/clock`, message stamps), never wall-clock.

## Bring-up

4 terminals, each sourced (`source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash`):

```bash
# once per checkout:
cd ~/huitzilin_ws && colcon build --symlink-install

# T1 — perception world (exports GZ_SIM_RESOURCE_PATH itself)
./scripts/week3_world.sh
# T2 — SITL
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553
# T3 — perception stack
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
# T4 — arm → takeoff → patrol (odom-polled climb)
./scripts/week3_flyup.sh
```

Sanity gate before recording: `/oak/points` stable at 15 Hz sim (66 ms stamp spacing,
no dropouts) at rest and in flight, and the detector logs
`differencing frame -> fixed (odom TF available)` — if it falls back to camera-frame
mode, the odom attitude is broken; fix that before capturing anything.

## Capture loop — one bag per scenario

Scenario IDs and parameters come from
`src/huitzilin_perception/config/scenario_matrix.yaml` (positives S01–S12, negatives
N01–N05; **S11/S12/N05 are the held-out test split**). Per scenario:

```bash
./scripts/capture_scenario.sh <ID>   # wraps the pattern below; Ctrl-C ~8–10 s sim after spawn
```

or manually:

```bash
ros2 bag record -s mcap -o /data/huitzilin_bags/week3_<ID> \
  /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
BAG_PID=$!
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=<ID> -p speed_mps:=… -p approach_angle_deg:=… \
  -p miss_distance_m:=… -p offset_forward_m:=…      # skip for N01/N02/N05
# wait time_to_closest_s + 4.0 s sim time, then:
kill -SIGINT $BAG_PID
```

Special cases:
- **N02** — no spawn; time the window to span a patrol waypoint turn.
- **N04** — needs ~10 m *vertical* miss; `spawn_projectile.py` only does lateral.
  Set `takeoff_alt_m: 12.0` in `bridge.yaml` (restore 2.0 after), or spawn manually
  via `gz service … EntityFactory` with a low Z. Don't silently mislabel it.
- **N05** — 60 s of clean patrol, no spawn.

After **every** bag, write `/data/huitzilin_bags/week3_<ID>.label.yaml` (fields map
1:1 from the matrix row):

```yaml
scenario_id: S02
label: positive            # positive | negative
closest_approach_m: 0.0
time_to_closest_s: 0.75
detection_window_s: 4.0    # widen for slow scenarios if needed
bag_start_sim_t: 0.0       # first /clock value in the bag (ros2 bag info)
```

All 17 sidecars must exist — `score_bags.py` fails explicitly on missing bags, but a
missing *sidecar* silently drops the scenario from the recall math.

## Tuning + regression

```bash
./scripts/run_regression.sh /data/huitzilin_bags train   # tune against this
./scripts/run_regression.sh /data/huitzilin_bags test    # score once; never tune on it
```

Knobs in `src/huitzilin_perception/params/detector.yaml`:

| Symptom | Knob |
|---|---|
| Missing fast/oblique positives | lower `diff_threshold_m`, widen `roi_half_angle_deg` |
| Missing wide-miss positives | raise `roi_max_range_m`, lower `cluster_min_points` |
| False triggers on patrol turns | confirm egomotion compensation active, raise `diff_threshold_m` slightly |
| False triggers on clutter | raise `min_publish_score` or `cluster_min_points` |

Before touching thresholds for S08 specifically: run a single-bag
`debug_funnel:=true` trace first (see `docs/JOURNAL.md` Week 3 open items).
Commit `detector.yaml` changes with a message naming the FN/FP they fixed.
Evidence docs mirror `docs/week2_patrol_evidence.md`: run summary, metrics table,
interpretation, source/date line.
