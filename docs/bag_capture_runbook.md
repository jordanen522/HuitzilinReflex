# Bag capture & regression runbook (Dell)

Owns: how to (re)capture the labeled scenario library and run the detector regression.
Native-Ubuntu Dell only. All timing in **sim time** (`/clock`, message stamps).

## Bring-up

4 terminals, each sourced (`source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash`):

```bash
cd ~/huitzilin_ws && colcon build --symlink-install   # once per checkout

# T1 — perception world (exports GZ_SIM_RESOURCE_PATH itself)
./scripts/week3_world.sh
# T2 — SITL: the standard fan-out, see CLAUDE.md
# T3 — perception stack
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
# T4 — arm → takeoff → patrol (odom-polled climb)
./scripts/week3_flyup.sh
```

Sanity gate before recording: `/oak/points` stable at 15 Hz sim (66 ms stamp spacing, no
dropouts) at rest and in flight, and the detector logs
`differencing frame -> fixed (odom TF available)`. A fallback to camera-frame mode means
the odom attitude is broken — fix that before capturing anything.

## Capture loop — one bag per scenario

Scenario IDs and parameters come from `src/huitzilin_perception/config/scenario_matrix.yaml`
(positives S01–S12, negatives N01–N05; **S11/S12/N05 are the held-out test split**).

```bash
./scripts/capture_scenario.sh <ID>   # Ctrl-C ~8–10 s sim after spawn
```

Manual equivalent:

```bash
ros2 bag record -s mcap -o /data/huitzilin_bags/week3_<ID> \
  /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
BAG_PID=$!
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=<ID> -p speed_mps:=… -p approach_angle_deg:=… \
  -p miss_distance_m:=… -p offset_forward_m:=…      # skip for N01/N02/N05
# wait time_to_closest_s + 4.0 s sim, then:
kill -SIGINT $BAG_PID
```

Special cases:
- **N02** — no spawn; time the window to span a patrol waypoint turn.
- **N04** — needs ~10 m *vertical* miss. `spawn_projectile.py` has
  `offset_vertical_m` (the matrix row sets `-10.0`) and `capture_scenario.sh` passes it
  automatically — no manual `gz service … EntityFactory` spawn, and no editing the
  parameter out. What you *do* still have to do by hand is fly high enough: set
  `takeoff_alt_m: 12.0` in `bridge.yaml` and restore 2.0 afterwards, or the spawn lands
  underground and `capture_scenario.sh` aborts.
- **N05** — 60 s of clean patrol, no spawn.

Every bag needs a `/data/huitzilin_bags/week3_<ID>.label.yaml` sidecar.
**`capture_scenario.sh` writes it for you** — hand-write one only when you captured
the bag manually with the block above. Fields map 1:1 from the matrix row:

```yaml
scenario_id: S02
label: positive            # positive | negative
closest_approach_m: 0.0
time_to_closest_s: 0.75
detection_window_s: 4.0    # widen for slow scenarios
```

There is no `bag_start_sim_t` field. Nothing writes one and nothing reads one; a
sidecar carrying it is not more complete, just inconsistent with the tooling.

All 17 sidecars must exist. `score_bags.py` fails explicitly on a missing *bag*, but a
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

The current operating point is tuned and the held-out test split passes at recall 100%.
**The library is saturated** — it cannot referee further threshold changes, and every
trigger-side lever has already been measured as a null (see `CLAUDE.md`). Commit
`detector.yaml` changes with a message naming the FN/FP they fixed.

## Open detector items

- **S08 false negative** (14 m/s near-miss, train split) — never root-caused. Two blind
  threshold attempts regressed other scenarios without fixing it. Any re-tune must start
  from a single-bag `debug_funnel:=true` trace on S08, not threshold guessing.
- **N02 / N03 / N05 false positives** (patrol-turn and background-clutter triggers) —
  they hurt precision but don't fail the recall gate. Uninvestigated.
