# Week 3 — Capture & Tuning Runbook (Dell box)

Execution checklist for the items in `docs/WEEK3_PLAN.md` §4 "Remaining" that need a live
Gazebo depth stream: **W3-01, W3-04 verify, W3-07, W3-10, W3-15 (tune), W3-17, W3-18
(wire), W3-21, W3-22**. Run all of this on the native-Ubuntu Dell Inspiron — the WSL2/Iris
Xe box cannot render Gazebo depth at rate (`CLAUDE.md` sharp edges).

All timing in this runbook is **sim time** (via `/clock` and message stamps), never
wall-clock, per the same sharp-edge note.

---

## 0. Pre-flight (W3-01)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/huitzilin_ws
colcon build --symlink-install
source install/setup.bash

# Confirm a clean baseline lap before touching perception
ros2 launch huitzilin_sim week2_sitl.launch.py
# (3-terminal Gazebo + sim_vehicle.py bring-up per CLAUDE.md "Build & run")
```

Arm → takeoff → start_patrol, confirm one clean lap (mean ~29.5 s per
`docs/week2_patrol_evidence.md`). Don't proceed to perception until this is green —
a regression here means depth capture would be debugging two problems at once.

## 1. Depth stream verify (W3-04 verify / W3-07)

```bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/models:$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/worlds:$GZ_SIM_RESOURCE_PATH"

gz sim -s -r "$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/worlds/huitzilin_runway.sdf"

# new terminal
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
```

In RViz: add `PointCloud2` on `/oak/points` (QoS: best-effort, keep-last 1, fixed frame
`camera_optical_frame` or `base_link` with TF). Arm/takeoff/start_patrol, fly one lap,
confirm the point cloud is stable (no dropouts, no frozen frames) for the whole lap.
This is a go/no-go gate — if the stream is unstable, fix it before recording any bags.

## 2. Capture loop (W3-10)

Bag root: **`/data/huitzilin_bags`** (already the hardcoded default in `score_bags.py`,
`run_regression.sh`, and the launch file's `score` mode — use this path, don't invent a
new one).

```bash
mkdir -p /data/huitzilin_bags
```

For **each** of the 17 scenarios in
`src/huitzilin_perception/config/scenario_matrix.yaml`, repeat:

```bash
# 1. Start the bag
ros2 bag record -s mcap -o /data/huitzilin_bags/week3_<ID> \
  /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
BAG_PID=$!

# 2. Trigger the scenario (positives only — see param table below; skip for clean negatives)
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=<ID> \
  -p speed_mps:=<speed_mps> \
  -p approach_angle_deg:=<approach_angle_deg> \
  -p miss_distance_m:=<miss_distance_m> \
  -p offset_forward_m:=<offset_forward_m>

# 3. Let it run time_to_closest_s + 4.0s (sim time) past spawn, then stop
kill -SIGINT $BAG_PID
```

### 2a. Per-scenario param table (copy straight from `scenario_matrix.yaml`)

| ID | speed_mps | approach_angle_deg | miss_distance_m | offset_forward_m | Spawn? | Notes |
|---|---|---|---|---|---|---|
| S01 | 4.0 | 0.0 | 0.0 | 6.0 | yes | baseline slow direct-hit |
| S02 | 8.0 | 0.0 | 0.0 | 6.0 | yes | primary design-point speed |
| S03 | 14.0 | 0.0 | 0.0 | 6.0 | yes | fast, fewer frames to cluster |
| S04 | 8.0 | 30.0 | 0.0 | 6.0 | yes | oblique right |
| S05 | 8.0 | -30.0 | 0.0 | 6.0 | yes | oblique left |
| S06 | 14.0 | 30.0 | 0.0 | 6.0 | yes | worst-case oblique fast |
| S07 | 8.0 | 0.0 | 0.5 | 6.0 | yes | near-miss, must still trigger |
| S08 | 14.0 | 0.0 | 0.5 | 6.0 | yes | fast near-miss, hardest window |
| S09 | 8.0 | 0.0 | 1.5 | 6.0 | yes | wide miss, still must flag |
| S10 | 4.0 | -20.0 | 1.2 | 5.0 | yes | slow+oblique+wide edge case |
| S11 | 8.0 | 5.0 | 0.1 | 7.0 | yes | **held-out test** |
| S12 | 12.0 | -25.0 | 0.4 | 6.0 | yes | **held-out test** |
| N01 | — | — | — | — | no | clean negative, no spawn |
| N02 | — | — | — | — | no | spawn nothing; fly a patrol turn during the bag window instead |
| N03 | 8.0 | 180.0 | 0.0 | -6.0 | yes | spawns behind drone, outside FOV |
| N04 | 8.0 | 0.0 | 10.0 | 6.0 | yes | spawn ~10 m below glide path — see note below |
| N05 | — | — | — | — | no | **held-out test** — 60 s clean patrol segment |

- **N02**: no `spawn_projectile` call. Instead, trigger a patrol waypoint transition while
  the bag is recording, so egomotion has to be compensated.
- **N04**: `miss_distance_m` in the matrix is lateral; this scenario needs *vertical* miss
  instead. `spawn_projectile.py` doesn't currently support a vertical offset — for this one
  scenario, spawn manually 10 m below the drone's altitude using
  `gz service -s /world/huitzilin_runway/create ...` directly (same EntityFactory pattern as
  `spawn_projectile.py:139-156`), or temporarily edit the spawn altitude in a throwaway copy
  of the script. Flag this as a known gap if you don't have time to fix it properly — don't
  silently mislabel the scenario.
- **N05**: just record 60 s of normal patrol, no spawn, no manoeuvre.

### 2b. Label sidecar — write immediately after stopping each bag

`score_bags.py` (`score_bags.py:71-84`) requires
`/data/huitzilin_bags/week3_<ID>.label.yaml` for every scenario. Field mapping straight
from the matrix row (example below uses S02's actual matrix values):

```yaml
# /data/huitzilin_bags/week3_S02.label.yaml  (example — fill per scenario)
scenario_id: S02                        # matrix: id
label: positive                         # matrix: label  (positive | negative)
closest_approach_m: 0.0                 # matrix: closest_approach_m
time_to_closest_s: 0.75                 # matrix: time_to_closest_s
detection_window_s: 4.0                 # default; widen for slow scenarios (S01, S10) if needed
bag_start_sim_t: 0.0                    # sim seconds of the FIRST /clock message in the bag —
                                         # read it with: ros2 bag info /data/huitzilin_bags/week3_S02
                                         # or `ros2 topic echo --once /clock` captured at record start
```

Write all 17 sidecars before moving to tuning — `score_bags.py` skips any scenario whose
bag or sidecar is missing and silently drops it from the recall/precision math, which would
quietly inflate the score.

## 3. Threshold tuning (W3-15)

```bash
./scripts/run_regression.sh /data/huitzilin_bags train
```

Inspect the per-scenario table for FN (missed positives) and FP (false-triggered
negatives). Primary knobs in `src/huitzilin_perception/params/detector.yaml`:

| Symptom | Likely knob |
|---|---|
| Missing fast/oblique positives (S03, S06, S08) | lower `diff_threshold_m`, widen `roi_half_angle_deg` |
| Missing wide-miss positives (S09, S10) | raise `roi_max_range_m`, lower `cluster_min_points` |
| False triggers on patrol turns (N02) | confirm `compensate_egomotion: true`, raise `diff_threshold_m` slightly |
| False triggers on background clutter | raise `min_publish_score` or `cluster_min_points` |

Re-run `run_regression.sh ... train` after each change until train-split recall ≥ 95%
with an FP rate you're willing to defend. **Commit `detector.yaml` with a note on exactly
which thresholds moved and why** (W3-15's commit message should reference the FN/FP it
fixed) — do not tune against `test` split scenarios (S11, S12, N05).

## 4. Held-out scoring (W3-17)

```bash
./scripts/run_regression.sh /data/huitzilin_bags test
```

Copy the printed recall/precision/FP-rate table into a new `docs/week3_detection_evidence.md`,
mirroring `docs/week2_patrol_evidence.md`'s format: run summary, metrics table, brief
interpretation, source/date line.

## 5. Confirm regression gate (W3-18)

With the bag library now populated, `run_regression.sh` is the CI-ready regression gate.
Verify it exits 0 against `test`, and update `docs/WEEK3_PLAN.md` §3/§4 to mark W3-18 as
fully done (was `[~]` partial — code existed, library didn't).

## 6. Acceptance run (W3-21)

From a **fresh shell** (new terminal, re-source everything, no leftover env state):

```bash
source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
```

Arm → takeoff → start_patrol → spawn a couple of representative scenarios live, confirm
`/threat/centroid` fires in RViz with the marker. Record this as evidence: an
`ros2 bag record` of the run plus an RViz screenshot showing a detected centroid marker.

## 7. Retro + Week 4 handoff (W3-22)

Append to `docs/JOURNAL.md`, matching the style of the existing "Week 3 — 2026-06-21"
entry: capture-session date, final recall/FP numbers, any deferred items (e.g. N04's
vertical-offset gap if not fixed), and what Week 4 (Kalman filter + dodge trigger) inherits
from this pipeline (`/threat/centroid` contract, detector params, bag library location).
