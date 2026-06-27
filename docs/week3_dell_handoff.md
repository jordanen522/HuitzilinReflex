# Week 3 Dell-box handoff — HuitzilinReflex perception capture

Paste this whole file to the agent/session that will run on the native Ubuntu Dell box.
It's self-contained: you don't need any other context from the planning conversation that
produced it.

## What this is

3.5″ ducted micro-quadrotor project. Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** ·
ArduPilot Copter 4.5+ **SITL** · pymavlink · Python 3.12 · Ubuntu 24.04. Repo:
`~/huitzilin_ws/src/` (assume already cloned — if not, clone the project repo here first
and `git pull` to get the latest `docs/` and `src/huitzilin_perception/` changes).

**Goal of this session:** execute Week 3's remaining perception tasks — all of them require
a live Gazebo depth render, which only works at usable rate on this machine (native GPU),
not on the Windows/WSL2 dev box where the code was written.

**IMPORTANT — check this first:** the commands below assume `docs/week3_capture_runbook.md`,
`docs/architecture.md`, and `docs/WEEK3_PLAN.md` are up to date on this machine. They were
edited but **not yet committed/pushed** as of this handoff. If `git log` doesn't show those
doc updates, either `git pull` after the user pushes them, or just follow this handoff doc
directly — it repeats everything needed.

## Sharp edges (do not skip — these have bitten before)

- **`FRAME_CLASS=0` = silent no-lift.** Always load `sitl_frame.parm` via
  `--add-param-file` when running `sim_vehicle.py`. Never `ARMING_CHECK 0`.
- **Port mismatch = `TimeoutError: no heartbeat`.** `sim_vehicle.py --out` must fan out to
  both `:14552` and `:14553`.
- **Patrol autostart is `false` by design** — always start via `/huitzilin/start_patrol`
  *after* takeoff, never rely on node default.
- **Never `Ctrl-Z` a launch** — it holds the SITL TCP socket; restart Gazebo+SITL together
  if the link goes half-broken.
- **All timing/scoring uses sim time** (`/clock`, message stamps) — never wall-clock. This
  machine has a real GPU so it should run close to real-time, but verify, don't assume.
- Depth rendering **must** happen on this box (native Ubuntu + GPU). If you're not on the
  native Dell Inspiron 3670 (i5-8400/UHD630) or equivalent native-GPU Linux box, stop —
  this won't work over WSL2/Iris Xe.

## Sequence

### 0. Build + baseline sanity (W3-01)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/huitzilin_ws
colcon build --symlink-install
source install/setup.bash

# 3 terminals — confirm a clean Week-2 patrol lap BEFORE touching perception:
gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

ros2 launch huitzilin_sim week2_sitl.launch.py
```

Call `/huitzilin/arm` → `/huitzilin/takeoff` → `/huitzilin/start_patrol`. Confirm one clean
lap (~29.5 s mean, per `docs/week2_patrol_evidence.md`) before proceeding.

### 1. Depth stream verify (W3-04 verify / W3-07)

```bash
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/models:$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/worlds:$GZ_SIM_RESOURCE_PATH"

gz sim -s -r "$(ros2 pkg prefix huitzilin_perception)/share/huitzilin_perception/worlds/huitzilin_runway.sdf"

# new terminal:
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
```

In RViz, add `PointCloud2` on `/oak/points` (best-effort QoS). Arm/takeoff/start_patrol,
fly a lap, confirm the depth stream is stable (no dropouts/frozen frames) for the whole
lap. **Go/no-go gate** — don't record bags on an unstable stream.

### 2. Capture 17 labeled scenarios (W3-10)

```bash
mkdir -p /data/huitzilin_bags
```

For each scenario ID below: start a bag, trigger the scenario, stop the bag, then write a
label sidecar. Pattern:

```bash
ros2 bag record -s mcap -o /data/huitzilin_bags/week3_<ID> \
  /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
BAG_PID=$!

ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=<ID> -p speed_mps:=<speed> -p approach_angle_deg:=<angle> \
  -p miss_distance_m:=<miss> -p offset_forward_m:=<offset>

# wait time_to_closest_s + 4.0s sim time, then:
kill -SIGINT $BAG_PID
```

Scenario params (id → speed_mps, approach_angle_deg, miss_distance_m, offset_forward_m,
spawn?, note) — copied from `src/huitzilin_perception/config/scenario_matrix.yaml`:

| ID | speed | angle | miss | offset | spawn? | note |
|---|---|---|---|---|---|---|
| S01 | 4.0 | 0.0 | 0.0 | 6.0 | yes | baseline slow direct-hit |
| S02 | 8.0 | 0.0 | 0.0 | 6.0 | yes | primary design speed |
| S03 | 14.0 | 0.0 | 0.0 | 6.0 | yes | fast |
| S04 | 8.0 | 30.0 | 0.0 | 6.0 | yes | oblique right |
| S05 | 8.0 | -30.0 | 0.0 | 6.0 | yes | oblique left |
| S06 | 14.0 | 30.0 | 0.0 | 6.0 | yes | fast oblique, worst case |
| S07 | 8.0 | 0.0 | 0.5 | 6.0 | yes | near-miss, must trigger |
| S08 | 14.0 | 0.0 | 0.5 | 6.0 | yes | fast near-miss |
| S09 | 8.0 | 0.0 | 1.5 | 6.0 | yes | wide miss, must trigger |
| S10 | 4.0 | -20.0 | 1.2 | 5.0 | yes | slow+oblique+wide |
| S11 | 8.0 | 5.0 | 0.1 | 7.0 | yes | **held-out test** |
| S12 | 12.0 | -25.0 | 0.4 | 6.0 | yes | **held-out test** |
| N01 | — | — | — | — | no | clean negative |
| N02 | — | — | — | — | no | spawn nothing; fly a patrol turn during the bag |
| N03 | 8.0 | 180.0 | 0.0 | -6.0 | yes | spawns behind drone, outside FOV |
| N04 | 8.0 | 0.0 | 10.0 | 6.0 | yes | needs vertical offset — see note below |
| N05 | — | — | — | — | no | **held-out test** — 60s clean patrol |

- **N04 gap:** `spawn_projectile.py` only supports lateral `miss_distance_m`, not vertical
  offset. For N04 you need ~10 m of *vertical* miss. Either spawn manually via
  `gz service -s /world/huitzilin_runway/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --req '...'`
  with a manually computed low Z, or patch a throwaway copy of `spawn_projectile.py` to
  offset Z instead of lateral Y for this one case. Don't silently mislabel it if you skip it.
- **N02:** trigger a patrol waypoint transition (not a projectile spawn) during the bag.

After each bag, write `/data/huitzilin_bags/week3_<ID>.label.yaml`:

```yaml
scenario_id: S02            # from matrix: id
label: positive               # from matrix: label
closest_approach_m: 0.0       # from matrix: closest_approach_m
time_to_closest_s: 0.75       # from matrix: time_to_closest_s
detection_window_s: 4.0       # default; widen for slow scenarios if needed
bag_start_sim_t: 0.0          # first /clock value in the bag — `ros2 bag info <bag>` or
                               # capture `ros2 topic echo --once /clock` at record start
```

Get `closest_approach_m`/`time_to_closest_s`/`label` for every ID from
`src/huitzilin_perception/config/scenario_matrix.yaml` directly — values above are just
the S02 example. **All 17 sidecars must exist** or `score_bags.py` silently drops the
missing scenario from the recall math.

### 3. Tune thresholds (W3-15)

```bash
./scripts/run_regression.sh /data/huitzilin_bags train
```

Adjust `src/huitzilin_perception/params/detector.yaml`:

| Symptom | Knob |
|---|---|
| Missing fast/oblique positives (S03,S06,S08) | lower `diff_threshold_m`, widen `roi_half_angle_deg` |
| Missing wide-miss positives (S09,S10) | raise `roi_max_range_m`, lower `cluster_min_points` |
| False triggers on patrol turns (N02) | confirm `compensate_egomotion: true`, raise `diff_threshold_m` slightly |
| False triggers on clutter | raise `min_publish_score` or `cluster_min_points` |

Iterate until **train-split recall ≥ 95%** with an FP rate you can defend. Commit
`detector.yaml` with a message describing which threshold moved and why. **Never tune
against the test split** (S11, S12, N05).

### 4. Score held-out set (W3-17)

```bash
./scripts/run_regression.sh /data/huitzilin_bags test
```

Copy the printed table into a new `docs/week3_detection_evidence.md` (mirror the format of
the existing `docs/week2_patrol_evidence.md`: run summary, metrics table, short
interpretation, source/date line).

### 5. Confirm CI gate (W3-18)

Confirm `run_regression.sh ... test` exits 0. Update `docs/WEEK3_PLAN.md` to mark W3-18
fully done.

### 6. Acceptance run (W3-21)

Fresh shell, full re-source, `ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true`,
fly + trigger a couple of scenarios live, confirm `/threat/centroid` fires with an RViz
marker. Record an `ros2 bag record` + RViz screenshot as evidence.

### 7. Retro (W3-22)

Append an entry to `docs/JOURNAL.md` (match the existing "Week 3 — 2026-06-21" entry
style): date, final recall/FP numbers, anything deferred (e.g. N04 gap if unresolved), and
what Week 4 (Kalman filter + dodge trigger) inherits (`/threat/centroid` contract, tuned
`detector.yaml`, `/data/huitzilin_bags` library location).

## When done, report back

- Final train/test recall, precision, FP rate numbers.
- Whether N04 was captured properly or deferred.
- Any threshold changes made to `detector.yaml` and why.
- Whether `run_regression.sh ... test` exits 0 (CI-gate ready).
- Any new sharp edges discovered (GPU quirks, timing surprises) — these belong in
  `CLAUDE.md`.

## Reference (full detail, same content as this handoff plus rationale)

`docs/week3_capture_runbook.md`, `docs/architecture.md` (contracts), `docs/WEEK3_PLAN.md`
(task tracker), `docs/JOURNAL.md` (history/rationale), `CLAUDE.md` (repo-wide sharp edges).
