# CLAUDE.md

Non-obvious facts only. Architecture, roadmap, and BOM live in `docs/` and
`HuitzilinReflex_v2.md`. This file owns: bring-up commands, sharp edges, measured nulls.

## What this is

3.5″ ducted micro-quadrotor that patrols, signals, and reflexively dodges projectiles.
Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink · Python 3.12 · Ubuntu 24.04.

**Current phase: Week 5** (hardware bring-up — FC swap; `docs/hardware_bringup.md`).
The Week 5 **software** lane is complete (supervisor, payload, clock guard, hardware
config overlays, hardware preflight); everything still open is gated on the FC swap.
Weeks 5-9 are planned as parallel hardware/software lanes in `docs/weeks_5_9_plan.md`;
the software lane is the larger half and is not blocked by the FC swap.

Weeks 1–4 closed: patrol loop (Wk2), detection pipeline scored against a labeled bag
library (Wk3), Kalman filter + dodge trigger (Wk4). Week 4's result is a **capability
envelope, not a success rate** — always report it split by ball speed, never blended:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

95 scored throws over five batteries. The counts do **not** divide by
`week4_battery.yaml` as it ships today (`repeats: 3` × 5 scenarios × 5 batteries = 75,
not 78): battery composition drifted across the five runs. These are measured results —
do not "correct" them to match the yaml, and do not re-derive the yaml from them.
Latency mean 95–115 ms/battery against a 150 ms
budget; ~25% of individual dodges exceed it (max 282 ms) but this costs no outcomes,
because tca at commit is 0.18–0.29 s — latency is not the binding term.

**14 m/s is the edge of the reaction envelope, not a defect.** The ball crosses the ~3 m
detection range in 0.22 s, while confirming three track updates at ~14.5 Hz costs 0.21 s
plus 0.08 s pipeline. Clearing the 0.30 m hit radius in the remaining ~0.08 s needs
3.6 m/s of *instantaneous* escape. The bound is **sensing** — detection range and frame
rate — so Week 6's real-OAK-D bring-up is the decisive measurement.

## Build & run (WSL or native Ubuntu)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/huitzilin_ws && colcon build --symlink-install && source install/setup.bash

# 3 terminals:
gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

ros2 launch huitzilin_sim week2_sitl.launch.py
```

After takeoff — service types matter, all three exactly as below:
```bash
ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger
ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}'
```

Optional Week 5 nodes (neither runs by default):
```bash
# The supervisor needs perception: it watches /oak/points, which week2_sitl
# does not publish. Started from a bare week2_sitl it sits in permanent
# SENSOR_DROPOUT. Use a Week 3/4 entry point — both forward the argument:
ros2 launch huitzilin_perception week4_evasion.launch.py \
  with_patrol:=true with_supervisor:=true

ros2 run huitzilin_perception payload --ros-args   --params-file src/huitzilin_perception/params/payload.yaml
```
The supervisor logs which watches are armed at startup
(`watching: odom(1.0s) patrol_state(2.0s) cloud(1.0s) | disabled: cmd_vel`).
A timeout of `0.0` in `supervisor.yaml` disables that watch; `cmd_vel` ships
disabled because position-mode patrol publishes no ROS setpoint stream to
watch. See the sharp edge below.

Unit tests: `./scripts/run_tests.sh` (both packages; forwards pytest args).
`.github/workflows/tests.yml` runs the ROS-free subset on every push.
Preflight: `./scripts/preflight_check.sh` (SITL) · `./scripts/preflight_hw.sh` (hardware;
warns rather than fails, always exits 0).
Perception stack (depth world + detector, Dell only): `docs/bag_capture_runbook.md`.
Dodge batteries: `docs/dodge_battery_runbook.md`.

## Frame convention (critical)

- ArduPilot/MAVLink **NED**; all ROS 2 topics **ENU** (REP-103); body commands **FLU**.
  +2 m altitude = `z = -2.0`.
- Conversion lives in **one place only**: `MavBridge.ned_to_enu` / `enu_to_ned` in
  `mav_bridge.py`. No other node invents its own.
- Velocity setpoints → `MAV_FRAME_BODY_OFFSET_NED`. Position → `MAV_FRAME_LOCAL_NED`.
- Mirrored RViz markers = bridge conversion bug, not the marker code.

Full frame table and TF tree: `docs/frames.md`.

## Sharp edges (read before touching SITL)

- **`FRAME_CLASS=0` = silent no-lift.** Fresh EEPROM arms and accepts takeoff but throttle maxes with zero lift (`PreArm: Motors: Check frame class and type`). Always load `sitl_frame.parm` via `--add-param-file` (`FRAME_CLASS=1`, `FRAME_TYPE=1`). Never `ARMING_CHECK 0` — it hides the message. By hand: `param set FRAME_CLASS 1`, `FRAME_TYPE 1`, then `reboot`.
- **Port mismatch = `TimeoutError: no heartbeat`.** `bridge.yaml` listens on `:14552`, `patrol.yaml` on `:14553`; `sim_vehicle.py --out` must fan out to both. `first_flight.py` connects on `:14551`; MAVProxy's default `:14550` is for QGC only.
- **Patrol autostart is `false` in `patrol.yaml` intentionally.** `patrol_node.py` defaults `autostart=True`; the yaml overrides it. Autostarting floods GUIDED with position setpoints during takeoff and the drone never leaves the ground. Start via `/huitzilin/start_patrol` *after* takeoff.
- **Don't blind force-arm** (`param2=21196`). Fix the root cause (frame/EKF).
- **Never `Ctrl-Z` a launch.** A suspended job holds the SITL TCP socket. Restart Gazebo+SITL together if the FDM link goes half-broken.
- **Inline comments inside `.parm` files break MAVProxy.** Use comment-only lines.
- **Judge all timing gates in sim time**, never wall-clock. Gazebo runs ~24% real-time under WSL2 (no GPU passthrough); the Dell (native Ubuntu, no discrete GPU) drops to ~0.33 RTF under depth rendering. Use `/clock` / message stamps.
- **`max_step_size` must stay `0.001`** (1000 Hz) — `0.004` causes a SITL "Main loop slow" PreArm failure.
- **Depth rendering only works on the native-Ubuntu Dell box.** WSL2/Iris Xe cannot render Gazebo depth at rate; SITL/flight logic runs on either box.
- **`iris_depth` must carry the flight plugins.** It merge-includes the *bare* `iris_with_standoffs` (no flight plugins). Symptom if missing: SITL spams `No JSON sensor message received`, `link 1 down`, nothing on `:9002`, no lift — while Gazebo steps fine and `/oak/points` streams. Fix (79c2e9b): `ArduPilotPlugin` (fdm 127.0.0.1:9002) + `LiftDrag`×8 + `ApplyJointForce`×4 + `JointStatePublisher` ported into `iris_depth/model.sdf` with the `iris_with_standoffs::` prefix **stripped** (merge flattens to top level).
- **Depth camera runs at 15 Hz, not 30.** `ros_gz_bridge` PointCloudPacked→PointCloud2 can't sustain 30 Hz / 640×480 on the Dell. 640×480 is kept to match the real OAK-D Lite; 15 Hz is the standard for the bag library and tuning.
- **Bags recorded before `b0eedd5` lack attitude in `/huitzilin/odom`** — the detector falls back to camera-frame differencing on them. Never score against pre-`b0eedd5` bags.
- **The bag library is saturated** (recall 100%). It cannot referee threshold changes — a change that helps or hurts will read as no-change.
- **`debug_dump_dir` costs ~40 ms/frame.** No latency number from a dumping run is valid.
- **Never derive velocity from `/gz/dynamic_poses` arrival times** — arrival is not emission; use the pose stamps.
- **Never edit a shipped yaml for a diagnostic.** `--symlink-install` makes the installed copy a symlink into `src/`, so you are editing the real config. Copy to `/tmp` and `sed` that.
- **`use_sim_time` is no longer in any yaml.** It is a launch argument only. A node started with `use_sim_time:=true` and no `/clock` now logs FATAL and exits 1 after a 5 s grace window instead of silently freezing at t=0 (`huitzilin_sim/clock_guard.py`). `use_sim_time:=false` with a `/clock` present only warns — that is the Week 7 HITL shape. `test_hw_config.py` fails if anyone re-adds the key to a params file.
- **`run_regression.sh`'s clock warm-up is load-bearing — do not delete it.** It plays a bag with `--topics /clock --loop` *before* starting the detector, then kills it once the guard passes. Without it the harness has no `/clock` until `score_bags` reaches its first bag, well past the guard's 5 s grace, so the detector dies at startup and every bag replays into a corpse: `TP=0`, **recall 0.0%** — a dead node, never a detection regression, and never a reason to touch `detector.yaml`. It works because the guard is *one-shot* (`install_clock_guard` destroys its own timer on the first non-WAIT verdict), so a temporary clock suffices. `--topics /clock` keeps `/oak/points` and `/huitzilin/odom` silent, so the persistent background map is never seeded with out-of-scenario frames. Broken this way from `ae02696` until the warm-up landed.
- **`set -u` breaks `/opt/ros/jazzy/setup.bash`.** Source ROS first, then enable it (`run_regression.sh`), or leave it off (`preflight_hw.sh`). A script that sources ROS under `-u` exits at that line.
- **Hardware config lives in `hw_*` files, never as edits.** `params/hw_bridge.yaml`, `hw_frame.parm`, `hw_detector.yaml`, `hw_evasion.yaml` are overlays; the sim files stay the regression path. `test_hw_config.py` asserts each overlay's node name and keys exist in the file it overlays — in ROS 2 a mistyped node name silently loads *nothing*.
- **`RTL_ALT` is centimetres; `FENCE_ALT_MAX` is metres.** `hw_frame.parm` pins `RTL_ALT 400` (4 m) under the 5 m ceiling, because ArduPilot's 15 m default would make a fence-breach RTL climb through the fence it is answering.
- **`supervisor_node` reports no faults while disarmed.** Half the watched topics are legitimately silent on the bench. Faults are gated on `armed`, and no fault path can reach EVADE — that is asserted over all states × faults × `armed`, not by inspection. (EVADE *is* reachable disarmed via the threat edge, but commands nothing; pinned by `test_disarmed_threat_reaches_evade_but_commands_nothing`.)
- **A supervisor watch on a topic nothing publishes is a permanent fault.** `_age` reads a never-seen topic as infinitely stale — correctly, since that is how a publisher which died before its first message is caught — so an *unpublished* topic is indistinguishable from a dead one. A timeout of `0.0` in `supervisor.yaml` is the only way to say "this configuration does not produce that topic". `cmd_vel_timeout_s` ships `0.0`: `patrol.yaml` runs `mode: "position"`, where `patrol_node` sends setpoints over its own MAVLink connection and never creates the `/huitzilin/cmd_vel` publisher. Watching it drove `SETPOINT_STALL → FAILSAFE → LOITER → RTL_LAND` one second after every arm. Set it to `1.0` only alongside `mode: "velocity"`.
- **`ECC/` at repo root** is an unrelated plugin marketplace (untracked). Ignore it.

## Measured nulls — do not re-run

Each was measured over a full battery; the baseline was best on every column. Re-running
these is the most common way to lose a session here.

- `roi_max_range_m` 5→8 · `min_track_updates` 3→2 · `cluster_min_points` 5→3
- frame rate 320×240 @ 30 Hz · `dodge_speed_mps` 1.5→4.0
- the multi-hypothesis tracker (it works correctly — it just bought zero tca)
- the command path (probe returned ok 16/16)
- vertical escape (the dodge is already mostly vertical; r = −0.077 with min_dist)

Also refuted as explanations: "sparse ball detections", "oblique aim geometry bug",
cloud mis-registration, and "2 m/s² is a hard limit" (the airframe does ~3 m/s²).

**Never cite the PSC_ACC_XY / WPNAV_ACCEL / ANGLE_MAX experiment** — it is invalid.
`--defaults` does not override `eeprom.bin`, and those parameter names do not exist in
this build (it is `ATC_ANGLE_MAX`).

Why range levers keep failing: tca is bounded by *sensing*, not by thresholds or by
dodge authority. Only a real camera with more range or frame rate moves it.

## Key docs

| Doc | Contents |
|---|---|
| `HuitzilinReflex_v2.md` | Master doc: objectives, BOM, 9-week roadmap |
| `docs/architecture.md` | Node graph + message/service contracts |
| `docs/frames.md` | Coordinate frames + TF tree |
| `docs/state_machine.md` | State/transition table |
| `docs/requirements.md` | REQ-01…REQ-16 + non-goals |
| `docs/SAFETY_CASE.md` | FMEA, geofence/RTL, kill-switch, safety/legal rules |
| `docs/SETUP.md` | Install from scratch |
| `docs/weeks_5_9_plan.md` | Weeks 5-9 plan, split into hardware and software lanes with cross-lane gates |
| `docs/hardware_bringup.md` | Weeks 5–6 physical checklist: FC swap, radio bind, Pi power, payload wiring |
| `docs/bag_capture_runbook.md` | Bag re-capture + regression/tuning procedure (Dell) |
| `docs/dodge_battery_runbook.md` | Dodge battery + sweep procedure (Dell) |

Development history is in git, not in the tree: `git log -p docs/`.
