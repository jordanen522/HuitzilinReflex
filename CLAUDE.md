# CLAUDE.md

Guidance for Claude working in this repo. Non-obvious facts only — derivable things
(architecture, roadmap, BOM, topic tables) live in `docs/` and `HuitzilinReflex_v2.md`.

## What this is

3.5″ ducted micro-quadrotor that patrols, signals, and reflexively dodges projectiles.
Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink · Python 3.12 · Ubuntu 24.04.
**Current phase: Week 4 (Kalman filter + dodge trigger).** Weeks 1–3 complete: autonomous
patrol loop (Wk2) and detection pipeline scored against a labeled bag library (Wk3,
closed 2026-07-15 — see `docs/JOURNAL.md`).

## Build & run (inside WSL or native Ubuntu)

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

After takeoff — service types matter, all three use SetBool or Trigger exactly as below:
```bash
ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger
ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}'
```
Preflight check: `./scripts/preflight_check.sh`
Perception stack (depth world + detector, Dell box only): `docs/week3_capture_runbook.md`.

## Frame convention (critical)

- ArduPilot/MAVLink: **NED**. +2 m altitude = `z = -2.0`.
- All ROS 2 topics: **ENU** (REP-103). Body commands: **FLU**.
- Conversion lives in **one place only**: `MavBridge.ned_to_enu` / `enu_to_ned` in `mav_bridge.py`.
  No other node invents its own conversion.
- Velocity setpoints → `MAV_FRAME_BODY_OFFSET_NED`. Position setpoints → `MAV_FRAME_LOCAL_NED`.
- Mirrored RViz markers = bridge conversion bug, not the marker code.

## Sharp edges (read before touching SITL)

- **`FRAME_CLASS=0` = silent no-lift.** Fresh EEPROM arms and accepts takeoff but throttle maxes with zero lift (`PreArm: Motors: Check frame class and type`). Fix: always load `sitl_frame.parm` via `--add-param-file` (`FRAME_CLASS=1`, `FRAME_TYPE=1`). Never use `ARMING_CHECK 0` — that hides the message. If setting by hand: `param set FRAME_CLASS 1`, `FRAME_TYPE 1`, then `reboot`.
- **Port mismatch = `TimeoutError: no heartbeat`.** `bridge.yaml` listens on `:14552`, `patrol.yaml` on `:14553`; `sim_vehicle.py --out` must fan out to both. MAVProxy's own default `:14550` is for `first_flight.py` / QGC only.
- **Patrol autostart is `false` in `patrol.yaml` intentionally.** `patrol_node.py` defaults `autostart=True` but the yaml overrides it — autostarting floods GUIDED with position setpoints during takeoff and the drone never leaves the ground. Always start via `/huitzilin/start_patrol` *after* takeoff.
- **Don't blind force-arm** (`param2=21196`). Fix the root cause (frame/EKF) instead.
- **Never `Ctrl-Z` a launch.** A suspended job holds the SITL TCP socket. Restart Gazebo+SITL together if the FDM link goes half-broken.
- **Inline comments inside `.parm` files break MAVProxy.** Use comment-only lines.
- **Judge all timing gates in sim time**, never wall-clock. Gazebo runs at ~24% real-time under WSL2 (no GPU passthrough); the Dell (native Ubuntu, no discrete GPU) drops to ~0.33 RTF under depth rendering. Use `/clock` / message stamps.
- **`max_step_size` must stay `0.001`** (1000 Hz) — `0.004` causes a SITL "Main loop slow" PreArm failure.
- **Depth rendering only works on the native-Ubuntu Dell box.** WSL2/Iris Xe cannot render Gazebo depth at rate; SITL/flight logic runs on either box.
- **`iris_depth` must carry the flight plugins.** It merge-includes the *bare* `iris_with_standoffs` (no flight plugins). Symptom if missing: SITL spams `No JSON sensor message received`, `link 1 down`, nothing on `:9002`, no lift — while Gazebo steps fine and `/oak/points` streams. Fix (79c2e9b): `ArduPilotPlugin` (fdm 127.0.0.1:9002) + `LiftDrag`×8 + `ApplyJointForce`×4 + `JointStatePublisher` ported into `iris_depth/model.sdf` with the `iris_with_standoffs::` prefix **stripped** (merge flattens to top level).
- **Depth camera runs at 15 Hz, not 30.** `ros_gz_bridge` PointCloudPacked→PointCloud2 can't sustain 30 Hz / 640×480 on the Dell. `iris_depth` `update_rate=15` (640×480 kept to match the real OAK-D Lite); 15 Hz is the standard for the bag library + tuning.
- **Bags recorded before b0eedd5 lack attitude in `/huitzilin/odom`** — the detector falls back to camera-frame differencing on them. Never score against pre-b0eedd5 bags.
- **`src/mavlink_bridge` (Week 1) is superseded** by `huitzilin_sim` and slated for deletion. Do not extend it.
- **`ECC/` at repo root** is an unrelated plugin marketplace (untracked). Ignore it.

## Key docs

| Doc | Contents |
|---|---|
| `HuitzilinReflex_v2.md` | Master doc: objectives, BOM, perception design, 9-week roadmap |
| `docs/architecture.md` | Node graph + message/service contract table |
| `docs/frames.md` | Coordinate frames + TF tree |
| `docs/state_machine.md` | State/transition table |
| `docs/SAFETY_CASE.md` | FMEA, geofence/RTL, kill-switch |
| `docs/SETUP.md` | Install + run/acceptance |
| `docs/JOURNAL.md` | Compacted week log — results, open items, what Week 4 inherits |
| `docs/week3_capture_runbook.md` | Bag re-capture + regression/tuning procedure (Dell) |
