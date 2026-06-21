# CLAUDE.md

Guidance for Claude working in this repo. Non-obvious facts only — derivable things (architecture,
roadmap, BOM, topic tables) live in `docs/` and `HuitzilinReflex_v2.md`.

## What this is

3.5″ ducted micro-quadrotor that patrols, signals, and reflexively dodges projectiles.
Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink · Python 3.12 · Ubuntu 24.04.
**Current phase: Week 3 (perception) — scaffolded, detection node not yet written.** Week 2 (autonomous patrol loop) is complete.

## Build & run (inside WSL or native Ubuntu)

```bash
source /opt/ros/jazzy/setup.bash
cd ~/huitzilin_ws && colcon build --symlink-install && source install/setup.bash

# 3 terminals:
gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf

sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

ros2 launch huitzilin_sim week2_sitl.launch.py
```

After takeoff: call `/huitzilin/arm` → `/huitzilin/takeoff` → `/huitzilin/start_patrol`.
Preflight check: `./scripts/preflight_check.sh`

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
- **Judge all timing gates in sim time**, never wall-clock. Gazebo runs at ~24% real-time under WSL2 (no GPU passthrough); wall-clock numbers look ~4× off. Use `/clock` / message stamps. The Dell Inspiron (native Ubuntu) gets better real-time but still no discrete GPU — keep measuring in sim time there too.
- **`mavlink_bridge` (Week 1 package) is superseded** by `huitzilin_sim`. Do not extend it.
- **`ECC/` at repo root** is an unrelated plugin marketplace (untracked). Ignore it.

## Key docs

| Doc | Contents |
|---|---|
| `HuitzilinReflex_v2.md` | Master doc: objectives, full BOM, perception design, 9-week roadmap |
| `docs/architecture.md` | Node graph + full message/service contract table |
| `docs/frames.md` | Coordinate frames + TF tree |
| `docs/state_machine.md` | Full state/transition table |
| `docs/SAFETY_CASE.md` | FMEA, geofence/RTL, kill-switch |
| `docs/SETUP.md` | Full install + run/acceptance |
| `docs/JOURNAL.md` | Week-by-week log — best source for "why is it this way" |
| `HuitzilinReflex_Week3_Playbook.docx` | Week 3 perception task cards (W3-01…W3-22) |
| `docs/WEEK3_PLAN.md` | Week 3 reconciled plan — playbook vs. actual repo state |
