# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

HuitzilinReflex is an autonomous-patrol / threat-evasion quadrotor project: a ROS 2 (Jazzy) stack
bridging to ArduPilot (SITL or real flight controller) via pymavlink, simulated in Gazebo Harmonic.
Current phase is **Week 2** — autonomous patrol loop flying in SITL. Perception (OAK-D depth),
evasion, and payload (LED/buzzer) are future work (Week 3–6); their topics/nodes exist only as
provisional contracts in the docs, not as code yet.

Development happens on Windows 11 + WSL2 (Ubuntu 24.04) with IntelliJ Remote Development connected
into WSL. All ROS/Gazebo/ArduPilot commands below run **inside WSL**, not native Windows.

The `ECC/` directory at the repo root is an unrelated, separately-cloned tool plugin
marketplace (untracked) — ignore it when working on this project.

## Build & run (inside WSL)

```bash
# one-time env (see docs/SETUP.md for full install of ROS2/ArduPilot/Gazebo/ros_gz/pymavlink)
source /opt/ros/jazzy/setup.bash

# build
cd ~/huitzilin_ws   # repo root = colcon workspace
colcon build --symlink-install
source install/setup.bash
```

Bringing up the full Week 2 stack requires 3 terminals/processes:

```bash
# 1) Gazebo headless
gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf

# 2) ArduPilot SITL — port fan-out MUST match bridge.yaml/patrol.yaml, and the
#    frame param file MUST be loaded or the quad arms but never produces lift
#    (see "Known sharp edges" below)
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14552 \
  --out udp:127.0.0.1:14553

# 3) Whole Week 2 ROS 2 stack (bridge + patrol + telemetry logger), or run nodes individually
ros2 launch huitzilin_sim week2_sitl.launch.py
# or:
ros2 run huitzilin_sim mav_bridge --ros-args -p connection:=udp:127.0.0.1:14552
```

Preflight sanity check before starting work: `./scripts/preflight_check.sh`
(verifies ROS 2 / Gazebo / ardupilot_gz workspace / pymavlink heartbeat).

Week 1 manual flight script (legacy, still works against MAVProxy's own `:14550` out port):
`python3 scripts/first_flight.py`

Telemetry CSV → ground-track plot: `python3 scripts/plot_telemetry.py`

There is no formal test suite for `huitzilin_sim`; `src/mavlink_bridge` (the retired Week 1
package) has the standard ROS 2 ament lint tests (`test_copyright.py`, `test_flake8.py`,
`test_pep257.py`) runnable via `colcon test`.

## Architecture

Two ROS 2 packages under `src/`:

- **`huitzilin_sim`** — the active stack (Week 2+). Contains:
  - `mav_bridge.py` — pure pymavlink class (`MavBridge`), no ROS dependency. Owns the single
    connection to ArduPilot, all arm/mode/takeoff/setpoint/telemetry calls, and the **only**
    NED↔ENU conversion helpers (`ned_to_enu`/`enu_to_ned`) in the whole codebase.
  - `mav_bridge_node.py` — ROS 2 wrapper around `MavBridge`. Subscribes `/huitzilin/cmd_vel`
    (FLU), publishes `/huitzilin/odom` (ENU) + `/huitzilin/state` (JSON), exposes
    `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` services. Runs a 10 Hz watchdog
    that zero-holds (never coasts/lunges) if `cmd_vel` goes stale past `cmd_timeout_s`.
  - `patrol_node.py` — loops a closed set of NED waypoints (`patrol.yaml`), either by streaming
    absolute position setpoints directly to ArduPilot (`mode: position`, its own separate
    MAVLink connection) or by publishing `/huitzilin/cmd_vel` (`mode: velocity`). Owns
    `/huitzilin/start_patrol`. Publishes `/huitzilin/mission_marker` for RViz.
  - `telemetry_logger.py` — subscribes odom/state, writes CSV for `plot_telemetry.py`.
  - `launch/week2_sitl.launch.py` — brings up bridge + patrol + telemetry logger together.
  - `params/bridge.yaml`, `params/patrol.yaml` — per-node MAVLink UDP listen ports and patrol
    waypoints/speeds; **must stay in sync** with the `--out` ports passed to `sim_vehicle.py`.
  - `params/sitl_frame.parm`, `config/huitzilin_3p5_6s.parm` — ArduPilot parameter files.
    `sitl_frame.parm` is the minimal frame-only file always loaded via `--add-param-file`;
    `huitzilin_3p5_6s.parm` is the fuller Pass-A param superset for the real airframe.

- **`mavlink_bridge`** — Week 1 package, **superseded by `huitzilin_sim`**. Kept for reference;
  do not extend it. It subscribed `/cmd/evade`, which in Week 4 will be folded into
  `mav_bridge`'s velocity path instead.

### Frame convention (critical, easy to get backwards)

- ArduPilot/MAVLink: **NED** (North-East-Down).
- Every ROS 2 topic: **ENU** (East-North-Up) per REP-103, body commands in **FLU**.
- The conversion happens in exactly one place: `MavBridge.ned_to_enu` / `enu_to_ned` in
  `mav_bridge.py`. Every other node (patrol, future perception/evasion) works exclusively in ENU
  and must not invent its own conversion. See `docs/frames.md`.
- Velocity setpoints to ArduPilot use `MAV_FRAME_BODY_OFFSET_NED`; absolute position setpoints
  use `MAV_FRAME_LOCAL_NED`. Setpoints must be re-sent faster than ~3s or ArduPilot times them out.

### Topic/service contracts

Full table in `docs/architecture.md`. Active (`/huitzilin/*`) namespace:
`cmd_vel` (patrol→bridge), `odom`/`state` (bridge→all), `mission_marker` (patrol→RViz),
`arm`/`takeoff`/`set_mode` (services on bridge), `start_patrol` (service on patrol).
Future-week topics (`/oak/*`, `/threat/*`, `/cmd/evade`, `/payload/alarm`) are provisional —
don't assume code for them exists.

### Flight state machine

DISARMED → ARMING → TAKEOFF → PATROL ⇄ EVADE → RTL/LAND → DISARMED, with any undefined fault
condition defaulting to FAILSAFE (calm hover → RTL → land, never an evasive maneuver). See
`docs/state_machine.md`. Only PATROL is implemented today; EVADE/FAILSAFE/RTL transitions are
design-only.

## Known sharp edges (read before touching SITL bring-up)

- **`FRAME_CLASS=0` on a fresh EEPROM is the classic silent failure**: the drone arms and
  "accepts" takeoff but throttle maxes out with zero lift (`PreArm: Motors: Check frame class
  and type`). Fix is always loading `sitl_frame.parm` (`FRAME_CLASS=1`/`FRAME_TYPE=1`) via
  `--add-param-file`, never `ARMING_CHECK 0` (that just hides the message).
- **Port mismatches cause `TimeoutError: no heartbeat`.** `bridge.yaml` listens on
  `udpin:0.0.0.0:14552`, `patrol.yaml` on `:14553`; `sim_vehicle.py --out` must fan out to both.
- Inline comments inside `.parm` files break the MAVProxy parser — use comment-only lines.
- Never `Ctrl-Z` a launch; a suspended job holds the SITL TCP socket. Restart Gazebo+SITL
  together if the FDM link goes half-broken.
- Gazebo headless is the norm here — GUI runs at ~24% real-time under WSL2 (no GPU passthrough),
  which is why timeouts in the code (e.g. takeoff) use generous wall-clock bounds.
- `patrol`'s `autostart` defaults to `false` intentionally — letting it autostart floods GUIDED
  with position setpoints during takeoff. Start it explicitly via `/huitzilin/start_patrol`
  after takeoff completes.

## Other docs worth reading before larger changes

- `docs/architecture.md` — node graph + full message/service contract table.
- `docs/requirements.md` — functional/non-functional/safety requirements (REQ-01..16) and
  explicit non-goals (no event camera, no multi-layer LiDAR, no custom depth math).
- `docs/SAFETY_CASE.md` — safety argument.
- `docs/JOURNAL.md` — week-by-week log of what was built, what broke, and why; the best source
  for "why is it done this way" questions.
- `docs/WEEK2_PLAN.md` — current week's gated task list.
