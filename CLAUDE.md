# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
It is the consolidated context for the project — the docs under `docs/` and the master doc
`HuitzilinReflex_v2.md` go deeper, but everything needed to orient is here.

## What this is

**HuitzilinReflex** ("Hummingbird Reflex") is a 9-week (Summer 2026) project building a compact,
agile, **3.5″ ducted micro-quadrotor** that autonomously patrols an area, signals with light/sound,
and **reflexively dodges incoming projectiles** using an onboard stereo-depth stack. The ducts fully
isolate the props, so projectile-evasion test cycles can run without exposed blades. The name is
Classical Nahuatl for "hummingbird" (root of *Huitzilopochtli*); the bird hovers, darts, and
reverses in a fraction of a second — "loiter like a hover, dodge like a dart."

Two engineering pillars:

1. **Autonomous patrol & signaling** — persistent pathing with a strobe (WS2812B LED) + siren (piezo) warning payload.
2. **Kinematic evasion** — a low-latency loop that detects an incoming projectile, predicts its intercept, and commands a sharp dodge.

**Design philosophy: simulate first, fly last — integrate, don't fabricate.** The expensive/fragile
parts (airframe, depth camera) are pre-assembled plug-and-play hardware; engineering effort
concentrates on the perception + evasion loop, almost all of which is de-risked in simulation before
a real prop spins. The single real fabrication step is one flight-controller swap (Week 5).

### Software stack (pinned)

ROS 2 **Jazzy** · Gazebo **Harmonic** (8.11.0) · ArduPilot Copter 4.5+ **SITL** · **pymavlink** ·
Python 3.12 · Ubuntu 24.04. The ROS 2 ↔ ArduPilot bridge is custom pymavlink (no MAVROS / no
micro-ROS agent in the loop). SITL is board-agnostic, so sim work does not depend on the FC swap.

### Current status (as of 2026-06-19)

**Week 2 is essentially complete** — the `huitzilin_sim` stack autonomously flies a closed patrol
loop in SITL through our own ROS 2 ↔ pymavlink bridge (`arm → takeoff 2 m → 5×5 m square → loop`).
Evidence: 43 continuous laps, mean lap **29.51 s**, stdev **0.93 s** (`docs/week2_patrol_evidence.md`).
Remaining Week 2 items: W2-05 (Pass-B model fidelity) deferred to Week 7–8; W2-18 (fresh-checkout
sign-off) was in teammate testing. **Week 3 (perception) is the next phase and is not yet coded** —
see `HuitzilinReflex_Week3_Playbook.docx`. Perception / evasion / payload topics exist only as
**provisional contracts** in the docs, not as code.

### Dev environment

- Primary box: **Windows 11 + WSL2 (Ubuntu 24.04)**, IntelliJ Remote Development connected into WSL.
  All ROS/Gazebo/ArduPilot commands run **inside WSL**, not native Windows.
- Second box: a **native-Ubuntu Dell Inspiron 3670** (i5-8400, Intel UHD630 — a real GPU), also ROS 2 Jazzy.
- **GPU caveat:** WSL2 has no GPU passthrough on the laptop's Intel Iris Xe, so Gazebo runs
  **headless at ~24% real-time** there. From Week 3 on, render the simulated depth camera on the
  native Dell box (real GPU); keep SITL/flight logic on either. **Judge all timing/rate gates in
  *sim time* (`/clock`, message stamps), never wall-clock** — at 0.24× real-time, wall-clock
  measurements look ~4× off and will "fail" gates that actually pass.

The `ECC/` directory at the repo root is an unrelated, separately-cloned tool/plugin marketplace
(untracked) — ignore it when working on this project.

## Build & run (inside WSL)

```bash
# one-time env (see docs/SETUP.md for full install of ROS2/ArduPilot/Gazebo/ros_gz/pymavlink)
source /opt/ros/jazzy/setup.bash

# build  (repo root IS the colcon workspace — packages live in src/)
cd ~/huitzilin_ws
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

After takeoff completes, start patrol explicitly: call `/huitzilin/arm`, then `/huitzilin/takeoff`,
then `/huitzilin/start_patrol`.

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

- **`huitzilin_sim`** — the active stack (Week 2+). ament_python; deps
  `rclpy geometry_msgs nav_msgs std_msgs std_srvs visualization_msgs`. Entry points (`setup.py`):
  `mav_bridge`, `patrol`, `telemetry_logger`. Contains:
  - `mav_bridge.py` — pure pymavlink class (`MavBridge`), no ROS dependency. Owns the single
    connection to ArduPilot, all arm/mode/takeoff/setpoint/telemetry calls, and the **only**
    NED↔ENU conversion helpers (`ned_to_enu`/`enu_to_ned`) in the whole codebase. Has a
    `--selftest` CLI path (connect → EKF wait → GUIDED → arm → takeoff 2 m → nudge → LAND).
    `type_mask` constants: `MASK_VEL_ONLY` (4039), `MASK_POS_ONLY` (4088), `MASK_POS_YAW`.
  - `mav_bridge_node.py` — ROS 2 wrapper around `MavBridge`. Subscribes `/huitzilin/cmd_vel`
    (FLU), publishes `/huitzilin/odom` (ENU) + `/huitzilin/state` (JSON), exposes
    `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` services. Runs a 10 Hz watchdog
    that zero-holds (never coasts/lunges) if `cmd_vel` goes stale past `cmd_timeout_s`.
  - `patrol_node.py` — loops a closed set of NED waypoints (`patrol.yaml`), either by streaming
    absolute position setpoints directly to ArduPilot (`mode: position`, its own separate
    MAVLink connection on `:14553`) or by publishing `/huitzilin/cmd_vel` (`mode: velocity`). Owns
    `/huitzilin/start_patrol`. Publishes `/huitzilin/mission_marker` for RViz.
  - `telemetry_logger.py` — subscribes odom/state, writes CSV for `plot_telemetry.py`.
  - `launch/week2_sitl.launch.py` — brings up bridge + patrol + telemetry logger together
    (the optional SITL+Gazebo include block is commented out; keep the sim in its own terminal).
  - `params/bridge.yaml`, `params/patrol.yaml` — per-node MAVLink UDP listen ports and patrol
    waypoints/speeds; **must stay in sync** with the `--out` ports passed to `sim_vehicle.py`.
  - `params/sitl_frame.parm`, `config/huitzilin_3p5_6s.parm` — ArduPilot parameter files.
    `sitl_frame.parm` is the minimal frame-only file (`FRAME_CLASS=1`/`FRAME_TYPE=1`) always loaded
    via `--add-param-file`; `huitzilin_3p5_6s.parm` is the fuller Pass-A param superset for the
    real airframe (note: ArduPilot param names differ from the playbook — e.g. `WP_SPD` not
    `WPNAV_SPEED`).

- **`mavlink_bridge`** — Week 1 package, **superseded by `huitzilin_sim`**. Kept for reference;
  do not extend it. It subscribed `/cmd/evade`, which in Week 4 will be folded into
  `mav_bridge`'s velocity path instead.

### Frame convention (critical, easy to get backwards)

- ArduPilot/MAVLink: **NED** (North-East-Down). A +2 m altitude is `z = -2.0` (down negative).
- Every ROS 2 topic: **ENU** (East-North-Up) per REP-103, body commands in **FLU**.
- The conversion happens in exactly one place: `MavBridge.ned_to_enu` / `enu_to_ned` in
  `mav_bridge.py`. Every other node (patrol, future perception/evasion) works exclusively in ENU
  and must not invent its own conversion. Mirrored/rotated RViz markers ⇒ the bridge conversion is
  off, not the marker code. See `docs/frames.md`.
- Velocity setpoints to ArduPilot use `MAV_FRAME_BODY_OFFSET_NED`; absolute position setpoints
  use `MAV_FRAME_LOCAL_NED`. Setpoints must be re-sent faster than ~3 s or ArduPilot times them out
  (the watchdog/send loop must stay lean and non-blocking).

### Topic/service contracts

Full table in `docs/architecture.md`. Active (`/huitzilin/*`) namespace, all Reliable QoS:

| Interface | Type | Direction | Frame |
|---|---|---|---|
| `/huitzilin/cmd_vel` | `geometry_msgs/Twist` | patrol → bridge | body **FLU** |
| `/huitzilin/odom` | `nav_msgs/Odometry` | bridge → all | `odom` (ENU) |
| `/huitzilin/state` | `std_msgs/String` (JSON) | bridge → all | N/A |
| `/huitzilin/mission_marker` | `visualization_msgs/MarkerArray` | patrol → RViz | `odom` (ENU) |
| `/huitzilin/arm` | `std_srvs/SetBool` | → bridge | service |
| `/huitzilin/takeoff` | `std_srvs/Trigger` | → bridge | service |
| `/huitzilin/set_mode` | `std_srvs/Trigger` (+`mode` param) | → bridge | service |
| `/huitzilin/start_patrol` | `std_srvs/SetBool` | → patrol | service |

Future-week topics (provisional, **no code yet**): `/oak/points`, `/oak/depth` (Wk3),
`/threat/centroid`, `/threat/intercept`, `/cmd/evade` (Wk4, folds into the bridge velocity path),
`/payload/alarm` (Wk6). A `supervisor_node` (state machine, Wk2+) is also planned.

### Flight state machine

DISARMED → ARMING → TAKEOFF → PATROL ⇄ EVADE → RTL/LAND → DISARMED, with any undefined fault
condition defaulting to FAILSAFE (calm hover → RTL → land, **never** an evasive maneuver). See
`docs/state_machine.md`. Only PATROL is implemented today; EVADE/FAILSAFE/RTL transitions are
design-only.

## Hardware (as purchased — full BOM in `HuitzilinReflex_v2.md` Appendix A, ~$1,150)

- **Airframe:** GEPRC CineLog35 V2 HD, 3.5″ ducted, 6S, **BNF / ELRS 2.4 GHz**, 142 mm wheelbase.
  Stock propulsion: SPEEDX2 2105.5 motors, HQProp D-T90 props.
- **Flight controller (Week 5 swap):** **MicoAir H743 V2 AIO** (STM32**H743** + integrated 4-in-1
  45 A AM32 ESC) **replaces** the stock Betaflight-class GEP F722-45A. Same 25.5×25.5 mm mount; the
  H7 board is required to run ArduPilot, driven by pymavlink over USB/UART.
- **Companion computer:** Raspberry Pi 5 (4 GB) + Active Cooler. Runs ROS 2 + the evasion node.
- **Depth sensor:** Luxonis **OAK-D Lite** — on-chip Myriad X VPU computes stereo depth **in-camera**
  and streams finished depth/point clouds over **USB-3** (USB-2 cable would throttle it). The Pi
  never runs per-pixel depth math.
- **Power:** Pololu **D24V50F5** 5 V/5 A BEC steps the 6S pack down for the Pi **in flight** (never
  off the FC rail); set `usb_max_current_enable=1` in the Pi's `config.txt`. Bench dev uses the 27 W
  USB-C PSU. Flight battery: CNHL 1300 mAh 6S.
- **Radio:** RadioMaster Pocket (ELRS 2.4 GHz) — manual control, failsafe, **kill-switch**.
- **Payload:** WS2812B addressable LED strip (via a 3.3→5 V level shifter, e.g. 74AHCT125) + piezo
  siren (via a transistor circuit), both on Pi GPIO.

## Perception & evasion pipeline (design — Weeks 3–4, not yet built)

`OAK-D Lite (on-chip stereo depth) → USB-3 → Pi 5 → depthai-ros republishes PointCloud2/Depth →
evasion node (spatial slice / differential clustering → centroid → predictive Kalman filter →
intercept prediction) → on threat: GPIO alarm + pymavlink SET_POSITION_TARGET_LOCAL_NED ~1.5 m/s
velocity spike → ArduPilot velocity-loop dodge`. A projectile appears as a sudden drastic cluster of
depth-value differentials. Target: detect within 200 ms, ≥80% evasion success. Non-goals: no event
camera, no multi-layer LiDAR, no custom depth math (use the on-chip DepthAI pipeline).

## Roadmap (9 weeks, simulation-first; each week has a Definition of Done)

- **Wk 0 — Procurement** (done): full BOM ordered.
- **Phase A — Foundations & Simulation (Wk 1–4):**
  - **Wk 1** (done): architecture, safety case, sim toolchain (SITL+Gazebo+ROS 2). *DoD:* sim quad arms/takes off/holds via pymavlink on a fresh checkout.
  - **Wk 2** (≈done): airframe tuning + ROS 2↔pymavlink bridge + patrol path-follower. *DoD:* autonomous closed patrol loop in Gazebo with logged telemetry.
  - **Wk 3** (next): simulated stereo-depth sensor + synthetic thrown-object scenarios + detection node + labeled rosbag library. *DoD:* detection node flags ≥95% of simulated incoming clusters with a quantified false-positive rate, scored repeatably, on a fresh checkout. (`HuitzilinReflex_Week3_Playbook.docx`, tasks W3-01…W3-22.)
  - **Wk 4:** predictive Kalman filter + close the loop (detection → intercept → velocity-spike → mocked GPIO). *DoD:* SITL drone dodges a battery of simulated projectiles above target success, latency within budget.
- **Phase B — Hardware bring-up (Wk 5–6):**
  - **Wk 5:** the one fabrication step — FC swap to MicoAir H743, flash/configure ArduPilot, bind radio, wire the BEC, mount Pi+OAK-D. *DoD:* clean ArduPilot bench arm-up (props off), failsafes + kill-switch verified, Pi on BEC with no FC-rail draw.
  - **Wk 6:** wire payload (LED + siren), bring up real OAK-D over USB-3 (depthai-ros), characterize real stereo noise, Remote ID. *DoD:* live depth from the real sensor; payload triggers within latency budget.
- **Phase C — Integration, flight & validation (Wk 7–9):**
  - **Wk 7:** HITL (real FC, sim world) + tethered/netted hover. *DoD:* stable tethered hover, full stack logging.
  - **Wk 8:** incremental real flight (manual hover → patrol → evasion) **only inside netting with soft projectiles**. *DoD:* ≥1 clean autonomous patrol + successful evasion, fully logged.
  - **Wk 9:** full validation matrix, as-built docs, post-mortem. *DoD:* reproducible build doc + validation report.

*Stretch/cut order: keep Wk 1–4 (sim) and Wk 5 (safe HW checkout) sacred; trim real free-flight evasion in Wk 8 first (demo it in HITL/SITL instead).*

## Requirements & safety (summary; full detail in `docs/requirements.md`, `docs/SAFETY_CASE.md`)

Functional REQ-01..06 (autonomous patrol, detect projectile via OAK-D, predict intercept, evade
within 200 ms, fire alarm, resume patrol). Non-functional REQ-07..09 (≤200 ms detect→evade latency,
≥80% evasion success, ≤200 g payload). Constraints REQ-10..13 (CineLog35 V2 airframe, H743 FC, Pi 5,
OAK-D Lite). Safety REQ-14..16 (safe link-loss response, kill-switch cuts motors, all faults land in
a safe state).

Safety case highlights: geofence cylinder (10 m radius, 5 m AGL; `FENCE_ENABLE/TYPE/RADIUS/ALT_MAX`);
RTL on link loss / low battery / fence breach; a dedicated RC channel mapped to ArduPilot motor
emergency stop (kill-switch must be in hand for all powered tests); projectile testing only inside
netting with soft projectiles, props-off on the bench. **Fail-safe default on any fault = calm hover
→ RTL → land, never an evasive lunge.** Privacy: depth only by default, footage stays local, never
pointed at non-consenting people; the warning payload is a signal, never used to harass.

## Known sharp edges (read before touching SITL bring-up)

- **`FRAME_CLASS=0` on a fresh EEPROM is the classic silent failure**: the drone arms and
  "accepts" takeoff but throttle maxes out with zero lift (`PreArm: Motors: Check frame class
  and type`). Fix is always loading `sitl_frame.parm` (`FRAME_CLASS=1`/`FRAME_TYPE=1`) via
  `--add-param-file`, never `ARMING_CHECK 0` (that just hides the message). This was the real Week 2
  takeoff bug. (If setting by hand: `param set FRAME_CLASS 1`, `FRAME_TYPE 1`, then `reboot`.)
- **Port mismatches cause `TimeoutError: no heartbeat`.** `bridge.yaml` listens on
  `udpin:0.0.0.0:14552`, `patrol.yaml` on `:14553`; `sim_vehicle.py --out` must fan out to both.
  MAVProxy's own default `:14550` still carries the Week 1 `first_flight.py` / QGC link.
- **Don't blind force-arm** (`param2=21196`) — fix the pre-arm cause (frame/EKF) instead. The bridge
  arms cleanly via `/huitzilin/arm` once EKF/GPS is ready.
- **Patrol autostart:** `patrol_node.py` declares `autostart` default `True` (demo-friendly), but the
  shipped `patrol.yaml` sets it **`false` intentionally** — autostarting floods GUIDED with position
  setpoints during takeoff and the drone never leaves the ground. Start patrol via
  `/huitzilin/start_patrol` *after* takeoff (commit 26a2264).
- Inline comments inside `.parm` files break the MAVProxy parser — use comment-only lines.
- Never `Ctrl-Z` a launch; a suspended job holds the SITL TCP socket. Restart Gazebo+SITL
  together if the FDM link goes half-broken.
- Gazebo headless is the norm here — GUI runs at ~24% real-time under WSL2 (no GPU passthrough),
  which is why timeouts in the code (e.g. takeoff bumped to 90 s) use generous wall-clock bounds.
  Visualize in **RViz** markers, not the Gazebo GUI.
- Avoid a stray `src/src/` duplicate after a git pull — it breaks `colcon build`.

## Docs map

- `HuitzilinReflex_v2.md` — master project doc: name/objectives, full hardware BOM, perception/evasion design, 9-week roadmap with DoDs, safety/legal/ethics.
- `docs/architecture.md` — node graph + full message/service contract table.
- `docs/requirements.md` — functional/non-functional/safety requirements (REQ-01..16) and explicit non-goals.
- `docs/frames.md` — coordinate frames + TF tree.
- `docs/state_machine.md` — full state/transition table.
- `docs/SAFETY_CASE.md` — FMEA, geofence/RTL, kill-switch, enclosure rules, traceability.
- `docs/SETUP.md` — full install of ROS 2 / ArduPilot / Gazebo / ros_gz / pymavlink + run/acceptance.
- `docs/WEEK2_PLAN.md` — Week 2 gated task list (W2-01…W2-19) + playbook reconciliation.
- `docs/week2_patrol_evidence.md` — the 43-lap patrol evidence.
- `docs/JOURNAL.md` — week-by-week log of what was built, what broke, and why; best source for "why is it done this way".
- `HuitzilinReflex_Week3_Playbook.docx` — the upcoming Week 3 perception task cards (W3-01…W3-22).
