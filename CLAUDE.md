# CLAUDE.md

Non-obvious facts only. Architecture and BOM live in `docs/` and `HuitzilinReflex_v2.md`;
the measured answer lives in `docs/RESULTS.md`. This file owns bring-up commands, the frame
convention, and the sharp edges that bite anyone who checks this out and runs it.

## What this is

3.5″ ducted micro-quadrotor that patrols, signals, and reflexively dodges projectiles.
Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink ·
Python 3.12 · Ubuntu 24.04.

**Status: complete. The simulation project closed 2026-08-10.** Weeks 1–6 are done — patrol
loop, detection pipeline scored against a labeled bag library, Kalman filter + dodge
trigger, and the 20 m/s answer. Hardware bring-up is out of scope.

## The result

Full write-up and every caveat: **`docs/RESULTS.md`**. In short:

**A save is a sigmoid in tca, and the threshold does not depend on ball speed.** Over 310
on-course hover throws from 30 cells:

```
logit P(save) = -22.271 + 27.910 * tca      LD50 0.798 s (CI 0.779-0.817)
range         = speed * (tca_required + t_dead)
t_dead        = 0.178 s at 60 Hz, 0.270 s at 12 Hz   (rate-dependent — always quote the rate)
```

P = 0.5 at 0.80 s, 0.9 at 0.88 s, and 155/155 above 1.00 s. Ball speed adds nothing as a
covariate across nine speeds 14–29 m/s (z = +0.93, n.s.) — only the *range* needed to buy
the time scales with speed. At 20 m/s that is **21.1 m** at P=0.90, and a 26 m sensor scores
**28/29** head-on in hover with **0 false dodges in 31** clear-miss throws.

The bound is **sensor reach and rate**; every maneuver-side lever was measured and refuted.
The part that closes the gap is an AR0234 global-shutter mono + 10 mm M12 (See3CAM_20CUG,
$89, 13.5 g — lighter than the 61 g OAK-D Lite), and the same lens narrows the defended
sector to ~±10° usable. The aircraft **as built** carries a ~3.4 m OAK-D Lite, which caps it
at ~3.2 m/s.

Separately, the Week 4 envelope with the **real** depth detector on **patrol** — the only
non-oracle measurement, and never to be merged with a hover/oracle table:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

95 scored throws over five batteries. The counts do **not** divide by `week4_battery.yaml`
as it ships (`repeats: 3` × 5 scenarios × 5 batteries = 75, not 78): battery composition
drifted across the five runs. These are measured results — do not "correct" them to match
the yaml, and do not re-derive the yaml from them.

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

Optional nodes (neither runs by default):
```bash
# The supervisor needs perception: it watches /oak/points, which week2_sitl
# does not publish. Started from a bare week2_sitl it sits in permanent
# SENSOR_DROPOUT. Use a Week 3/4 entry point — both forward the argument:
ros2 launch huitzilin_perception week4_evasion.launch.py \
  with_patrol:=true with_supervisor:=true

ros2 run huitzilin_perception payload --ros-args \
  --params-file src/huitzilin_perception/params/payload.yaml
```
The supervisor logs which watches are armed at startup
(`watching: odom(1.0s) patrol_state(2.0s) cloud(1.0s) | disabled: cmd_vel`).
A timeout of `0.0` in `supervisor.yaml` disables that watch; `cmd_vel` ships disabled
because position-mode patrol publishes no ROS setpoint stream to watch.

The oracle lane — a synthetic far-range sensor, used for every result in `docs/RESULTS.md`:
```bash
# detector is REPLACED, never run alongside it.
# A SENSOR IS THREE AXES: reach, sector and rate. This command pins reach only;
# `oracle_rate_hz` (default 14.5) and `fov_half_angle_deg` (in
# params/oracle_detector.yaml, default 45.0) take their defaults silently.
# Quote all three beside every number, and pin them explicitly when it matters.
ros2 launch huitzilin_perception week6_oracle.launch.py \
  with_patrol:=true detection_range_m:=26.0

./scripts/run_dodge_battery.sh week6

# Any escape or save measurement: HOVER. Patrol cannot deliver a hit at range.
EXTRA_ARGS="-p hover_mode:=true" ./scripts/run_dodge_battery.sh week6
```

Unit tests: `./scripts/run_tests.sh` (both packages; forwards pytest args).
`.github/workflows/tests.yml` runs the ROS-free subset on every push.
Preflight: `./scripts/preflight_check.sh`.
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

## Sharp edges

- **`FRAME_CLASS=0` = silent no-lift.** Fresh EEPROM arms and accepts takeoff but throttle maxes with zero lift (`PreArm: Motors: Check frame class and type`). Always load `sitl_frame.parm` via `--add-param-file` (`FRAME_CLASS=1`, `FRAME_TYPE=1`). Never `ARMING_CHECK 0` — it hides the message.
- **Port mismatch = `TimeoutError: no heartbeat`.** `bridge.yaml` listens on `:14552`, `patrol.yaml` on `:14553`; `sim_vehicle.py --out` must fan out to both. MAVProxy's default `:14550` is for QGC only.
- **Patrol autostart is `false` in `patrol.yaml` intentionally.** `patrol_node.py` defaults `autostart=True`; the yaml overrides it. Autostarting floods GUIDED with position setpoints during takeoff and the drone never leaves the ground. Start via `/huitzilin/start_patrol` *after* takeoff.
- **Never `Ctrl-Z` a launch.** A suspended job holds the SITL TCP socket. Restart Gazebo+SITL together if the FDM link goes half-broken.
- **Inline comments inside `.parm` files break MAVProxy.** Use comment-only lines.
- **Judge all timing gates in sim time**, never wall-clock. Gazebo runs ~24% real-time under WSL2 (no GPU passthrough); the Dell (native Ubuntu, no discrete GPU) drops to ~0.33 RTF under depth rendering. Use `/clock` / message stamps.
- **`max_step_size` must stay `0.001`** (1000 Hz) — `0.004` causes a SITL "Main loop slow" PreArm failure.
- **Depth rendering only works on the native-Ubuntu Dell box.** WSL2/Iris Xe cannot render Gazebo depth at rate; SITL/flight logic runs on either box.
- **Depth camera runs at 15 Hz, not 30.** `ros_gz_bridge` PointCloudPacked→PointCloud2 can't sustain 30 Hz / 640×480 on the Dell. 640×480 is kept to match the real OAK-D Lite.
- **`iris_depth` must carry the flight plugins.** It merge-includes the *bare* `iris_with_standoffs` (no flight plugins). Symptom if missing: SITL spams `No JSON sensor message received`, `link 1 down`, nothing on `:9002`, no lift — while Gazebo steps fine and `/oak/points` streams. Fixed in 79c2e9b by porting `ArduPilotPlugin` + `LiftDrag`×8 + `ApplyJointForce`×4 + `JointStatePublisher` into `iris_depth/model.sdf` with the `iris_with_standoffs::` prefix **stripped** (merge flattens to top level).
- **The bag library is saturated** (recall 100%). It cannot referee threshold changes — a change that helps or hurts will read as no-change.
- **Bags recorded before `b0eedd5` lack attitude in `/huitzilin/odom`** — the detector falls back to camera-frame differencing. Never score against pre-`b0eedd5` bags.
- **`debug_dump_dir` costs ~40 ms/frame.** No latency number from a dumping run is valid.
- **Never derive velocity from `/gz/dynamic_poses` arrival times** — arrival is not emission; use the pose stamps.
- **Never edit a shipped yaml for a diagnostic.** `--symlink-install` makes the installed copy a symlink into `src/`, so you are editing the real config. Copy to `/tmp` and `sed` that.
- **`use_sim_time` is a launch argument only, never a yaml key.** A node started with `use_sim_time:=true` and no `/clock` logs FATAL and exits 1 after a 5 s grace window instead of silently freezing at t=0 (`huitzilin_sim/clock_guard.py`). `test_hw_config.py` fails if anyone re-adds the key to a params file.
- **`run_regression.sh`'s clock warm-up is load-bearing — do not delete it.** It plays a bag with `--topics /clock --loop` *before* starting the detector, then kills it once the guard passes. Without it the harness has no `/clock` until `score_bags` reaches its first bag, well past the guard's 5 s grace, so the detector dies at startup and every bag replays into a corpse: `TP=0`, **recall 0.0%** — a dead node, never a detection regression, and never a reason to touch `detector.yaml`. It works because the guard is *one-shot*, so a temporary clock suffices.
- **`set -u` breaks `/opt/ros/jazzy/setup.bash`.** Source ROS first, then enable it.
- **Hardware config lives in `hw_*` files, never as edits.** `params/hw_bridge.yaml`, `hw_frame.parm`, `hw_detector.yaml`, `hw_evasion.yaml` are overlays; the sim files stay the regression path. `test_hw_config.py` asserts each overlay's node name and keys exist in the file it overlays — in ROS 2 a mistyped node name silently loads *nothing*.
- **`RTL_ALT` is centimetres; `FENCE_ALT_MAX` is metres.** `hw_frame.parm` pins `RTL_ALT 400` (4 m) under the 5 m ceiling, because ArduPilot's 15 m default would make a fence-breach RTL climb through the fence it is answering.
- **`ATC_ANGLE_MAX` is in DEGREES in this build, not centidegrees** — `param set ATC_ANGLE_MAX 45`, never 4500. Related renames that make parameters look absent: `PSC_JERK_XY`→`PSC_JERK_NE`, `WPNAV_ACCEL`→`WP_ACC`. `PSC_ACC_XY`, `ANGLE_MAX` and `PSC_VELXY_P` genuinely do not exist here.
- **`supervisor_node` reports no faults while disarmed.** Half the watched topics are legitimately silent on the bench. Faults are gated on `armed`, and no fault path can reach EVADE — asserted over all states × faults × `armed`. (EVADE *is* reachable disarmed via the threat edge, but commands nothing.)
- **A supervisor watch on a topic nothing publishes is a permanent fault.** `_age` reads a never-seen topic as infinitely stale — correctly, since that is how a publisher which died before its first message is caught. A timeout of `0.0` is the only way to say "this configuration does not produce that topic". `cmd_vel_timeout_s` ships `0.0` because `patrol.yaml` runs `mode: "position"`, where `patrol_node` sends setpoints over its own MAVLink connection and never creates the `/huitzilin/cmd_vel` publisher. Watching it drove `SETPOINT_STALL → FAILSAFE → LOITER → RTL_LAND` one second after every arm. Set it to `1.0` only alongside `mode: "velocity"`.
- **`oracle_detector` and `detector` must never run together.** Both publish `/threat/centroid`, so the tracker would get two uncorrelated views of one ball. `week6_oracle.launch.py` never includes `week3_perception`, and pins `with_supervisor:=false` because it publishes no `/oak/points` to watch.
- **A sensor is reach, sector AND rate — pinning one axis describes the wrong instrument.** Quoting only `detection_range_m` leaves `oracle_rate_hz`/`rate_hz` and `fov_half_angle_deg` (or the depth lane's `fov_half_*_deg` + `image_*_px`) silently at defaults. This cost a full run: a fidelity gate pinned to the real detector's 3.4 m reach still failed, because it flew the *proposed* AR0234 optics (±13.5°, 60 Hz) against a Week 4 reference flown on the OAK-D Lite (±33.65°, 15 Hz) — 16.6 % of the solid angle, ~1.6 clouds/throw against ~2.8, versus `min_track_updates: 3`. Matching rate alone does not rescue it; matching all three does. The reach-only command had been copied into five places.
- **`counterfactual_min_m` is BLANK on `NO_DODGE` rows, and that blank is a trap.** Join on it naively and every no-fire silently leaves the *denominator*, turning a save rate into a rate over only the throws that fired. On the 26 m / 20 m/s cell that converted a true **21/30 (70 %)** into **21/21 (100 %)** — better than the oracle's own 28/29, purely from a blank cell. When no dodge fired the drone never deviated, so the actual path *is* the counterfactual: substitute `counterfactual_min_m := actual_min_m`, and count a `NO_DODGE` inside the hit radius as a **loss**, not as missing data. `scripts/hz_counterfactual.py` keeps emitting the blank on purpose (recorded CSVs stay reproducible); the substitution belongs in whatever scores them.
- **`detection_range_m` is an INPUT, not a result**, and it is read once at startup. `oracle_detector_node.py.__init__` reads it into an attribute and installs **no** parameter callback — `ros2 param set` is accepted and ignored. Every range change needs a full stack restart. Pass it as a float: `detection_range_m:=5` is inferred as an integer and rejected.
- **An oracle cell must raise `offset_forward_m` with its range** — `offset_forward_m ≥ detection_range_m + 8.5` at 20 m/s, or the ball enters the gate from inside it and the cell delivers a shorter sensor than its label. Verify per run: `first_det_range_m` in the CSV must match the launched `detection_range_m` to ~0.2 m.
- **`hover_mode` is the only configuration that has ever put the ball on a hit course.** Under patrol, 0 of 98 throws arrived inside 0.109 m; with `EXTRA_ARGS="-p hover_mode:=true"`, 40/40 did. The throw-window gate holds each throw until the drone reaches a **rolling-max** cruise (`min_cruise_frac` 0.95 of a 3.49 m/s max against a 2.09 m/s median), so the lead extrapolates a peak the drone never sustains. Hover also drops the straight-leg requirement, which is what made ranges ≥18 m unmeasurable.
- **Score saves on the counterfactual, never on `dodged`.** A throw is on a hit course only if `counterfactual_min_m` ≤ 0.30 m. A fire count and a save rate are different numbers, and a cell with no on-course throws measured **nothing** — never report it as 0/N.
- **Kill lab stacks by installed path, never a guessed `_node` name.** `evasion_nod[e]`/`patrol_nod[e]` match nothing; seven stacks once accumulated and a whole queue returned plausible numbers with no error. The decisive contamination detector is the implied rate `(track_updates - 1) / track_age_s` reading *above* the launched oracle rate.

## Key docs

| Doc | Contents |
|---|---|
| `docs/RESULTS.md` | **The answer**: the tca law, the sensor requirement, and every closed question |
| `HuitzilinReflex_v2.md` | Master doc: objectives, BOM, roadmap |
| `docs/architecture.md` | Node graph + message/service contracts |
| `docs/frames.md` | Coordinate frames + TF tree |
| `docs/state_machine.md` | State/transition table |
| `docs/requirements.md` | REQ-01…REQ-16 + non-goals |
| `docs/SAFETY_CASE.md` | FMEA, geofence/RTL, kill-switch, safety/legal rules |
| `docs/SETUP.md` | Install from scratch |
| `docs/bag_capture_runbook.md` | Bag re-capture + regression/tuning procedure (Dell) |
| `docs/dodge_battery_runbook.md` | Dodge battery + sweep procedure (Dell) |

Development history is in git, not in the tree: `git log -p docs/`. The experiment harness
and 193 raw result files lived in `lab/` until the project closed — `git show 3cf0bbb:lab/…`.
