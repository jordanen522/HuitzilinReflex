# CLAUDE.md

Non-obvious facts only. Architecture, roadmap, and BOM live in `docs/` and
`HuitzilinReflex_v2.md`. This file owns: bring-up commands, sharp edges, measured nulls,
and the measured dodge envelope.

## What this is

3.5″ ducted micro-quadrotor that patrols, signals, and reflexively dodges projectiles.
Stack: ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink · Python 3.12 · Ubuntu 24.04.

**Current phase: Week 5** (hardware bring-up — FC swap; `docs/hardware_bringup.md`).
The Week 5 **software** lane is complete (supervisor, payload, clock guard, hardware
config overlays, hardware preflight); everything still open is gated on the FC swap.
Weeks 5-9 are planned as parallel hardware/software lanes in `docs/weeks_5_9_plan.md`;
the software lane is the larger half and is not blocked by the FC swap.
The Week 6 **sim** lane has since answered the project's central question ahead of the
hardware — see `## The dodge law` and `docs/week6_result.md`. What the real OAK-D bring-up
still owes is the measurement of its own reach and rate, not the envelope arithmetic.

Weeks 1–4 closed: patrol loop (Wk2), detection pipeline scored against a labeled bag
library (Wk3), Kalman filter + dodge trigger (Wk4). Week 4's result is a **capability
envelope, not a success rate** — always report it split by ball speed, never blended:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

Conditions, because a second table further down looks like this one and is not: **patrol**
flight, the **real depth detector** at its measured ~3.4 m reach, scored on `dodged`. The
oracle results in `## The dodge law` are a different experiment — synthetic sensor at a
settable range, stationary drone, scored on the counterfactual. Neither contradicts the
other. Never merge the two tables, and never compare a hover row against a patrol row.

95 scored throws over five batteries. The counts do **not** divide by
`week4_battery.yaml` as it ships today (`repeats: 3` × 5 scenarios × 5 batteries = 75,
not 78): battery composition drifted across the five runs. These are measured results —
do not "correct" them to match the yaml, and do not re-derive the yaml from them.
Latency mean 95–115 ms/battery against a 150 ms
budget; ~25% of individual dodges exceed it (max 282 ms) but this costs no outcomes,
because tca at commit is 0.18–0.29 s — latency is not the binding term.

**14 m/s is not a special number.** Week 4's failure there is one point on a law that has
since been measured directly — `## The dodge law` below. The conclusion held (the bound is
**sensing reach**); the arithmetic that used to sit here did not, and its "3.6 m/s of
instantaneous escape" figure should not be quoted again.

## The dodge law (measured 2026-08-08/09, hover + oracle)

**A save is a THRESHOLD in tca, not a rate, and the threshold does not depend on ball
speed.** 60 throws in `hover_mode` on a true hit course, `dodge_speed_mps` 3.0:

| ball speed | oracle range (INPUT) | saved |
|---|---|---|
| 14 m/s | 8 m | 0/10 |
| 14 m/s | 12 m | 0/10 |
| 14 m/s | 16 m | 3/8 |
| 14 m/s | 20 m | 9/9 |
| 20 m/s | 21 m | 3/10 — every save at tca ≥ 0.80 s |
| 20 m/s | 16 m (negative control) | 0/10 — tca 0.41–0.69 s, all below threshold |

Highest failure / lowest save: **0.79 / 0.83 s** at 14 m/s, **0.81 / 0.80 s** at 20 m/s.
No overlap either time. The 20 m/s run was pre-registered before flying, with its own
negative control. Fitted effective escape `delta = 0.295·tca^3.15` (14 m/s) and
`0.337·tca^3.69` (20 m/s) independently give required tca **0.83–0.86 s**.

**Only the RANGE needed to buy that time scales with ball speed:**

    range = speed × (tca + 0.177)        # 0.177 s = pipeline + commit, hover

**8.3 m @ 8 m/s · 14.5 m @ 14 m/s · ~20–21 m @ 20 m/s.** Inverting for the real OAK-D
Lite's measured **~3.4 m** gives a maximum dodgeable ball speed of **~3.5 m/s in hover**.
That, not 14 m/s, is the honest edge of the sensor the aircraft actually carries.

Two caveats bound the number. **Hover only** — the 0.177 s assumes a stationary drone, so
closing speed is the ball alone; a patrolling drone closes faster (effective 24 m/s against
a 14 m/s ball) and needs roughly 1.7x more range. And `dodge_speed_mps` was 3.0, not the
shipped 1.5 — though that lever plateaus, so it does not move the threshold (see nulls).

**The part that closes the gap is named, and it is not a longer-range OAK-D.** The deficit
is **angular resolution**, not headline range: the ball spans 8.9 px at the measured 3.4 m
and 1.44 px at 21 m. An **AR0234 global-shutter mono + 10 mm M12** (e-con See3CAM_24CUG,
$99, 26 g — *lighter* than the 61 g OAK-D Lite) gives 0.300 mrad/px, 11.1 px at 21 m, and
26 m of reach = **26.8 m/s**. Full derivation, rejected alternatives and the FOV/stereo
costs: `docs/week6_result.md`.

**Four dead claims that keep resurfacing. All were fitted on patrol escape displacement,
all are inflated 3–20x, never quote them:**
- "escape = 0.219·tca, so 0.30 m needs tca ~1.35 s" — the real curve is `0.295·tca^3.15`
  and the answer is 0.86 s.
- "no throw ever escaped 0.30 m, best 0.2616 m" — the 20 m cell reached 0.917 m actual,
  delta 0.737 m.
- "0/27 saved at 14 m/s at every range" — true only *below* the threshold; the ranges
  flown then bought tca ≤ 0.5 s.
- "18/28/37 m of reach", and the `(v_ball + 4)` closing term — superseded by the formula
  above.

Report split by ball speed, never blended — that rule now applies to this table too.

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

Week 6 lane — the 20 m/s push (all default-off or sim-only; the ≤8 m/s path is unchanged):
```bash
# Synthetic far-range sensor. detector is REPLACED, never run alongside it.
ros2 launch huitzilin_perception week6_oracle.launch.py \
  with_patrol:=true detection_range_m:=12.0     # 12 m buys tca 0.68 s at 14 m/s — under
                                                # the 0.83 s threshold. Saves need 20 m.
./scripts/run_dodge_battery.sh week6      # B08-B10 at 20 m/s, B11/B12 continuity

# Any escape or save measurement: HOVER. Patrol cannot deliver a hit at range.
EXTRA_ARGS="-p hover_mode:=true" ./scripts/run_dodge_battery.sh week6
```
**`detection_range_m` is an INPUT, not a result.** A dodge scored under the oracle is a
claim about the tracker, trigger and airframe *given* a sensor with that reach — never
evidence such a sensor exists. Quote the range beside every number, and run the fidelity
gate first (`detection_range_m:=3.4` must reproduce the measured 0/17 at 14 m/s and the
78/78 shape at 8 m/s) before believing anything it says about 20 m/s. **Run the gate in the
same mode as the run it validates** — B11/B12 are patrol, geometrically identical to
B02/B03 by design, and a hover gate cannot reproduce a patrol number.

New levers, all shipping OFF so every recorded result stays reproducible:
`evade_accel_ff_mps2` (0.0 = today's velocity-only setpoint; >0 adds an acceleration
feedforward on `/cmd/evade_accel` — **now flown and null, see below; keep it off**),
`allow_upward_escape` (false), `alert_min_track_updates`
(inert until something publishes `/threat/cue`). `evade_rate_hz` in `bridge.yaml` is the
one change that is on by default: the bridge now sends a dodge on receipt instead of
waiting up to 100 ms for its 10 Hz watchdog tick.

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
- **`oracle_detector` and `detector` must never run together.** Both publish
  `/threat/centroid`, so the tracker would get two uncorrelated views of one ball.
  `week6_oracle.launch.py` never includes `week3_perception`, so there is no detector to
  forget to disable; it also pins `with_supervisor:=false`, because it publishes no
  `/oak/points` for `supervisor.yaml` to watch.
- **`detection_range_m` is read once, at startup.** `oracle_detector_node.py.__init__`
  (~line 123) reads it into an attribute and the node installs **no** parameter callback —
  `ros2 param set` is accepted and ignored. Every range change needs a full stack restart.
  Pass it as a float: `detection_range_m:=5` is inferred as an integer and rejected.
- **`hover_mode` is the only configuration that has ever put the ball on a hit course.**
  Under patrol, 0 of 98 throws arrived inside 0.109 m; with `EXTRA_ARGS="-p
  hover_mode:=true"`, 40/40 did (counterfactual 0.07–0.18 m). The fault was never the lead
  *direction* (that is ~5° off, not the ~90° an ordinal-matching artefact once suggested):
  the throw-window gate holds each throw until the drone reaches a **rolling-max** cruise
  (`min_cruise_frac` 0.95 of a 3.49 m/s max against a 2.09 m/s median), so the lead
  extrapolates a peak the drone never sustains — over-lead +1.5 m at 1.2 s of ball flight,
  +2.6 m at 1.5 s, 96–98% along-track. Hover also drops the straight-leg requirement, which
  is what made ranges ≥18 m unmeasurable (patrol skipped 9/10 there). Hover is the
  *trustworthy* instrument, not the *generous* one — it shows a real dead time patrol does
  not, out to t ≈ 0.20 s.
- **Score saves on the counterfactual, never on `dodged`.** A throw is on a hit course only
  if `counterfactual_min_m` ≤ 0.30 m. A fire count and a save rate are different numbers,
  and scoring every throw against a dead-centre worst case reads a working system as broken.
  A cell with no on-course throws measured **nothing** — never report it as 0/N.
- **Escape displacement is not fit to referee a lever.** Patrol's counterfactual extrapolates
  a straight line through a vehicle that is tracking waypoints, so its own curvature lands in
  the escape term: median fit residual **0.0112 m patrol vs 0.0011 m hover**, and at matched
  tca 0.385 s the two disagree **19x** (0.1268 vs 0.0066 m). On top of that, two *identical*
  control arms drifted **1.58x** apart — larger than the effect they were controlling for.
  So: measure escape in hover, A/B within one session, fly the control twice, and prefer the
  dataflash velocity step (`PSCN`/`PSCE` `DVN`/`DVE` vs `VN`/`VE`), which compares command
  against achievement inside a single dodge and has no session drift. **Any past null scored
  on patrol escape displacement with n ≤ 8 is unsafe.**
- **An oracle cell must raise `offset_forward_m` with its range** —
  `offset_forward_m ≥ detection_range_m + 8.5` at 20 m/s. At the old 15.0 the ball entered the
  gate from inside it, so every flown "12 m" cell delivered first detection at 9.9–11.3 m: a
  ~10–11 m sensor wearing a 12 m label. Verify every run — `first_det_range_m` in the CSV must
  match the launched `detection_range_m` to ~0.2 m. Fixed to 22.0 in `e1e77d3` (a 1.40 s
  straight leg at 20 m/s); a 20 m gate needs 30.0. If B08–B10 come back mostly `SKIPPED, not
  thrown`, suspect the throw window and waypoint spacing, never the tracker — a skip is
  absence of data, never a failed dodge.
- **`ECC/` at repo root** is an unrelated plugin marketplace (untracked). Ignore it.

## Measured nulls — do not re-run

Each was measured over a full battery; the baseline was best on every column. Re-running
these is the most common way to lose a session here.

- `roi_max_range_m` 5→8 · `min_track_updates` 3→2 · `cluster_min_points` 5→3
- frame rate 320×240 @ 30 Hz
- the multi-hypothesis tracker (it works correctly — it just bought zero tca). Track
  *continuity* is refuted separately: the associator does not fragment the ball, 1.05 spawns
  per throw, and MHT and one unbroken filter commit at the same tca.
- the command path (probe returned ok 16/16). Dodge direction is ~5° off — not a suspect.
- vertical escape (the dodge is already mostly vertical; r = −0.077 with min_dist) —
  scored on patrol min_dist, so treat as unestablished rather than settled.
- **`evade_accel_ff_mps2`** — 12-trial in-flight A/B, no dodge gain (0.107 m pre-fix vs
  0.091 m post-fix; both arms commanded real motion). The 28/28 zero-metre result at settled
  hover does **not** reproduce in flight, so it never invalidated the Week 6 lane.
- **`WP_ACC` above 2.5** — flown 5/20/60 sweep. Peak demand 1.43 m/s² against a 5.0 clamp;
  at the 2.5 default it binds for only 7% of the window. Real effect only *downward*:
  starving it to 1.0 cuts escape ~3x.
- **`dodge_speed_mps`** — the shipped 1.5 is not costing saves. Within-session hover A/B,
  14 m/s, oracle 16 m, 10 repeats per arm: **1.5 → 6/10 · 3.0 → 6/9 · 6.0 → 6/10.** Holding
  the exponent pooled and fitting only per-arm scale, **4x the command buys 1.095x the
  escape** (log residuals −0.045/−0.001/+0.046 against sd 0.28–0.59 — inside noise). Escape
  goes as ~`command^0.065`; clearing 0.30 m by command alone would need ~350x shipped. This
  *supersedes* the old "1.5→4.0" null, which was scored on patrol escape displacement and
  was never safe; the conclusion survives, the evidence for it changed.

Also refuted as explanations: "sparse ball detections", "oblique aim geometry bug",
cloud mis-registration, and "2 m/s² is a hard limit" (the airframe does ~3 m/s²).

**Never cite the PSC_ACC_XY / WPNAV_ACCEL / ANGLE_MAX experiment** — it is invalid.
`--defaults` does not override `eeprom.bin`, and `PSC_ACC_XY` and `ANGLE_MAX` do not exist
in this build. Its conclusion is right anyway, for a different reason: `WPNAV_ACCEL` *does*
exist, renamed `WP_ACC`, and has since been swept properly and refuted (above).

**The real parameter table, fetched 2026-08-06** (read-only inventory of all 1382 params
from a throwaway SITL in its own directory; nothing was set, no `eeprom.bin` was touched).
This replaces guesswork about what the shaping limits are actually called:

| Name | Value | Note |
|---|---|---|
| `ATC_ANGLE_MAX` | **30** | **DEGREES in this build, not centidegrees.** `param set ATC_ANGLE_MAX 45`, never 4500. Permits g·tan(30°) = 5.66 m/s². |
| `PSC_ANGLE_MAX` | 0 | 0 = "use ATC_ANGLE_MAX", so the position controller has the full 30°. |
| `PSC_JERK_NE` | **5** | m/s³, horizontal. The XY→NE rename is why `PSC_JERK_XY` "does not exist". |
| `PSC_JERK_D` | 5 | m/s³, vertical — bounds a thrust-pop escape the same way. |
| `WP_ACC` | 2.5 | m/s², range 0.50–5.00. **The renamed `WPNAV_ACCEL`** — the same naming trap as `PSC_JERK_XY`→`PSC_JERK_NE`. `ModeGuided::pva_control_start()` passes it into the horizontal controller, so it bounds every GUIDED velocity setpoint. Swept and refuted; see the nulls above. |
| `PSC_ACC_XY`, `ANGLE_MAX`, `PSC_VELXY_P` | ABSENT | confirmed against the full table. `WPNAV_ACCEL` is **not** absent — it is `WP_ACC`. |

**`PSC_JERK_NE` is NOT the dodge-authority limit — REFUTED, do not re-run.** A flown
5/20/60 A/B settles it twice over. (1) *The limit is not being reached.* Pure jerk-limited
displacement `j·t³/6` at 5 m/s³ is 0.0225 m at t = 0.30 s; measured baseline escape is
0.103 m — **4.6x above its own jerk envelope**, and 9.2x at t = 0.20 s. A limit you already
operate far above is not binding, and that holds without any between-arm comparison.
(2) *The response is not monotonic.* Mean escape at t = 0.30 s: jerk 5 → 0.103 m,
20 → **0.077 m**, 60 → 0.137 m. The middle arm is lowest and within-arm spread exceeds any
between-arm difference. The parameter facts in the table above are verified and stand; only
this hypothesis is dead.

**Every flight-authority lever is now refuted, and one row retires them all together.**
Dataflash over the controlled `WP_ACC` sweep: **achieved Δv EXCEEDS commanded Δv in every
arm** — 0.93 vs 0.81, 1.26 vs 1.17, 1.47 vs 0.97 m/s. The vehicle over-delivers on what it
is asked for. That closes, at once: the airframe, the attitude loop, `ATC_ANGLE_MAX` (21–28°
used of 30 available), `PSC_JERK_NE`, and `WP_ACC`. The attitude/ACRO-step experiment that
used to be "the only maneuver-side question left" is answered without flying it.

**The dodge only ever asks for ~1 m/s of horizontal step** — and asking for more does not
help, because `dodge_speed_mps` plateaus (above). Useful dataflash facts while you are in
there: `GUIP.Type` 2 = position target (patrol), 4 = velocity target (dodge); `ATT.Roll` /
`Pitch` are already DEGREES, do not convert; the velocity target persists ~2.4 s, not the
1.0 s `dodge_duration_s`, so the ramp is never truncated.

So the bound is **sensor reach and nothing else.** The dodge needs tca ≥ ~0.83 s; in hover
tca is `range/speed − 0.177`; and no threshold, no parameter, and no command magnitude on
the maneuver side moves either term. Only a camera that sees the ball further out does —
`range = speed × (tca + 0.177)`. This is the same conclusion the file has carried since
Week 4, now with a number attached to it instead of an argument, and with the part named
(`docs/week6_result.md`).

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
| `docs/week6_result.md` | **The 20 m/s answer**: the tca threshold law, every refuted lever, and the sensor that closes the gap |
| `docs/weeks_5_9_plan.md` | Weeks 5-9 plan, split into hardware and software lanes with cross-lane gates |
| `docs/hardware_bringup.md` | Weeks 5–6 physical checklist: FC swap, radio bind, Pi power, payload wiring |
| `docs/bag_capture_runbook.md` | Bag re-capture + regression/tuning procedure (Dell) |
| `docs/dodge_battery_runbook.md` | Dodge battery + sweep procedure (Dell) |

Development history is in git, not in the tree: `git log -p docs/`.
