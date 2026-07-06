# Project Journal — HuitzilinReflex

## Week 1 — 2026-06-15

### What was accomplished
- Completed all design docs: requirements, node graph, message contracts, state machine, frames, safety case
- Installed and verified: ArduPilot SITL, Gazebo Harmonic, ardupilot_gazebo plugin, ros_gz bridge, QGroundControl
- First scripted flight via pymavlink: arm → takeoff → hold position confirmed in QGC
- ROS 2 mavlink_bridge node built and verified receiving /cmd/evade commands

### What worked
- WSL2 + IntelliJ Remote Development workflow solid for coding
- SITL + Gazebo headless connection stable
- pymavlink connecting via udp:127.0.0.1:14551

### What didn't work / workarounds
- Gazebo GUI runs at ~24% real-time due to Intel Iris Xe GPU not supported in WSL2 — running headless as workaround
- ROS 2 node needed pymavlink installed to system Python (`sudo pip install pymavlink --break-system-packages`)
- arm throttle force needed manually in MAVProxy before script can arm

### Open questions for Week 2
- NED/ENU conversion handling in the bridge
- Command stream rate for position targets
- How to automate the force arm for scripted testing

### Versions
- ROS 2: Jazzy
- Gazebo: Harmonic 8.11.0
- ArduPilot: latest main
- Python: 3.12.3

## Week 2 — 2026-06-17

### What was accomplished
- `huitzilin_sim` stack flies an autonomous closed patrol loop in SITL through our
  own ROS 2 ↔ pymavlink bridge: `arm → takeoff (2 m) → 5×5 m square → loop`.
- One-command bring-up via `week2_sitl.launch.py` (bridge + patrol + telemetry logger).
- Services working: `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/start_patrol`.

### Evidence (W2-13/15/17)
- 43 full laps, ~21 min continuous, **mean lap 29.51 s, stdev 0.93 s** — no drift.
  Details in `docs/week2_patrol_evidence.md`.
- Per-leg ~7–9 s over 5 m (~0.6 m/s effective; position mode decelerates into the
  0.6 m accept radius). CSV → `scripts/plot_telemetry.py` for the ground-track figure.

### The takeoff bug — root cause and fixes
The drone armed but would not climb (`takeoff accepted` then `Thr 100`, zero lift).
After a long hunt the real cause was **`FRAME_CLASS=0` on a fresh EEPROM** — with no
airframe defined, ArduPilot never mixes the motors for a quad, so throttle maxes with
no lift (`PreArm: Motors: Check frame class and type`). Fix: `FRAME_CLASS=1`/
`FRAME_TYPE=1`, persisted via `params/sitl_frame.parm` loaded with `--add-param-file`
(the full Pass-A set lives in `config/huitzilin_3p5_6s.parm`, a superset).
Secondary issues found and fixed along the way:
- **Telemetry port mismatch:** bridge listens on `udpin:14552`, patrol on `:14553`;
  SITL must fan out `--out udp:…:14552 --out udp:…:14553` or nodes get no heartbeat.
- **`patrol autostart: true`** flooded GUIDED with position setpoints during takeoff →
  set to `false`; start patrol via the service after takeoff (commit 26a2264).
- **Takeoff timeout** was wall-clock 30 s ≈ 7 sim-s at 24% real-time → bumped to 90 s.
- Misc: never use `Ctrl-Z` on a launch (suspended job holds the SITL TCP socket);
  restarting Gazebo+SITL together clears a half-broken FDM link.

### Answers to Week 1 open questions
- **NED/ENU:** single conversion boundary in `mav_bridge` (`ned_to_enu`/`enu_to_ned`);
  topics are ENU, MAVLink/patrol setpoints are NED (`d` negative = up).
- **Command rate:** patrol streams position setpoints at 10 Hz (W2-09 gate pending).
- **Automate force-arm:** solved by fixing the frame instead of forcing — bridge arms
  cleanly via `/huitzilin/arm` once EKF/GPS is ready.

### Week 2 gate status (2026-06-18)
- ✓ W2-01: Week 1 baseline verified (heartbeat sys=1 comp=0).
- ✓ W2-02: huitzilin_sim builds clean (removed spurious src/src/ duplicate).
- ✓ W2-03: MAVLink endpoint confirmed udpin:0.0.0.0:14552, documented in bridge.yaml.
- ✓ W2-04: Pass-A params loaded; fixed parm file (wrong param names: WP_SPD not WPNAV_SPEED).
- ⏸ W2-05: Deferred — iris_with_standoffs used as-is; real mass/motor tune deferred to Week 7-8.
- ✓ W2-06: Stable hover ±0.01m variance at 2.19m for 30s.
- ✓ W2-07: mav_bridge.py selftest passed (arm→takeoff→nudge→land). Force arm fix applied.
- ✓ W2-08: Velocity round-trip confirmed (~120ms sim-time at 24% real-time).
- ✓ W2-09: ~6 Hz sim-time setpoint rate, no AP timeout.
- ✓ W2-10: Bridge node publishing /huitzilin/odom and /huitzilin/state.
- ✓ W2-11: Fail-safe hold confirmed — drone holds on cmd_vel dropout.
- ✓ W2-12: patrol.yaml defines 5×5m square at 2m altitude.
- ✓ W2-13: patrol_node looping cleanly through all 4 waypoints.
- ✓ W2-14: RViz mission markers showing 4 green spheres.
- ✓ W2-15: Telemetry CSV logging confirmed (2148 rows).
- ✓ W2-16: One-command launch via week2_sitl.launch.py.
- ✓ W2-17: Rosbag recorded (week2_patrol_0.mcap).
- ⏳ W2-18: Fresh-checkout validation in progress (teammate testing).
- ✓ W2-19: Retro and Week 3 handoff written (see below).

### Week 2 Retro

**What worked**
- ROS 2 ↔ pymavlink bridge solid; arm/takeoff/patrol all scripted via services.
- Position-mode patrol extremely stable — 43 laps, mean lap 29.51s, stdev 0.93s.
- Telemetry logger + rosbag pipeline ready for Week 3.
- Force-arm fix (param2=21) eliminates the manual MAVProxy workaround.

**What was harder than expected**
- FRAME_CLASS=0 on fresh EEPROM caused silent no-lift bug (fix: always load sitl_frame.parm).
- Param names differ from playbook (WP_SPD not WPNAV_SPEED; PSC_* not present).
- Inline comments in .parm files break the MAVProxy parser — use comment-only lines.
- Port contention: each node needs its own --out UDP port from SITL.
- src/src/ duplicate caused colcon build failure after git pull.

**What's deferred**
- W2-05 Pass-B model fidelity (real mass/inertia/motorConstant) — deferred to Week 7-8.
- W2-18 fresh-checkout validation — teammate testing in progress.

### Week 3 Handoff

Week 3 (perception pipeline) inherits:
- A working autonomous patrol loop in SITL via `ros2 launch huitzilin_sim week2_sitl.launch.py`.
- A stable bridge with `/huitzilin/odom`, `/huitzilin/state`, `/huitzilin/cmd_vel` contracts.
- Rosbag + CSV telemetry tooling ready for labeled scenario capture.
- The body-velocity path (send_velocity_body) proven and ready for the Week 4 evasion reflex.
- A Gazebo model to hang a depth sensor on (iris_with_standoffs, Pass-B fidelity deferred).
- Open: W2-05 Pass-B model, W2-18 fresh-checkout sign-off.

### Versions
- Same as Week 1 (ROS 2 Jazzy, Gazebo Harmonic 8.11.0, ArduPilot main, Python 3.12.3)
---

## Week 3 — 2026-06-21

### Carry-over close-out (W3-02)

**W2-18 (fresh-checkout sign-off):** No second machine / teammate available for a formal
sign-off run. Logging the gap explicitly: the week2_sitl.launch.py bring-up procedure is
fully documented in `docs/SETUP.md`; the acceptance evidence (43 laps, mean 29.51 s) is in
`docs/week2_patrol_evidence.md`. W2-18 is parked as "evidence documented, formal sign-off
deferred to Week 4 when the native Dell box runs the perception stack end-to-end."

**W2-05 (Pass-B model fidelity):** Decision confirmed — depth sensor is hung on the stock
`iris_with_standoffs` Gazebo model as-is. Real mass / inertia / motorConstant tuning stays
in Week 7–8. The detection algorithm must not depend on exact airframe dynamics.

### Week 3 goal

Detection node flags ≥ 95% of simulated incoming clusters with a quantified false-positive
rate, scored repeatably against a labeled rosbag library, reproducible from a fresh checkout.

### Environment reminder

All depth-sensor rendering done on the native Dell Inspiron 3670 (i5-8400, UHD 630,
native Ubuntu 24.04). WSL2 / Iris Xe box cannot render Gazebo depth at rate. All timing
gates measured in sim time via `/clock` and message stamps — never wall-clock.

---

## Week 3 — 2026-07-06: 60%-recall regression root-caused (egomotion)

Train-split regression on the Dell: recall 60% (S01/S04/S07/S08 FN), floor 95%.
DEBUG_FUNNEL showed ~half of all frames dying at `fg > fg_max_points=5000`.

### Root causes (three independent defects)

1. **`compensate_egomotion` was declared but never implemented.** The detector
   subscribed `/huitzilin/odom`, stored the message, and never read it. Background
   differencing therefore ran in the *moving camera frame*: at ~2 m/s patrol the
   whole scene shifts ≈ `diff_threshold_m` (0.15 m) per frame, plus metres under
   pitch/yaw, so the entire cloud became "foreground" and the flood guard threw
   away the very frames containing the ball. Detection succeeded only when the
   ball coincided with a momentarily steady camera — which is exactly why the
   slow (S01), oblique (S04), and near-miss (S07/S08) scenarios failed.
   **Fix:** odom is folded into the tf buffer (`odom → base_link`) and clouds are
   re-expressed in `odom` before differencing (`fixed_frame` param). Pure math
   extracted to `cloud_geometry.py` with unit tests proving camera-frame
   differencing floods (>20% fg from egomotion alone) while odom-frame stays <1%.

2. **`/huitzilin/odom` carried no orientation** (all-zero quaternion — the ROS
   message default) and only 10 Hz. `mav_bridge` already received `ATTITUDE` from
   SITL but never packed it. **Fix:** `MavBridge.ned_rpy_to_enu_quat()` (NED/FRD →
   ENU/FLU, unit-tested), `get_state()` caches last-known telemetry (recv_match is
   lossy), `stream_rate_hz` 10 → 30 so odom outpaces the 15 Hz depth cloud.
   **Consequence: all existing bags lack attitude → the detector detects the
   invalid quaternion and falls back to the old camera-frame mode. The bag
   library must be re-captured with the new mav_bridge before the regression can
   pass.**

3. **`score_bags` booked a missing bag as a false positive.** N04 (never captured
   — needs a ≥ 12 m flight) produced "FP=1 / FP rate 25%" on a run with zero
   actual false positives. **Fix:** missing bags are now harness errors, excluded
   from the confusion matrix, and fail the run explicitly.

Also fixed: `rcl_shutdown already called` traceback on every harness teardown
(guard `rclpy.ok()` + catch `ExternalShutdownException`), and a booby-trapped git
index on the Windows box (stale pre-W3-13 file versions staged — a blind commit
would have reverted the detector; defused with `git reset`).

### Verification
- 12 unit tests green (`test_cloud_geometry.py`, `test_frames.py`) — run on the
  Windows box without ROS; they run under `colcon test` too.
- Live regression still requires the Dell: rebuild, re-capture S01–S10/N01–N03
  with attitude-bearing odom, capture N04 at `takeoff_alt_m: 12.0`, re-run
  `./scripts/run_regression.sh /data/huitzilin_bags train`.
