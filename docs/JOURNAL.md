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

### Still open (Week 2 gates)
- ★ W2-08/W2-09: velocity round-trip + ≥5 Hz / 3-s rule (Week 4 depends on this).
- ★ W2-06: stable ≥30 s hover with the tuned `huitzilin_3p5_6s.parm`.
- W2-14: RViz mission markers. W2-05: forked duct model (Pass B).
- ★ W2-18: fresh-checkout reproduction from docs alone.

### Versions
- Same as Week 1 (ROS 2 Jazzy, Gazebo Harmonic 8.11.0, ArduPilot main, Python 3.12.3)