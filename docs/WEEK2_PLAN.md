# HuitzilinReflex — Week 2 Reconciled Plan

*Generated 2026-06-15. This is the execution plan for Week 2, reconciling the two source docs (`HuitzilinReflex_Week2_Playbook.docx` and `HuitzilinReflex_Week2_Checklist.md`) against the **actual repo** and the **actual dev environment**. Where the playbook and your Week 1 work disagree, the resolution is stated here. Decision on record: **follow the playbook contracts** (new `huitzilin_sim` package, `/huitzilin/*` topics) rather than Week 1 naming.*

---

## 1. Where we are

Week **2 of 9**, Phase A (Foundations & Simulation). Everything this week is **pure simulation — no hardware, no props, no real propeller spins.** The whole week drives toward one provable sentence.

> **Week 2 Definition of Done:** *The drone autonomously flies a closed patrol loop in Gazebo, driven through our ROS 2 → pymavlink bridge, with telemetry logged to a rosbag and a CSV you can replay/plot — demonstrated on a fresh checkout.*

**The bright line:** patrol only. No perception, no OAK-D, no evasion this week — that is Weeks 3–4. If a task tempts you toward detecting or dodging anything, it's misfiled.

Stack (pinned, from Week 1): Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · ArduPilot Copter 4.5+ SITL · pymavlink. SITL is board-agnostic, so none of this depends on the Week 5 flight-controller swap.

---

## 2. Reconciliation — playbook vs. your actual repo

The playbook was written against an idealized layout. Your Week 1 repo differs in a few concrete ways. These are resolved up front so they don't surface mid-week as "bugs."

| # | Item | Playbook assumes | Your repo (Week 1, as-built) | Resolution for Week 2 |
|---|---|---|---|---|
| R1 | Workspace | A separate `~/huitzilin_ws/` colcon workspace | The repo root **is** the colcon workspace; packages live in `src/` (`src/mavlink_bridge`, plus stray `src/__init__.py`) | Build from the **repo root**: `colcon build --symlink-install`. Mentally map every `~/huitzilin_ws/` in the playbook to the repo root. |
| R2 | Our package | New `huitzilin_sim` (ament_python) | Existing `mavlink_bridge` package with `mavlink_bridge_node.py` | **Create `src/huitzilin_sim/`** as the canonical Week 2 package. Migrate the useful Week 1 node logic into `mav_bridge_node.py`. Leave `mavlink_bridge` in place until `huitzilin_sim` is green, then retire it in its own commit. |
| R3 | Command topic | `/huitzilin/cmd_vel` (Twist) | Week 1 node subscribes `/cmd/evade` | Adopt `/huitzilin/cmd_vel`. **`/cmd/evade` is a Week 4 evasion seam** — park it, don't wire it this week. |
| R4 | MAVLink endpoint | `udp:127.0.0.1:14550` (fallback `tcp:5760`) | `docs/SETUP.md` launches SITL with a **single `--out udp:127.0.0.1:14551`**; `first_flight.py` and the Week 1 node both bind `:14551` | The playbook's `14550` is **wrong for this repo** — your working port is `14551`. Because that single `--out` is shared, QGC + MAVProxy + the bridge fought over it (Week 1 "heartbeats vanish"). Fix: add a **second `--out udp:127.0.0.1:14552`** to the `sim_vehicle.py` line and point the bridge at `:14552`, leaving `:14551` for QGC/MAVProxy. Record the final string in `params/bridge.yaml`. |
| R5 | Frames | NED inside bridge; ENU on `/odom`; body-FLU on `cmd_vel`; **single conversion point** | (open question in Week 1 journal) | Adopt as written. All NED↔ENU math lives **only** in the bridge. This is also what Week 4 evasion needs ("dodge left" is a body concept). |
| R6 | Arming | `arm()` after `wait_ekf_ready()` | Week 1 needed a **manual force-arm** in MAVProxy before scripts could arm | Solve scripted arming in W2-07: wait for EKF/GPS ready first, and/or relax SITL `ARMING_CHECK` for dev runs. Avoid blind force-arm (`param2=21196`) — fix the pre-arm cause. Closes a Week 1 open question. |
| R7 | pymavlink install | "latest" | Week 1 installed to system Python via `pip install pymavlink --break-system-packages` | Document this in `SETUP.md` and declare the dep so the **fresh-checkout** test (W2-18) actually reproduces. |

---

## 3. Environment reality & timing caveats (read before Day 3)

Your dev box is **WSL2 on Windows with an Intel Iris Xe GPU**. There's no GPU passthrough in WSL2, so (per the Week 1 journal) **Gazebo runs headless at ~24% real-time.** That single fact changes how several Week 2 gates must be judged:

- **Judge timing gates in *sim time*, not on a wristwatch.** At ~0.24× real-time, the W2-08 gate ("first motion < 200 ms") and the W2-09 gate ("≥ 5 Hz command rate") will look ~4× off if you measure wall-clock. Run the ROS nodes with `use_sim_time` consistent across the graph, drive measurements off `/clock` and the telemetry timestamps, and evaluate the gates against **simulation timestamps** (or rosbag stamps). Otherwise you'll "fail" a gate that actually passes.
- **The ArduPilot ~3 s setpoint timeout is in autopilot/sim time.** Good news: streaming setpoints at 10 Hz wall-clock means ArduPilot sees them very frequently in sim time, so the timeout won't bite — *as long as the send loop stays tight and non-blocking*. A blocking call in the send path is what makes the 3 s rule bite intermittently. Keep `_tick_setpoint` lean.
- **Stay headless; visualize in RViz.** Run Gazebo server-only and do the W2-14 patrol-loop visualization in **RViz markers** (lighter on the GPU than the Gazebo GUI). The playbook's "Gazebo runs at 2 fps → enable GPU accel" troubleshooting row has **no fix on this hardware** — headless + RViz is the workaround, not chasing GL acceleration.
- **Run the final demo patiently.** A 2-lap patrol at 0.24× takes ~4× wall-clock. Budget for it; don't shorten the loop just to make the wall-clock run feel faster.

---

## 4. Target layout (what we're building toward, in this repo)

```
HuitzilinReflex/                      # repo root = colcon workspace
├── src/
│   ├── mavlink_bridge/               # Week 1 (retire after huitzilin_sim is green)
│   └── huitzilin_sim/                # NEW — ament_python, all Week 2 code
│       ├── package.xml  setup.py  setup.cfg
│       ├── huitzilin_sim/
│       │   ├── mav_bridge.py         # pure pymavlink helper (no ROS)
│       │   ├── mav_bridge_node.py    # ROS 2 wrapper + watchdog
│       │   ├── patrol_node.py        # patrol path-follower
│       │   └── telemetry_logger.py   # CSV logger
│       ├── launch/  week2_sitl.launch.py
│       ├── params/  bridge.yaml  patrol.yaml
│       ├── models/  huitzilin_duct/  # forked & tuned iris (Pass B)
│       ├── config/  huitzilin_3p5_6s.parm
│       └── worlds/  patrol_yard.sdf  # optional bounded world
└── docs/  (architecture, requirements, safety_case, frames, state_machine, JOURNAL.md, WEEK2_PLAN.md)
```

Full runnable scaffolds for every file above are in the source checklist §7–§10 / playbook Appendix E. **Extend those scaffolds; don't rewrite from zero.** If you change a topic/type/service name, update the contracts (§2 R3–R5 here, and the playbook §3) in the same commit.

---

## 5. The week, day by day (W2-01 … W2-19)

Ordered by dependency. `★` = hard gate; don't proceed past it unchecked. Each day ends on a gate.

### Day 1 (Mon) — Verify, scaffold, baseline · *Lane A, all hands*

- [ ] **W2-01** Run the Week 1 preflight; confirm the baseline still flies. `ros2 launch ardupilot_gz_bringup iris_runway.launch.py` → quad spawns, SITL connects, holds. A one-line pymavlink check prints a HEARTBEAT.
  - ★ **Gate:** baseline launches and holds on a fresh checkout. *If red, it's a Week 1 regression — fix first, full stop.*
- [ ] **W2-02** Scaffold `src/huitzilin_sim/` (ament_python; deps `rclpy geometry_msgs nav_msgs std_msgs std_srvs visualization_msgs`). Lay out `launch/ params/ models/ config/ worlds/`. Register entry points **and `data_files`** in `setup.py` now (forgetting `data_files` is why launch/param files "vanish" after install). `colcon build --symlink-install` from the repo root, commit the empty scaffold.
- [ ] **W2-03** Confirm & document the endpoint (see R4). Verify a second MAVLink client attaches; choose the string (recommend a dedicated `--out=udp:127.0.0.1:14552`); record it in `params/bridge.yaml` and the journal.
  - ★ **Gate (Day 1):** fresh checkout launches the sim *and* a one-line pymavlink script prints a heartbeat from the chosen endpoint.

### Day 2 (Tue) — Airframe tuning · *Lane A*

Make the sim behave like the real 3.5″ duct (CineLog35 V2: 142 mm, 6S, SPEEDX2 2105.5, D-T90). **Two passes — do Pass A first so Lanes B/C aren't blocked.**

- [ ] **W2-04** *Pass A (params-only, fast).* Create `config/huitzilin_3p5_6s.parm`: `FRAME_CLASS=1` (quad), `FRAME_TYPE=1` (X), `MOT_THST_EXPO≈0.55`, `MOT_HOVER_LEARN=2`, `INS_GYRO_FILTER≈80–120 Hz`, conservative `WPNAV_SPEED/ACCEL/SPEED_UP/DN`. Load into SITL; re-confirm arm → takeoff → hold.
- [ ] **W2-05** *Pass B (model fidelity, can run parallel to Days 3–4).* Fork `iris_with_ardupilot` into `models/huitzilin_duct/`; edit mass (~0.55–0.75 kg est. — **note it's an estimate** until the real airframe is weighed in Week 5), arm length ≈ 0.071 m, inertia (scale from iris, `Izz≈2·Ixx`), `motorConstant` for ~1.0–1.3 kgf/motor. Re-verify hover. *Lower priority than the bridge — fine to slip into the weekend buffer.*
- [ ] **W2-06** Validate a stable hover: arm, climb to 2 m, hold ≥ 30 s, watch for oscillation/sag. Commit params/model.
  - ★ **Gate (Day 2):** stable ≥ 30 s hover with tuned params (Pass A minimum), committed. *(A slow up-down "breathing" is usually mass/thrust mismatch, not PIDs — fix mass first.)*

### Day 3 (Wed) — The pymavlink bridge · *Lane B (the heart of the week)*

- [ ] **W2-07** Drop in `mav_bridge.py` (pure pymavlink: connect, set_mode, arm, takeoff, send_velocity_body, send_position_ned, get_state, NED↔ENU helpers). Solve scripted arming per R6. Run standalone: `python3 -m huitzilin_sim.mav_bridge --connect <endpoint> --selftest` → connect → EKF wait → GUIDED → arm → takeoff 2 m → 1 m/s nudge → LAND.
- [ ] **W2-08** ★ **Velocity round-trip:** command `vx=1.0 m/s` body-frame; confirm measured `LOCAL_POSITION_NED.vx` tracks it; **first motion < 200 ms in sim time** (see §3). Log the commanded-vs-measured trace.
- [ ] **W2-09** ★ **Command rate / 3 s rule:** measure send rate + jitter; **≥ 5 Hz steady** (run 10 Hz); no AP setpoint-timeout, no stutter. Measure against sim time (§3).
- [ ] **W2-10** Wrap as `mav_bridge_node.py`: subscribe `/huitzilin/cmd_vel`, publish `/huitzilin/odom` + `/huitzilin/state`, offer `/huitzilin/arm` (SetBool) / `/huitzilin/takeoff` (Trigger) / `/huitzilin/set_mode`. All NED↔ENU conversion stays **inside the bridge**. Load `params/bridge.yaml`.
- [ ] **W2-11** Wire fail-safe: watchdog re-sends last `cmd_vel` at `cmd_rate_hz`; if stale beyond `cmd_timeout_s` → **zero-velocity hold** (calm hold, never a coast/lunge). Mode services flip LOITER/RTL/LAND.
  - ★ **Gate (Day 3):** `ros2 topic pub /huitzilin/cmd_vel …` repeatably moves the sim drone; `/huitzilin/odom` streams; round-trip + rate checks logged as passing; stopping the pub triggers the zero-hold.

### Day 4 (Thu) — Patrol path-follower · *Lane C (parallel to B since Tue via a mock bridge)*

- [ ] **W2-12** Define `params/patrol.yaml`: a closed loop (e.g. 5 m × 5 m square at 2 m alt → `d = -2.0`), `accept_radius_m≈0.6`, `cruise_speed_ms≈1.5`, `loop: true`. **Waypoints are NED — a +2 m altitude is `z = -2.0`; mixing the sign flies it into the ground.**
- [ ] **W2-13** Drop in `patrol_node.py`. Default to **position-target mode** (sends `SET_POSITION_TARGET_LOCAL_NED` position setpoints; leans on Day-2 `WPNAV_*` tuning — simplest, robust). Logs "reached WP n" per corner and loops. Guard: patrol only in GUIDED + armed + EKF-ok. *(Velocity-pursuit mode is optional, to exercise the full velocity path for Week 4.)*
- [ ] **W2-14** Publish `/huitzilin/mission_marker` (MarkerArray) and check the loop in **RViz** (markers + drone track). Mirrored/rotated markers ⇒ NED→ENU is off — fix it in the bridge, not the marker code.
  - ★ **Gate (Day 4):** starting patrol flies the drone corner-to-corner; node logs "reached WP n" for each corner, at least once around.

### Day 5 (Fri) — Close the loop, log it, prove the DoD · *Lane D leads*

- [ ] **W2-15** Telemetry: record a rosbag (`/huitzilin/odom /huitzilin/cmd_vel /huitzilin/state /huitzilin/mission_marker`) **and** a CSV via `telemetry_logger.py`. Record *before* you fly, not after the interesting moment.
- [ ] **W2-16** `week2_sitl.launch.py` one-command bring-up (bridge + patrol + logger). Keep the sim in its own terminal. Confirm `data_files` installed the params so launch finds them.
- [ ] **W2-17** End-to-end recorded run from a **fresh shell**: arm → takeoff → autonomous patrol → ≥ 2 laps → induce a `cmd_vel` dropout (confirm hold) → RTL/LAND. Capture a screen recording + the bag/CSV.
- [ ] **W2-18** ★ **Fresh-checkout validation (the real DoD):** a teammate clones on a clean Ubuntu 24.04 + Jazzy machine and reproduces the patrol loop from the docs alone. Log every gap; fix docs/scripts until it works unaided.
- [ ] **W2-19** Update `docs/JOURNAL.md` (what worked, exact params/commits, Pass A/B reached); record the Week 2 vlog; write the Week 3 handoff note.
  - ★ **Gate (Week 2 DoD):** fresh checkout → launch → autonomous closed patrol loop in Gazebo via the ROS 2→pymavlink bridge, **telemetry logged**.

**Buffer (weekend):** harden flaky bits, finish Pass-B model fidelity if it slipped, expand the acceptance matrix. **Do not** start Week 3 perception early at the cost of an un-demonstrated Week 2 DoD.

---

## 6. Acceptance matrix (prove the week)

Run top-to-bottom on Day 5, then again on a fresh checkout for sign-off. Every row pass/fail; capture evidence (plot PNG, "reached WP" log lines, bag name) in the commit/journal.

| # | Test | Pass criterion |
|---|---|---|
| 1 | Fresh-checkout build | `colcon build` clean from repo root; entry points present |
| 2 | Sim baseline | quad spawns, SITL connects, holds position |
| 3 | Bridge connects | `/huitzilin/odom` publishes; `/huitzilin/state` shows EKF ok |
| 4 | Arm/takeoff service | drone arms and climbs to `takeoff_alt_m` |
| 5 | Velocity round-trip | measured vx tracks command; first motion < 200 ms **(sim time)** |
| 6 | Command rate | ≥ 5 Hz steady; no AP timeout / no stutter **(sim time)** |
| 7 | Fail-safe hold | after `cmd_timeout_s`, drone holds (zero vel), no coast/lunge |
| 8 | Patrol loop | flies all corners, logs "reached WP n", **loops ≥ 2 laps** |
| 9 | Drift bound | stays within `accept_radius_m` of each corner; loop closes |
| 10 | Telemetry | rosbag + CSV written; quick-plot renders loop + vx trace |
| 11 | Clean shutdown | LAND/RTL descends/returns calmly; disarms; no flyaway |
| ★ | **WEEK 2 DoD** | fresh checkout → autonomous closed patrol loop via the bridge, telemetry logged |

---

## 7. Week 1 open questions → closed by Week 2

The Week 1 journal left three open questions; here's where each is resolved:

- *NED/ENU conversion handling in the bridge* → **R5** + W2-10: single conversion point inside the bridge; NED in, ENU on `/odom`, body-FLU on `cmd_vel`.
- *Command stream rate for position targets* → W2-09: ≥ 5 Hz (run 10 Hz), watch the ~3 s timeout, measured in sim time (§3).
- *How to automate the force arm for scripted testing* → **R6** + W2-07: wait for EKF/GPS ready and/or relax SITL `ARMING_CHECK`; avoid blind force-arm.

---

## 8. Recommended first actions (start here)

1. **Sanity-check the floor (W2-01).** From a fresh shell, source the stack and launch `iris_runway.launch.py`. If it doesn't hold position, stop and fix Week 1 before anything else.
2. **Pin the endpoint (W2-03).** Confirm what SITL actually emits and whether Week 1's `14551` or a dedicated `14552 --out` is cleanest; write it into `params/bridge.yaml`.
3. **Scaffold `huitzilin_sim` (W2-02)** and commit the empty package — gives Lanes B and C a place to land code.
4. Then Lane B starts the bridge (W2-07) and Lane C develops the patrol against a mock bridge in parallel.

---

## 9. Sources

- `HuitzilinReflex_Week2_Checklist.md` (repo) — full task list + runnable starter code (§7–§10).
- `HuitzilinReflex_Week2_Playbook.docx` (repo) — task cards W2-01…W2-19, acceptance matrix, Appendix E code.
- `HuitzilinReflex_v2.md` (repo) — master project doc, 9-week roadmap, hardware BOM.
- `docs/JOURNAL.md` (repo) — Week 1 result, environment notes, open questions.
- ArduPilot ROS 2 + Gazebo: https://ardupilot.org/dev/docs/ros2-gazebo.html · Guided-mode commands (3 s rule, type_mask): https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

*Week 2 of 9. "Loiter like a hover, dodge like a dart." This week we earn the loiter; the dart comes later. Patrol first, reflexes after.*
