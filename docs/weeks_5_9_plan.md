# Weeks 5–9 Plan — Hardware / Software Split

Weeks 1–4 are closed (`CLAUDE.md`). Everything from here runs on two lanes that
progress in parallel and block each other at named points:

- **HW lane (`H<week>-<n>`)** — physical: soldering, flashing, wiring, mounting, binding.
- **SW lane (`S<week>-<n>`)** — code, config, measurement, tuning, docs.

The lanes are separated because they are differently gated. The HW lane is gated by
*parts and hands*; the SW lane is gated by *measurements that only exist once the HW
lane reaches a specific step*. Most SW work in Weeks 5–6 does **not** need a flyable
aircraft, and is listed accordingly so it can start immediately.

## What this doc owns vs. what it defers

| Content | Owner |
|---|---|
| Week 5–6 physical steps, step-by-step | `docs/hardware_bringup.md` §1–9 — **not restated here** |
| Week 7–9 physical steps | this doc (nowhere else covers them) |
| The full software track, Weeks 5–9 | this doc |
| Cross-lane blocking, per-week DoD | this doc |
| Safety/legal rules | `docs/SAFETY_CASE.md` (binding; this doc never overrides it) |
| Node/topic contracts | `docs/architecture.md` |
| Week 1–4 results and measured nulls | `CLAUDE.md` |

**Cut order if behind** (from `HuitzilinReflex_v2.md` §5): Weeks 1–5 are sacred. Trim
real evasion flights (Week 8) first — demo evasion in HITL rather than rush an unsafe
test. In lane terms: the SW lane never gets cut before the HW lane, because a demo that
runs in HITL still needs every Week 5–7 software item.

---

## The three findings that shape this plan

Recorded here because each one changes what the weeks contain, and each was found by
auditing the tree rather than by reading the roadmap.

**1. Two promised nodes do not exist.** `docs/architecture.md` lists `supervisor_node`
as phase "Wk2+" and `payload_node` as "Wk6". Neither is in `src/`. `supervisor_node` is
the component that owns every fault response in `SAFETY_CASE.md` §1 — sensor dropout,
Pi brownout, GUIDED setpoint-stream loss — and `docs/state_machine.md` describes a state
machine that nothing currently implements. Today `evasion_node` publishes
`/payload/alarm` to no subscriber. Both are scheduled below: supervisor in Week 5
(it blocks Week 7), payload in Week 6.

**2. The whole stack is pinned to sim time.** `evasion_node.py` states "All timing is
SIM time"; commit `cceb0d5` deliberately put the flight nodes on `/clock`, and
`detector.yaml` ships `use_sim_time: true`. Real hardware publishes no `/clock`. Every
timing gate in the system — `latency_budget_s` 0.15, `track_timeout_s` 0.5,
`patrol_handoff_s` 0.8, `cmd_timeout_s` 0.7, `bg_map_ttl_s` 20.0 — is currently
denominated in sim seconds. This is the quietest available way for the stack to
misbehave on hardware, so it is a Week 5 gate (S5-4), not a Week 7 discovery.

**3. Nothing has ever run on arm64.** All Week 1–4 numbers come from x86 (WSL2 laptop,
native-Ubuntu Dell). The 2026-07-27 profiling found `voxel_downsample` (66→5-17 ms) and
`cluster_all` (12-71→2-20 ms) were the entire latency overrun on x86; frames now land at
14–55 ms. The Pi 5 will not reproduce those numbers. Week 4's conclusion that latency is
*not* the binding term rests on tca (0.18–0.29 s) exceeding latency — on the Pi that
margin has to be re-earned, not assumed (S6-7).

---

## Cross-lane dependency map

```
W5  H5-1 FC swap ──┬─> H5-2 flash/config ──> H5-3 bind ──> H5-6 bench arm
                   └─> H5-4 Pi power ──> H5-5 mount ──────────┐
    S5-1 Pi deploy ─────────────────────────────────────┐     │
    S5-2 hw config ──> S5-3 param file ──> (needs H5-2) │     │
    S5-4 clock audit ──> blocks EVERY later SW item ────┤     │
    S5-5 supervisor ────────────────> blocks W7 ────────┤     │
                                                        v     v
W6  S6-1 payload_node ──> H6-1 payload wiring ──> S6-2 latency check
    H5-4+H5-5 ──> S6-3 depthai bring-up ──> S6-4 RANGE/RATE MEASUREMENT
                                            └──> S6-5 real bag library
                                                 └──> S6-6 KF re-tune
                                            S6-7 Pi profiling
    H6-2 Remote ID / registration (independent; blocks all outdoor flight)

W7  H5-6 + S5-5 + S6-6 ──> S7-1 HITL launch ──> H7-1 tethered hover
                                            ──> S7-4 Week 8 scoring decision

W8  H7-1 + H6-2 ──> H8-1 enclosure ──> S8-2 real throw campaign

W9  everything ──> S9-1 validation matrix ──> S9-3 post-mortem
```

---

# Week 5 — FC swap, avionics & power

**Theme:** the one fabrication step. The HW lane is the critical path; the SW lane is
the largest single block of work in the remaining plan and is *not* blocked by it —
S5-1, S5-2, S5-4 and S5-5 can all start on day one against SITL.

## HW lane — Week 5

Physical steps are already written step-by-step in `docs/hardware_bringup.md`. Do not
duplicate them here; work the checklist there and tick these gates.

- [ ] **H5-1 — FC swap (F722 → MicoAir H743 V2 AIO).** Run `hardware_bringup.md` §1 end
      to end. Gate: every joint visually re-checked against the reference photos, no
      shorts across ESC pads. *Blocks: H5-2, H5-4.*
      - The §0 pre-steps are non-optional: photograph stock wiring first, battery
        disconnected throughout, static-safe surface.
      - Label motor corners before desoldering — ArduPilot quad-X mapping depends on it.
      - This is the one step worth outsourcing to an FPV shop if fine-pitch soldering
        isn't comfortable. Everything in Weeks 5–9 assumes it is done.
- [ ] **H5-2 — Flash & configure ArduPilot.** `hardware_bringup.md` §2. Gate: all four
      motors spin in correct order and direction under the GCS motor test, props off.
      *Blocked by: H5-1. Blocks: H5-3, H5-6.*
      - `FRAME_CLASS=1` / `FRAME_TYPE=1` set explicitly. The SITL sharp edge
        (`FRAME_CLASS=0` = silent no-lift, arms and accepts takeoff with zero lift)
        applies identically to the real board.
      - A backward motor is fixed by swapping two of its three phase wires, not by
        remapping in firmware.
      - Load the parameters from S5-3's file, not by hand — hand-entry is how a safety
        param goes missing.
- [ ] **H5-3 — Bind radio + failsafes + kill-switch.** `hardware_bringup.md` §3. Gate:
      RC-loss failsafe demonstrably fires with props off, and the kill-switch is on its
      own dedicated momentary switch — never a shared/mode switch (`SAFETY_CASE.md` §3).
      *Blocked by: H5-2. Blocks: H5-6.*
- [ ] **H5-4 — Pi power off the flight battery.** `hardware_bringup.md` §4. Gate:
      multimeter reads clean 5.0–5.2 V at the regulator output under no load, *before*
      the Pi is connected. *Blocked by: H5-1 (battery leads). Blocks: H5-5, S6-3.*
      - Pololu D24V50F5 input to the 6S rail, never to the FC's own rail.
      - Output to the Pi's 5 V/GND **GPIO pins**, not USB-C — GPIO bypasses PD
        negotiation, which is exactly why `usb_max_current_enable=1` is required
        (S5-1) or the OAK-D gets starved.
- [ ] **H5-5 — Mount the companion stack.** `hardware_bringup.md` §5. Gate: Pi + OAK-D
      mounted on M2.5 standoffs, USB-3 cable routed clear of prop wash and the ELRS
      antenna. *Blocked by: H5-4. Blocks: S6-3.*
- [ ] **H5-6 — Bench validation.** `hardware_bringup.md` §6, in order. This is the gate
      before any Week 7 HITL or Week 8 flight work. *Blocked by: H5-3, H5-5.*
      - [ ] Props off: arm via GCS, IMU/EKF healthy, all failsafes confirmed.
      - [ ] Props off: Pi boots and *stays up* on BEC power alone, USB-C PSU
            disconnected. This is the real test of H5-4.
      - [ ] Props off: OAK-D enumerates as **SuperSpeed** in `lsusb -t` / `dmesg`.
            High-Speed means USB-2 negotiation — re-seat cable/port, do not proceed.
      - [ ] Only after all three: props on, held down, brief static throttle-up.

## SW lane — Week 5

- [ ] **S5-1 — Stand the stack up on the Pi 5 (arm64).** Nothing in this project has
      ever been built or run on ARM. *Blocks: everything on the Pi.*
      - [ ] Install Ubuntu 24.04 arm64 + ROS 2 Jazzy on the 64 GB microSD.
      - [ ] `colcon build --symlink-install` both packages (`huitzilin_sim`,
            `huitzilin_perception`). Expect missing/rebuilt native deps — record which.
      - [ ] Run the existing unit tests on the Pi: `test_frames`, `test_kalman`,
            `test_ballistics`, `test_cloud_geometry`, `test_background_map`,
            `test_throw_window`, `test_stage_profiler`. Any arch-dependent failure is a
            real finding, not a nuisance.
      - [ ] Set `usb_max_current_enable=1` in `config.txt` (pairs with H5-4).
      - [ ] Decide and record where telemetry/bags land on the SD card, and what the
            retention rule is — `SAFETY_CASE.md` §4 requires footage deleted after 30
            days unless kept for analysis.
- [ ] **S5-2 — Hardware config files (new files, never edits).** `--symlink-install`
      makes the installed yaml a symlink into `src/`, so editing a shipped yaml edits
      the real config. Every hardware value goes in a *new* file.
      - [ ] `params/hw_bridge.yaml` — `connection:` moves from `udpin:0.0.0.0:14552` to
            the H743's serial device. Prefer a stable `/dev/serial/by-id/...` path over
            `/dev/ttyACM0`, which renumbers.
      - [ ] Preserve the coupling `patrol_handoff_s` (0.8) **>** `cmd_timeout_s` (0.7).
            If either moves on hardware, both move. Violating it makes the
            zero-velocity handback fight patrol's position setpoints.
      - [ ] Re-examine `stream_rate_hz: 30.0`. Its stated justification is that odom
            must outpace the 15 Hz depth cloud for egomotion compensation. If S6-4
            measures the real camera above 15 Hz, this value no longer satisfies its
            own reason and must rise.
      - [ ] `params/hw_detector.yaml` — two flips already documented in `detector.yaml`
            itself: `cloud_convention` `gz_flu` → `optical` (DepthAI already emits
            optical-convention clouds), and `cloud_reliable` `true` → `false` (DepthAI
            publishes best-effort). Both are Week 6 to *verify*, Week 5 to *stage*.
      - [ ] `params/hw_evasion.yaml` — carry Week 4 tuning across unchanged for now.
            It gets re-derived in S6-6 against real noise; do not pre-guess it.
      - [ ] Raise `dodge_floor_m` from its sim value of 1.0 m. Its comment says
            "Raise for real flight" — the sim floor exists because a descending throw
            drove the drone into the runway at 2 m AGL. Set the real value against the
            actual enclosure in H8-1, but never leave it at 1.0.
- [ ] **S5-3 — `params/hw_frame.parm` for the real vehicle.** One file, loaded to the
      board, so no safety parameter depends on someone remembering to type it.
      - [ ] Frame: `FRAME_CLASS=1`, `FRAME_TYPE=1`.
      - [ ] Geofence (`SAFETY_CASE.md` §2): `FENCE_ENABLE=1`, `FENCE_TYPE=3`,
            `FENCE_RADIUS=10`, `FENCE_ALT_MAX=5`, `FENCE_ACTION=1`.
      - [ ] Failsafes: `FS_THR_ENABLE=1`, `BATT_FS_LOW_ACT=2`.
      - [ ] Kill-switch: `RC7_OPTION=31` (motor emergency stop) or another free channel.
      - [ ] **Resolve a live contradiction before flashing:** `SAFETY_CASE.md` §2 sets
            `FENCE_ALT_MAX=5` and separately describes RTL climbing to `RTL_ALT`
            (default **15 m**). An RTL triggered by a fence breach would climb straight
            through the fence ceiling it is responding to. Pick `RTL_ALT` below
            `FENCE_ALT_MAX`, set it explicitly in this file, and correct
            `SAFETY_CASE.md` §2 to match. *Do not flash the board with both values as
            currently written.*
      - [ ] **No inline comments** — MAVProxy breaks on them. Comment-only lines only.
      - [ ] Never `ARMING_CHECK 0` and never blind force-arm (`param2=21196`). Both hide
            the message that tells you what is actually wrong.
- [ ] **S5-4 — Sim-time → wall-clock audit.** *Blocks every later SW item.* See finding
      2 above. The goal is that a node started against real hardware is either correct
      or loudly wrong, never silently drifting.
      - [ ] Enumerate every `use_sim_time` declaration and every launch file that sets
            it. Make it a launch argument with a hardware default of `false`, rather
            than a value baked into `detector.yaml`.
      - [ ] Audit every duration parameter for which clock it is denominated in:
            `latency_budget_s` 0.15, `track_timeout_s` 0.5, `patrol_handoff_s` 0.8,
            `cmd_timeout_s` 0.7, `recover_hold_s` 0.5, `dodge_duration_s` 1.0,
            `trigger_horizon_s` 1.5, `prediction_horizon_s` 3.0, `bg_map_ttl_s` 20.0.
      - [ ] Make a missing `/clock` under `use_sim_time:=true` a hard startup failure.
            Silently falling back to wall-clock is how a timing gate quietly changes
            meaning by a factor of ~4 (Gazebo runs ~24% real-time under WSL2, ~0.33 RTF
            on the Dell under depth rendering).
      - [ ] Restate the Week 4 latency figures in *wall* ms alongside sim ms, so Week 6
            and Week 8 numbers are comparable to something.
- [ ] **S5-5 — Write `supervisor_node`.** Promised in `architecture.md` at 1 Hz,
      publishing `/huitzilin/set_mode` and `/huitzilin/start_patrol`; does not exist.
      *Blocks: W7.*
      - [ ] Implement the state table in `docs/state_machine.md`: DISARMED, ARMING,
            TAKEOFF, PATROL, EVADE, RTL/LAND, FAILSAFE, with the transitions as listed.
      - [ ] Implement the detection column of the `SAFETY_CASE.md` §1 FMEA: topic
            timeout (sensor dropout), heartbeat timeout (RC/link loss, Pi brownout),
            voltage monitor (low battery), FC status (FC failsafe), geofence breach
            (flyaway), bridge watchdog (GUIDED setpoint-stream loss).
      - [ ] Enforce the fail-safe default explicitly: any undefined fault → FAILSAFE →
            RTL → land. **Never** an evasive maneuver on a fault. The reflex is for
            projectiles only.
      - [ ] Unit-test the transition table, including that no fault path can reach EVADE.
- [ ] **S5-6 — Hardware preflight script.** `scripts/preflight_check.sh` is a Week 2
      SITL script (checks Gazebo, `ardu_ws`, a SITL heartbeat). Add
      `scripts/preflight_hw.sh`: serial link present, heartbeat on the real FC, all
      expected params readback-matched against `hw_frame.parm`, OAK-D at SuperSpeed, Pi
      not throttled, disk space for logs, kill-switch channel reading live.
- [ ] **S5-7 — Keep SITL green.** The SITL path stays the regression environment for the
      rest of the project. Confirm `week2_sitl.launch.py` and the Week 4 battery still
      run unchanged after S5-2/S5-4 land. A hardware refactor that breaks sim costs the
      only reference you have.

## Week 5 DoD

- Clean bench arm-up on the real H743, props off, IMU/EKF healthy (H5-6).
- Failsafes and kill-switch verified by test, not by inspection (H5-3).
- Pi runs on BEC power alone with no draw from the FC rail (H5-6).
- ROS 2 stack builds and unit-tests pass on the Pi 5 (S5-1).
- `use_sim_time` is a launch argument with a hardware default, and no timing parameter's
  clock is ambiguous (S5-4).
- `supervisor_node` exists with a tested transition table (S5-5).
- `RTL_ALT` / `FENCE_ALT_MAX` contradiction resolved in both the param file and
  `SAFETY_CASE.md` (S5-3).

---

# Week 6 — Payload wiring & real OAK-D bring-up

**Theme:** the decisive measurement of the project. `CLAUDE.md` states the Week 4 dodge
envelope is bounded by *sensing* — detection range and frame rate — not by thresholds or
dodge authority. Every range-side lever is a measured null. S6-4 is what moves the
envelope, and it is a headline result, not a checkbox.

## HW lane — Week 6

- [ ] **H6-1 — Payload wiring.** `hardware_bringup.md` §7. *Blocked by: S6-1 (write the
      node first so there is something to test against).*
      - [ ] WS2812B: 5 V + GND from the Pi, data line through a 3.3→5 V level shifter
            (74AHCT125 or equivalent) before DIN. The Pi's 3.3 V logic is out of spec
            for the strip directly. **This part is not yet purchased** — order it now
            (`HuitzilinReflex_v2.md` Appendix A closing note).
      - [ ] Piezo siren through an NPN transistor switch with flyback-safe wiring, never
            straight off a GPIO pin.
- [ ] **H6-2 — Remote ID & registration.** `hardware_bringup.md` §9. *Independent of
      everything else; blocks all outdoor flight.* Start it early — it has external
      turnaround time that no amount of bench work shortens.
      - [ ] Register the aircraft with the FAA; mark it with the registration number.
      - [ ] Enable Remote ID broadcast and verify it actually transmits (Remote ID
            receiver app on a phone is the fastest check).
      - [ ] Re-read `SAFETY_CASE.md` geofence/RTL/kill-switch against the **as-built**
            aircraft, not the sim model.
      - [ ] Confirm the test site is clear of controlled airspace.

## SW lane — Week 6

- [ ] **S6-1 — Write `payload_node`.** The `/payload/alarm` publisher has had no
      subscriber since Week 4. *Blocks: H6-1 validation.*
      - [ ] Subscribe `/payload/alarm` (`std_msgs/Bool`, Reliable), drive WS2812B via
            `rpi_ws281x` and the siren GPIO. On-event, per `architecture.md`.
      - [ ] Per `SAFETY_CASE.md` §1, a GPIO/payload fault is severity Low: log and
            continue. A dead LED must never take down the flight stack.
      - [ ] Promote `/payload/alarm` from "provisional (Wk5–6)" to active in
            `architecture.md` once it lands.
      - [ ] Payload is a **safety signal only** — never used to follow or harass a
            person (`SAFETY_CASE.md` §4). State it in the node's docstring.
- [ ] **S6-2 — Validate payload trigger latency.** `hardware_bringup.md` §7 calls for a
      plain GPIO toggle script *before* wiring into the evasion node. Measure the alarm
      path separately from the dodge path — REQ-05 (alarm on detection) and REQ-04
      (evade within 150 ms) are different requirements and must not be measured as one.
- [ ] **S6-3 — Real OAK-D bring-up over `depthai-ros`.** `hardware_bringup.md` §8.
      *Blocked by: H5-4, H5-5.* Replaces the `ros_gz_bridge` path in
      `perception_bridge.yaml`; that file stays as the sim path.
      - [ ] Install `depthai-ros` on the Pi; confirm depth + point-cloud topics publish.
      - [ ] Confirm the Myriad X VPU is doing the stereo work — Pi CPU stays low. Any
            per-pixel depth reconstruction on the Pi means the pipeline is
            misconfigured, and contradicts the project's own non-goal ("no custom depth
            math").
      - [ ] Verify the two staged flips from S5-2 are correct against the real stream:
            `cloud_convention: optical`, `cloud_reliable: false`.
- [ ] **S6-4 — MEASURE THE ENVELOPE'S BINDING TERM.** *The decisive result of the
      project.* Report it as a headline, in its own commit, with the numbers split out.
      - [ ] Measure **delivered** frame rate at the ROS layer — not the camera's rated
            spec. Sim delivered 15 Hz because `ros_gz_bridge` could not sustain 30 Hz at
            640×480; the real USB path has its own, different ceiling.
      - [ ] Measure **effective detection range** on the real sensor. Note the
            distinction: `roi_max_range_m` is gated at 5.0 m, but `CLAUDE.md` reports
            the ball is only actually detected across ~3 m. The real number that matters
            is where a ball-sized object stops producing a usable cluster, not where the
            gate sits.
      - [ ] Recompute the reaction envelope from the measured pair. The Week 4 arithmetic
            to redo: ball crosses ~3 m at 14 m/s in 0.22 s; three track updates at
            ~14.5 Hz cost 0.21 s; pipeline 0.08 s; clearing the 0.30 m hit radius in the
            remaining ~0.08 s needs 3.6 m/s of *instantaneous* escape. Substitute the
            measured range and rate and state where the new speed ceiling lands.
      - [ ] Re-examine `min_track_updates: 3` **only in light of this measurement.**
            3→2 is a recorded measured null in sim — but it was null *at 15 Hz*. If the
            real rate differs materially, the null does not transfer, and this is the
            one condition under which re-testing it is not a repeat of a closed
            experiment. Say so explicitly in the commit message.
      - [ ] Characterize what sim never modelled: low-texture dropouts, depth holes at
            range, frame-rate dips under load. Record where depth stops being
            trustworthy — that range is the real analogue of `roi_max_range_m`.
- [ ] **S6-5 — Build a real bag library.** The sim library is **saturated** (recall
      100%) and cannot referee a threshold change — a change that helps or hurts reads
      as no-change. Every detector tuning decision from here needs real bags.
      - [ ] Capture labelled bags with the real camera, following the structure in
            `docs/bag_capture_runbook.md`.
      - [ ] Include the failure modes sim could not produce (S6-4): low-texture scenes,
            depth holes, rate dips.
      - [ ] Keep the library **unsaturated on purpose** — include hard cases the current
            detector fails, or it will be as unable to referee as the sim library.
      - [ ] Re-point `score_bags.py` / `run_regression.sh` at the real library. Keep the
            sim library scoreable for comparison; never mix the two in one score.
      - [ ] Carry the pre-`b0eedd5` rule forward: any bag without attitude in
            `/huitzilin/odom` forces camera-frame differencing and must never be scored.
- [ ] **S6-6 — Re-derive the Kalman measurement model from real noise.** *Blocks: W7.*
      - [ ] Measure real centroid noise and feed it into `meas_std_m` (sim: 0.15) and
            `process_accel_std` (sim: 3.0). **Do not carry the sim-tuned values over
            unexamined** — this is the explicit instruction in `hardware_bringup.md` §8.
      - [ ] Re-check `max_assoc_sigma_m: 0.75`. Its derivation is arithmetic on the
            frame rate — "0.75 m covers 11 m/s of ball travel at the 15 Hz detector
            rate". If S6-4 changes the rate, this number is stale by construction.
      - [ ] Re-measure the detector's false-positive rate on real data. The
            multi-hypothesis tracker exists because sim measured ~1.4 FP/s; that rate
            is a property of the sim scene and will differ in a real room.
- [ ] **S6-7 — Re-profile the pipeline on the Pi 5.** See finding 3. Turn
      `profile_stages: true` (it is cached at construction — restart the node; `ros2
      param set` has no effect) and re-run the stage breakdown on arm64.
      - [ ] Expect `voxel_downsample` and `cluster_all` to dominate again; they were the
            entire x86 overrun.
      - [ ] Leave `debug_dump_dir` **empty**. It costs ~40 ms/frame and no latency number
            from a dumping run is valid. Same for `debug_funnel` during profiling.
      - [ ] State plainly whether latency is *still* not the binding term on the Pi. The
            Week 4 conclusion holds only while latency stays under tca (0.18–0.29 s).
            If Pi latency pushes past it, the binding term has changed and the Week 8
            plan changes with it.

## Week 6 DoD

- Live depth frames from the real OAK-D on the Pi, VPU doing the stereo work (S6-3).
- Payload LED + siren trigger from `/payload/alarm`, measured against their own latency
  budget (S6-1, S6-2, H6-1).
- **Measured real detection range and delivered frame rate, with the recomputed reaction
  envelope stated in terms of ball speed** (S6-4).
- A real, unsaturated, labelled bag library the regression harness scores against (S6-5).
- KF measurement model re-derived from real noise, not inherited (S6-6).
- A clear statement of whether latency is the binding term on the Pi (S6-7).
- Aircraft registered, Remote ID verified transmitting (H6-2).

---

# Week 7 — HITL & tethered hover

**Theme:** real flight controller, simulated world. The first week where a mistake can
move a physical propeller under autonomous command, so the supervisor and the failsafes
stop being code review items and become test items.

## HW lane — Week 7

Not covered by `hardware_bringup.md` — written here.

- [ ] **H7-1 — Tether and net rig.** *Blocked by: H5-6.*
      - [ ] Build or source a tether that constrains altitude and lateral excursion to
            less than the geofence, so the fence is the second line of defence and not
            the first.
      - [ ] Anchor it so a full-authority dodge command cannot drag the anchor. The dodge
            adds escape to cruise and is capped at `dodge_max_speed_mps: 6.0` — size the
            rig against the cap, not against the 1.5 m/s nominal.
      - [ ] Confirm the tether cannot foul a duct, the ELRS antenna, or the OAK-D USB
            cable at any attitude.
- [ ] **H7-2 — Full-stack bench dress rehearsal, props off.** Everything powered exactly
      as it will fly: BEC only, OAK-D live, payload wired, FC on serial to the Pi. Run
      for a sustained period and watch for thermal throttling on the Pi 5 and brownouts
      on the 5 V rail under inference load (`SAFETY_CASE.md` §1: Pi brownout, severity
      High).
- [ ] **H7-3 — Kill-switch discipline drill.** Operator's thumb on the switch for every
      armed state, no exceptions (`SAFETY_CASE.md` §3). Rehearse the abort criteria
      until aborting is reflexive: unexpected path → kill; person enters area → land;
      payload malfunction → land, props off, inspect; loss of visual line-of-sight →
      kill.

## SW lane — Week 7

- [ ] **S7-1 — `week7_hitl.launch.py`.** Real H743 over serial + Gazebo world.
      *Blocked by: S5-5, S6-6.*
      - [ ] This is the awkward clock case and the reason S5-4 exists: the FC runs in
            real time while the world runs in sim time. Decide *explicitly* which clock
            each node consumes, and write that decision down here in the Week 7
            section and in the launch file's docstring. It must be explicit, never
            implied by a launch argument's default.
      - [ ] The NED↔ENU conversion still lives in exactly one place
            (`MavBridge.ned_to_enu` / `enu_to_ned`). HITL is a classic place for a
            second conversion to sneak in. Mirrored RViz markers = bridge bug, as ever.
- [ ] **S7-2 — Exercise the supervisor against real faults.** Not a code review — pull
      the plug and watch.
      - [ ] Unplug the OAK-D mid-run → sensor dropout → FAILSAFE.
      - [ ] Kill the detector process → topic timeout → FAILSAFE.
      - [ ] Stop the `cmd_vel` stream → bridge watchdog → zero-velocity hold → LOITER.
      - [ ] RC off → link loss → FAILSAFE → RTL.
      - [ ] Confirm in every case that the response is FAILSAFE and **never** a dodge.
- [ ] **S7-3 — Measure end-to-end latency on the real aircraft, in wall time.** First
      honest REQ-07 measurement: real camera, real Pi, real serial link to a real FC.
      Report mean and the exceedance tail, the way Week 4 did (mean 95–115 ms, ~25%
      over 150 ms, max 282 ms) — a mean alone hides the tail that matters.
- [ ] **S7-4 — DECISION: how Week 8 gets scored.** *Resolve before Week 8 starts.*
      `dodge_battery.py` scores every throw against simulator ground truth — it subscribes
      `/gz/dynamic_poses` and exits non-zero if that stream is absent. Real throws have
      no truth stream, so the 78/78 figure has no direct real-world successor unless a
      scoring method is chosen deliberately. The options:
      - **(a) Outcome scoring — hit / no-hit / no-dodge.** Cheapest, needs no new rig,
        survives contact with reality. Loses the miss-distance detail that Week 4 used
        to root-cause failures (closest approach was 4 cm at one point, which is how the
        manoeuvre was distinguished from the trigger).
      - **(b) External camera + manual annotation.** Recovers approximate miss distance.
        Slow, and adds a calibration problem between the camera's frame and `odom`.
      - **(c) Onboard closest-approach estimate from the KF's own track.** Free, already
        computed, comparable in *form* to the sim metric — but it scores the estimator
        using the estimator, so a tracking failure and a dodge failure become
        indistinguishable. Usable as a secondary metric, dangerous as the primary one.
      - Recommendation: **(a) as the primary metric, (c) logged alongside as diagnostic,
        never as the headline.** Whatever is chosen, REQ-08 still binds: report split by
        ball speed, never blended.
      - [ ] Refactor `dodge_battery.py` so the scoring backend is swappable, rather than
            forking a second harness that will drift from the first.
- [ ] **S7-5 — Set real-flight geometry parameters.** `dodge_floor_m` (sim 1.0),
      `takeoff_alt_m` (sim 2.0) and the fence radius were all chosen for an open sim
      runway. A netted enclosure is smaller than the 10 m fence radius in
      `SAFETY_CASE.md` §2. Derive all three from the actual enclosure (H8-1) and record
      the derivation.

## Week 7 DoD

- Stable tethered hover with the full stack running and logging (`HuitzilinReflex_v2.md`
  §5).
- Every FMEA fault injected and answered with FAILSAFE — never a dodge (S7-2).
- End-to-end latency measured in wall time on the real aircraft, mean **and** tail
  (S7-3).
- Week 8 scoring method decided, written down, and implemented behind a swappable
  backend (S7-4).

---

# Week 8 — Incremental real flight

**Theme:** the week most likely to be cut, and the week where cutting is the right call
if Week 7 leaves anything unresolved. Progression is strictly manual hover →
autonomous patrol → evasion, and each stage is a gate, not a formality.

## HW lane — Week 8

- [ ] **H8-1 — Netted enclosure.** *Blocked by: H6-2.* Projectile evasion testing only
      inside netting with soft projectiles (`SAFETY_CASE.md` §4) — this is binding, not
      advisory. Measure the enclosure and hand the dimensions to S7-5.
- [ ] **H8-2 — Soft projectiles.** Match the sim ball where it matters: the detector's
      size gates are metric (`cluster_max_extent_m: 0.35`, and the 80 mm ball spans
      ≤ ~0.15 m after 0.02 m voxelization). A real ball of a very different size or a
      low-texture surface will not be detected by a pipeline tuned for that one.
- [ ] **H8-3 — Battery and charge discipline.** Charge on the ISDT inside the fireproof
      bag, never unattended; storage charge between sessions; 18650s cased
      (`SAFETY_CASE.md` §4). Two 1300 mAh 6S packs is a short session — plan the flight
      cards around pack count, not around ambition.
- [ ] **H8-4 — Spares triage.** Two spare prop pairs shipped with the airframe. Decide
      in advance what damage ends the session rather than deciding it after a crash.

## SW lane — Week 8

- [ ] **S8-1 — Staged flight progression.** Each stage fully logged; do not advance on a
      stage that produced a surprise.
      - [ ] Manual hover, kill-switch in hand.
      - [ ] Autonomous patrol (REQ-01). Compare against the sim baseline: 43 laps,
            mean 29.51 s.
      - [ ] Evasion, only after both above are clean.
- [ ] **S8-2 — Real throw campaign.** Scored by the S7-4 method.
      - [ ] Report **split by ball speed, never blended** (REQ-08, `CLAUDE.md`). The sim
            result is 78/78 at ≤ 8 m/s, 0/17 at 14 m/s, 0/12 false dodges — the real
            report must be legible against that shape.
      - [ ] Include a false-dodge battery. A dodge that fires at nothing is a failure
            mode with its own count, and sim scored it separately for that reason.
      - [ ] Sweep toward the speed ceiling S6-4 predicted, and record where it actually
            lands. That comparison — predicted vs. measured ceiling — is the single most
            interesting number the project can produce.
- [ ] **S8-3 — Re-tune the KF against real flight noise.** Distinct from S6-6: that was
      bench noise, this is noise under prop wash, vibration, and real egomotion.
      - [ ] Re-check egomotion compensation specifically. It depends on odom carrying a
            valid attitude quaternion, and disabling it caused a 60%-recall regression
            in sim.
      - [ ] Re-check the persistent background map on a real scene. `bg_map_ttl_s: 20.0`
            and `bg_map_leaf_m: 0.10` were tuned against a Gazebo world; a real room
            with moving air and changing light is a different problem.
- [ ] **S8-4 — Confirm return-to-patrol.** REQ-06: the drone returns to patrol after a
      successful evasion. `auto_resume_patrol: true` and the `patrol_handoff_s` /
      `cmd_timeout_s` coupling are what implement it — verify on the real vehicle, where
      the handback fights a real controller rather than a simulated one.
- [ ] **S8-5 — Do not re-run the measured nulls.** `CLAUDE.md` lists them and calls
      re-running them "the most common way to lose a session here": `roi_max_range_m`
      5→8, `min_track_updates` 3→2, `cluster_min_points` 5→3, 320×240 @ 30 Hz,
      `dodge_speed_mps` 1.5→4.0, the multi-hypothesis tracker, the command path,
      vertical escape. **Exception:** any null whose sim derivation depended on the
      15 Hz rate or the ~3 m range is re-openable *if and only if* S6-4 measured
      something materially different — and the commit must say which measurement
      re-opened it. Never cite the PSC_ACC_XY / WPNAV_ACCEL / ANGLE_MAX experiment; it
      is invalid (`--defaults` does not override `eeprom.bin`, and the real parameter is
      `ATC_ANGLE_MAX`).

## Week 8 DoD

- One clean autonomous patrol, fully logged (REQ-01).
- One successful evasion under the S7-4 scoring method, fully logged (REQ-02–06).
- Results reported split by ball speed, with false dodges counted separately (REQ-08).
- Predicted (S6-4) vs. measured speed ceiling stated side by side.

---

# Week 9 — Validation, documentation & retro

**Theme:** make the result reproducible and the surprises legible. This is the week that
determines whether the project reads as a finished piece of engineering or a pile of
runs.

## HW lane — Week 9

- [ ] **H9-1 — As-built wiring documentation.** Photograph and diagram the finished
      aircraft: motor phase order by corner, battery leads, BEC input and output, Pi
      GPIO pinout, level shifter, siren transistor circuit, OAK-D routing. The Week 5
      reference photos captured what was removed; this captures what replaced it.
- [ ] **H9-2 — Mass budget check.** REQ-09 caps payload overhead at 200 g. Weigh the
      as-built aircraft against the bare airframe and record the delta against the
      ~150–200 g budget in `HuitzilinReflex_v2.md` §2.
- [ ] **H9-3 — Storage state.** Packs at storage charge, 18650s cased, aircraft
      disarmed and unpowered.

## SW lane — Week 9

- [ ] **S9-1 — Validation matrix, REQ-01 … REQ-16.** One row per requirement: the
      evidence, where the log lives, and pass/fail/not-demonstrated. "Not demonstrated"
      is an honest and acceptable verdict; a fudged pass is not.
      - [ ] REQ-04 and REQ-07 (150 ms) cite the wall-clock measurement from S7-3, not
            the sim-time figure.
      - [ ] REQ-08 cites the split-by-speed table from S8-2.
      - [ ] REQ-14/15/16 cite the fault injections from S7-2.
- [ ] **S9-2 — Reproducible build doc.** `docs/SETUP.md` covers install from scratch for
      the sim path. Extend or complement it so a reader can go from the BOM to a flying
      aircraft: Pi image, ROS 2 arm64 build, `hw_*.yaml`, `hw_frame.parm`, bring-up
      order. The plan's own dependency map is the outline.
- [ ] **S9-3 — Sim-vs-real post-mortem.** The intellectual payload of the project.
      - [ ] The headline comparison: sim said the envelope was bounded by sensing at
            ~3 m and 15 Hz. State what the real sensor measured and whether the
            prediction held.
      - [ ] Which measured nulls transferred to hardware and which did not, and why.
      - [ ] Which sim-only artefacts wasted time and would be caught earlier next time
            (the `ros_gz_bridge` 15 Hz ceiling, the saturated bag library, the
            `FRAME_CLASS=0` silent no-lift, sim-time timing gates).
      - [ ] Where a real failure mode had no sim analogue at all.
- [ ] **S9-4 — Fold the real results into `CLAUDE.md`.** Its Week 4 envelope table and
      measured-nulls list are the project's working memory. Update them so the next
      session starts from hardware truth rather than sim truth — including a new
      measured-nulls section for anything hardware proved dead.
- [ ] **S9-5 — Update the contract docs to as-built.** `architecture.md` still marks
      `payload_node` and `supervisor_node` as future and `/payload/alarm` as
      provisional. Promote them, and correct any topic, rate or QoS that hardware
      changed.
- [ ] **S9-6 — Final vlog.** The project is vlogged weekly
      (`HuitzilinReflex_v2.md`). The strongest available story is not "the drone dodges"
      — it is that Week 4 predicted the envelope was bounded by sensing, named the
      measurement that would settle it, and Week 6 went and made that measurement.

## Week 9 DoD

- Validation report covering REQ-01 … REQ-16 with evidence per row (S9-1).
- Reproducible build doc: BOM → flying aircraft (S9-2).
- As-built wiring documentation (H9-1).
- Sim-vs-real post-mortem written (S9-3).
- `CLAUDE.md` and `architecture.md` reflect the as-built system (S9-4, S9-5).

---

## Standing rules for the whole period

Carried from `CLAUDE.md` and `SAFETY_CASE.md` because they are the ones most likely to
be violated during a hardware push, when the temptation to shortcut is highest.

1. **Never edit a shipped yaml for a diagnostic.** `--symlink-install` makes the
   installed copy a symlink into `src/` — you are editing the real config. Copy to
   `/tmp` and `sed` that.
2. **Never `ARMING_CHECK 0`, never blind force-arm.** Both hide the message naming the
   actual fault.
3. **No latency number from a `debug_dump_dir` run is valid** (~40 ms/frame). Same
   caution for `debug_funnel` and `profile_stages`.
4. **Report the envelope split by ball speed, never blended** (REQ-08).
5. **Props-off is the default on the bench**, at all times.
6. **Kill-switch in hand for every armed state**, no exceptions.
7. **Any undefined fault → FAILSAFE → RTL → land. Never a dodge.** The reflex is for
   projectiles only.
8. **Re-running a measured null is the most common way to lose a session** — unless
   S6-4 changed the measurement the null rested on, and the commit says so.
