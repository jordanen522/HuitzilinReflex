# Project HuitzilinReflex

### Autonomous Agile Patrol & Projectile-Evasion Drone — Summer 2026

9-week project: a 3.5″ ducted micro-drone that patrols a designated area, signals with
light/sound, and autonomously dodges incoming projectiles using an onboard stereo depth
stack. Vlogged weekly. **v2 — as-built:** hardware list reflects parts actually
purchased; the one structural change from v1 is that the airframe's stock Betaflight
F722 flight controller is **replaced** with an ArduPilot-capable H7 board (Week 5).

**Name:** *Huitzilin* (Classical Nahuatl) = "hummingbird" — the only bird that hovers in
place and darts omnidirectionally in a fraction of a second; in Aztec belief, fallen
warriors returned as hummingbirds. *Reflex* is literal: the sense → dodge loop fires as a
reaction, not a decision. "Hummingbird Reflex" — loiter like a hover, dodge like a dart.

---

## 1. Overview & Objectives

3.5-inch ducted micro-platform keeping full CS/robotics complexity (perception, state
estimation, control) while staying physically safe: integrated ducts fully isolate the
props, so rapid evasion test cycles run without exposed blades.

1. **Autonomous patrol & signaling** — persistent pathing with strobe + siren payload.
2. **Kinematic evasion** — low-latency detection → intercept prediction → sharp dodge.

**Philosophy:** simulate first, fly last; *integrate, don't fabricate*. The fragile parts
(airframe, depth stack) are plug-and-play hardware; effort concentrates on the
perception + evasion loop, de-risked in sim. The one fabrication step is the FC swap.

---

## 2. Hardware Base & Payload (as purchased)

| Subsystem | Part | Notes |
|---|---|---|
| **Airframe** | GEPRC CineLog35 V2 HD, 3.5″ ducted, 6S, BNF ELRS 2.4 GHz | 142 mm wheelbase; GEPRC ELRS receiver pre-installed; 2 spare prop pairs |
| **Propulsion** (stock) | SPEEDX2 2105.5 motors, HQProp D-T90 props | ~150–200 g payload overhead |
| **Flight controller** (replacement, Wk5) | MicoAir H743 V2 AIO — STM32H743 + integrated 4-in-1 45 A AM32 ESC | Replaces stock GEP F722-45A (Betaflight-only, not an ArduPilot target). Same 25.5×25.5 mm mount and 45 A rating — like-for-like swap; integrated ESC = no separate ESC |
| **Companion computer** | Raspberry Pi 5 (4 GB) + Active Cooler + 64 GB microSD | ROS 2 + evasion node; depth arrives pre-computed |
| **Bench power (Pi)** | Official 27 W USB-C PSU | Desk power through Week 5 bring-up; flight power is the Pololu BEC below |
| **Flight power (Pi)** | Pololu D24V50F5 5 V/5 A step-down | 6S → 5 V/5 A to Pi GPIO pins; see §4 |
| **Depth sensor** | Luxonis OAK-D Lite + USB-C↔USB-A 3.1 Gen 2 cable | On-chip stereo depth over USB 3 (USB-2 cable would throttle it) |
| **Flight battery** | CNHL 1300 mAh 130C 6S ×2 (XT60) | Matches recommended 1050–1300 mAh 6S |
| **Radio** | RadioMaster Pocket (ELRS 2.4 GHz) + 2× Samsung 30Q 18650 | Manual control, failsafe, kill-switch |
| **Charging** | ISDT 608AC + Zeee fireproof bag | |
| **Tools/wiring** | Soldering kit (w/ multimeter), jumper wires, component kit, M2.5 nylon standoffs | FC swap, payload wiring, buzzer transistor circuit, Pi/OAK-D mounting |

**Payload — warning systems**

- **LEDs:** BTF-Lighting WS2812B strip (5 V, 144 LED/m), driven from Pi GPIO via
  `rpi_ws281x`. Pi data line is 3.3 V vs the strip's ~5 V logic — add a 3.3→5 V level
  shifter (e.g. 74AHCT125).
- **Siren:** Tokatuker 2–12 V piezo (120 dB @ 12 V), on a Pi GPIO through a transistor
  circuit; toggled the moment a threat vector is computed.

---

## 3. Perception & Evasion Pipeline

No event cameras, no LiDAR, no hand-built depth math: the OAK-D Lite's Myriad X VPU
computes stereo depth **in-camera** and streams finished depth/point clouds to the Pi
over USB 3 — the Pi never does per-pixel depth work.

Detection (incoming projectile = sudden cluster of depth differentials) → tracking
(centroid into a predictive Kalman filter for velocity) → trigger (predicted intercept
inside the drone's boundary → alarm GPIO + a high-rate ~1.5 m/s velocity-spike command).

Node graph, topics, rates, and the full contract table: `docs/architecture.md`.

---

## 4. Critical Avionics, Power & Signal Notes

- **H743 required:** real-time ArduPilot control laws + high-rate pymavlink velocity
  commands need the H7 class; the stock F722 is Betaflight-only.
- **FC swap = soldering job (Wk5):** 4 motors (3 wires each), battery leads, HD-VTX/camera
  + ELRS receiver connections move to the MicoAir; same mount. Photograph stock wiring
  first; outsource to an FPV shop if fine-pitch soldering isn't comfortable.
- **Control interface:** native `pymavlink` over USB/UART, no middleware agent.
- **Pi power isolation:** Pi 5 can spike to 5 V/5 A during inference and must never draw
  from the FC rail. Pololu BEC feeds the Pi's 5 V/GND GPIO pins; because that bypasses
  USB-C PD negotiation, set `usb_max_current_enable=1` in `config.txt` or the Pi
  throttles its USB ports and starves the OAK-D.
- **Manual FPV (optional):** the RunCam Wasp is DJI HD — manual video flying needs DJI
  goggles; line-of-sight flying doesn't.
- **Stereo noise budget:** no active IR (no MPI), but low-texture dropouts, range limits,
  and frame-rate dips are handled in the Kalman filter tuning, not a denoise stage.

---

## 5. Roadmap (Simulation-First)

Weeks 1–4 are pure simulation, hardware bring-up runs in parallel on the bench with props
off, and real flight is staged late.

Completed milestones:

| Milestone | Landed |
|---|---|
| Procurement | Full BOM ordered (Appendix A); all Week-5 hardware in hand |
| Architecture, safety case, simulation environment | SITL, Gazebo and ROS 2 up; scripted arm, takeoff and hold via pymavlink |
| ROS 2 to pymavlink bridge and patrol | Autonomous closed patrol loop with logged telemetry: 43 laps, mean 29.51 s. Airframe fidelity deferred |
| Perception pipeline | Simulated OAK-D depth, synthetic scenarios, detection node, 17-bag labelled library and regression harness; held-out recall 100% |
| Evasion logic and Kalman filter in the loop | Predictive Kalman filter, multi-hypothesis tracker and dodge trigger, closing detection to intercept to velocity spike to alarm (mocked GPIO) over a 7-scenario battery. The resulting envelope is bounded by sensing |
| Software lane | Supervisor state machine, payload node, clock guard, hardware config overlays, hardware preflight |
| Sensor-requirement study | The reach and rate a 20 m/s dodge needs, measured in simulation ahead of the hardware |

The sensor-requirement study is the one that answers the project's central question:
probability of a save is a sigmoid in time-to-closest-approach with an LD50 of 0.798 s,
independent of ball speed, so a 20 m/s dodge needs 21.1 m of reach on an 80 mm ball where
the as-built OAK-D Lite caps the aircraft at ~3.2 m/s. The full derivation, the sensor
spec it implies, and the sector cost are in `docs/RESULTS.md`.

**Weeks 7–9 are out of scope for the simulation phase.** They are the physical work: HITL
and tethered hover, incremental real flight inside a netted enclosure with soft
projectiles, and sim-versus-real validation. What hardware would have to settle is listed
in `docs/RESULTS.md` §9.

---

## 6. Safety, Legal & Ethical

The binding rules — FAA/Remote ID, LiPo handling, netted-enclosure testing, privacy and
consent, abort criteria, and the fail-safe default — are owned by `docs/SAFETY_CASE.md`.
Design constraint they impose here: the ducts make blade contact safe, not a flyaway, so
enclosure and kill-switch discipline is what actually bounds evasion testing.

---

## Appendix A — Bill of Materials (as purchased)

| # | Item | Qty | Price | Vendor |
|---|---|---|---|---|
| 1 | GEPRC CineLog35 V2 HD (RunCam Wasp), 6S — ELRS 2.4 GHz | 1 | $315.99 | GetFPV |
| 2 | Raspberry Pi 5 (4 GB) | 1 | $130.00 | Adafruit |
| 3 | Raspberry Pi 5 Active Cooler | 1 | $13.50 | Adafruit |
| 4 | SanDisk Extreme 64 GB microSD | 1 | $31.99 | Amazon |
| 5 | Luxonis OAK-D Lite | 1 | $169.00 | Luxonis |
| 6 | USB-C ↔ USB-A 3.1 Gen 2 cable (3 ft) | 1 | $5.99 | Amazon |
| 7 | CNHL 1300 mAh 130C 6S LiPo (2-pack, XT60) | 1 | $62.99 | Amazon |
| 8 | ISDT 608AC balance charger | 1 | $59.99 | Amazon |
| 9 | Zeee fireproof LiPo bag | 1 | $12.74 | Amazon |
| 10 | BTF-Lighting WS2812B LED strip (5 V) | 1 | $16.99 | Amazon |
| 11 | Tokatuker piezo siren (2–12 V) | 1 | $9.99 | Amazon |
| 12 | Soldering iron kit (w/ multimeter) | 1 | $16.14 | Amazon |
| 13 | Elegoo jumper-wire kit | 1 | $6.98 | Amazon |
| 14 | MOGAOPI component kit (1390 pc) | 1 | $25.99 | Amazon |
| 15 | Raspberry Pi 27 W USB-C PSU | 1 | $23.99 | Amazon |
| 16 | RadioMaster Pocket (ELRS 2.4 GHz) | 1 | $79.99 | Amazon |
| 17 | MicoAir H743 V2 AIO flight controller | 1 | $104.99 | Pyrodrone |
| 18 | Pololu 5 V/5 A regulator (D24V50F5) | 1 | $37.99 | Amazon |
| 19 | COMRUN M2.5 nylon standoff kit | 1 | $9.99 | Amazon |
| 20 | Samsung 30Q 18650 cell | 2 | $13.98 | 18650 Battery Store |
| | **Item subtotal (pre-tax/shipping)** | | **$1,149.21** | |

**Proposed BOM change, not purchased.** Replace the OAK-D Lite (item 5) with a stereo
pair of e-con See3CAM_20CUG (AR0234 global shutter, mono, $89 each, 13.5 g each) on
matched 10 mm M12 lenses, mounted on a rigid, thermally stable baseline bar. Net ~$218
and roughly −15 g, so the upgrade is lighter than the part it replaces. This is what
buys 26 m of reach on an 80 mm ball, and therefore the 20 m/s objective; the OAK-D
Lite's measured ~3.4 m caps the aircraft at ~3.2 m/s. The See3CAM_24CUG is the colour
variant and is the wrong part for this role. Reasoning and the sector cost:
`docs/RESULTS.md` §4.1 and §5.

*Prices June 2026, several on sale. Not yet purchased: 3.3→5 V level shifter for the LED
data line; DJI FPV goggles if manual FPV flight is desired in Week 8.*
