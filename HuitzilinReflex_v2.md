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

Weeks 1–4 pure simulation; hardware bring-up parallel on the bench (props off); real
flight staged late. Each week has a Definition of Done.

**Done** (results and the Week 4 envelope: `CLAUDE.md`):

| Week | Delivered |
|---|---|
| 0 — Procurement ✔ | Full BOM ordered (Appendix A); all Week-5 hardware in hand |
| 1 — Architecture, safety case, sim env ✔ | SITL + Gazebo + ROS 2 up; scripted arm/takeoff/hold via pymavlink |
| 2 — ROS 2 ↔ pymavlink bridge + patrol ✔ | Autonomous closed patrol loop, logged telemetry (43 laps, mean 29.51 s). Airframe fidelity deferred to Weeks 7–8 |
| 3 — Perception pipeline ✔ (2026-07-15) | Simulated OAK-D depth, synthetic scenarios, detection node, 17-bag labeled library + regression harness; held-out test recall 100% |
| 4 — Evasion logic & KF in the loop ✔ (2026-07-27) | Predictive KF + multi-hypothesis tracker + dodge trigger; closed detection → intercept → velocity-spike → alarm (mocked GPIO) over a 7-scenario battery. All four DoD criteria met; result is a capability envelope bounded by sensing, so Week 6 is what moves it |

The Week 6 **sim** lane has since answered the project's central question ahead of the
hardware. A save is a **threshold in time-to-closest-approach at ~0.83 s**, independent of
ball speed; in hover `range = speed × (tca + 0.177)`; every maneuver-side lever is
refuted. As built, the maximum dodgeable ball speed is **~3.5 m/s**, and 20 m/s needs
~21 m of reach on a 7 cm ball — which an **AR0234 global-shutter mono + 10 mm M12** buys
for $99 and 26 g, *lighter* than the OAK-D Lite it replaces. The deficit was angular
resolution, never headline range. Full derivation: `docs/week6_result.md`.

**Remaining** — planned in detail, hardware lane vs software lane, in
`docs/weeks_5_9_plan.md`:

- **Week 5 — FC swap, avionics & power** (the one fabrication step). Swap F722 → H743,
  flash ArduPilot, motor test (props off), bind radio + failsafes + kill-switch, wire
  Pololu BEC, mount Pi + OAK-D. *DoD:* clean bench arm-up, failsafes verified, Pi powered
  with no FC-rail draw.
- **Week 6 — Payload *wiring* & real OAK-D bring-up.** WS2812B (level-shifted) + siren on
  GPIO; depthai-ros streaming over verified USB-3; characterize real stereo noise into
  the KF model; Remote ID / registration. *DoD:* live depth frames + payload triggers
  within latency budget. (The `payload_node` software landed early, in the Week 5 SW
  lane — Week 6 is the hardware it drives.) The Week 6 **sim** lane is separate and
  already closed (`docs/week6_result.md`); what this bullet still owes is the real
  sensor's own reach and rate, the two inputs that law consumes.
- **Week 7 — HITL & tethered hover.** Real H743 + simulated world; tethered/netted hover.
  *DoD:* stable tethered hover with full stack running and logging.
- **Week 8 — Incremental real flight.** Manual hover → autonomous patrol → evasion, only
  inside a netted enclosure with soft projectiles; re-tune KF against real stereo noise.
  *DoD:* one clean autonomous patrol + successful evasion, fully logged.
- **Week 9 — Validation, documentation & retro.** Validation matrix, as-built wiring doc,
  final vlog, sim-vs-real post-mortem. *DoD:* reproducible build doc + validation report.

> **Cut order if behind:** Weeks 1–5 are sacred. Trim real evasion flights first (Week
> 8) — demo evasion in HITL/SITL rather than rush an unsafe test.

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

*Prices June 2026, several on sale. Not yet purchased: 3.3→5 V level shifter for the LED
data line; DJI FPV goggles if manual FPV flight is desired in Week 8.*
