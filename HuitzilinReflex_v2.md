# Project HuitzilinReflex

### Autonomous Agile Patrol & Projectile-Evasion Drone — Summer 2026

> A 9-week development project building a compact, agile, ducted micro-drone that patrols a designated area, signals with light/sound, and autonomously dodges incoming projectiles using an onboard stereoscopic depth stack. Vlogged weekly.

> **v2 — as-built.** This revision replaces the placeholder hardware choices with the parts actually purchased, and re-points the roadmap at those parts. The one structural change from v1: the airframe's stock flight controller (a Betaflight-class F722) is **replaced** with an ArduPilot-capable H7 board, because the autonomy depends on it. See §2 and Week 5.

---

## On the Name

**Huitzilin** (Classical Nahuatl, *huītzilin*) means **"hummingbird."** It's the root of *Huitzilopochtli* — "Hummingbird of the South / Left" — the Aztec sun-and-war deity. In Aztec belief the bird carried far more weight than its size suggests: warriors who fell in battle were said to accompany the sun across the sky and, in time, to return to the world reborn as hummingbirds. The hummingbird *was* the fallen soldier — small, relentless, and back on the wing. Naming the platform after it frames the drone as something that stands a post, takes the hit, and keeps flying.

The bird is also a near-perfect behavioral mascot. It is the only bird that can hover in place, fly straight backward, and reverse direction in a fraction of a second; its wingbeats and reaction times are among the fastest of any vertebrate. That combination — **persistent hovering presence plus explosive, omnidirectional darting** — is exactly the profile the drone targets: loiter calmly on patrol, then snap into a sharp evasive maneuver the microsecond a threat vector appears.

**Reflex** names the second half of the system, and it's meant literally. A reflex is not a decision — it's the response that fires before thought catches up, the way a hand snaps back from a flame before the brain registers the heat. The drone doesn't deliberate over an incoming rock, brick, bottle, or can. It doesn't classify the object, weigh options, or run a plan. The sense → dodge loop is wired tight enough that the maneuver happens as a reaction, not a choice. It just does. So the project name reads, plainly, as *"Hummingbird Reflex."*

---

## 1. Overview & Objectives

HuitzilinReflex is a 3.5-inch ducted micro-development platform that retains full CS/robotics mathematical complexity (perception, state estimation, control) while staying physically safe to test indoors and out. Thick integrated ducts fully isolate the props, so rapid projectile-evasion test cycles can be run without exposed spinning blades.

**Primary objectives**

1. **Autonomous patrol & signaling** — persistent pathing and target tracking, with an integrated strobe + siren warning payload.
2. **Kinematic evasion** — a low-latency trajectory engine that detects an incoming projectile, predicts its intercept point, and commands a sharp dodge.

**Design philosophy:** simulate first, fly last — and *integrate, don't fabricate*. The expensive, fragile parts (the airframe, the depth stack) are pre-assembled, plug-and-play hardware, so effort concentrates where the engineering risk actually lives: the perception + evasion loop. Almost all of that can be developed and de-risked in simulation before a single real propeller spins. The one piece of real fabrication is a single flight-controller swap (Week 5), needed to put ArduPilot on the aircraft. The roadmap below reflects that.

---

## 2. Hardware Base & Payload (as purchased)

| Subsystem | Part purchased | Notes |
|---|---|---|
| **Airframe** | **GEPRC CineLog35 V2 HD** (RunCam Wasp / DJI HD), 3.5" ducted, 6S, **BNF — ELRS 2.4 GHz** | 142 mm wheelbase; integrated ducts isolate the blades for safe rapid-evasion testing. Ships bind-and-fly with a **GEPRC ELRS 2.4 GHz receiver already installed**, and includes 2 spare prop pairs. |
| **Propulsion** (stock, on airframe) | **SPEEDX2 2105.5** motors, **HQProp D-T90** props, 6S | Comes assembled; leaves ~150–200 g payload overhead. The stock 8-bit 45 A 4-in-1 ESC is integrated into the AIO board that gets replaced (see flight controller). |
| **Flight controller** *(replacement — see Week 5)* | **MicoAir H743 V2 AIO** — STM32**H743** + integrated **4-in-1 45 A AM32 ESC**, 2–6S | **Swapped in for the stock GEP F722-45A AIO.** The stock board runs Betaflight and cannot run ArduPilot; the H743 is the required **H7-class** board, runs ArduPilot, and is driven by **pymavlink** over USB/UART. Same **25.5 × 25.5 mm** mount and 45 A rating as the stock board, so it's a like-for-like board swap. Integrated ESC = no separate ESC needed. |
| **Companion computer** | **Raspberry Pi 5 (4 GB)** + official **Active Cooler** + **64 GB SanDisk Extreme** microSD | Runs ROS 2 (native workspace) + the evasion node; depth arrives pre-computed, so CPU stays free for the Kalman loop. Active Cooler prevents thermal throttling under sustained inference. |
| **Bench power (Pi)** | Official **Raspberry Pi 27 W USB-C PSU** | Desk power for the Pi during Phases A–B (SITL/dev and bench bring-up). |
| **Flight power (Pi)** | **Pololu 5 V / 5 A step-down regulator (D24V50F5)** | Steps the 6S flight battery down to a clean **5 V / 5 A** for the Pi *in flight*. Wired to the Pi's 5 V / GND GPIO pins with `usb_max_current_enable=1` set (see §4). Replaces drawing power from the FC rail. |
| **Depth sensor** | **Luxonis OAK-D Lite** + **USB-C ↔ USB-A 3.1 Gen 2 (10 Gbps)** cable | On-chip stereo depth over USB 3 to the Pi. The USB-3 cable (camera USB-C → Pi USB-A) is required for full frame-rate/resolution; a USB-2 cable would throttle it. |
| **Flight battery** | **CNHL 1300 mAh 130C 6S** LiPo (2-pack, XT60) | Matches the airframe's recommended 1050–1300 mAh 6S pack. |
| **Radio** | **RadioMaster Pocket (ELRS 2.4 GHz)** transmitter + **2× Samsung 30Q 18650** cells | Binds to the airframe's built-in GEPRC ELRS 2.4 GHz receiver. The 18650 cells power the radio (charges them internally via USB-C). Used for manual control, failsafe, and kill-switch. |
| **Charging & storage** | **ISDT 608AC** balance charger (1–6S, XT60) + **Zeee** fireproof LiPo bag | Safe charge / discharge / storage of the 6S packs. |
| **Build tools & wiring** | Soldering iron kit (w/ multimeter), Elegoo jumper-wire kit, MOGAOPI component kit (resistors/transistors), COMRUN M2.5 nylon standoff kit | For the FC swap, payload wiring, the buzzer transistor circuit, and mounting the Pi + OAK-D to the frame. |

**Payload — Warning Systems**

**Flashing lights**

- Component: **BTF-Lighting WS2812B** addressable RGB LED strip (5 V, 144 LED/m).
- Interface: Raspberry Pi 5 GPIO (5 V, GND, Data), driven from Python (`rpi_ws281x`) inside the ROS 2 pipeline for low-latency pulsing / strobe patterns.
- **Wiring note:** the Pi's GPIO data line is 3.3 V but the WS2812B expects ~5 V logic, so a small **3.3 V → 5 V level shifter** (e.g. 74AHCT125) is recommended for reliable data. It can be built up from the component kit or added as a cheap board.

**Siren / alarm**

- Component: **Tokatuker 2–12 V piezo siren** (120 dB @ 12 V; still very loud at 5 V), 3.5 mm leads.
- Interface: a Pi 5 digital GPIO pin through a **transistor circuit** (built from the MOGAOPI component kit) to limit current draw.
- Software: toggled programmatically the moment an incoming threat vector is calculated.

---

## 3. Perception & Evasion Pipeline (ROS 2 + Stereo Depth)

The hardware is deliberately simplified — no event cameras, no multi-layer LiDAR, and no hand-built depth math. Depth maps are produced **inside the camera**: the OAK-D Lite's onboard Myriad X VPU computes stereoscopic depth on-chip and streams finished depth/point-cloud frames to the Pi over USB 3. The companion computer never touches raw sensor data — it receives finished depth, so no per-pixel depth reconstruction runs on the Pi at all.

**Data flow**

```
[ OAK-D Lite Camera ]
        |  on-chip Myriad X VPU: stereo depth computed in-camera
        |  USB 3 (USB-C → USB-A) → companion computer (finished depth + point cloud)
        ▼
[ Raspberry Pi 5 (4 GB) — Companion Computer ]
    ├── depthai-ros Node   → republishes PointCloud2 / Depth Image (already computed)
    └── Custom Evasion Node → subscribes to PointCloud2
            ├── Spatial slice: filter noise, extract moving cluster
            ├── Predictive Kalman filter: project intercept coordinate
            └── GPIO trigger: fire buzzer + LED strobes simultaneously
        |   pymavlink: inject SET_POSITION_TARGET_LOCAL_NED over USB/UART
        ▼
[ Flight Controller — MicoAir H743 AIO running ArduPilot ]
    ├── Velocity-loop override: execute high-frequency dodge
    └── Integrated 4-in-1 45 A ESC → SPEEDX2 motors
```

**Evasion software breakdown**

1. **Object detection** — the ROS 2 node scans the incoming depth frames; an incoming projectile shows up as a sudden, drastic cluster of depth-value differentials.
2. **Trajectory tracking** — the cluster's centroid (X, Y, Z) is piped continuously into a predictive Kalman filter to recover velocity.
3. **Action trigger** — if the predicted intercept path crosses the drone's spatial boundary, the node toggles the alarm GPIO and a `pymavlink` script issues a high-rate `SET_POSITION_TARGET_LOCAL_NED` command for an immediate ~1.5 m/s velocity spike to clear the object.

---

## 4. Critical Avionics, Power & Signal Notes

- **Flight controller — MicoAir H743 V2 AIO:** the STM32**H743** is the required H7-class processor; it runs real-time ArduPilot control laws and services high-rate `pymavlink` velocity commands without stalling the loop. Its **integrated 4-in-1 45 A AM32 ESC** drives the stock SPEEDX2 motors, so no separate ESC is required. It replaces the stock GEP F722-45A AIO, which is a Betaflight-class F7 board and is not an ArduPilot target.
- **The swap is a soldering job (Week 5):** the four motors (3 wires each), the battery leads, and the HD-VTX/camera + ELRS-receiver connections move from the stock AIO to the MicoAir. Same 25.5 × 25.5 mm mount means it drops into the existing frame standoffs. If fine-pitch soldering isn't comfortable, hand this one step to a local FPV/hobby shop.
- **Control interface — pymavlink:** a native Python 3 `pymavlink` script connects directly to ArduPilot on the H743 and injects high-rate `SET_POSITION_TARGET_LOCAL_NED` velocity commands. No middleware agent sits between the evasion node and the flight controller.
- **Isolated power — Pololu 5 V/5 A BEC:** the Pi 5 can spike to **5 V / 5 A** during peak inference and must **never** draw from the flight controller's rail. The Pololu **D24V50F5** takes the 6S battery (well within its ~38 V input ceiling) down to 5 V / 5 A, wired to the Pi's **5 V / GND GPIO pins**. Because that bypasses USB-C Power Delivery negotiation, set **`usb_max_current_enable=1`** in the Pi's `config.txt` so the Pi grants full current to its USB ports (otherwise it throttles them and starves the OAK-D).
- **Manual video feed (optional):** the airframe's RunCam Wasp is part of the **DJI HD digital FPV** system. Manual FPV flying (Week 8) therefore needs **DJI FPV goggles**; if all manual checks are flown **line-of-sight**, goggles are not required.
- **Depth quality — stereo vision:** stereoscopic depth uses no active IR emission, so there's no Multi-Path Interference to fight. Stereo carries its own failure modes (low-texture surfaces, depth dropouts at range, frame-rate dips), so that noise budget lives in the Kalman filter's tuning rather than in a separate denoising stage.

---

## 5. Roadmap (Simulation-First)

**Roughly the first half is pure simulation.** Hardware bring-up runs in parallel on the bench (props off), and real flight is staged incrementally and late. Each week has an explicit *Definition of Done* (DoD) so you can't "feel finished" without hitting a checkpoint.

### Week 0 — Procurement (complete)

- Full bill of materials finalized, verified, and ordered (see **Appendix A**). All parts needed through Week 5 are accounted for.
- Key procurement decision locked: the **MicoAir H743 AIO** flight controller is purchased to replace the stock board; the **Pololu 5 V/5 A BEC** powers the Pi in flight; the **RadioMaster Pocket (ELRS)** + 18650 cells drive manual control and bind to the airframe's built-in ELRS receiver.
- *DoD:* every Week 5 hardware item in hand; no open ordering dependencies.

### Phase A — Foundations & Simulation (Weeks 1–4)

**Week 1 — Architecture, Safety Case & Sim Environment**

- Lock requirements, message contracts (topics/types), and the node graph.
- Write the safety case: failure modes, geofence/RTL behavior, kill-switch, test-enclosure rules.
- Stand up the toolchain: ArduPilot **SITL** + **Gazebo** + ROS 2 in a **native, flat ROS 2 workspace** on your OS. (SITL is board-agnostic, so sim work is identical whether the target is the F722 or the H743 — no dependency on the FC swap.) Working natively sidesteps container pass-through for serial ports (the FC link) and the GPU (Gazebo rendering).
- *DoD:* a simulated quad arms, takes off, and holds position in Gazebo, driven through `pymavlink`, on a teammate's fresh checkout.

**Week 2 — Flight Dynamics & ROS 2 ↔ pymavlink Bridge in SITL**

- Tune the simulated airframe (mass, inertia, motor model) to approximate the real 3.5" duct on 6S (SPEEDX2 2105.5 / D-T90).
- Build and harden the ROS 2 → ArduPilot control path using a native `pymavlink` script; validate `SET_POSITION_TARGET_LOCAL_NED` round-trips and command rate.
- Prototype the patrol path-follower as a ROS 2 node against the sim.
- *DoD:* drone autonomously flies a closed patrol loop in Gazebo with logged telemetry.

**Week 3 — Perception Pipeline on Synthetic & Recorded Depth**

- Add a simulated **stereo depth** sensor in Gazebo (model it on OAK-D Lite resolution/FOV/frame-rate); generate synthetic "thrown object" scenarios (vary speed, angle, size, miss-distance).
- Build the detection node: differential clustering and centroid extraction directly on the depth stream.
- Start a labeled rosbag library of evasion scenarios for regression testing.
- *DoD:* detection node flags ≥95% of simulated incoming clusters with a quantified false-positive rate.

**Week 4 — Evasion Logic & Kalman Filter in the Loop**

- Implement the predictive Kalman filter; tune process/measurement noise against the synthetic scenarios.
- Close the loop: detection → intercept prediction → velocity-spike command → GPIO/alarm trigger (mocked in sim).
- Sweep parameters (dodge magnitude, trigger threshold, latency budget) and chart hit-vs-miss outcomes.
- *DoD:* in SITL, the drone dodges a defined battery of simulated projectiles above a target success rate, with end-to-end latency measured and within budget.

### Phase B — Hardware Bring-Up (Weeks 5–6, parallel to sim refinement)

**Week 5 — Flight-Controller Swap, Avionics & Power**

This week is the one real fabrication step. The airframe arrives BNF, but its brain is replaced:

- **Swap the flight controller:** remove the stock **GEP F722-45A AIO** and install the **MicoAir H743 V2 AIO** in the same 25.5 × 25.5 mm mount. Move over the 4 motors (3 wires each), battery leads, and the HD-VTX/camera + ELRS-receiver connections. *(Photograph the stock wiring first; if soldering isn't comfortable, outsource this one step.)*
- **Flash & configure ArduPilot** on the H743: set frame to quad-X, run the **motor test** to confirm rotation direction/order, calibrate IMU/accelerometer, and configure the receiver protocol.
- **Bind the radio:** pair the **RadioMaster Pocket (ELRS)** to the airframe's ELRS receiver; set up failsafes and the kill-switch.
- **Power the Pi:** wire the **Pololu 5 V/5 A BEC** from the battery to the Pi's GPIO 5 V/GND and set `usb_max_current_enable=1`. (Bench dev still uses the 27 W USB-C PSU.)
- **Mount** the Pi + OAK-D to the frame with the M2.5 standoffs.
- Bench-test motors **props off**; verify IMU, calibration, RC link, failsafes, kill-switch.
- *DoD:* clean ArduPilot bench arm-up (props off), all failsafes verified, RadioMaster ↔ drone bound, and the Pi powered from the BEC with **no FC-rail draw**.

**Week 6 — Payload Wiring & Real OAK-D Lite Bring-Up**

- Wire the **WS2812B** strip (via the 3.3→5 V level shifter) and the **piezo siren** (transistor circuit from the component kit) to Pi GPIO; validate timing/latency.
- Bring up the real **OAK-D Lite** over the **USB-3** cable via `depthai-ros`; confirm on-chip depth streams to the Pi and that the CPU stays free of any depth reconstruction. (Confirm the link enumerates as USB-3, not USB-2.)
- Characterize real-world stereo behavior (low-texture dropouts, range limits, frame-rate) and feed it into the Kalman noise model.
- Implement Remote ID; confirm registration/regulatory items (see §6).
- *DoD:* live depth frames published from the real sensor; payload triggers fire within the latency budget on real GPIO.

### Phase C — Integration, Flight & Validation (Weeks 7–9)

**Week 7 — Hardware-in-the-Loop & Tethered Hover**

- Run HITL (real **MicoAir H743** FC, simulated world) to validate timing on real silicon before free flight.
- Tethered / netted hover tests; sensor calibration in the real flight envelope.
- *DoD:* stable tethered hover with the full software stack running and logging.

**Week 8 — Incremental Real Flight**

- Stage it: manual hover → autonomous patrol loop → evasion, **only inside a netted enclosure**, projectiles soft and controlled.
- *Manual hover note:* flying by video needs **DJI FPV goggles** (the RunCam Wasp is DJI's HD system); if flying line-of-sight, goggles aren't required.
- Compare real evasion outcomes against the sim regression suite; fix the sim-to-real gaps.
- Use the Phase B schedule buffer to **re-tune the predictive Kalman filter** against real stereo edge noise and frame-rate drops, rather than rushing.
- *DoD:* at least one clean autonomous patrol + successful evasion in the enclosure, fully logged.

**Week 9 — Validation, Documentation & Retro**

- Run the full validation matrix; write up results, the **as-built wiring/architecture** (including the FC swap), and tuning notes.
- Produce the final vlog/build documentation and a post-mortem on sim-vs-real accuracy.
- *DoD:* reproducible build doc + validation report a stranger could follow.

> **Stretch / cut order if you fall behind:** keep Weeks 1–4 (sim) and 5 (FC swap + safe hardware checkout) sacred. The first things to trim are the *real* evasion flights in Week 8 — you can demo evasion convincingly in HITL/SITL and defer free-flight dodging rather than rush an unsafe test.

---

## 6. Safety, Legal & Ethical Considerations

These aren't optional paperwork — for this platform they shape the design.

- **Flight rules (US/FAA):** register the aircraft, broadcast **Remote ID**, and respect the standard constraints — keep it within visual line of sight, don't fly autonomously over people or moving vehicles without the appropriate waiver, and stay clear of controlled airspace.
- **LiPo safety:** charge the 6S packs on the **ISDT 608AC** in the **Zeee fireproof bag**, on a non-flammable surface, never unattended. Store at storage charge. The 18650 cells for the radio travel in a case, never loose.
- **Test only in enclosures:** all projectile-evasion testing happens inside netting, with soft projectiles and a hardware kill-switch within reach. The ducts make blade contact safe; they do not make an uncommanded flyaway safe.
- **People-tracking is sensitive:** "patrol," "target tracking," and "curfew enforcement" mean this thing can record and follow people. Decide up front what it records, where footage goes, how long it's kept, and don't point it at anyone who hasn't consented. Treat the warning payload as a *signal*, never as a way to harass or corner a person.
- **Fail safe, not aggressive:** the default on any sensor dropout, link loss, or low battery should be a calm geofenced hover → RTL/land, never an evasive lunge.

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

*Prices captured June 2026; several were sale prices (notably the airframe, ~$200 off list). Excludes tax and per-vendor shipping; orders span five sellers. Possible small add-ons not yet purchased: a 3.3→5 V level shifter for the LED data line, and DJI FPV goggles if manual FPV flight is desired in Week 8.*

---

*Project HuitzilinReflex — "Hummingbird Reflex." Loiter like a hover, dodge like a dart.*
