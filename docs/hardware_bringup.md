# Hardware Bring-Up (Weeks 5–6)

Physical assembly checklist for turning the purchased BOM into a bench-testable
aircraft + companion stack. Sim work (Weeks 1–4) does not block this — it can
run in parallel, props off, on the bench.

## 0. Before touching anything

- [ ] Work on a non-conductive, static-safe surface. Discharge yourself before handling the FC/ESC.
- [ ] Battery disconnected for the entire FC swap. Only reconnect for the bench motor test at the end.
- [ ] Photograph the stock GEP F722-45A AIO wiring (all 4 motor phases, battery leads, HD-VTX/camera plug, ELRS receiver plug) before desoldering anything — this is your only reference once it's off.

## 1. Flight controller swap (GEP F722 → MicoAir H743 V2 AIO)

- [ ] Desolder/unplug the 4 motor wires (3 phases each) from the stock AIO. Label which motor came from which corner (front-left/front-right/rear-left/rear-right) — motor order matters for ArduPilot's quad-X mapping.
- [ ] Desolder the battery (XT60 pigtail) leads from the stock AIO.
- [ ] Move the HD-VTX/RunCam Wasp connector and the ELRS receiver connector to the corresponding pads on the MicoAir H743.
- [ ] Remove the stock AIO from the frame (4 standoff screws, 25.5×25.5 mm pattern).
- [ ] Mount the MicoAir H743 in the same position using the same standoffs — it's a like-for-like mechanical fit.
- [ ] Resolder motors to the H743's integrated 4-in-1 ESC pads, same corner mapping as labeled above.
- [ ] Resolder the battery pigtail to the H743's battery pads.
- [ ] Visually re-check every joint against your reference photos before power-up. No shorts across ESC pads.

If fine-pitch soldering isn't comfortable, this is the one step worth outsourcing to a local FPV/hobby shop — everything else in Weeks 5–9 assumes the swap is already done.

## 2. Flash & configure ArduPilot on the H743

- [ ] Flash ArduCopter firmware (H7-class target) via the MicoAir's USB-C, using the ArduPilot firmware flashing tool.
- [ ] Set `FRAME_CLASS=1` (Quad), `FRAME_TYPE=1` (X) explicitly — don't rely on defaults. (Same failure mode as the sim SITL sharp edge in `CLAUDE.md`: `FRAME_CLASS=0` silently accepts arm/takeoff with zero lift.)
- [ ] Run the built-in motor test (Mission Planner/QGC) — props still off — and confirm all 4 motors spin in the correct order/direction for quad-X. Fix wiring, not software, if a motor is backward — swap 2 of the 3 phase wires rather than remapping in firmware.
- [ ] Calibrate accelerometer/IMU (six-position calibration).
- [ ] Set the receiver protocol for ELRS on the RC input.
- [ ] Set up geofence/RTL and failsafe parameters per `docs/SAFETY_CASE.md` before any prop-on test.

## 3. Bind the radio

- [ ] Put the airframe's built-in GEPRC ELRS 2.4 GHz receiver into bind mode.
- [ ] Bind the RadioMaster Pocket to it.
- [ ] Configure a hard kill-switch on a dedicated momentary switch, not a shared/mode switch.
- [ ] Trigger an RC-loss failsafe test (props off) and confirm the configured failsafe action actually fires.

## 4. Power the Pi from the flight battery

- [ ] Wire the Pololu D24V50F5 (5 V/5 A step-down) input to the 6S battery rail — **not** to the FC's own power rail.
- [ ] Wire the regulator's 5 V/GND output to the Pi 5's 5 V/GND GPIO pins (not the USB-C port — GPIO power bypasses PD negotiation).
- [ ] In `config.txt`, set `usb_max_current_enable=1` so the Pi grants full current to USB (otherwise the OAK-D gets throttled/starved).
- [ ] Confirm with a multimeter that the regulator outputs a clean 5.0–5.2 V under no load before connecting the Pi.
- [ ] For bench dev without the battery connected, keep using the 27 W USB-C PSU — don't rely on the BEC path until it's verified.

## 5. Mount the companion stack

- [ ] Mount the Pi 5 (+ Active Cooler) and the OAK-D Lite to the frame using the M2.5 nylon standoffs.
- [ ] Route the USB-C→USB-A 3.1 Gen 2 cable from the OAK-D to the Pi. Keep it clear of prop wash paths and away from the ELRS antenna.
- [ ] Route the Pololu regulator's input leads to the battery XT60 without adding a second connector inline if avoidable — resistance/voltage sag matters at 5 A peak.

## 6. Bench validation (props off, then props on)

- [ ] Props off: arm via GCS, confirm IMU/EKF healthy, confirm all failsafes as configured in step 3.
- [ ] Props off: confirm Pi boots and stays up on BEC power alone (battery connected, USB-C PSU disconnected) — this is the real test of step 4.
- [ ] Props off: confirm OAK-D enumerates as USB-3 (not USB-2) on the Pi — check `lsusb -t` or `dmesg` for SuperSpeed vs High-Speed. A USB-2 negotiation throttles depth throughput and needs the cable/port re-seated.
- [ ] Only after all of the above pass clean: reinstall the stock props (2 spare pairs shipped with the airframe) and do a brief, held-down static throttle-up test to confirm motor order and thrust direction before anything free-hovers.

## 7. Payload wiring (Week 6, can run in parallel with §6)

- [ ] WS2812B LED strip: 5 V + GND from Pi GPIO, data line through a 3.3→5 V level shifter (74AHCT125 or equivalent) before the strip's DIN — the Pi's 3.3 V logic is out of spec for the strip directly.
- [ ] Piezo siren: through a transistor switch circuit (NPN + flyback-safe wiring) off a Pi GPIO pin, not directly off the pin — the siren draws more current than a GPIO pin can safely source.
- [ ] Validate both trigger within the target latency budget once wired, using a simple GPIO toggle script before wiring them into the real evasion node.

## 8. Real OAK-D Lite bring-up (Week 6)

Sim used a Gazebo depth camera at 640×480 / 15 Hz. This step replaces it with the
real sensor and re-measures the noise the Kalman filter was tuned against.

- [ ] Install `depthai-ros` on the Pi and bring up the OAK-D Lite over the USB-3 cable. Confirm depth/point-cloud topics publish.
- [ ] Confirm the on-chip VPU is doing the stereo work — Pi CPU should stay low. Any per-pixel depth reconstruction on the Pi means the pipeline is misconfigured.
- [ ] Measure the *delivered* frame rate and resolution at the ROS layer, not the camera's rated spec. Sim already found `ros_gz_bridge` could not sustain 30 Hz at 640×480, and the real USB path has its own ceiling. The detector and the tracker's `min_track_updates` gate are timed against whatever this turns out to be.
- [ ] Characterize the failure modes sim did not model: low-texture dropouts, depth holes at range, frame-rate dips under load. Record where depth stops being trustworthy — that range is the real analogue of `roi_max_range_m`.
- [ ] Feed the measured noise back into the Kalman measurement covariance. Do not carry the sim-tuned values over unexamined.

> Week 4 closed with the dodge envelope bounded by *detection* range and frame
> rate, not by tuning (`CLAUDE.md`). Whatever this step measures sets where the
> real envelope lands, so treat these numbers as a headline result.

## 9. Remote ID & regulatory (Week 6)

- [ ] Register the aircraft with the FAA and mark it with the registration number.
- [ ] Implement/enable Remote ID broadcast and verify it actually transmits (a Remote ID receiver app on a phone is the quickest check).
- [ ] Re-read `docs/SAFETY_CASE.md` geofence/RTL/kill-switch sections against the *as-built* aircraft, not the sim model.
- [ ] Confirm the intended test site is clear of controlled airspace, and that all projectile testing is planned inside netting per `HuitzilinReflex_v2.md` §6.

## Sequencing note

Steps 1–3 must happen in order (swap → flash/configure → bind); the battery
only needs to be connected for the tail end of step 3 onward. Step 4 (Pi
power) and step 7 (payload) don't depend on the FC being flown yet and can be
done any time after step 1. Step 6 is the gate before any Week 7 HITL or
Week 8 real flight work — don't skip straight to flight once wiring "looks
right."

Steps 8 and 9 are Week 6 and independent of each other; step 8 needs only the Pi
powered and the OAK-D mounted (steps 4–5), not a flyable aircraft, so it can
start while the FC work is still in progress. Step 9 must be complete before any
outdoor flight, regardless of how the bench work is going.
