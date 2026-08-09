# Week 6 result — can HuitzilinReflex dodge a 20 m/s projectile?

**Status: answered.** Not with the sensor it carries. With a $99, 26 g sensor swap —
which is *lighter* than the part it replaces — yes, with margin.

This document exists because the project's success criteria demand one of two outcomes:
either demonstrate reliable 20 m/s evasion, or **prove quantitatively that it is
unattainable, state the closest achievable ball speed, and name the exact
hardware/architectural change that closes the gap.** What follows is the second, plus the
measurement that shows the first is reachable after the change.

Every number here is measured in SITL unless marked as a derivation. Where a number is
derived, the derivation is shown so it can be checked rather than believed.

---

## 1. The answer in one table

| Question | Answer |
|---|---|
| Can it dodge 20 m/s **as built**? | **No.** |
| What is the binding constraint? | **Sensor reach.** Every maneuver-side lever is refuted — see §4. |
| Closest achievable ball speed as built | **~3.5 m/s** in hover, with the OAK-D Lite's measured 3.4 m reach |
| What does 20 m/s actually require? | **~21 m** of detection range on a 7 cm ball at ≥15 Hz |
| The exact change that closes it | **AR0234 global-shutter mono + 10 mm M12 lens** (e-con See3CAM_24CUG, $99, 26 g, 60 fps) → 26 m reach → **26.8 m/s** |
| What it costs | 73° → 32° horizontal FOV, and stereo depth (replaced by a known-size range prior) |

The headline is counter-intuitive and worth stating plainly: **the deficit is angular
resolution, not range.** The OAK-D Lite is not "a short-range camera." It is a camera whose
pixels are 6.2× too coarse to resolve a tennis ball at 21 m. Nothing marketed as
longer-range fixes that unless its IFOV is finer, which is why most of §5 is rejections.

---

## 2. The measurement: a save is a threshold in tca

`tca` is time-to-closest-approach at the moment the dodge commits. It is the independent
variable of this entire investigation.

### 2.1 What is being scored

A throw is scored **saved** if and only if:

    counterfactual_min_m <= 0.30 m   AND   actual_min_m > 0.30 m

`counterfactual_min_m` is how close the ball *would* have come with no dodge, recovered by
extrapolating the drone's pre-dodge constant-velocity track against the ball's recorded
ground truth. A throw with `counterfactual > 0.30 m` was never on a hit course and
**measured nothing about dodging** — it is excluded, not counted as a failure.

This matters more than it sounds. The battery's own `dodged` field counts *trigger
firings*, not saves. Scoring on `dodged` reads a working system as broken, and scoring
every throw against a dead-centre worst case is how the earlier 8 m/s "34% success"
figure arose — those balls were already passing at 0.20–0.30 m.

### 2.2 The result

60 throws in `hover_mode` on a true hit course, `dodge_speed_mps` 3.0, oracle sensor:

| ball speed | oracle range (an **INPUT**) | saved |
|---|---|---|
| 14 m/s | 8 m | 0/10 |
| 14 m/s | 12 m | 0/10 |
| 14 m/s | 16 m | 3/8 |
| 14 m/s | 20 m | **9/9** |
| 20 m/s | 21 m | 3/10 — every save at tca ≥ 0.80 s |
| 20 m/s | 16 m (negative control) | **0/10** — tca 0.41–0.69 s, all below threshold |

Binned by tca rather than by range, the 14 m/s data is a step function:

| tca bin | n | saved | rate |
|---|---|---|---|
| 0.2–0.4 | 8 | 0 | 0% |
| 0.4–0.6 | 8 | 0 | 0% |
| 0.6–0.8 | 9 | 0 | 0% |
| **0.8–1.0** | **8** | **8** | **100%** |
| 1.0–1.2 | 3 | 3 | 100% |
| 1.2–1.6 | 1 | 1 | 100% |

**Highest failure 0.79 s, lowest save 0.83 s, no overlap.** This is a threshold, not a rate.

### 2.3 It is speed-independent — and that was pre-registered

The 20 m/s run was designed, and its prediction written down, **before** it was flown, with
its own negative control at a range chosen to fall below the threshold.

- Crossing at 20 m/s: **0.81 s failed / 0.80 s saved**, against 0.79/0.83 at 14 m/s.
- Fitted effective escape, independently at each speed:
  `delta = 0.295·tca^3.15` (14 m/s, R² 0.65, n=31) and `0.337·tca^3.69` (20 m/s).
- Required tca from the fits: **0.86 s** and **0.83 s**.

**Required tca does not depend on ball speed. Only the range needed to buy it does.**

### 2.4 Converting tca to a sensor requirement

In hover the drone is stationary, so closing speed is the ball alone:

    range = speed × (tca + 0.177)

The 0.177 s is pipeline plus commit overhead. Substituting tca = 0.80 s:

| ball speed | required reach |
|---|---|
| 8 m/s | 8.3 m |
| 14 m/s | 14.5 m |
| 20 m/s | **20.7 m** |

Inverting for the real OAK-D Lite's measured **~3.4 m**:

    max dodgeable ball speed = 3.4 / (0.80 + 0.177) = 3.48 m/s

**That is the closest achievable ball speed as built, and it is the honest headline** —
not the 14 m/s that Week 4's patrol battery happened to fail at.

### 2.5 Caveats that bound these numbers

- **Hover only.** A patrolling drone closes faster (effective 24 m/s against a 14 m/s
  ball); required range scales with *closing* speed, roughly 1.7× more at 14 m/s.
- **`dodge_speed_mps` was 3.0**, not the shipped 1.5 — but that lever plateaus (§4.5), so
  it does not move the threshold.
- Conversion of raw displacement into miss distance is ~0.6 (median, the 16 m cell — the
  only cell where displacement is large enough for the ratio to mean anything).
- **`detection_range_m` is an INPUT, never a result.** A dodge scored under the oracle is a
  claim about the tracker, trigger and airframe *given* a sensor with that reach. It is
  never evidence that such a sensor exists. §5 is where that debt is paid.

---

## 3. Two instrument faults that had to be fixed before any of this was measurable

Both invalidated earlier results. They are recorded because the corrected numbers only make
sense against them.

### 3.1 Patrol could not deliver a hit at range — 0 of 98 throws

Across 98 patrol throws, **not one** arrived within 0.109 m. The dodge was being scored
against balls that were going to miss anyway.

The cause was not the aim direction — measured lead error is ~5°, and the ~90° figure
that circulated earlier was an ordinal-matching artefact in the analysis, not a real
error. The cause was the **throw-window gate**: it holds each throw until the drone
reaches `min_cruise_frac` 0.95 of a **rolling-max** cruise. Measured max was 3.49 m/s
against a 2.09 m/s median, so the lead extrapolated a peak speed the drone never
sustained — over-lead +1.5 m at 1.2 s of ball flight, +2.6 m at 1.5 s, 96–98% along-track.

`-p hover_mode:=true` fixed it with no source change: **40/40 on a true hit course**,
counterfactuals 0.07–0.18 m. It also removed the straight-leg requirement that had made
ranges ≥18 m unmeasurable (patrol skipped 9 of 10 throws there).

Hover is the *trustworthy* instrument, not the *generous* one — it exposes a real dead
time out to t ≈ 0.20 s that patrol's extrapolation had been hiding.

### 3.2 Patrol escape displacement was inflated 3–20×

Patrol's counterfactual extrapolates a straight line through a vehicle that is tracking
waypoints, so the vehicle's own path curvature lands in the escape term:

| | patrol | hover |
|---|---|---|
| median fit residual | 0.0112 m | **0.0011 m** |
| escape at matched tca 0.385 s | 0.1268 m | **0.0066 m** |

A 19× disagreement at matched tca. On top of that, two *identically configured* control
arms drifted **1.58× apart** — larger than the effects they were controlling for.

**Consequence: any null scored on patrol escape displacement with n ≤ 8 is unsafe.** One
entry on the project's own do-not-re-run list (`dodge_speed_mps` 1.5→4.0) was exactly
that, which is why it was re-tested properly in §4.5.

### 3.3 The oracle was under-delivering its own range

`offset_forward_m` was 15.0 while the requested range was 12 m, so the ball entered the
detection gate from *inside* it: first detection landed at 9.9–11.3 m. Every pre-fix
"12 m" cell had actually measured a ~10–11 m sensor. Fixed by requiring
`offset_forward_m ≥ detection_range_m + 8.5` at 20 m/s.

**Verification rule, permanently:** `first_det_range_m` in the CSV must match the launched
`detection_range_m` to ~0.2 m, every run.

---

## 4. Why the answer is sensing: every maneuver-side lever, refuted

This is the load-bearing half of the "prove it is unattainable" requirement. If any of
these had worked, the answer would be software, not hardware.

### 4.1 The single result that retires most of them at once

From dataflash over a controlled `WP_ACC` sweep, comparing commanded velocity change
against achieved:

| arm | commanded Δv | achieved Δv |
|---|---|---|
| 1 | 0.81 m/s | **0.93 m/s** |
| 2 | 1.17 m/s | **1.26 m/s** |
| 3 | 0.97 m/s | **1.47 m/s** |

**The vehicle over-delivers on what it is asked for, in every arm.** A system that exceeds
its own command is not limited by anything downstream of the command. That single row
closes the airframe, the attitude loop, `ATC_ANGLE_MAX` (21–28° used of 30 available),
`PSC_JERK_NE` and `WP_ACC` simultaneously — and answers the attitude/ACRO-step experiment
without flying it.

**The dodge only ever asks for ~1 m/s of horizontal step.**

### 4.2 `PSC_JERK_NE` — refuted twice over

This was the most credible remaining hypothesis: at 5 m/s³ the controller would need
5.66/5 = 1.13 s to reach the acceleration the 30° tilt ceiling permits, four times the
whole tca window. A flown 5/20/60 A/B settles it:

1. **The limit is not being reached.** Pure jerk-limited displacement `j·t³/6` at 5 m/s³ is
   0.0225 m at t = 0.30 s. Measured baseline escape is 0.103 m — **4.6× above its own jerk
   envelope**, and 9.2× at t = 0.20 s. A limit you already operate far above is not
   binding, and this holds without any between-arm comparison at all.
2. **The response is not monotonic.** Mean escape at t = 0.30 s: jerk 5 → 0.103 m,
   20 → **0.077 m**, 60 → 0.137 m. The middle arm is the lowest, and within-arm spread
   exceeds any between-arm difference.

### 4.3 `WP_ACC` — the renamed `WPNAV_ACCEL`

Peak demand is 1.43 m/s² against a 5.0 clamp; at the 2.5 default it binds for only 7% of
the window. The only real effect is *downward*: starving it to 1.0 cuts escape ~3×.

Worth recording as a naming trap: `WPNAV_ACCEL` was previously believed absent from this
build. It exists, renamed `WP_ACC`, exactly as `PSC_JERK_XY` became `PSC_JERK_NE`.
`PSC_ACC_XY`, `ANGLE_MAX` and `PSC_VELXY_P` genuinely are absent.

### 4.4 `evade_accel_ff_mps2` — null in flight

A 12-trial in-flight A/B found no dodge gain (0.107 m pre-fix vs 0.091 m post-fix; both
arms commanded real motion). An earlier 28/28 zero-metre result at *settled hover* did not
reproduce in flight, so it never invalidated the lane.

### 4.5 `dodge_speed_mps` — the last lever, and it plateaus

Within-session hover A/B at 14 m/s, oracle 16 m, 10 repeats per arm, readbacks verified
against the driver log, baseline eeprom intact on entry and exit of every arm.

| arm | commanded | on-course | saved | escape scale `a` |
|---|---|---|---|---|
| dspd15 | 1.5 (shipped) | 10 | 6 (60%) | 0.3644 — 1.000× |
| dspd30 | 3.0 | 9 | 6 (67%) | 0.3806 — 1.044× |
| dspd60 | 6.0 | 10 | 6 (60%) | 0.3991 — 1.095× |

**4× the command buys 1.095× the escape.** Saves are flat and non-monotonic.

A methodological note, because it nearly went the other way. The pre-registered readout
compared arms inside a "matched" tca band of 0.70–0.92 s. That band is 0.22 s wide while
escape scales as tca³·⁵, and the arms did not sample it evenly (median tca 0.790 / 0.850 /
0.835). Read naively it showed a 22.6% rise at 4× command. That rise is the sampling, not
the command. Controlling properly — pool one exponent across all arms, fit only per-arm
scale:

    pooled:  delta = 0.3811 · tca^3.55        R² 0.54, n = 27
    predicted delta at tca 0.85:  0.2045 / 0.2136 / 0.2240 m   — none reach 0.30
    log-residual vs pooled:  -0.045 / -0.001 / +0.046,  sd 0.28-0.59  => inside noise

Escape scales as roughly **command^0.065**. Clearing 0.30 m at tca 0.85 by command alone
would need ~350× the shipped value — about 500 m/s.

### 4.6 A real low-tca failure mode, found in passing

Two throws had the dodge move the drone **toward** the ball: dspd15 rep 0 (tca 0.710,
delta −0.1456, counterfactual 0.296 — on the 0.30 m boundary, so arguably ill-conditioned)
and **dspd60 rep 7 (tca 0.620, delta −0.0615, counterfactual 0.160 — nowhere near the
boundary)**. The second kills the boundary-artefact explanation for the first.

Both sit at the lowest tca in their arm, and the worse one came at the *highest* command.
**Below the threshold the dodge is not merely insufficient; it is sometimes harmful.** This
is untested as its own hypothesis and touches `horizontal_away_bearing()` / `away_hint`,
which stay out of conclusions until a controlled experiment tests them.

### 4.7 The rest of the null list

Refuted earlier and not re-opened: `roi_max_range_m` 5→8 · `min_track_updates` 3→2 ·
`cluster_min_points` 5→3 · 320×240 @ 30 Hz · the multi-hypothesis tracker · track
continuity (the associator does not fragment the ball — 1.05 spawns per throw, and MHT and
one unbroken filter commit at the same tca) · the command path (probe ok 16/16) · dodge
direction (~5° error) · vertical escape.

---

## 5. The sensor that closes the gap

### 5.1 The requirement, stated in the units that decide it

A 0.07 m ball at 21 m subtends **θ = 0.07/21 = 3.333 mrad = 0.191°**.

The quantity that matters is **IFOV** = pixel pitch / focal length, in rad/px. Pixels
across the ball is `N = θ / IFOV`, and range at a given N is `R = 0.07 / (N · IFOV)`.

**Calibrate N from the one measured number rather than guessing.** The OAK-D Lite mono pair
is OV7251, 640×480, 3.0 µm, HFOV 73° → active width 1.92 mm → f = 1.297 mm →
**IFOV = 2.312 mrad/px**. At the measured 3.4 m reach, the ball spans **8.9 px**. That is
the pipeline's empirical detection threshold. At 21 m the same camera gets **1.44 px**.

Stereo is separately dead at that range: disparity = B·f/Z = 0.075 × 432 / 21 =
**1.54 px** (against 9.5 px at 3.4 m). A 150 mm airframe cannot fix that, and prop flex of
0.1 px would corrupt what little disparity remained.

**Holding 8.9 px at 21 m needs IFOV ≤ 0.375 mrad — 6.2× finer than today.** At the same
3.0 µm pitch, that is a 6.2× longer lens.

### 5.2 The recommendation

> **onsemi AR0234CS, 1920×1200 global shutter, 3.0 µm, with a 10 mm M12 lens.**
> e-con Systems **See3CAM_24CUG** — **$99**, **26 g with lens**, 30×30×26 mm,
> **0.86–2.02 W**, USB 3.1 Type-C, **60 fps** at full resolution.

| lens | IFOV | FOV_H | px on ball @ 21 m | reach @ 8.9 px | max ball speed |
|---|---|---|---|---|---|
| 6 mm | 0.500 mrad | 51.3° | 6.7 | 15.7 m | 16.1 m/s |
| 8 mm | 0.375 mrad | 39.6° | 8.9 | 21.0 m | 21.5 m/s (marginal) |
| **10 mm** | **0.300 mrad** | **32.1°** | **11.1** | **26.2 m** | **26.8 m/s** |
| 12 mm | 0.250 mrad | 27.0° | 13.3 | 31.4 m | 32.2 m/s |

Speed column is `reach / 0.977`, i.e. tca 0.80 + 0.177 s overhead. **The model reproduces
the measured 3.48 m/s from the OAK-D's 3.4 m** — a consistency check, not independent
evidence.

**Take 10 mm rather than 8 mm.** At 8 mm the answer depends on whether the true threshold
is 8.9 px (21.5 m/s, passes) or 10 px (19.1 m/s, fails). 10 mm clears 20 m/s under either,
so the ambiguity does not have to be resolved before buying.

It is **lighter than the part it replaces** — 26 g against the OAK-D Lite's 61 g — at
comparable power.

### 5.3 Two improvements that come free with the same part

- **60 fps instead of 15.** Confirming three track updates drops from 0.207 s to 0.050 s.
  That is 0.157 s off the 0.177 s overhead term — worth **3.1 m of equivalent reach** at
  20 m/s, for zero grams. **Re-derive the 0.177 s constant after the swap; it will not be
  0.177 s.**
- **Bearing becomes very precise.** A 0.2 px centroid is 0.075 mrad = **1.6 mm of
  cross-range at 21 m**. The miss-distance vector that decides *which way* to dodge is
  essentially exact.

### 5.4 What it costs, and how to pay it

**Field of view: 73° → 32°.** Restore it by tiling — two units at ±16° give 64° for 52 g
and ~4 W; three give 96°.

**Stereo depth.** Replace it with the known-size prior, `R = 0.07 / (N · IFOV)`:

- Range: with N = 8.9 px and ~0.3 px edge precision, σ_R/R = 3.4% → **±0.71 m at 21 m**.
- Closing speed: a linear fit of R(t) over 0.2 s at 60 fps (n=12) gives **±3.6 m/s**, i.e.
  tca to ±18%. Adequate to trigger, and it tightens rapidly as the ball closes.

**Honest limitation:** pure looming (τ = θ/θ̇) does *not* work at 21 m. Apparent diameter
grows only 0.29 px per frame there — below the noise floor. Looming becomes reliable inside
roughly 10 m. At long range the system runs on the size prior plus bearing.

**Do not plan on a CNN.** The published embedded floor for small-object detection is
~20×30 px at 24 fps in 1280×720 — 3–4× more pixels than will be available. At 5–13 px the
right tool is IMU/homography-compensated temporal differencing plus connected components:
search at half-res (960×600 @ 60 Hz ≈ 35 Mpx/s, comfortable on a Pi 5 with NEON) and
re-measure the blob at full resolution inside a small ROI, so N stays high where it counts.

### 5.5 Rejected alternatives, each with the reason it fails

| Candidate | Why it fails |
|---|---|
| **All flyable lidar** | **Point density, not range.** Livox Mid-360: 200 kpts/s over 21,240 deg² = 9.4 pts·deg⁻²·s⁻¹; the ball subtends 0.0287 deg² at 21 m → **0.018 points per 15 Hz frame**. Landing 3 points needs ~1,570 pts·deg⁻²·s⁻¹ in-sector; the best sub-250 g unit (Hesai JT16, 48) is 33× short *before* restricting the sector. |
| **Single-chip mmWave** (IWR6843) | Fails twice, independently. A plastic ball is ~−37 dBsm against a human's ~0 → ~4–10 m. And 15° azimuth resolution = **5.5 m of cross-range at 21 m**, so no usable dodge direction. Doppler is its one genuine strength and rescues neither failure. |
| **OAK-D LR** | Optics are fine (6.9 px), but ~700 g and 5.5 W — multiples of the entire AUW. |
| **OAK 4 D / 4 S** | 325–674 g, up to 25 W. |
| **Prophesee GenX320** | Pixels are fine; there are only 320 of them, so ≤20° coverage at any usable N. Space-bandwidth product is what fails. |
| **Thermal** (FLIR Boson) | A ball is at ambient temperature. No contrast. |
| **Acoustic array** | 61 ms of propagation delay from 21 m, and tens of degrees of bearing accuracy. |

### 5.6 The runner-up, which is a real option

**Prophesee EVK4 HD (IMX636) + 16 mm C-mount** — 40 g, ~1.5 W, <100 µs pixel latency,
120 dB dynamic range, 11.0 px at 21 m over 22.1°. Strictly better than the AR0234 on
latency and on rejecting static clutter; strictly worse on FOV, on cost (quote-only,
~$3–5k), and on how much perception code must be written from scratch.

This is the sensor family behind UZH's ball-dodging quadrotor (Falanga, Kleber &
Scaramuzza, *Science Robotics* 2020) — but their demonstration was a ball at 3 m and
10 m/s, so they never faced this range problem. **Take this only if the AR0234 flies and
the residual failure turns out to be latency or clutter rather than reach.**

---

## 6. The assumption this all rests on, and the $26 experiment that retires it

**That 8.9 px is the pipeline's detection threshold is derived from a single measured
range.** It bundles pixel count together with *contrast* and with the stereo-clustering
step. A 9 px dark ball against open sky is trivial; the same ball against a treeline at
21 m is not. If the 3.4 m figure was partly a contrast or clustering limit rather than a
pure angular-resolution limit, every range in §5.2 is an optimistic ceiling.

**Retire it before committing to airframe integration.** An **Arducam OV9281** ($26–36,
5–10 g, MIPI CSI-2, 120 fps) with an 8 mm M12 lens has the *identical* 0.375 mrad IFOV —
same 3.0 µm pitch, same focal length. Bolt it to a bench, throw balls at it, and measure
first-detection range directly. The answer transfers to the AR0234, and it costs a weekend
and less than the lens.

If cash or CSI lanes are the constraint, note that three OV9281s cost less than one of
anything else here and give the same pixels-on-target over 27° each.

---

## 7. Provenance

**Measured in SITL:** every save count, tca value, escape displacement, fitted curve, and
dataflash velocity comparison in §2 and §4. Conditions are stated per table; ranges are
oracle **inputs**.

**Derived, arithmetic shown and independently re-checked:** every IFOV, FOV,
pixels-on-target, disparity, lidar hits-per-frame, and implied ball speed in §5. The FOV
model reproduces Arducam's published 40° H for an 8 mm M12 on 1/2.5" to within 1%, and the
speed model reproduces the measured 3.48 m/s.

**Vendor specifications:** all masses, powers, prices, resolutions, pixel pitches, frame
rates and interfaces in §5.

**Judgment calls, flagged as such:** the 8.9 px detection threshold (§6); the assumption
that an event sensor detects at N ≈ 4; the −37 dBsm ball RCS estimate (a first-principles
optical-region calculation with a crude dielectric correction — could be ±10 dB, but radar
fails on angular resolution regardless).

The lab toolchain that produced every measurement, and the raw result CSVs, are in
[`lab/`](../lab/). Sharp edges and the do-not-re-run list are in
[`CLAUDE.md`](../CLAUDE.md).
