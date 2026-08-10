# Hardware Readiness Package

**Status: the simulation campaign is closed.** This document is what to buy, how to
wire it, what to configure, what to measure, and which simulation predictions the
hardware is expected to confirm or destroy.

Everything here is a measured number or is derived from one. Where a figure is an
*input* to a simulation rather than a result, it says so. Where the simulation
cannot answer a question, it says that too, rather than inventing a model.

---

## 1. The answer

| Question | Answer |
|---|---|
| Can it dodge a 20 m/s projectile? | **Yes** — in hover, head-on, given a sensor with **26 m reach at 60 Hz**. 10/10 counterfactual saves, reproduced across three independent cells. |
| Can it as built? | **No.** The OAK-D Lite's measured ~3.4 m reach caps it at **~3.2 m/s**. |
| What is the binding constraint? | **Sensor reach and sensor rate.** Every maneuver-side lever is refuted — see §6. |
| What closes the gap? | A global-shutter mono camera on a 10 mm M12 lens. **$89–178, 13.5–27 g — lighter than the 61 g OAK-D Lite it replaces.** |
| What does it cost? | The defended sector collapses from ±45° to **±13.5° nominal, ~±10° usable**. This is an architectural consequence, not a bug — §4. |

---

## 2. The governing law

Fitted over **310 on-course hover throws from 30 cells**, selected by rule rather than
by hand (per-throw gates: counterfactual ≤ 0.30 m, fit residual ≤ 0.005 m, `n_post`
within one-pose-bridge bounds).

```
logit P(save) = -22.271 + 27.910 * tca
LD50 = 0.798 s   (95% CI 0.779 - 0.817)
```

The observed data, binned — the fit is only worth as much as this:

| tca | saved | fit |
|---|---|---|
| ≤ 0.60 s | **0 / 36** | 0.00 |
| 0.70 s | 1 / 24 = 0.04 | 0.06 |
| 0.80 s | 28 / 48 = 0.58 | 0.51 |
| 0.90 s | 43 / 47 = 0.91 | 0.95 |
| ≥ 1.00 s | **155 / 155** | 1.00 |

**It is a sigmoid, not a step.** The old "no overlap between saves and failures" claim
is refuted by 33 throws in the 0.77–0.91 s band.

**It does not depend on ball speed.** Adding ball speed as a second covariate over
308 throws spanning 14–29 m/s:

| covariate | coefficient | SE | z |
|---|---|---|---|
| tca | +27.28 | 4.21 | **+6.47** |
| ball speed | +0.056 | 0.060 | **+0.93** (not significant) |

Only the range needed to *buy* that time scales with speed.

### 2.1 Dead time

```
tca = range / speed - t_dead
t_dead = track_age_s + latency_ms/1000
```

Measured over the same 310 throws: **p50 0.179 s, p90 0.350 s, p95 0.422 s.**

Dead time is a **filter** property, not a pipeline property: `track_age` is 70–94% of
it and latency is a stable ~19 ms. `min_track_updates` is *not* the gate (cells commit
at 4.6–17.3 updates); the binding gate is `miss_m < threat_radius_m` waiting on
velocity convergence. Dead time is also **rate-dependent**: 0.178 s at 60 Hz,
0.270 s at 12 Hz.

### 2.2 The reach requirement

```
range = speed * (tca_required + t_dead)
tca_required = 0.877 s at P=0.90     0.903 s at P=0.95
```

| ball speed | P=0.90, p50 dead | P=0.95, p90 dead |
|---|---|---|
| 14 m/s | 14.8 m | 17.5 m |
| **20 m/s** | **21.1 m** | **25.1 m** |
| 26 m/s | 27.4 m | 32.6 m |

**26 m is chosen because it covers P=0.95 at 20 m/s even at the 90th-percentile dead
time.** It is a derived requirement with margin, not a lucky cell.

Inverting: 26 m → **24.6 m/s at P=0.90**. The speed ladder confirmed this independently
— 26 m/s scored exactly 9/10.

---

## 3. Exact sensor requirement

| Spec | Requirement | Why, and where measured |
|---|---|---|
| Shutter | **Global** | A rolling shutter smears a 20 m/s target across the frame; the centroid is then not a position. Not simulated — this is a hard requirement from first principles. |
| Sensor | AR0234 class, ≥1600×1300 active, 3.0 µm pitch | Sets IFOV with the lens below. |
| Lens | **10 mm M12** | 0.300 mrad/px. An 80 mm ball spans **11.1 px at 21 m**, ~8.9 px at the OAK-D's 3.4 m. This is the *angular resolution* deficit, which is what actually limits reach — not headline range. |
| Reach | **≥26 m** on an 80 mm ball | §2.2. |
| Rate | **≥60 Hz at full resolution** | 12 Hz costs 0.09 s of dead time = **1.8 m of extra reach** at 20 m/s. Measured, not assumed. |
| Timestamping | **At exposure, not arrival** | The latency model assumes the centroid keeps its exposure stamp so the filter predicts across the delay. An arrival-stamped pipeline additionally *biases* the state estimate and is a worse sensor than anything measured here. **This is a firmware/driver requirement.** |
| Depth | 0.30 m at 26 m (stereo pair) | Mono apparent-size depth is 0.85 m at 26 m and scored **7/10 vs stereo's 10/10**. See §7 for the one-vs-two-camera decision. |
| Bearing | ≤0.05 m cross-range at 26 m (6.4 px) | See §7. |
| Mass | 13.5 g (mono) / 27 g (pair) | Against the **61 g** OAK-D Lite being removed — the change *saves* mass. |
| Part | e-con **See3CAM_20CUG** (mono, $89) | The See3CAM_24CUG is **colour**, verified; do not order it for this role. |

### 3.1 What the depth number does *not* mean

The simulated 0.30 m is a *model input*. Real passive stereo at 26 m is
extrinsics-limited to roughly 12 arcsec of relative alignment, which is demanding.
Mono apparent-size depth is 0.85–1.4 m. **A dodge scored under the oracle is a claim
about the tracker, trigger and airframe *given* a sensor with that reach — never
evidence that such a sensor exists.** That is what the hardware must establish.

---

## 4. The field-of-view consequence — architectural, decide before ordering

**Reach and field of view are the same optical trade.** The lens that buys 26 m
narrows the horizontal cone to ±13.5° (and ±11.0° vertical). Flown, at 26 m and
20 m/s, identical in every respect except the modelled cone:

| approach angle | ±45° cone | **±13.5° cone (the real lens)** |
|---|---|---|
| 0° head-on | 8/8 saved | **8/8 saved** |
| 10° | 7/8 saved | **4/8 — 4 never fired** |
| 20° | 7/8 saved | **0/8 — zero detections on all eight** |

Every ±13.5° loss is a **NO-FIRE**, not a failed dodge: the ball was never seen.
Under ±45°, the two losses were ordinary sigmoid losses at tca 0.80 and 0.84 s, so
**oblique geometry alone does not break the dodge — only the optics do.**

Two effects stack into the cone and make the *usable* sector narrower than the
nominal one. A gravity-compensated 20 m/s lob sits ~2.6 m high at the 26 m detection
point (≈5.6° of elevation), and `in_view` is a cone, not a horizontal sector — so a
10° horizontal approach presents at ~11.5°, leaving only ~2° of margin. The dodge
itself then perturbs heading enough to consume it, which is why 10° alternated
fire/no-fire while 0° never missed and 20° never fired.

**Plan for a defended sector of about ±10°, and treat 20° as undefended.**

The options, and none of them is free:

1. **Accept it.** The aircraft must be pointed at the threat. Cheapest; changes the product.
2. **Second camera pair**, boresighted outward. Doubles cost, mass and compute.
3. **More pixels.** Reach ∝ 1/IFOV, so a higher-resolution sensor buys reach *and*
   sector. This is the only option that improves both, and it is the one to price first
   if a wide sector is required.

Do **not** solve it with a wider lens on the same sensor: `reach × half-angle ≈ 351 m·deg`
is fixed by pixel count, so ±20° would cut reach to 17.6 m and cap the system at
~17 m/s — below the 20 m/s objective.

---

## 5. Exact software configuration

Hardware config lives in `hw_*` overlay files, never as edits to the sim files —
the sim files stay the regression path. `test_hw_config.py` asserts each overlay's
node name and keys exist in the file it overlays, because in ROS 2 a mistyped node
name silently loads *nothing*.

| File | Role |
|---|---|
| `params/hw_bridge.yaml` | MAVLink endpoints |
| `params/hw_detector.yaml` | detector, real camera |
| `params/hw_evasion.yaml` | tracker + trigger |
| `params/hw_frame.parm` | ArduPilot frame/fence |

Settings that matter, with the reason attached:

| Parameter | Value | Why |
|---|---|---|
| `dodge_speed_mps` | **1.5** (shipped) | Command magnitude plateaus: escape goes as ~`command^0.065`, and a 4× command buys 1.095×. Raising it is not a lever. |
| `dodge_duration_s` | **1.0** (shipped) | Null at operational tca — doubling the window changes escape by 1.00× where these dodges commit. |
| `min_track_updates` | 3 | Not the binding gate; cells commit at 4.6–17.3 updates. |
| `threat_radius_m` | 0.75 → see §7 | This *is* the gate that holds the dodge. |
| `trigger_horizon_s` | **1.5** | `should_dodge` requires `tca ≤ trigger_horizon_s`, which **hard-caps exploitable reach at `v × (1.5 + t_dead)` = 33.6 m at 20 m/s.** 26 m is under the cap; **any longer lens requires raising this or the extra reach is discarded.** |
| `meas_std_xyz_m` | **[0, 0, 0]** (isotropic) | Correcting the tracker's covariance to match real mono depth was tested and **refuted** — it made the filter sluggish, not accurate. §6. |
| `RTL_ALT` | 400 (**cm**) | Under the 5 m fence. ArduPilot's 15 m default would make a fence-breach RTL climb through the fence it is answering. `FENCE_ALT_MAX` is in **metres** — the units differ. |
| `ATC_ANGLE_MAX` | 30 (**degrees** in this build) | Not centidegrees. Never write 4500. |
| `use_sim_time` | launch argument only | Never in a yaml; `clock_guard.py` exits 1 rather than freezing at t=0. |

---

## 6. Levers that are closed — do not re-open on hardware

Each was measured over a full battery. Re-running these is the most common way to
lose time here.

- **Every flight-authority lever, retired by one measurement.** Dataflash over a
  controlled `WP_ACC` sweep shows **achieved Δv exceeds commanded Δv in every arm**
  (0.93 vs 0.81, 1.26 vs 1.17, 1.47 vs 0.97 m/s). The vehicle over-delivers on what it
  is asked for. That closes at once: the airframe, the attitude loop, `ATC_ANGLE_MAX`
  (21–28° used of 30 available), `PSC_JERK_NE`, and `WP_ACC`.
- **`dodge_speed_mps`** — 1.5/3.0/6.0 within-session A/B: 6/10, 6/9, 6/10.
- **`dodge_duration_s`** — null at the tca these dodges commit at.
- **Tracker covariance correction** — 3-cell drift-controlled A/B, isotropic 7/10 and
  7/10 against matched 6/10, and the tca spread **doubled** instead of narrowing.
- **`evade_accel_ff_mps2`**, the multi-hypothesis tracker, track continuity, the
  command path (16/16 ok), dodge direction (~5° error, not a suspect).

**The dodge only ever asks for ~1 m/s of horizontal step, and the vehicle delivers it.**
Nothing on the maneuver side is the problem.

---

## 7. Decisions pending the final queue

*(filled on completion — see §10 provenance)*

| Question | Cell | Decides |
|---|---|---|
| Is 10/10 real at n=30? | `ref30` | Whether the BOM rests on a Wilson lower bound of 0.72 or 0.88. |
| Does a wider trigger gate buy reach back? | `trA/trB/trC` + `fdTR` | Whether `threat_radius_m` 1.25 cuts dead time — every 0.01 s is 0.2 m of reach at 20 m/s — and what it costs in false fires. |
| Does bearing precision matter? | `bear10` | Whether centroid/calibration accuracy is a spec driver at all. |
| Do degradations compound? | `stack` | The only multi-factor cell: real cone, depth 0.30, bearing 0.05, 30 ms latency, shipped `dodge_speed`, all at once. |
| **One camera or two?** | `mono30` | Whether mono's *time* deficit can be paid for with reach ($89, 13.5 g, no extrinsics) instead of a second sensor ($178, 27 g, 12 arcsec extrinsics). |
| Does mono false-fire? | `fdmonoISO/MCV` | Whether the faster/overconfident arm buys its speed with false dodges. |

---

## 8. Exact test procedure on hardware

**Order matters. Do not skip to throwing balls.**

### 8.1 Bench, disarmed, props off
1. Camera streams at **full resolution and ≥60 Hz sustained** — measure, do not trust
   the datasheet. `ros2 topic hz`.
2. Confirm centroids carry an **exposure** timestamp, not arrival. Compare stamp
   against wall clock under artificial CPU load; the offset must not grow.
3. `./scripts/preflight_hw.sh` (warns rather than fails, always exits 0 — read it).
4. `test_hw_config.py` passes.
5. Supervisor reports no faults while armed on the bench; note that it reports none
   while **disarmed** by design, since half the watched topics are legitimately silent.

### 8.2 Static range calibration — the number the whole project rests on
6. Place an **80 mm ball** on a stand. Walk it out in 2 m steps from 4 m to 30 m.
7. At each station record: detection y/n, pixel span, reported depth, reported bearing,
   and the **frame-to-frame scatter** of each over ≥100 frames.
8. **The deliverable is the range at which detection becomes unreliable**, plus σ_depth
   and σ_bearing as functions of range.

### 8.3 Angular sector
9. Repeat at 20 m, sweeping the ball's bearing off boresight in 2° steps until
   detection fails. **Record the angle both horizontally and vertically** — the cone is
   ±13.5°/±11.0° nominal and the vertical is the tighter one.

### 8.4 Moving target, no aircraft
10. Throw the ball past a **static, powered** aircraft on a stand. Log tracks. Confirm
    first detection range matches §8.2 and that `track_age` at first valid velocity
    matches the simulated 0.179 s at 60 Hz.

### 8.5 Flight — netted, tethered, ascending speed only
11. Hover, head-on, **8 m/s** first. Then 14. Then 20. Never start at 20.
12. Score on the **counterfactual**, never on `dodged`.

---

## 9. Exact measurements, and the criteria

| Measurement | Source | Pass |
|---|---|---|
| Sustained frame rate | `ros2 topic hz` on the centroid topic | ≥60 Hz, no dropouts >2 frames |
| Detection range, 80 mm ball | §8.2 | **≥26 m** |
| σ_depth at 26 m | §8.2 | ≤0.30 m stereo (or per §7 for mono) |
| σ_bearing at 26 m | §8.2 | ≤0.05 m (6.4 px) |
| Horizontal sector | §8.3 | ≥±13.5° detected; **≥±10° usable** |
| Vertical sector | §8.3 | ≥±11.0° |
| Exposure-stamp integrity | §8.1 | offset stable under load |
| `track_age_s` at commit | flight CSV | ≤0.20 s at 60 Hz |
| End-to-end latency | flight CSV `latency_ms` | **≤30 ms preferred.** Latency is not free — it adds to dead time one-for-one, so **every 10 ms costs 0.2 m of required reach at 20 m/s**. Flown at 23 m: 0 ms → 10/10, 30 ms → 10/10, **60 ms → 6/10** (commit tca fell to 0.828 s). At 26 m the margin absorbs it; do not read "30 ms was free" as latency being harmless. |
| `first_det_range_m` | flight CSV | within 1.0 m of the static figure |
| tca at commit, 20 m/s | flight CSV | **≥0.90 s** |
| Save rate, 20 m/s head-on | counterfactual | **≥9/10** |
| False dodges | fd geometry, 3.0 m miss | **0** |

**Stop criteria.** Any of: detection range below 21 m (P=0.90 floor at 20 m/s);
sustained rate below 30 Hz; commit tca below 0.80 s at 20 m/s. Below those, the
20 m/s objective is not reachable with that sensor and the lens or sensor must change
before more flying.

---

## 10. What the hardware must confirm — the falsifiable predictions

These are the simulation's claims. Each can be destroyed by the corresponding
measurement, and if one is destroyed, so is the conclusion resting on it.

1. **P(save) is a sigmoid in tca with LD50 0.798 s**, and is speed-independent from
   14 to 29 m/s. Falsified by: saves clustering by ball speed at matched tca.
2. **Dead time is ~0.179 s at 60 Hz** and rises to ~0.270 s at 12 Hz. Falsified by:
   `track_age` at commit materially above 0.20 s on hardware at 60 Hz.
3. **26 m of reach yields ≥9/10 at 20 m/s head-on.** The headline claim.
4. **A 10 mm lens detects an 80 mm ball to 26 m at ~11 px.** Falsified in five minutes
   by §8.2 — and this is the single measurement most likely to disappoint, because it
   depends on the detector's real behaviour on a small, low-contrast blob, which the
   oracle does not model at all.
5. **The defended sector is ±13.5° nominal / ~±10° usable**, and 20° is undefended.
6. **Latency converts to reach at 0.2 m per 10 ms at 20 m/s.** Flown at 23 m:
   0 ms → 10/10, 30 ms → 10/10, 60 ms → 6/10 as commit tca fell to 0.828 s, which
   the pooled fit and the measured first-detection range predict to within 0.008 s.
   Falsified by: a latency cost that is *not* explained by the tca it removes.
7. **No maneuver-side parameter improves the outcome** (§6). Falsified by: any
   authority lever producing a real gain on hardware.

**Prediction 4 is the one to test first.** It is cheap, it is static, and every other
number in this document is conditional on it.

---

## 11. What simulation could not answer, and why

These are **not** open experiments. They are questions the simulation is structurally
unable to answer, and they are therefore hardware validation items.

- **The drone in motion.** The whole envelope is measured in **hover**, because patrol
  cannot deliver a hit at range — 0 of 98 throws arrived inside 0.109 m, against 40/40
  in hover. That is a harness limitation, not a result. Analytically a patrolling drone
  closes faster and needs roughly 1.7× the reach, but that figure is **unvalidated**.
- **Real detector behaviour on a real blob.** The oracle is given the ball's position
  and told to add noise. It does not model contrast, motion blur, background clutter,
  false positives, or the detector missing a small fast object entirely.
- **Wind, lighting, weather.** No model exists. Building one would simulate our own
  assumptions rather than measure anything.
- **Stereo extrinsics stability** under vibration and thermal cycling.
- **Compute headroom** on the real Pi with the real detector at 60 Hz.

---

## 12. Provenance

Every number above traces to a CSV under `~/hz_lab/results/` on the Dell and to a
commit in this repository. The rules the results were scored under:

- Score on the **counterfactual**: saved = `counterfactual_min_m ≤ 0.30` AND
  `actual_min_m > 0.30`. Never on `dodged`. A cell with no on-course throws measured
  **nothing** and is never reported as 0/N.
- Report split by ball speed, never blended.
- Never pool or compare **hover** against **patrol** — patrol counterfactuals carry
  10× the fit residual.
- `detection_range_m` is an **INPUT**. Quote it beside every number.
- Never derive a metric by subtracting the tracker's estimated `tca_s` from ground
  truth; use `track_age_s + latency_ms/1000`.

See also: `docs/week6_result.md` (the full derivation and every refuted lever),
`docs/hardware_bringup.md` (the physical checklist), `docs/SAFETY_CASE.md`.
