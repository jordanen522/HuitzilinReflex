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
| Bearing | ≤0.05 m cross-range at 26 m (6.4 px) | **Derived budget, not a measured requirement** — the cell that would have tested bearing sensitivity (`bear10`) was not flown. Treat as conservative; it may be loose. |
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
| `threat_radius_m` | **0.75 (shipped — do not change)** | This *is* the gate that holds the dodge: the wait for `miss_m < threat_radius_m` is most of dead time. Widening it to 1.25 was the one untested reach lever and its false-fire cost is unknown (§7.2). Fly the bracket before touching it. |
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

## 7. The final queue — what it answered, and what it did not

The campaign was **stopped by decision on 2026-08-10**, not by exhaustion of the queue.
Two cells of the planned nine flew; the rest are optional confidence-building that would
not change the architecture, the BOM, or any specification in this document. This section
records both halves honestly, because an unflown cell is an **open question**, never a
null result.

### 7.1 Answered

| Question | Cell | Result |
|---|---|---|
| Is 10/10 real at n=30? | `ref30` | **28/29 on-course saves at 26 m, 20 m/s, hover.** 30 fired, **0 NO-FIREs**, 1 `DODGE_FAILED`, 1 `WOULD_HAVE_MISSED` excluded from the denominator by rule. Commit tca mean 1.096 s (0.770–1.310). Latency mean 19 ms, max 23. **Wilson 95% lower bound 0.83**, against the 0.72 a 10/10 cell supports. |
| Does mono false-fire? | `fdmonoISO` + `fdmonoMCV` | **0 false dodges in 31 clear-miss throws**, both arms. Isotropic: 0/10 at 1.5 m miss, 0/5 at 3.0 m. Matched-covariance: 0/10 and 0/6. Both still fired **6/6** on the 0.5 m near-miss control, so the discipline is not deafness. Mono does **not** buy its speed with false fires, under either tracker covariance. Neither cell contains an on-course throw, so neither reports a save rate. |

All three cells pass all four contamination detectors (implied rate below launched,
`n_post` inside one-pose-bridge bounds, first-detection range within 0.3 m of launched,
response probe dodges equal to fires).

`fdmonoMCV` was in flight when the campaign was stopped; it was allowed to land rather
than discarded, and completed all 22 throws.

### 7.2 Not answered — carried forward as open questions

| Question | Cell | Status and what stands in its place |
|---|---|---|
| Does a wider trigger gate buy reach back? | `trA/trB/trC`, `fdTR` | **Unflown.** `threat_radius_m` stays at the shipped **0.75**. The hypothesis — that 1.25 shortens the velocity-convergence wait and so cuts dead time — is **untested in either direction**, and its false-fire cost is unmeasured. Do not change this parameter on hardware without flying the bracket. |
| Does bearing precision matter? | `bear10` | **Unflown.** The ≤0.05 m cross-range figure in §3 is therefore a **derived budget, not a measured requirement**. It may well be loose. |
| Do degradations compound? | `stack` | **Unflown.** See §7.4 — composed from single-factor results by model, which is weaker evidence than the cell would have been. |
| **One camera or two?** | `mono30` | **Unflown.** See §7.3 — the decision is therefore made on the evidence that exists, and it goes the conservative way. |

### 7.3 One camera or two — recommendation: **two (stereo pair)**

The evidence actually in hand, at 26 m and 20 m/s in hover:

| arm | saves | how the losses failed |
|---|---|---|
| stereo, σ_depth 0.30 m | **10/10**, and 28/29 at n=30 | — |
| mono, σ_depth 0.85 m | **7/10** | all three were **late commits** (tca 0.31 / 0.68 / 0.75 s), not weak escapes |
| mono, false fires | **0/15** | — |
| tracker covariance matched to mono | 6/10 vs isotropic 7/10 and 7/10 | tca spread **doubled**; refuted |

Mono does not dodge worse — it **decides later**, because a 0.85 m depth axis makes the
velocity estimate converge slowly. That deficit is in *time*, and time has two currencies:
buy accuracy with a second camera ($178, 27 g, extrinsics to ~12 arcsec), or buy time with
a longer lens on one camera ($89, 13.5 g, no extrinsics). `mono30` existed to test the
second currency at 30 m, where the extra 4 m returns 0.20 s. **It was not flown, so that
option is unvalidated.**

**Specify the stereo pair.** It is the only arm with a measured ≥9/10 at the design point.
Mono-plus-reach remains a live cost-down path worth ~$89 and ~13.5 g, but it is a
*hypothesis*, and it carries two costs that must be priced together: 30 m needs an
~11.5 mm lens, narrowing the cone from ±13.5° to ~±11.7° when ~±10° is already the usable
sector (§4), and `trigger_horizon_s` must rise above 1.5 s or the extra reach is discarded
(§5). If the hardware confirms §8.2 comfortably, one 10-cell run at 30 m with the mono
noise model rescaled by z² is the whole experiment.

### 7.4 Do the degradations compound? — composed, not measured

`stack` was to fly the real cone, 0.30 m depth, 0.05 m bearing, 30 ms latency and the
shipped `dodge_speed_mps` all at once. Composing the single-factor results instead:

| factor | measured cost at the design point |
|---|---|
| ±13.5° cone, **head-on** | none — 8/8 |
| σ_depth 0.30 m (stereo) | none — this is the reference condition |
| 30 ms latency | +0.030 s dead time = 0.6 m of reach; flown at 23 m it cost nothing (10/10) |
| `dodge_speed_mps` 1.5 vs 3.0 | none — the command plateaus (§6) |
| σ_bearing 0.05 m | **unmeasured** |

At 26 m and 20 m/s the budget is `tca = 26/20 − t_dead − latency`. At p50 dead time that
is 1.09 s, where the pooled sigmoid gives P ≈ 0.999; at the **p90** dead time of 0.350 s it
is 0.92 s, where P ≈ 0.97. So the head-on design point retains margin even when the
degradations are stacked pessimistically.

**This is a model composition, and it is weaker than the cell would have been** in exactly
one way: it assumes the factors are additive in dead time and independent. Nothing measured
contradicts that, and nothing measured confirms it either. Off-axis it does not hold at all
— the cone term is not additive, it is a cliff (§4).

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

---

## 13. Bill of materials — the sensing change

Only the sensing path changes. The airframe, flight controller, and companion computer
are unchanged, because **no maneuver-side lever was ever found wanting** (§6).

| Item | Part | Qty | Unit | Mass | Note |
|---|---|---|---|---|---|
| Camera | e-con **See3CAM_20CUG** — AR0234 global shutter, **mono** | **2** | $89 | 13.5 g | Stereo pair per §7.3. The See3CAM_24CUG is **colour** — verified; do not order it for this role. |
| Lens | **10 mm M12**, matched pair | 2 | ~$20 | ~5 g | 0.300 mrad/px ⇒ 26 m reach on an 80 mm ball, and ±13.5°/±11.0° of cone. Both come from the same choice. |
| Stereo mount | Rigid baseline bar, thermally stable | 1 | — | ~10 g | Extrinsic stability to ~12 arcsec is the demanding requirement, not the optics. |
| Cabling | USB 3.0, 2× short | 2 | — | ~8 g | Full-resolution 60 Hz needs USB 3.0 bandwidth per camera; verify the Pi's controller sustains both. |
| **Removed** | OAK-D Lite | −1 | — | **−61 g** | |

**Net mass change: roughly −15 g.** The upgrade is lighter than the part it replaces.
Net cost ~$218.

**Cost-down option, unvalidated:** one camera + an ~11.5 mm lens (−$89, −13.5 g, no
extrinsics requirement at all), contingent on flying the `mono30` cell and accepting a
~±11.7° cone. See §7.3.

---

## 14. Final status

**Software / simulation validation: COMPLETE.**
**20 m/s architecture demonstrated in simulation.**
**Physical hardware validation is now the blocking dependency.**

The physical 20 m/s objective has **not** been achieved and must not be reported as
achieved. What has been established is that a specified, purchasable sensor configuration
makes it reachable, and that nothing on the maneuver side stands in the way.

### 14.1 What simulation proved

- P(save) is a **sigmoid in tca**, `logit P = −22.271 + 27.910·tca`, LD50 **0.798 s**
  (CI 0.779–0.817), over **310 on-course hover throws from 30 cells**.
- The law is **speed-independent** across nine speeds from 14 to 29 m/s (ball speed as a
  covariate: z = +0.93, not significant; tca: z = +6.47).
- **26 m of reach at 60 Hz yields 28/29 saves at 20 m/s head-on**, Wilson lower bound
  **0.83**.
- **Zero false dodges** across every validated miss-by-design control set — most recently
  0 in 31 across both mono tracker-covariance arms, with the near-miss control still
  firing 6/6.
- Dead time is **0.179 s at 60 Hz**, rising to 0.270 s at 12 Hz, and is a *filter*
  property — `track_age` is 70–94% of it.
- **Latency converts to reach at 0.2 m per 10 ms** at 20 m/s; ≤30 ms is free at 26 m,
  60 ms is not.
- Robustness holds **head-on**; the ±13.5° lens leaves ~±10° usable and **20° undefended**,
  and every off-axis loss is a no-fire, never a failed dodge.
- **Every maneuver-side lever is refuted**, retired together by achieved Δv exceeding
  commanded Δv in every arm of a controlled sweep.

### 14.2 What only hardware can settle

1. **Whether a 10 mm lens actually detects an 80 mm ball at 26 m.** Every other number
   here is conditional on this one, and it is the cheapest to test (§8.2). It is also the
   most likely to disappoint, because the oracle models no contrast, blur, or clutter.
2. Real σ_depth and σ_bearing at range — the 0.30 m and 0.05 m figures are **model inputs**.
3. Delivered frame rate at full resolution, sustained, on the real Pi.
4. Real end-to-end latency, and whether centroids carry an **exposure** stamp.
5. Real detector behaviour: false positives, missed small fast objects, background clutter.
6. Real airframe response outside hover — the entire envelope is measured in **hover**,
   because the harness cannot deliver a hit at range under patrol. The analytic ~1.7×
   reach penalty for a moving drone is **unvalidated**.
7. Wind, lighting, vibration, thermal drift of the stereo extrinsics.
8. Real projectile validation at ascending speed: 8, then 14, then 20 m/s.

### 14.3 Simulation questions deliberately left open

Not blockers, and none of them changes the architecture or the BOM — recorded so they are
not mistaken for settled: the `threat_radius_m` bracket, bearing sensitivity, the stacked
multi-factor cell, and `mono30`. All four are described in §7.2, and each would be a
single ~8-minute cell if hardware later makes one of them worth resolving.
