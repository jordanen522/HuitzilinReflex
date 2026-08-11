# Results — can HuitzilinReflex dodge a 20 m/s projectile?

**Yes, in simulation, given a sensor with 26 m of reach at 60 Hz.** This document is the
whole answer: the measured law, what it implies for the sensor, and what the campaign
closed. The simulation phase is complete and closed on 2026-08-10.

Every figure here is measured unless it says otherwise. Where a number is an *input* to a
simulation rather than a result, it says so.

---

## 1. The answer

| Question | Answer |
|---|---|
| Can it dodge a 20 m/s projectile? | **Yes** — in hover, head-on, given **26 m reach at 60 Hz**. **28/29** on-course saves at n=30, Wilson 95% lower bound **0.83**; 10/10 reproduced across three independent cells. |
| Can it as built? | **No.** The OAK-D Lite's measured ~3.4 m reach caps it at **~3.2 m/s**. |
| What is the binding constraint? | **Sensor reach and sensor rate.** Every maneuver-side lever is refuted — §7. |
| What closes the gap? | A global-shutter mono camera on a 10 mm M12 lens. **$89–178, 13.5–27 g — lighter than the 61 g OAK-D Lite it replaces.** |
| What does it cost? | The defended sector collapses from ±45° to **±13.5° nominal, ~±10° usable** — §5. |
| False dodges? | **0 in 31** clear-miss throws, while the 0.5 m near-miss control still fired 6/6. |

---

## 2. The law — P(save) is a sigmoid in tca

Fitted over **310 on-course hover throws from 30 cells**, selected by rule rather than by
hand (per-throw gates: counterfactual ≤ 0.30 m, fit residual ≤ 0.005 m, `n_post` within
one-pose-bridge bounds).

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

**It is a sigmoid, not a step.** Never say "tca ≥ X guarantees a save" — say P = 0.5 at
0.80 s, 0.9 at 0.88 s, indistinguishable from 1 above 1.00 s.

**The curve is conditional on detection.** It pools only throws that *fired*, because a
no-fire has no tca. The narrow-lens no-fires in §5 are a separate, additive failure mode
this curve does not describe.

### 2.1 It does not depend on ball speed

Ball speed added as a second covariate over 308 throws spanning **nine speeds, 14–29 m/s**:

| covariate | coefficient | SE | z |
|---|---|---|---|
| tca | +27.28 | 4.21 | **+6.47** |
| ball speed | +0.056 | 0.060 | **+0.93** (not significant) |

Nothing adds information beyond tca — range, requested rate, delivered rate, depth sigma,
bearing sigma, `dodge_speed_mps`, `track_updates`, `track_age_s`, `latency_ms` and
`counterfactual_min_m` each came back p = 0.135 to 0.994.

Only the range needed to *buy* that time scales with speed.

---

## 3. Dead time

```
tca     = range / speed - t_dead
t_dead  = track_age_s + latency_ms/1000
```

Measured over the same 310 throws: **p50 0.179 s, p90 0.350 s, p95 0.422 s.**

Dead time is a **filter** property, not a pipeline property: `track_age` is 70–94% of it
and latency is a stable ~19 ms. `min_track_updates` is *not* the gate (cells commit at
4.6–17.3 updates); the binding gate is `miss_m < threat_radius_m` waiting on velocity
convergence.

**Dead time is rate-dependent — 0.178 s at 60 Hz, 0.270 s at 12 Hz.** Always quote the rate
beside the constant. Rate was the last unvaried lever in the campaign: Gazebo's
`SceneBroadcaster` runs at 60 Hz, so a requested oracle rate is delivered as
`60 / ceil(60 / requested)`. A requested 14.5 Hz quantises to 12 Hz, which costs 0.09 s of
dead time — 1.8 m of required reach at 20 m/s.

---

## 4. From tca to a sensor requirement

```
range = speed * (tca_required + t_dead)
tca_required = 0.877 s at P=0.90     0.903 s at P=0.95
```

| ball speed | P=0.90, p50 dead | P=0.95, p90 dead |
|---|---|---|
| 14 m/s | 14.8 m | 17.5 m |
| **20 m/s** | **21.1 m** | **25.1 m** |
| 26 m/s | 27.4 m | 32.6 m |

**26 m is chosen because it covers P=0.95 at 20 m/s even at the 90th-percentile dead time.**
It is a derived requirement with margin, not a lucky cell. Inverting: 26 m → **24.6 m/s at
P=0.90**, which the speed ladder confirmed independently — 26 m/s scored exactly 9/10, and
28 m/s scored 7/10 at a commit tca of 0.811 s, essentially the LD50. **Zero no-fires at
either speed**: a 26 m sensor still *sees* a 28 m/s ball, it just cannot buy enough time.
The design point has ~30% speed margin and degrades gracefully rather than falling off a
cliff.

### 4.1 The sensor

| Spec | Requirement | Why |
|---|---|---|
| Shutter | **Global** | A rolling shutter smears a 20 m/s target across the frame; the centroid is then not a position. Not simulated — a hard requirement from first principles. |
| Sensor | AR0234 class, ≥1600×1300 active, 3.0 µm pitch | Sets IFOV with the lens below. |
| Lens | **10 mm M12** | 0.300 mrad/px. An 80 mm ball spans **11.1 px at 21 m**, ~8.9 px at the OAK-D's 3.4 m. The deficit is *angular resolution*, not headline range. |
| Reach | **≥26 m** on an 80 mm ball | §4. |
| Rate | **≥60 Hz at full resolution** | 12 Hz costs 0.09 s of dead time = 1.8 m of extra reach at 20 m/s. |
| Timestamping | **At exposure, not arrival** | The latency model assumes the centroid keeps its exposure stamp. An arrival-stamped pipeline additionally *biases* the state estimate. A firmware/driver requirement. |
| Depth | 0.30 m at 26 m (stereo pair) | Mono apparent-size depth is 0.85 m at 26 m and scored **7/10 vs stereo's 10/10**. |
| Bearing | ≤0.05 m cross-range at 26 m (6.4 px) | **Derived budget, not a measured requirement** — the cell that would have tested it was not flown. |
| Mass | 13.5 g (mono) / 27 g (pair) | Against the **61 g** OAK-D Lite removed — the change *saves* mass. |
| Part | e-con **See3CAM_20CUG** (mono, $89) | The See3CAM_24CUG is **colour**, verified; do not order it for this role. |

Specify the **stereo pair** — it is the only arm with a measured ≥9/10 at the design point.
Mono does not dodge worse, it **decides later**: all three mono losses were late commits
(tca 0.31 / 0.68 / 0.75 s), not weak escapes, because a 0.85 m depth axis makes the velocity
estimate converge slowly. Mono-plus-more-reach is a live cost-down path worth ~$89 and
~13.5 g, but it is a hypothesis — the cell that would have tested it at 30 m was not flown.

**The simulated 0.30 m depth is a model input.** Real passive stereo at 26 m is
extrinsics-limited to roughly 12 arcsec of relative alignment; mono apparent-size depth is
0.85–1.4 m. The load-bearing mitigation is that **tca from looming is diameter- and
calibration-invariant**, so a real trigger should gate on tau rather than on metres.

---

## 5. Reach and field of view are one optical trade

The lens that buys 26 m narrows the horizontal cone to ±13.5° (±11.0° vertical). Flown at
26 m and 20 m/s, identical in every respect except the modelled cone:

| approach angle | ±45° cone | **±13.5° cone (the real lens)** |
|---|---|---|
| 0° head-on | 8/8 saved | **8/8 saved** |
| 10° | 7/8 saved | **4/8 — 4 never fired** |
| 20° | 7/8 saved | **0/8 — zero detections on all eight** |

Every ±13.5° loss is a **no-fire**, not a failed dodge — the ball was never seen. Under ±45°
the two losses were ordinary sigmoid losses at tca 0.80 and 0.84 s, so **oblique geometry
alone does not break the dodge; only the optics do.**

The *usable* sector is narrower than the nominal one. A gravity-compensated 20 m/s lob sits
~2.6 m high at 26 m (≈5.6° of elevation) and `in_view` is a cone, not a horizontal sector, so
a 10° horizontal approach presents at ~11.5° — leaving ~2° of margin that the dodge's own
heading perturbation then spends. **Plan for a defended sector of about ±10°, and treat 20°
as undefended.**

Do **not** answer this with a wider lens on the same sensor:

```
reach x half-angle ~ 351 m.deg     (AR0234 class, 1600 px, 3.0 um)
```

is fixed by pixel count, so ±20° would cut reach to 17.6 m and cap the system at ~17 m/s —
below the objective. **Only more pixels buy reach and sector together.**

---

## 6. Latency, and the trigger horizon

**Latency is not free.** It adds to dead time one-for-one, so **every 10 ms costs 0.2 m of
required reach at 20 m/s.** Flown at 23 m: 0 ms → 10/10 (tca 0.953), 30 ms → 10/10
(tca 0.986), **60 ms → 6/10** (tca 0.828). The pooled fit predicts the 60 ms collapse to
within 0.008 s. **Budget ≤30 ms.** At ≤8 m/s latency is not the binding term; at 20 m/s it
is a reach tax.

`trigger_horizon_s` is **1.5 s**, and `should_dodge` requires `tca ≤ trigger_horizon_s` —
which **hard-caps exploitable reach at `v × (1.5 + t_dead)` = 33.6 m at 20 m/s.** 26 m is
under the cap; any longer lens requires raising this or the extra reach is discarded.

---

## 7. Closed questions — do not re-open

Each was measured over a full battery.

- **Every flight-authority lever, retired by one measurement.** Dataflash over a controlled
  `WP_ACC` sweep shows **achieved Δv exceeds commanded Δv in every arm** (0.93 vs 0.81,
  1.26 vs 1.17, 1.47 vs 0.97 m/s) — the vehicle over-delivers on what it is asked for. That
  closes at once: the airframe, the attitude loop, `ATC_ANGLE_MAX` (21–28° used of 30
  available), `PSC_JERK_NE`, and `WP_ACC`. The dodge only ever asks for ~1 m/s of horizontal
  step.
- **`dodge_speed_mps`** — 1.5/3.0/6.0 within-session hover A/B: 6/10, 6/9, 6/10. Escape goes
  as ~`command^0.065`; 4× the command buys 1.095× the escape.
- **`dodge_duration_s`** — null at the tca these dodges commit at. The velocity target
  persists ~2.4 s regardless, so the ramp was never being truncated.
- **Tracker covariance correction** — pre-registered 3-cell drift-controlled A/B. Matching
  the tracker's `R` to anisotropic mono depth does **not** rescue it (isotropic 7/10 and
  7/10 vs matched 6/10) and the tca spread **doubled**.
- **`evade_accel_ff_mps2`** — 12-trial in-flight A/B, no dodge gain.
- **The multi-hypothesis tracker** — works correctly, bought zero tca.
- **Track continuity** — the associator does not fragment the ball (1.05 spawns per throw);
  MHT and one unbroken filter commit at the same tca.
- **The command path** (probe returned ok 16/16) and **dodge direction** (~5° error, not a
  suspect).
- Detector thresholds: `roi_max_range_m` 5→8, `min_track_updates` 3→2, `cluster_min_points`
  5→3, frame rate 320×240 @ 30 Hz.

---

## 8. The Week 4 envelope — the only real-detector measurement

Everything above uses a synthetic oracle sensor. This table is the aircraft's **real** depth
detector at its measured ~3.4 m reach, on **patrol**, scored on `dodged` — 95 throws over
five batteries:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

**Never merge this table with a hover/oracle table, and never compare a hover row against a
patrol row.** They are different experiments: different sensor, different flight mode,
different scoring rule. Report split by ball speed, never blended.

---

## 9. What simulation did not settle

Not open experiments — questions the simulation is structurally unable to answer.

- **Whether a 10 mm lens actually detects an 80 mm ball at 26 m.** Every number in §4 is
  conditional on this one. It is also the most likely to disappoint, because the oracle
  models no contrast, motion blur, or background clutter.
- **Real σ_depth and σ_bearing at range** — the 0.30 m and 0.05 m figures are model inputs.
- **Real detector behaviour on a real blob**: false positives, missed small fast objects,
  clutter.
- **The drone in motion.** The whole envelope is measured in **hover**, because patrol
  cannot deliver a hit at range — 0 of 98 throws arrived inside 0.109 m, against 40/40 in
  hover. That is a harness limitation, not a result. Analytically a patrolling drone closes
  faster and needs roughly 1.7× the reach, but that figure is **unvalidated**.
- **Wind, lighting, vibration, thermal drift** of the stereo extrinsics; compute headroom on
  a real Pi at 60 Hz.

Four cells were queued and deliberately never flown when the campaign was stopped. Each is
an **open question, not a null result**, and none changes the architecture or the BOM: the
`threat_radius_m` 0.75→1.25 bracket (untested in either direction, and its false-fire cost
is unmeasured — do not change this parameter without flying the bracket), bearing
sensitivity, a stacked multi-factor degradation cell, and mono-at-30 m.

---

## 10. Scoring rules these results were measured under

- Score on the **counterfactual**: saved = `counterfactual_min_m ≤ 0.30` AND
  `actual_min_m > 0.30`. Never on `dodged`. A fire count and a save rate are different
  numbers. A cell with no on-course throws measured **nothing** and is never reported as 0/N.
- Report split by ball speed, never blended.
- Never pool or compare **hover** against **patrol** — patrol counterfactuals carry 10× the
  fit residual, and two identical patrol control arms once drifted 1.58× apart.
- `detection_range_m` is an **INPUT**, not a result. A dodge scored under the oracle is a
  claim about the tracker, trigger and airframe *given* a sensor with that reach — never
  evidence that such a sensor exists. Quote the range beside every number.
- Never derive dead time by subtracting the tracker's estimated `tca_s` from ground truth;
  use `track_age_s + latency_ms/1000`.
- Between-cell escape offset spans ×0.68 to ×1.36, so two 10-throw ceiling results cannot be
  separated by their escape numbers.

The raw CSVs behind every figure are in git history under `lab/results/`, removed from the
working tree when the project closed: `git show 3cf0bbb:lab/results/<file>`.
