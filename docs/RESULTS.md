# Results

Measured performance of the evasion stack in simulation. Two independent measurement
lanes, never mixed:

- **Oracle lane** — a synthetic sensor with settable reach/sector/rate reads Gazebo
  ground truth. Results are about the tracker, trigger, and airframe *given* such a
  sensor, not about any real detector.
- **Real-detector lane** — the depth-camera detection pipeline as flown (§8). The only
  lane that measures detection.

Figures are measured unless marked as a model input.

---

## 1. Summary

| Question | Answer |
|---|---|
| Can it dodge a 20 m/s projectile? | Yes — in hover, head-on, given 26 m reach at 60 Hz: 28/29 on-course saves (Wilson 95% lower bound 0.83), 10/10 reproduced across three independent cells. |
| Can it as built? | No. The OAK-D Lite's measured ~3.4 m reach caps it at ~3.2 m/s. |
| Binding constraint | Sensor reach and rate. Maneuver-side levers measured null (§7). |
| Gap closer | Global-shutter mono camera + 10 mm M12 lens: $89–178, 13.5–27 g vs the 61 g OAK-D Lite. |
| Cost of the fix | Defended sector narrows from ±45° to ±13.5° nominal, ~±10° usable (§5). |
| False dodges | 0 in 31 clear-miss throws; the 0.5 m near-miss control still fired 6/6. |

---

## 2. P(save) is a sigmoid in tca

Fitted over 310 on-course hover throws from 30 cells, selected by per-throw gates
(counterfactual ≤ 0.30 m, fit residual ≤ 0.005 m, `n_post` within pose-bridge bounds):

```
logit P(save) = -22.271 + 27.910 * tca
LD50 = 0.798 s   (95% CI 0.779 - 0.817)
```

Binned data:

| tca | saved | fit |
|---|---|---|
| ≤ 0.60 s | 0 / 36 | 0.00 |
| 0.70 s | 1 / 24 = 0.04 | 0.06 |
| 0.80 s | 28 / 48 = 0.58 | 0.51 |
| 0.90 s | 43 / 47 = 0.91 | 0.95 |
| ≥ 1.00 s | 155 / 155 | 1.00 |

It is a sigmoid, not a step: P = 0.5 at 0.80 s, 0.9 at 0.88 s, indistinguishable from 1
above 1.00 s. The curve is conditional on detection — it pools only throws that fired
(a no-fire has no tca), so the narrow-lens no-fires in §5 are a separate, additive
failure mode.

### 2.1 Speed-independent

Ball speed added as a second covariate over 308 throws spanning nine speeds, 14–29 m/s:

| covariate | coefficient | SE | z |
|---|---|---|---|
| tca | +27.28 | 4.21 | +6.47 |
| ball speed | +0.056 | 0.060 | +0.93 (n.s.) |

No other candidate covariate adds information beyond tca — range, requested rate,
delivered rate, depth sigma, bearing sigma, `dodge_speed_mps`, `track_updates`,
`track_age_s`, `latency_ms`, `counterfactual_min_m` each came back p = 0.135–0.994.
Only the range needed to buy the time scales with speed.

---

## 3. Dead time

```
tca     = range / speed - t_dead
t_dead  = track_age_s + latency_ms/1000
```

Over the same 310 throws: p50 0.179 s, p90 0.350 s, p95 0.422 s.

Dead time is a filter property, not a pipeline property: `track_age` is 70–94% of it,
latency a stable ~19 ms. `min_track_updates` is not the gate (cells commit at 4.6–17.3
updates); the binding gate is `miss_m < threat_radius_m` waiting on velocity convergence.

Dead time is rate-dependent — 0.178 s at 60 Hz, 0.270 s at 12 Hz — so quote the rate
beside the constant. Gazebo's `SceneBroadcaster` runs at 60 Hz, so a requested oracle
rate is delivered as `60 / ceil(60 / requested)`; a requested 14.5 Hz quantises to
12 Hz, costing 0.09 s of dead time — 1.8 m of required reach at 20 m/s.

---

## 4. Sensor requirement

```
range = speed * (tca_required + t_dead)
tca_required = 0.877 s at P=0.90     0.903 s at P=0.95
```

| ball speed | P=0.90, p50 dead | P=0.95, p90 dead |
|---|---|---|
| 14 m/s | 14.8 m | 17.5 m |
| **20 m/s** | **21.1 m** | **25.1 m** |
| 26 m/s | 27.4 m | 32.6 m |

26 m covers P=0.95 at 20 m/s even at 90th-percentile dead time — a derived requirement
with margin. Inverting: 26 m → 24.6 m/s at P=0.90, confirmed independently by the speed
ladder (26 m/s scored 9/10; 28 m/s scored 7/10 at a commit tca of 0.811 s ≈ the LD50,
with zero no-fires at either speed — the sensor still sees a 28 m/s ball, it just cannot
buy enough time). ~30% speed margin, graceful degradation.

### 4.1 The sensor

| Spec | Requirement | Why |
|---|---|---|
| Shutter | Global | A rolling shutter smears a 20 m/s target across the frame. Not simulated — first-principles requirement. |
| Sensor | AR0234 class, ≥1600×1300 active, 3.0 µm pitch | Sets IFOV with the lens below. |
| Lens | 10 mm M12 | 0.300 mrad/px; an 80 mm ball spans 11.1 px at 21 m (~8.9 px at the OAK-D's 3.4 m). The deficit is angular resolution, not headline range. |
| Reach | ≥26 m on an 80 mm ball | §4. |
| Rate | ≥60 Hz at full resolution | 12 Hz costs 0.09 s of dead time = 1.8 m of reach at 20 m/s. |
| Timestamping | At exposure, not arrival | Arrival stamping biases the state estimate; firmware/driver requirement. |
| Depth | 0.30 m at 26 m (stereo pair) | Mono apparent-size depth is 0.85 m at 26 m and scored 7/10 vs stereo's 10/10. |
| Bearing | ≤0.05 m cross-range at 26 m (6.4 px) | Derived budget, not measured — the cell that would have tested it was not flown. |
| Mass | 13.5 g (mono) / 27 g (pair) | vs the 61 g OAK-D Lite removed — net mass saving. |
| Part | e-con See3CAM_20CUG (mono, $89) | The See3CAM_24CUG is the colour variant — wrong part for this role. |

Specify the stereo pair — the only arm with a measured ≥9/10 at the design point. Mono
does not dodge worse, it decides later: all three mono losses were late commits (tca
0.31 / 0.68 / 0.75 s), because a 0.85 m depth axis slows velocity convergence.
Mono-plus-more-reach is a plausible cost-down (~$89, ~13.5 g) but untested — the 30 m
mono cell was not flown.

The simulated 0.30 m depth is a model input. Real passive stereo at 26 m is
extrinsics-limited (~12 arcsec relative alignment); mono apparent-size depth is
0.85–1.4 m. Mitigation: tca from looming is diameter- and calibration-invariant, so a
real trigger should gate on tau rather than metres.

---

## 5. Reach and field of view are one optical trade

The lens that buys 26 m narrows the horizontal cone to ±13.5° (±11.0° vertical). Flown
at 26 m and 20 m/s, identical except the modelled cone:

| approach angle | ±45° cone | ±13.5° cone (the real lens) |
|---|---|---|
| 0° head-on | 8/8 saved | 8/8 saved |
| 10° | 7/8 saved | 4/8 — 4 never fired |
| 20° | 7/8 saved | 0/8 — zero detections |

Every ±13.5° loss is a no-fire, not a failed dodge. Under ±45° the two losses were
ordinary sigmoid losses at tca 0.80 / 0.84 s — oblique geometry alone does not break the
dodge; only the optics do.

The usable sector is narrower than nominal: a gravity-compensated 20 m/s lob sits ~2.6 m
high at 26 m (≈5.6° elevation) and `in_view` is a cone, so a 10° horizontal approach
presents at ~11.5°, leaving ~2° of margin that the dodge's own heading perturbation then
spends. Plan for ~±10° defended; treat 20° as undefended.

A wider lens on the same sensor does not help:

```
reach x half-angle ~ 351 m.deg     (AR0234 class, 1600 px, 3.0 um)
```

is fixed by pixel count — ±20° would cut reach to 17.6 m and cap the system at ~17 m/s.
Only more pixels buy reach and sector together.

---

## 6. Latency and the trigger horizon

Latency adds to dead time one-for-one: every 10 ms costs 0.2 m of required reach at
20 m/s. Flown at 23 m: 0 ms → 10/10 (tca 0.953), 30 ms → 10/10 (tca 0.986), 60 ms →
6/10 (tca 0.828); the pooled fit predicts the 60 ms collapse to within 0.008 s.
Budget ≤30 ms. At ≤8 m/s latency is not the binding term; at 20 m/s it is a reach tax.

`trigger_horizon_s` is 1.5 s and `should_dodge` requires `tca ≤ trigger_horizon_s`,
which hard-caps exploitable reach at `v × (1.5 + t_dead)` = 33.6 m at 20 m/s. 26 m is
under the cap; a longer lens requires raising it or the extra reach is discarded.

---

## 7. Measured nulls

Levers measured over a full battery each, all null:

- **Flight authority.** Dataflash over a controlled `WP_ACC` sweep: achieved Δv exceeds
  commanded Δv in every arm (0.93 vs 0.81, 1.26 vs 1.17, 1.47 vs 0.97 m/s). This closes
  the airframe, the attitude loop, `ATC_ANGLE_MAX` (21–28° used of 30 available),
  `PSC_JERK_NE`, and `WP_ACC` at once; the dodge only asks for ~1 m/s of horizontal step.
- **`dodge_speed_mps`** — 1.5/3.0/6.0 hover A/B: 6/10, 6/9, 6/10. Escape goes as
  ~`command^0.065`; 4× the command buys 1.095× the escape.
- **`dodge_duration_s`** — null at the tca these dodges commit at; the velocity target
  persists ~2.4 s regardless.
- **Tracker covariance correction** — 3-cell drift-controlled A/B. Matching `R` to
  anisotropic mono depth does not help (isotropic 7/10, 7/10 vs matched 6/10) and the
  tca spread doubled.
- **`evade_accel_ff_mps2`** — 12-trial in-flight A/B, no gain.
- **Multi-hypothesis tracker** — works correctly, bought zero tca.
- **Track continuity** — the associator does not fragment the ball (1.05 spawns/throw);
  MHT and one unbroken filter commit at the same tca.
- **Command path** (probe ok 16/16) and **dodge direction** (~5° error).
- Detector thresholds: `roi_max_range_m` 5→8, `min_track_updates` 3→2,
  `cluster_min_points` 5→3, frame rate 320×240 @ 30 Hz.

---

## 8. Real-detector envelope (patrol)

Everything above uses the oracle sensor. This table is the real depth detector at its
measured ~3.4 m reach, on patrol, scored on `dodged` — 95 throws over five batteries:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

This table and the hover/oracle tables are different experiments — different sensor,
flight mode, and scoring rule — and are not comparable row-for-row. Report split by
ball speed, never blended.

---

## 9. Not settled by simulation

- Whether a 10 mm lens actually detects an 80 mm ball at 26 m — every number in §4 is
  conditional on this, and the oracle models no contrast, motion blur, or clutter.
- Real σ_depth / σ_bearing at range (0.30 m and 0.05 m are model inputs).
- Real detector behaviour on a real blob: false positives, small fast objects, clutter.
- The drone in motion. The envelope is measured in hover because patrol cannot deliver
  a hit at range (0 of 98 throws inside 0.109 m, vs 40/40 in hover) — a harness
  limitation, not a result. Analytically a patrolling drone needs roughly 1.7× the
  reach, unvalidated.
- Wind, lighting, vibration, thermal drift of stereo extrinsics; compute headroom on a
  real Pi at 60 Hz.

Untested cells (open questions, not nulls; none changes the architecture or BOM): the
`threat_radius_m` 0.75→1.25 bracket (false-fire cost unmeasured — fly the bracket before
changing the parameter), bearing sensitivity, a stacked multi-factor degradation cell,
mono at 30 m.

---

## 10. Scoring rules

- Score on the counterfactual: saved = `counterfactual_min_m ≤ 0.30` AND
  `actual_min_m > 0.30`, never on `dodged`. A fire count and a save rate are different
  numbers; a cell with no on-course throws measured nothing and is not reported as 0/N.
- Report split by ball speed.
- Never pool hover against patrol — patrol counterfactuals carry 10× the fit residual,
  and identical patrol control arms have drifted 1.58× apart.
- `detection_range_m` is an input. An oracle-scored dodge is a claim about the tracker,
  trigger, and airframe given a sensor of that reach. Quote the range beside every
  number.
- Derive dead time as `track_age_s + latency_ms/1000`, never by subtracting the
  tracker's estimated `tca_s` from ground truth.
- Between-cell escape offset spans ×0.68–×1.36, so two 10-throw ceiling results cannot
  be separated by their escape numbers.

## Data availability

Raw CSVs for the oracle/hover dataset (§§2–7) are in git history under `lab/results/`
(removed from the working tree at project close): `git show 3cf0bbb:lab/results/<file>`.
The §8 patrol table has no surviving per-throw CSV — a 20-row reference subset exists at
`git show 3cf0bbb:lab/results/ref_week4_battery.csv`, and the per-battery breakdown is
recorded in `git show 8e7bde3:docs/JOURNAL.md`. A fresh battery of the same kind:
`./scripts/run_dodge_battery.sh week4`.
