# Proposed CLAUDE.md corrections — AWAITING APPROVAL, NOT APPLIED

CLAUDE.md is deliberately not edited without an explicit request, so this file holds the
corrections the Week 6 campaign owes it. Each entry names the claim in CLAUDE.md as it
stands today, the measurement that supersedes it, and the replacement text.

Nothing here changes a conclusion the file reaches. The headline — **the bound is sensor
reach and nothing else** — survived every falsification attempt this campaign. What changed
is that the numbers under it are now measured over 310 throws instead of 60, and three
facts were discovered that the file does not mention at all (§7, §8, §14).

Ordered by how much damage the stale version does if quoted.

---

## 1. "No overlap either time" is refuted — the threshold is a SIGMOID

**Today:** "Highest failure / lowest save: **0.79 / 0.83 s** at 14 m/s, **0.81 / 0.80 s** at
20 m/s. No overlap either time."

That held on 60 throws. It does not hold on 310.

**Replacement:**

> **P(save) is a sigmoid in tca, not a step.** Pooled over **310 on-course hover throws
> across 30 contamination-screened cells**, selected per throw by rule (counterfactual
> ≤ 0.30 m, `fit_resid_m` ≤ 0.005, `n_post` inside one-pose-bridge bounds):
>
>     logit P(save) = -22.271 + 27.910 * tca      LD50 0.798 s (CI 0.779-0.817)
>
> | tca | observed |
> |---|---|
> | ≤ 0.60 s | 0/36 |
> | 0.70 s | 1/24 |
> | 0.80 s | 28/48 (fit 0.51) |
> | 0.90 s | 43/47 (fit 0.95) |
> | ≥ 1.00 s | **155/155** |
>
> Never say "tca ≥ X guarantees a save". Say P = 0.5 at 0.80 s, 0.9 at 0.88 s,
> indistinguishable from 1 above 1.00 s.

The earlier LD50 of 0.810 s (CI 0.783–0.842, n=125) is inside this interval — the law did
not move when the data multiplied 2.5x, it only tightened. That is the strongest single
result of the campaign and is worth stating as such.

**Caveat that must travel with the curve:** this pool contains only throws that FIRED,
because a no-fire has no tca. The curve is **conditional on detection**. The narrow-lens
no-fires in §7 are a separate, additive failure mode it does not describe.

## 2. The required-tca figure, and where the BOM comes from

**Today:** "Fitted effective escape `delta = 0.295·tca^3.15` (14 m/s) and `0.337·tca^3.69`
(20 m/s) independently give required tca **0.83–0.86 s**."

The escape fits stand (pooled: `0.337·tca^3.39`, both prior fits replicate within one SE).
But required tca should be read off the sigmoid, which is the direct measurement, rather
than inferred from escape displacement.

**Replacement:** required tca **0.877 s at P=0.90**, **0.903 s at P=0.95**. Then

    range = speed x (tca_required + t_dead)

| speed | P=0.90, p50 dead time | P=0.95, p90 dead time |
|---|---|---|
| 14 m/s | 14.8 m | 17.5 m |
| **20 m/s** | **21.1 m** | **25.1 m** |
| 26 m/s | 27.4 m | 32.6 m |

**This is where 26 m comes from.** It is derived from the pooled law with margin, not read
off a lucky cell. The inverse — 26 m buys 24.6 m/s at P=0.90 — was then confirmed
independently by the speed ladder (§9).

## 3. `0.177 s` is conditional on 60 Hz and must never be quoted bare

**Today:** `range = speed × (tca + 0.177)   # 0.177 s = pipeline + commit, hover`

Dead time is **rate-dependent**, and the file gives no hint of it.

**Replacement:** measured with the rule-compliant proxy `track_age_s + latency_ms/1000`
(never by subtracting the tracker's own `tca_s` from ground truth):

> **0.178 s at 60 Hz · 0.270 s at 12 Hz.** Pooled percentiles at 60 Hz: p50 **0.179 s**,
> p90 0.350, p95 0.422. Always quote the rate beside the constant.

Dead time is a **filter** property, not a pipeline one: `track_age` is 70–94% of it, and the
binding gate is `miss_m < threat_radius_m` waiting on velocity convergence —
`min_track_updates` is not the gate.

## 4. RATE was the unvaried lever, and it is invisible in the launch argument

Not in CLAUDE.md at all. It should be, because it silently invalidated a year of cells.

**Add:**

> **Gazebo's `SceneBroadcaster` runs at 60 Hz, so the delivered oracle rate is
> `60 / ceil(60 / requested)`, not the requested value.** A requested 14.5 Hz quantises to
> **12 Hz**. Every cell flown before 2026-08-09 delivered 12 Hz regardless of its label, and
> 12 Hz costs 0.09 s of dead time — 1.8 m of required reach at 20 m/s. Use
> `rate_hz: 0.0` to disable the limiter and get the full 60 Hz. Verify delivered rate per
> throw with the implied rate `(track_updates - 1) / track_age_s`.

## 5. `dodge_duration_s` joins the measured nulls

Swept, null. The velocity target persists ~2.4 s regardless, so the ramp was never being
truncated and the parameter had nothing to lengthen.

## 6. The tracker-covariance correction joins the measured nulls

Pre-registered, drift-controlled 3-cell A/B: matching the tracker's `R` to the anisotropic
sensor does **not** rescue mono depth (isotropic 7/10 and 7/10 vs matched 6/10), and the tca
spread **doubled**. Do not re-run.

## 7. NEW — reach and FOV are one optical trade, and this is a product decision

Nothing in CLAUDE.md mentions that the 26 m lens is also a 27° lens. This is the campaign's
most consequential hardware finding and the file is silent on it.

**Add:** two cells, identical but for the modelled cone, at 26 m / 20 m/s / hover:

| approach | ±45° (harness default) | **±13.5° (the real 10 mm lens)** |
|---|---|---|
| 0° | 8/8 | **8/8** |
| 10° | 7/8 | **4/8 — 4 never fired** |
| 20° | 7/8 | **0/8 — zero detections in all eight** |

> Every narrow-cone loss is a **NO-FIRE**, never a failed dodge; the two wide-cone losses
> were ordinary sigmoid losses at tca 0.80/0.84 s. **Oblique geometry alone does not break
> the dodge — only the optics do.**
>
>     reach x half-angle ~ 351 m.deg    (AR0234 class, 1600 px, 3.0 um)
>
> is fixed by pixel count, so ±20° would cut reach to 17.6 m and cap the system at ~17 m/s.
> **A wider lens is not the fix; only more pixels buy reach and sector together.** The
> usable sector is ~±10°, narrower than the nominal ±13.5°, because `in_view` is a cone and
> a gravity-compensated lob sits ~5.6° high at 26 m, leaving ~2° of margin at a 10° approach
> that the dodge's own heading perturbation then spends.

Yaw is **not** the mechanism — hold is 0.22° p95 against DesYaw. (An early p95 of 6° was a
wrap artefact of a single-centre unwrap; fold yaw about a *local* centre or the number is
garbage.)

## 8. NEW — latency converts to reach at a measured rate

**Add:**

> **Latency is not free.** It adds to dead time one-for-one, so **every 10 ms costs 0.2 m of
> required reach at 20 m/s.** Flown at 23 m: 0 ms → 10/10 (tca 0.953), 30 ms → 10/10
> (tca 0.986), **60 ms → 6/10** (tca 0.828). The model predicts the 60 ms collapse to within
> 0.008 s. Budget ≤30 ms.

This supersedes the 150 ms budget framing for the Week 6 regime. At ≤8 m/s latency still is
not the binding term; at 20 m/s it is a reach tax.

## 9. NEW — the speed ladder, and the margin at the design point

**Add:** at 26 m, hover, 20 m/s: 10/10. Then 26 m/s → 9/10 (commit tca 0.848 s) and
28 m/s → 7/10 (commit tca **0.811 s**, essentially the LD50).

> **Zero NO-FIREs at either speed** — a 26 m sensor still *sees* a 28 m/s ball, it just
> cannot buy enough time. So the design point has ~30% speed margin and degrades gracefully
> rather than falling off a cliff.

## 10. Speed-independence now has a direct test

**Today** it rests on comparing a 14 m/s cell against a 20 m/s cell.

**Replacement:** ball speed added as a second covariate over 308 throws and **nine distinct
speeds, 14–29 m/s**: tca +27.28 (SE 4.21, **z +6.47**); ball speed +0.056 (SE 0.060,
**z +0.93, not significant**). Nothing adds information beyond tca — range, requested rate,
delivered rate, depth sigma, bearing sigma, `dodge_speed_mps`, `track_updates`,
`track_age_s`, `latency_ms` and `counterfactual_min_m` each p = 0.135 to 0.994.

## 11. The as-built OAK-D figure moves from 3.5 to 3.2 m/s

Inverting the refit law at 3.4 m of measured reach gives **~3.2 m/s in hover**, not 3.5.
Same conclusion, corrected arithmetic.

## 12. The named part is COLOUR, and the sim's depth is optimistic

**Today:** "An **AR0234 global-shutter mono + 10 mm M12** (e-con See3CAM_24CUG, $99, 26 g)".

The See3CAM_24CUG is **colour**, verified. The mono equivalent is the **See3CAM_20CUG,
$89, 13.5 g**.

Also worth stating, because the sim does not model it: passive depth at 26 m is
**0.85–1.4 m mono / 0.27 m stereo**, and stereo is extrinsics-limited to ~12 arcsec —
**not** the 0.15 m the oracle assumes. The load-bearing mitigation is that **tca from
looming is diameter- and calibration-invariant**, so the trigger should gate on tau, not on
metres.

## 13. `lab_cell.sh` — kill by installed path, never a guessed node name

Operational, but it silently voided a six-cell queue and belongs with the sharp edges.

**Add:** `evasion_nod[e]` / `patrol_nod[e]` matched nothing, **seven stacks accumulated**,
and the queue returned plausible-looking numbers with no error anywhere. Kill by
**installed path**. The decisive contamination detector is the implied rate
`(track_updates - 1) / track_age_s` reading *above* the launched oracle rate — with
`n_post` (230–354 for one 60 Hz bridge; 1179–6317 when duplicated) as the confirmation.

## 14. Between-cell escape drift bounds what a 10-throw cell can prove

**Add:** between-cell escape offset spans **x0.68 to x1.36**. Two 10-throw ceiling results
therefore cannot be separated. This is why `hw26aniso` 10/10 vs `hw26` 9/9 is **not** a
demonstrated improvement — both are ceilings with no failures to separate, and their 1.48x
escape gap sits inside the drift.

---

## What does NOT change

- The bound is **sensor reach and nothing else**. Every flight-authority lever is refuted,
  and the dataflash row that retires them together (achieved Δv exceeds commanded Δv in
  every arm) stands.
- The Week 4 envelope table (78/78 at ≤8 m/s, 0/17 at 14 m/s, 0/12 false dodges) stands
  unchanged. It is a patrol/real-detector measurement and must still never be merged with
  any hover/oracle table.
- Every entry in "Measured nulls — do not re-run" stands; §5 and §6 only add to it.
- The four dead claims stand dead.
