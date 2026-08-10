# Week 6 result — can HuitzilinReflex dodge a 20 m/s projectile?

**Status: answered in simulation, against a *modelled* sensor.** Not with the camera the
aircraft carries. With a global-shutter camera and a longer lens — a part that is *lighter*
and *cheaper* than the one it replaces — yes: **10/10 on-course saves at 20 m/s and 9/10 at
24 m/s**, flown at a **26 m oracle detection range, which is an INPUT and not a result**,
with **0 false fires in 15 near-miss throws**.

This document exists because the project's success criteria demand one of two outcomes:
either demonstrate reliable 20 m/s evasion, or **prove quantitatively that it is
unattainable, state the closest achievable ball speed, and name the exact
hardware/architectural change that closes the gap.** What follows is the second, plus a
flown measurement showing the first is reachable after the change.

Everything here is tagged. **MEASURED** = flown in SITL and scored. **MODELLED** =
arithmetic or a fitted curve, with the derivation shown so it can be checked rather than
believed. **QUEUED BUT UNFLOWN** = a designed experiment that has not produced data;
nothing is predicted from those.

---

## 1. The answer in one table

| Question | Answer |
|---|---|
| Can it dodge 20 m/s **as built**? | **No.** |
| What is the binding constraint? | **Sensor reach and sensor rate.** Every maneuver-side lever is refuted — *magnitude* in §5, *duration* in §7.1. |
| Closest achievable ball speed as built | **~3.1–3.4 m/s** in hover at the OAK-D Lite's measured 3.4 m — and that is now a **50% point, not a guarantee** |
| What does 20 m/s actually require? | tca **0.81 s** for a coin flip, **0.90 s** for 90%. At the measured 60 Hz dead time that is **19.8 m / 21.5 m** of reach (MODELLED from the fit) |
| What has actually been flown at 20 m/s | **26 m INPUT: 10/10** with the *shipped* `dodge_speed_mps` 1.5 · **23 m: 8/10** · **21 m: 6/9** (MEASURED) |
| Is there margin above 20 m/s? | **24 m/s at 26 m: 9/10** (MEASURED). 26 and 28 m/s are QUEUED, UNFLOWN (§7.4). |
| The exact change that closes it | **AR0234 / OV2311-class global shutter, 3.0 µm pitch, 10 mm M12** → **0.300 mrad/px** → **26 m** of reach |
| What it costs | 73° → **27–32°** horizontal FOV (and only ~20–22° vertical), and stereo depth at range, replaced by a known-size prior |

The headline is counter-intuitive and worth stating plainly: **the deficit is angular
resolution, not range.** The OAK-D Lite is not "a short-range camera." It is a camera whose
pixels are 6.2× too coarse to resolve a thrown ball at 21 m. Nothing marketed as
longer-range fixes that unless its IFOV is finer, which is why most of §6 is rejections.

---

## 2. The measurement: a save is a *sigmoid* in tca

`tca` is time-to-closest-approach at the moment the dodge commits. It is the independent
variable of this entire investigation.

### 2.1 What is being scored

A throw is scored **saved** if and only if:

    counterfactual_min_m <= 0.30 m   AND   actual_min_m > 0.30 m

`counterfactual_min_m` is how close the ball *would* have come with no dodge, recovered by
extrapolating the drone's pre-dodge constant-velocity track against the ball's recorded
ground truth. A throw with `counterfactual > 0.30 m` was never on a hit course and
**measured nothing about dodging** — it is excluded from the denominator, not counted as a
failure.

This matters more than it sounds. The battery's own `dodged` field counts *trigger
firings*, not saves. Scoring on `dodged` reads a working system as broken, and scoring
every throw against a dead-centre worst case is how the earlier 8 m/s "34% success"
figure arose — those balls were already passing at 0.20–0.30 m.

### 2.2 The pooled law — MEASURED over 125 on-course hover throws

Pooled across **13 clean cells**, patrol excluded, off-course throws excluded from the
denominators:

    logit P(save) = -21.11 + 26.05 * tca

| quantity | value |
|---|---|
| **LD50** (P = 0.5) | **0.810 s**, 95% CI **0.783–0.842** (cluster bootstrap over cells) |
| 10→90% transition width | **0.169 s** (P=0.10 at 0.726 s, P=0.90 at 0.895 s) |
| likelihood-ratio test vs intercept-only | **χ² = 123.3**, 1 df, **p = 1.2 × 10⁻²⁸** |
| below 0.70 s | **0 / 40 saved** |
| above 1.00 s | **25 / 25 saved** |

### "No overlap" is REFUTED — the transition is real and it is 0.14 s wide

The earlier claim, **"highest failure 0.79 s, lowest save 0.83 s, no overlap"**, held on 60
throws. **It fails on 125.**

| | value |
|---|---|
| lowest save | **0.77 s** |
| highest failure | **0.91 s** |
| throws inside the 0.77–0.91 s band | **33** |
| saved inside the band | **22 (67%)** |
| cells showing *both* outcomes inside the band | **9 of 13** |

Both outcomes appear in most cells, so this is not one bad session. **Never write
"tca ≥ X guarantees a save" again.** The correct form is probabilistic: quote P(save) from
the fit, or quote the flown count with its denominator.

### 2.3 Speed-independence survived a genuine attempt to break it

Fitting the sigmoid separately by ball speed:

| ball speed | n | LD50 |
|---|---|---|
| 14 m/s | 66 | **0.797 s** |
| 20 m/s | 59 | **0.833 s** |

Bootstrap CI on the **difference**: **−0.041 to +0.083 s** — it contains zero, and it is
narrower than the transition width. Speed-independence is not merely unrefuted; it was
tested with enough n to have failed.

The same test was run against every other covariate the harness records. Added one at a
time to the tca-only logistic, **none is significant**: ball speed, oracle range, requested
rate, delivered rate, depth sigma, bearing sigma, `dodge_speed_mps`, `track_updates`,
`track_age_s`, `latency_ms`, `counterfactual_min_m` — **p from 0.135 to 0.994**.

**tca is the whole model.** Every other lever acts, if at all, only by changing tca.

### 2.4 Converting tca to a sensor requirement — and the dead time is RATE-DEPENDENT

In hover the drone is stationary, so closing speed is the ball alone:

    range = speed * (tca + t_dead)

**`t_dead` is not a constant.** The old `0.177 s` was a 60 Hz number quoted as if it were
universal. Measured with the rule-compliant proxy `track_age_s + latency_ms/1000` —
*never* ground truth minus the tracker's own `tca_s`, which would be circular:

| delivered rate | n | median `t_dead` |
|---|---|---|
| **60 Hz** | 40 | **0.178 s** |
| **12 Hz** | 90 | **0.270 s** |

The 0.092 s difference is worth **1.84 m of equivalent reach at 20 m/s**. It shows up
directly in a **matched 21 m pair** (same range, same sensor model, rate the only
difference): **12 Hz → median tca 0.730 s → 3/10 saved**; **60 Hz → median tca 0.890 s →
5/10 saved** (MEASURED).

So `range = speed × (tca + 0.177)` is **conditional on 60 Hz**. Quote the rate beside it or
do not quote it.

Required reach, MODELLED from the fit at the measured 60 Hz dead time:

| ball speed | reach for P = 0.50 | reach for P = 0.90 | reach for P = 0.95 |
|---|---|---|---|
| 8 m/s | 7.9 m | 8.6 m | 8.8 m |
| 14 m/s | 13.8 m | 15.0 m | 15.4 m |
| **20 m/s** | **19.8 m** | **21.5 m** | **22.0 m** |
| 24 m/s | 23.7 m | 25.7 m | 26.4 m |

At the 12 Hz dead time the 20 m/s row becomes **21.6 m / 23.3 m** instead.

Inverting for the real OAK-D Lite's measured **~3.4 m**:

    P=0.50 speed = 3.4 / (0.810 + t_dead)   ->  3.44 m/s at 60 Hz,  3.15 m/s at 12 Hz
    P=0.90 speed = 3.4 / (0.895 + t_dead)   ->  3.17 m/s at 60 Hz,  2.92 m/s at 12 Hz

The shipped depth path runs at **15 Hz**, nearer the 12 Hz measurement than the 60 Hz one,
so the low end of that bracket is the honest one. **Call it ~3.1–3.4 m/s for a coin flip,
and under 3 m/s for anything you would call reliable.** That is the closest achievable ball
speed as built — not the 14 m/s that Week 4's patrol battery happened to fail at.

### 2.5 The escape curve, refit at double n — HOVER only

| arm | fit | n | SE (log) | R² (log) |
|---|---|---|---|---|
| 14 m/s | `Δ = 0.333 · tca^3.27` | 58 | 0.30 | 0.673 |
| 20 m/s | `Δ = 0.339 · tca^3.60` | 56 | 0.25 | 0.791 |
| **pooled** | **`Δ = 0.337 · tca^3.39`** | **114** | **0.20** | **0.729** |

The prior fits (`0.295·tca^3.15` and `0.337·tca^3.69`) **replicate within one SE**. The
exponent is real and it is steep; that steepness is why the outcome is a threshold.

**Reconciling the curve with the LD50.** The pooled curve crosses a *full* 0.30 m at
**tca 0.967 s**, which is well above the 0.810 s LD50. That is not a contradiction: a throw
does not need 0.30 m of escape, it needs enough to clear the miss it was already going to
have. Median `counterfactual_min_m` is **0.127 m**, so the median **required** Δ is
**0.173 m**, and the pooled curve reaches that at **tca 0.822 s** — the LD50, to 0.012 s.

**Between-cell escape offset spans ×0.68 to ×1.36.** That is a 2.0× spread between cells
that differ in nothing that should matter. **Quote it whenever comparing two cells**, and
never referee an effect smaller than it across sessions.

### 2.6 Caveats that bound all of the above

- **Hover only.** A patrolling drone closes faster (effective 24 m/s against a 14 m/s
  ball); required range scales with *closing* speed, roughly 1.7× more at 14 m/s.
  `-p hover_mode:=true` is the only configuration that has ever delivered a hit at range.
- **`detection_range_m` is an INPUT, never a result.** A dodge scored under the oracle is a
  claim about the tracker, trigger and airframe *given* a sensor with that reach. It is
  never evidence that such a sensor exists. §6 is where that debt is paid.
- **The oracle has zero pipeline latency.** No camera does. §7.3.
- **The oracle is head-on through a ±45° cone**, a harness default, not a camera. §7.2.

---

## 3. The flown envelope — MEASURED at 20 m/s, hover, modelled stereo sensor

All cells below hold the reference sensor model `lab/params/oracle_stereo.yaml`
(**depth σ 0.30 m, bearing σ 0.02 m** — a synchronised AR0234-class stereo pair at ~150 mm
baseline), **26 m and 60 Hz unless the row says otherwise**, `hover_mode`, one lever varied
per cell, and every cell passed the per-cell contamination screen (§4.4).

### 3.1 False dodges at 26 m — the result that could have failed the project

A long-range sensor that fires at everything is worse than a short-range one. Near-miss
throws, scored on whether the trigger fired at all:

| aim miss distance | fires / scored | reading |
|---|---|---|
| 3.0 m | **0 / 6** | correct hold |
| 1.5 m | **0 / 9** | correct hold |
| 0.5 m | **6 / 6** | correct fire |

**0 false fires in 15. 6 correct fires in 6.** The discrimination boundary sits between
0.5 m and 1.5 m, which is where `threat_radius_m` 0.75 puts it. Aim error on the no-dodge
runs was **mean 0.01 m, max 0.10 m**, so the misses were the intended misses.

This **refutes an offline replay** that had predicted ~18% false fires at 1.5 m.

### 3.2 The range ladder

| oracle range (**INPUT**) | saved / on-course |
|---|---|
| 21 m | **6 / 9** |
| 23 m | **8 / 10** |
| 26 m | **10 / 10** |

MODELLED prediction from §2.4 for comparison, not as evidence: P = 0.83 at 21 m, 0.99 at
23 m, ~1.00 at 26 m. The ladder is consistent with the fit and was not used to build it.

### 3.3 The shipped configuration passes

Every earlier 20 m/s result was flown at `dodge_speed_mps` **3.0**, which is *not what
ships*. Re-flown at the shipped **1.5**, at 26 m: **10 / 10**. The configuration the
aircraft would actually carry is the configuration that scored.

### 3.4 Margin above 20 m/s, and rate

| lever | cell | result |
|---|---|---|
| ball speed 24 m/s (26 m) | `sp24` | **9 / 10** |
| rate 30 Hz instead of 60 (26 m) | `rt30` | **9 / 10**, tca mean **1.022 s** |

**20 m/s has margin above it**, and **60 fps is not required at 26 m** — 30 Hz clears it.
Rate still buys dead time (§2.4); it is just not binding once the reach is there.

MODELLED check: the pooled fit puts P = 0.90 at 24.4 m/s from a 26 m sensor at 60 Hz. The
flown 24 m/s cell returned 9/10. Consistent.

### 3.5 Measurement noise — the anisotropic claim, WEAKENED

Three cells at 26 m differing only in the oracle's per-axis noise:

| cell | depth σ / bearing σ | saved | tca 10–90 width |
|---|---|---|---|
| `hw26` | 0.15 m isotropic | **9 / 9** | **0.238 s** |
| `hw26aniso` | 0.30 / 0.02 m | **10 / 10** | **0.347 s** |
| `hw26mono` | 0.85 / 0.02 m | **7 / 10** | **0.507 s** |

**`hw26aniso` 10/10 against `hw26` 9/9 is NOT a demonstrated improvement.** Both arms are at
ceiling with no failures to separate them, and the 1.48× escape gap between them sits
*inside* the ×0.68–1.36 between-cell drift of §2.5. Any earlier reading of "anisotropic
noise helps" was reading drift.

**What the mono arm does show is spread, not mean.** Going from 0.15 m to 0.85 m of depth
noise widens the tca 10–90 band from 0.238 s to **0.507 s** while barely moving its centre.
That is the signature of a filter over-trusting a bad measurement: `evasion.yaml` declares
`meas_std_m` 0.15 m while the depth axis is really 0.85 m. The three `hw26mono` losses were
**late commits**, not weak escapes. Whether telling the filter the truth per-axis recovers
them is **QUEUED, UNFLOWN** (§7.5).

---

## 4. Instrument faults that had to be fixed before any of this was measurable

All four invalidated earlier results. They are recorded because the corrected numbers only
make sense against them.

### 4.1 Patrol could not deliver a hit at range — 0 of 98 throws

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

### 4.2 Patrol escape displacement was inflated 3–20×

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
that, which is why it was re-tested properly in §5.5.

### 4.3 The oracle was under-delivering its own range

`offset_forward_m` was 15.0 while the requested range was 12 m, so the ball entered the
detection gate from *inside* it: first detection landed at 9.9–11.3 m. Every pre-fix
"12 m" cell had actually measured a ~10–11 m sensor. Fixed by requiring
`offset_forward_m ≥ detection_range_m + 8.5` at 20 m/s.

**Verification rule, permanently:** `first_det_range_m` in the CSV must match the launched
`detection_range_m` to ~0.2 m, every run.

### 4.4 Seven concurrent ROS stacks voided a whole batch

A teardown bug let **seven ROS stacks run at once**. The kill patterns named
`evasion_nod[e]` and `patrol_nod[e]`, but colcon installs the executables as `evasion` and
`patrol`, so nothing matched and node processes outlived the `ros2 launch` that started
them. Duplicate pose bridges and duplicate evasion nodes then contaminated the data.

**Six cells were quarantined** to `results/VOID_dirty_stack/` and are not used anywhere in
this document. **All five historical headline cells were screened and are CLEAN.**

The screen is now per-cell and runs automatically (`lab/bin/lab_cell.sh`, "contamination
screen"). Cheapest signals first:

- **implied detection rate** `= (track_updates - 1) / track_age_s`. It cannot exceed the
  rate the oracle was launched with; if it does, more than one publisher is feeding the
  tracker.
- **`n_post`** — pose samples seen after the dodge. One 60 Hz bridge gives **230–354**.
- the **response-probe dodge count**, which must match the number of throws.

Related and equally load-bearing: `lab_cell.sh` reuses a stack that is still alive, so
`EXTRA_LAUNCH` overrides (`oracle_params`, `evasion_params`) silently land on nothing and
the battery re-flies the *previous* configuration under the new tag. `RANGE` and
`ORACLE_RATE` appear to work because the new banner echoes them, so nothing looks wrong
until the numbers are compared. `lab_queue.sh` owns the hard teardown and the
`EXPECT_BANNER` readback assertion that catches it in ~2 minutes instead of 25.

    nohup setsid bash ~/hz_lab/bin/lab_queue.sh ~/hz_lab/config/q1.txt \
      > /tmp/queue.log 2>&1 < /dev/null &

---

## 5. Why the answer is sensing: every maneuver-side *magnitude* lever, refuted

This is the load-bearing half of the "prove it is unattainable" requirement. If any of
these had worked, the answer would be software, not hardware.

Note the qualifier. Everything below varies the **magnitude** of the command. The one
lever that varies its **duration** has since been swept too, and is also null at the tca
these dodges commit at — §7.1.

### 5.1 The single result that retires most of them at once

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

### 5.2 `PSC_JERK_NE` — refuted twice over

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

### 5.3 `WP_ACC` — the renamed `WPNAV_ACCEL`

Peak demand is 1.43 m/s² against a 5.0 clamp; at the 2.5 default it binds for only 7% of
the window. The only real effect is *downward*: starving it to 1.0 cuts escape ~3×.

Worth recording as a naming trap: `WPNAV_ACCEL` was previously believed absent from this
build. It exists, renamed `WP_ACC`, exactly as `PSC_JERK_XY` became `PSC_JERK_NE`.
`PSC_ACC_XY`, `ANGLE_MAX` and `PSC_VELXY_P` genuinely are absent.

### 5.4 `evade_accel_ff_mps2` — null in flight

A 12-trial in-flight A/B found no dodge gain (0.107 m pre-fix vs 0.091 m post-fix; both
arms commanded real motion). An earlier 28/28 zero-metre result at *settled hover* did not
reproduce in flight, so it never invalidated the lane.

### 5.5 `dodge_speed_mps` — magnitude plateaus

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

**The truncation objection has been tested and does not overturn this.** The whole A/B was
measured inside a 1.0 s command window, and §7.1 asked whether the window rather than the
magnitude was what saturated. Doubling the window changes escape by **1.00×** at the tca
these dodges actually commit at (1.0–1.05 s). The 2× command ratio does grow with time —
1.10× at t = 1.0 s, 1.67× at t = 2.0 s — so magnitude and duration do interact **after the
ball has arrived**, which buys nothing. `command^0.065` stands as the operational number.

### 5.6 A real low-tca failure mode, found in passing

Two throws had the dodge move the drone **toward** the ball: dspd15 rep 0 (tca 0.710,
delta −0.1456, counterfactual 0.296 — on the 0.30 m boundary, so arguably ill-conditioned)
and **dspd60 rep 7 (tca 0.620, delta −0.0615, counterfactual 0.160 — nowhere near the
boundary)**. The second kills the boundary-artefact explanation for the first.

Both sit at the lowest tca in their arm, and the worse one came at the *highest* command.
**Below the threshold the dodge is not merely insufficient; it is sometimes harmful.** This
is untested as its own hypothesis and touches `horizontal_away_bearing()` / `away_hint`,
which stay out of conclusions until a controlled experiment tests them.

### 5.7 The rest of the null list

Refuted earlier and not re-opened: `roi_max_range_m` 5→8 · `min_track_updates` 3→2 ·
`cluster_min_points` 5→3 · 320×240 @ 30 Hz · the multi-hypothesis tracker · track
continuity (the associator does not fragment the ball — 1.05 spawns per throw, and MHT and
one unbroken filter commit at the same tca) · the command path (probe ok 16/16) · dodge
direction (~5° error) · vertical escape.

---

## 6. The sensor that closes the gap — MODELLED throughout

Nothing in this section is flown. It is arithmetic on vendor specifications and on the one
measured OAK-D reach, shown so it can be checked.

### 6.1 Get the ball diameter right first

**The projectile is 80 mm across**, not 70–72 mm:
`src/huitzilin_perception/models/projectile/model.sdf` gives `<sphere><radius>0.040`, 150 g,
baseball-class. Earlier revisions of this document used 0.07 m throughout. **Every optical
figure below states which diameter it assumes.**

**The flown results are unaffected.** The oracle detects from `/gz/dynamic_poses` ground
truth and never forms an image, so no save count, tca or escape number in §2–§5 depends on
the ball's apparent size.

### 6.2 The requirement, in the units that decide it

The quantity that matters is **IFOV** = pixel pitch / focal length, in rad/px. Pixels
across the ball is `N = D / (R · IFOV)`.

**Calibrate against the one measured number rather than guessing.** The OAK-D Lite mono
pair is OV7251, 640×480, 3.0 µm, HFOV 73° → active width 1.92 mm → f = 1.297 mm →
**IFOV = 2.312 mrad/px**. At the measured 3.4 m reach the ball spans:

| assumed diameter | N at 3.4 m | N at 21 m |
|---|---|---|
| 70 mm (legacy) | 8.90 px | 1.44 px |
| **80 mm (correct)** | **10.18 px** | **1.65 px** |

Stereo is separately dead at that range: disparity = B·f/Z = 0.075 × 432 / 21 =
**1.54 px** (against 9.5 px at 3.4 m). A 150 mm airframe cannot fix that, and prop flex of
0.1 px would corrupt what little disparity remained.

**Holding the measured detection threshold out to 21 m needs IFOV ≤ 0.374 mrad — 6.2× finer
than today.** That factor is *diameter-invariant*, and trivially so: the required IFOV is
`2.312 × 3.4/21`, and the ratio is just `21/3.4 = 6.18`. Whatever the ball's true size, it
is the same size in both terms and cancels. At the same 3.0 µm pitch, 6.2× finer means a
6.2× longer lens.

### 6.3 Reach, two ways — and why the answer is the same

There are two defensible ways to turn IFOV into reach, and the diameter correction moves
one of them and not the other.

**(a) The IFOV ratio against the measured reach — diameter-invariant, and conservative.**
`R = 3.4 m × 2.312 / IFOV_mrad`. The 80 mm ball is bigger *and* the implied pixel threshold
is proportionally higher, so D cancels exactly.

**(b) A fixed 8.9 px absolute threshold** — valid only if 8.9 px is a property of the
detector established independently, rather than a back-calibration from the 3.4 m
measurement under the wrong diameter. Under 80 mm this reads higher.

| lens (3.0 µm pitch) | IFOV | N on 80 mm @ 26 m | **reach (a), invariant** | reach (b), 8.9 px & 80 mm |
|---|---|---|---|---|
| 6 mm | 0.500 mrad | 6.2 px | 15.7 m | 18.0 m |
| 8 mm | 0.375 mrad | 8.2 px | 21.0 m | 24.0 m |
| **10 mm** | **0.300 mrad** | **10.3 px** | **26.2 m** | **30.0 m** |
| 12 mm | 0.250 mrad | 12.3 px | 31.4 m | 36.0 m |

**Take column (a).** It is the conservative reading, it is the one the diameter correction
cannot move, and **it already supports every flown cell**: the 26 m INPUT that scored 10/10
at 20 m/s and 9/10 at 24 m/s is exactly the 10 mm figure. Column (b) is the upside if the
$26 bench test in §8 confirms the threshold independently.

Converting reach to ball speed with the fitted sigmoid and the measured 60 Hz dead time
(`speed = reach / (tca + 0.178)`), and remembering these are **probabilities, not limits**:

| lens | reach (a) | speed at P = 0.50 | speed at P = 0.90 |
|---|---|---|---|
| 6 mm | 15.7 m | 15.9 m/s | 14.6 m/s |
| 8 mm | 21.0 m | 21.3 m/s | 19.6 m/s |
| **10 mm** | **26.2 m** | **26.5 m/s** | **24.4 m/s** |
| 12 mm | 31.4 m | 31.8 m/s | 29.3 m/s |

The 10 mm row's P = 0.90 speed of **24.4 m/s** sits directly on the flown **9/10 at
24 m/s** from §3.4. The model was not fitted to that cell.

**Take 10 mm rather than 8 mm.** At 8 mm, 20 m/s lands at P ≈ 0.90 with no headroom for
pipeline latency (§7.3) or a non-head-on approach (§7.2). 10 mm clears it with margin, and
the flown ladder was run at exactly that reach.

### 6.4 The part — and the mono/colour correction

> **onsemi AR0234CS or OV2311-class, 1/2.6", 3.0 µm global shutter, with a 10 mm M12 lens.**

| | e-con **See3CAM_24CUG** | e-con **See3CAM_20CUG** |
|---|---|---|
| sensor | AR0234CS, 1920×1200 | OV2311, 1600×1300 |
| **colour / mono** | **COLOUR — verified on e-con's page** | **mono** |
| price | $99 | **$89** |
| mass | 26 g with lens | 13.5 g **bare** (not directly comparable) |
| pixel pitch | 3.0 µm | 3.0 µm |
| **IFOV at 10 mm** | **0.300 mrad/px** | **0.300 mrad/px** |
| **reach (a)** | **26.2 m** | **26.2 m** |
| stated FOV | ~33° | ~27.5° |
| geometric FOV at 10 mm (derived) | 32.1° H / 20.4° V | 27.0° H / 22.1° V |

**Earlier revisions of this document named the See3CAM_24CUG as mono. It is not — it is a
colour part.** The mono equivalent is the **See3CAM_20CUG**. Both share the 3.0 µm pitch,
so **the reach argument is identical for either**; the choice is between colour information
and a lighter, cheaper, wider-vertical mono part. Mono is preferred for a differencing
detector, which wants raw luminance and no Bayer interpolation.

Either part is **lighter than the OAK-D Lite's 61 g** at comparable power (0.86–2.02 W,
USB 3.1 Type-C, 60 fps at full resolution for the 24CUG).

### 6.5 What the same part buys, and what it costs

**Bearing becomes very precise.** At 0.300 mrad/px, a 0.2 px centroid is
**0.060 mrad** — *not* the 0.075 mrad an earlier revision claimed. That is **1.3 mm of
cross-range at 21 m and 1.6 mm at 26 m**. The reference sensor model
(`lab/params/oracle_stereo.yaml`) deliberately models bearing σ at 0.02 m, **~12×
pessimistic**, and it still scored 10/10.

**Rate.** 60 fps is available, and the measured dead time is 0.178 s at 60 Hz against
0.270 s at 12 Hz (§2.4) — worth 1.84 m of equivalent reach at 20 m/s. But **60 fps is not
required at 26 m**: the flown 30 Hz cell returned 9/10 (§3.4). Rate is insurance, not the
gate.

**Field of view: 73° → 27–32° horizontal, and only ~20–22° vertical.** This is the real
price, and §7.2 is where it gets measured rather than argued.

**Stereo depth at range.** Replace it with the known-size prior, `R = D / (N · IFOV)`.
Fractional range error equals fractional diameter error, so `σ_R/R = σ_N/N`:

| assumed diameter | N at 21 m | σ_R at 21 m (0.3 px edge) | N at 26 m | σ_R at 26 m |
|---|---|---|---|---|
| 70 mm (legacy) | 11.1 px | **±0.57 m** | 9.0 px | ±0.87 m |
| **80 mm (correct)** | **12.7 px** | **±0.50 m** | **10.3 px** | **±0.76 m** |

An earlier revision quoted **±0.71 m at 21 m** by using **N = 8.9 px**, which is the *26 m*
pixel count, at *21 m*. The corrected figure is **±0.55 m**-ish either way, and slightly
better at the true 80 mm. The reference sensor model's stereo σ of 0.30 m is the
*synchronised-pair* case; 0.85 m in `oracle_mono.yaml` is the single-camera case, and that
is the arm that scored 7/10.

**Looming, scoped honestly.** The earlier flat rejection of τ = θ/θ̇ was derived at
**30 fps** and quoted 0.29 px/frame at 21 m. At the rate actually flown it is *smaller*,
not larger, because halving the frame interval halves the per-frame growth:

| | 70 mm | 80 mm |
|---|---|---|
| single-frame Δ at 21 m, 20 m/s, **30 fps** | 0.36 px | 0.42 px |
| single-frame Δ at 21 m, 20 m/s, **60 fps** | **0.18 px** | **0.21 px** |

(The earlier 0.29 px/frame does not reproduce from `ΔN = D/(R−v/fps)/IFOV − D/R/IFOV` at
any of these inputs; a straight re-derivation at 30 fps and 70 mm gives 0.36 px. Use the
table.)

**So the rejection stands, but only for a single frame pair, and it is worse at 60 fps than
at 30.** What works is the *multi-frame* fit, which is the same measurement written
differently: a linear fit of R(t) over 0.2 s at 60 fps (n = 12) gives closing speed to
**±3.6 m/s**, i.e. tca to ±18% — adequate to trigger, and tightening rapidly as the ball
closes. And τ = N/Ṅ from that fit is **diameter- and calibration-invariant**, so the right
thing to gate on at long range is **τ, not metres**. That invariance is exactly why the
80 mm correction does not move any flown result.

**Do not plan on a CNN.** The published embedded floor for small-object detection is
~20×30 px at 24 fps in 1280×720 — 3–4× more pixels than will be available. At 6–13 px the
right tool is IMU/homography-compensated temporal differencing plus connected components:
search at half-res (960×600 @ 60 Hz ≈ 35 Mpx/s, comfortable on a Pi 5 with NEON) and
re-measure the blob at full resolution inside a small ROI, so N stays high where it counts.

### 6.6 Rejected alternatives, each with the reason it fails

| Candidate | Why it fails |
|---|---|
| **All flyable lidar** | **Point density, not range.** Livox Mid-360: 200 kpts/s over 21,240 deg² = 9.4 pts·deg⁻²·s⁻¹; the ball subtends 0.0287 deg² at 21 m → **0.018 points per 15 Hz frame**. Landing 3 points needs ~1,570 pts·deg⁻²·s⁻¹ in-sector; the best sub-250 g unit (Hesai JT16, 48) is 33× short *before* restricting the sector. |
| **Single-chip mmWave** (IWR6843) | Fails twice, independently. A plastic ball is ~−37 dBsm against a human's ~0 → ~4–10 m. And 15° azimuth resolution = **5.5 m of cross-range at 21 m**, so no usable dodge direction. Doppler is its one genuine strength and rescues neither failure. |
| **OAK-D LR** | Optics are fine, but ~700 g and 5.5 W — multiples of the entire AUW. |
| **OAK 4 D / 4 S** | 325–674 g, up to 25 W. |
| **Prophesee GenX320** | Pixels are fine; there are only 320 of them, so ≤20° coverage at any usable N. Space-bandwidth product is what fails. |
| **Thermal** (FLIR Boson) | A ball is at ambient temperature. No contrast. |
| **Acoustic array** | 61 ms of propagation delay from 21 m, and tens of degrees of bearing accuracy. |

### 6.7 The runner-up, which is a real option

**Prophesee EVK4 HD (IMX636) + 16 mm C-mount** — 40 g, ~1.5 W, <100 µs pixel latency,
120 dB dynamic range, 11.0 px at 21 m on a 70 mm ball (12.6 px on 80 mm) over 22.1°.
Strictly better than the AR0234 on latency and on rejecting static clutter; strictly worse
on FOV, on cost (quote-only, ~$3–5k), and on how much perception code must be written from
scratch.

This is the sensor family behind UZH's ball-dodging quadrotor (Falanga, Kleber &
Scaramuzza, *Science Robotics* 2020) — but their demonstration was a ball at 3 m and
10 m/s, so they never faced this range problem. **Take this only if the AR0234 flies and
the residual failure turns out to be latency or clutter rather than reach.**

---

## 7. Open questions

§7.1 has since been answered and is kept here with its result. Everything below it names
an experiment and what it would distinguish, with **no outcome predicted**.

### 7.1 `dodge_duration_s` — ANSWERED, and it is a null at operational tca

This was the last maneuver-side lever standing, because everything in §5 varied command
*magnitude* and this varies its *duration*. It has now been swept, and it does not buy
saves.

**The mechanism was real.** `dodge_duration_s` is **1.0 s** while `trigger_horizon_s` is
**1.5 s**, and the two are uncoupled. In the flagship 26 m cell the tca at commit ran
**0.93–1.24 s**, so nine of ten throws were still inbound when the command ended, and at
the cutoff `evasion_node` publishes **zero velocity** — a body-frame zero is a brake, not a
coast. The prediction was that removing the brake would add escape where it counts.

**It does not, because the brake is largely absorbed by coasting.** Paired arms flown
adjacent in one queue (`q4b`), 26 m / 20 m/s / hover / `dodge_speed_mps` 3.0, medians:

| t (s) | 1.0 s command (`dur10b`, n = 10) | 2.0 s command (`dur20c`, n = 3) | ratio |
|---|---|---|---|
| 1.00 | 0.510 m | 0.512 m | **1.00** |
| 1.20 | 0.880 m | 0.925 m | 1.05 |
| 1.40 | 1.342 m | 1.416 m | 1.05 |
| 1.60 | 1.805 m | 1.980 m | 1.10 |
| 2.00 | 2.597 m | 3.165 m | 1.22 |

**tca at commit was 1.048 s and 1.007 s, so the operating point is t ≈ 1.0–1.05 s — where
the ratio is 1.00.** The separation only opens past 1.2 s, which is after the fact. The
1.0 s arm still reaches 2.60 m by t = 2.0 s despite its zero-velocity cutoff at 1.0 s: the
vehicle does not stop when told to stop, it decays.

**And the longer command has a cost.** `dur20c` dodged cleanly on throws 0–2 and then
**no-fired seven consecutive times**, with altitude wandering to **4.63 m** against the
5.0 m `escape_ceiling_m`. A 2.0 s command drifts the aircraft off station faster than it
recovers between throws, which is also why the treatment arm is n = 3.

**The thin arm is corroborated, not assumed.** An earlier independent cell at the same
settings (`dur20`, n = 10) reached **3.144 m** at t = 2.0 s against `dur20c`'s 3.165 m — a
0.7% difference across sessions, well inside the ×0.68–1.36 between-cell drift. The
treatment curve is solid; only the head-to-head is thin.

One confound is noted and not separated: `evasion_node` passes
`descent_len_m = dodge_speed_mps × dodge_duration_s` into the ground clamp, so doubling the
duration also re-aims the escape more horizontal. It does not matter to the conclusion —
the arms are indistinguishable at the operating point either way.

Queues: `lab/config/q4_duration.txt` (first attempt, control lost to the instrument crash
of §4.4) and the `q4b` redo that produced the table above.

### 7.2 Field of view — QUEUED, UNFLOWN

**Every save in this document was a head-on throw through a ±45° oracle cone.** That is a
harness default, not a property of any camera.

The 10 mm M12 that buys 26 m of reach is also the lens that narrows the frustum:

    OV2311 1600 x 3.0 um = 4.80 mm wide -> atan(2.40/10) = +/-13.5 deg horizontally
    OV2311 1300 x 3.0 um = 3.90 mm high -> atan(1.95/10) = +/-11.0 deg vertically

**Reach and coverage are the same optical purchase, and the campaign has only ever measured
one side of it.** The oracle models a symmetric cone, so ±13.5° is the horizontal figure
and is **optimistic for a ball arriving high or low**, where the true limit is ±11.0°.

`lab/config/q3_fov.txt` pairs `oracle_stereo.yaml` (±45°) against
`oracle_stereo_fov13.yaml` (±13.5°) over the same three approach angles (0°, 10°, 20°),
making the control internal: the ±45° arm shows what oblique geometry alone costs, the
±13.5° arm adds the optics on top, and the difference is attributable. **UNFLOWN.**

### 7.3 Pipeline latency — QUEUED, UNFLOWN

**The oracle publishes a centroid the instant the world moves. No camera does.** A real
AR0234 pipeline spends time on exposure, readout, USB3 transfer and detection before the
centroid is usable. That time comes off tca directly, and tca is the only term §2.3 says
matters.

A `detection_delay_s` model now exists (`oracle.DelayLine`). It models the **charitable
case**: the centroid keeps its **exposure** timestamp and only **arrives** late, so the
filter can predict forward across the delay and lose no accuracy — it simply cannot act on
data it does not have. A pipeline that stamps at *arrival* would additionally bias the
state estimate and would be a worse sensor than anything measured here.

**Therefore: exposure-time stamping is a hardware requirement**, and it must go into the
hardware readiness package as one, whatever the cells return.

`lab/config/q5_latency.txt` runs 0 / 30 / 60 ms at **23 m**, not 26 m — 26 m is at ceiling
and would report 10/10 three times while measuring nothing. **UNFLOWN.**

### 7.4 Speed ladder to 26 and 28 m/s — QUEUED, UNFLOWN

`lab/config/q6_ladder.txt` flies 26 and 28 m/s in **one cell, two arms**, so the two are
directly comparable with no between-cell drift. Combined with the flown 20 m/s (10/10) and
24 m/s (9/10) at the same 26 m range, it turns the range/speed relation from a fitted curve
into a four-point measured ladder — and it is a **falsification test** of §2.2, since the
fit makes a specific prediction at each rung. **UNFLOWN.**

### 7.5 Measurement covariance A/B — QUEUED, UNFLOWN

`meas_std_xyz_m` now lets the filter be told the per-axis truth instead of the hardcoded
isotropic 0.15 m. `lab/config/q2_covariance.txt` tests whether that recovers the three
`hw26mono` late commits (§3.5). It flies **OFFa → ON → OFFb**, bracketing the treatment
with two identical controls, precisely because two identical arms have drifted 1.58× apart
in this harness. If OFFa and OFFb disagree by more than ON-vs-OFF, the cell measured drift
and that is the reportable result. **UNFLOWN.**

---

## 8. The assumption this all rests on, and the $26 experiment that retires it

**The detection threshold in pixels is derived from a single measured range.** It bundles
pixel count together with *contrast* and with the stereo-clustering step. A 10 px dark ball
against open sky is trivial; the same ball against a treeline at 21 m is not. If the 3.4 m
figure was partly a contrast or clustering limit rather than a pure angular-resolution
limit, every range in §6.3 is an optimistic ceiling — and it is also what decides whether
column (a) or column (b) of that table is the right one.

**Retire it before committing to airframe integration.** An **Arducam OV9281** ($26–36,
5–10 g, MIPI CSI-2, 120 fps) with an 8 mm M12 lens has the *identical* 0.375 mrad IFOV —
same 3.0 µm pitch, same focal length. Bolt it to a bench, throw **80 mm** balls at it, and
measure first-detection range directly. The answer transfers to the AR0234, and it costs a
weekend and less than the lens.

If cash or CSI lanes are the constraint, note that three OV9281s cost less than one of
anything else here and give the same pixels-on-target over 27.0° each (1280 × 3.0 µm =
3.84 mm at 8 mm focal length) — which is also a partial answer to §7.2.

---

## 9. Provenance

**MEASURED in SITL:** every save count, tca value, escape displacement, fitted curve,
logistic fit, dead-time proxy, false-dodge count and dataflash velocity comparison in
§2–§5. Conditions are stated per table. **Every range is an oracle INPUT.** All cells used
are hover; patrol cells are excluded from every pooled statistic. All cells passed the
contamination screen of §4.4; the six quarantined cells are in `results/VOID_dirty_stack/`
and appear nowhere here.

**MODELLED, arithmetic shown and independently re-checked:** every IFOV, FOV,
pixels-on-target, disparity, σ_R, looming rate, lidar hits-per-frame, required reach and
implied ball speed in §6, plus the required-reach table in §2.4. The FOV model reproduces
Arducam's published 40° H for an 8 mm M12 on 1/2.5" to within 1%. Two independent
consistency checks the model was *not* fitted to: it reproduces the measured 3.4 m → ~3.4
m/s as-built figure, and its P = 0.90 speed of 24.4 m/s from a 26 m sensor lands on the
flown 9/10 at 24 m/s.

**QUEUED BUT UNFLOWN:** everything in §7.2–§7.5. No outcome is claimed for any of them.
§7.1 is flown; its treatment arm is n = 3, stated as such at the point of use.

**Vendor specifications:** all masses, powers, prices, resolutions, pixel pitches, frame
rates, colour/mono status and interfaces in §6.

**Repo facts:** ball diameter 80 mm from
`src/huitzilin_perception/models/projectile/model.sdf`; `threat_radius_m` 0.75,
`trigger_horizon_s` 1.5, `dodge_duration_s` 1.0, `dodge_speed_mps` 1.5, `meas_std_m` 0.15
from `src/huitzilin_perception/params/evasion.yaml`; sensor models from `lab/params/`.

**Judgment calls, flagged as such:** which of §6.3's two reach columns is correct (§8); the
assumption that an event sensor detects at N ≈ 4; the −37 dBsm ball RCS estimate (a
first-principles optical-region calculation with a crude dielectric correction — could be
±10 dB, but radar fails on angular resolution regardless).

The lab toolchain that produced every measurement, the queue files for every unflown
experiment, and the raw result CSVs are in [`lab/`](../lab/). Sharp edges and the
do-not-re-run list are in [`CLAUDE.md`](../CLAUDE.md).
