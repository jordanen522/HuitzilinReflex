# Held-out perception evaluation — PRE-REGISTRATION

**Written 2026-08-21. Nothing in the H-set has been captured, and nothing has
been scored, at the time this file was committed.** That is the point of it:
every definition, threshold and success criterion below is fixed *before* the
data exists, so the result cannot be produced by choosing a definition after
seeing the numbers.

This document is the contract. If a later result disagrees with it, the result
stands and this document is annotated — never the reverse. Amendments are made
by adding a dated section at the bottom, never by editing a definition in place;
`git log -p docs/perception_eval.md` is the audit trail.

---

## 1. What is being claimed, and what would falsify it

The claim under test:

> The frozen detector achieves 100 % recall on a held-out set of at least 17
> positive projectile scenarios, with false positives reported separately.

It is falsified by a single unattributed positive. There is no partial credit and
no "excluding S08"-style carve-out available: the set is enumerated in §4 before
capture, and every enumerated positive is in the denominator.

### Why the existing library cannot answer it

`config/scenario_matrix.yaml` already carries three splits, and the honest
accounting in its SPLITS block and in README.md is that none of them supports the
claim:

| split | contents | why it cannot carry the claim |
|---|---|---|
| `train` | S01–S10, N01–N04 | `detector.yaml`'s own comments name these IDs as the bags its thresholds were fitted on. A recall figure here is a fit statistic. |
| `test` | S11, S12, N05 | Genuinely held out from the Week 3 tuning — but **two positives**. 2/2 is not evidence of 100 % over 17. |
| `tune` | T01–T14 | Exists to be fitted against. Disqualified by construction. |

Full-library positive recall is **11/12** (S08, a 14 m/s near-miss, is a known
and never-root-caused false negative). A held-out claim of 100 % over 17
positives is therefore *not* a restatement of anything already measured. It
requires new data.

Three further reasons the existing bags cannot simply be relabelled held-out:

1. **They record no ground truth.** `capture_scenario.sh` records
   `/oak/depth /oak/points /clock /huitzilin/odom /threat/centroid` and nothing
   else. Without `/gz/dynamic_poses` there is no way to ask whether a detection
   was *of the ball* — see §3.
2. **Bags recorded before `b0eedd5` lack attitude in `/huitzilin/odom`**, so the
   detector falls back to camera-frame differencing. CLAUDE.md forbids scoring
   against them.
3. **They were captured on the OAK-D world**, which is not the sensor the final
   system flies. See §2.

---

## 2. Which system is under evaluation

**The final configuration, not a historical one.** Concretely:

| | |
|---|---|
| world | `worlds/huitzilin_runway_ar0234.sdf` |
| camera | `models/iris_ar0234/model.sdf` — 800×650, ±13.5 × 11.0°, 0.2–35 m clip, 30 Hz requested |
| error model | `depth_noise_node.py`, sigma(z) = 0.30 · (z/26)^2 m along the ray, correlated over 7 px |
| detector | `detector_node.py`, unmodified, on `params/rendered_detector.yaml` |
| launch | `launch/week7_rendered.launch.py` |

This is the same instrument the 20 m/s dodge claim is measured on, so the two
résumé bullets describe **one** system rather than two eras. Evaluating the
Week 4 OAK-D detector instead would be easier and would measure a configuration
the project no longer flies.

A sensor is reach × sector × rate. All three are pinned by the world file above
and must be quoted with every number produced under this protocol; there is no
`detection_range_m` in this lane, and reach is an output.

### 2.1 Freeze procedure

1. The evaluated configuration is the tree at a named commit. The commit SHA of
   `params/rendered_detector.yaml`, `huitzilin_perception/detector_node.py`,
   `huitzilin_perception/depth_noise*.py`, `models/iris_ar0234/model.sdf` and
   `worlds/huitzilin_runway_ar0234.sdf` is recorded in the run artifact **before
   the first H-bag is captured**.
2. Between the freeze and the single scoring pass, none of those files may
   change. If any of them changes for any reason, the H-set is burned: either the
   scoring pass has not happened yet (fine — re-freeze) or it has (then the H-set
   is spent, and a *new* held-out set must be captured for the new
   configuration).
3. The freeze is verified mechanically at scoring time: the scorer records
   `git rev-parse HEAD` and `git status --porcelain` for those paths, and a dirty
   tree is reported in the artifact rather than being cleaned up.

### 2.2 Where tuning is allowed to happen

Nowhere, on this data. If the rendered detector needs re-tuning — and the
prediction registered in `params/rendered_detector.yaml` says a range-dependent
difference threshold may well be needed — it is fitted on a **tune** split
captured in the same world, analogous to T01–T14, and the H-set is not opened
until that work is finished and re-frozen.

The parameters `rendered_detector.yaml` inherits from `detector.yaml` were
historically fitted on S01–S10/N01–N04. That is provenance, not contamination: it
makes those *bags* unusable as held-out data, and says nothing about the H-set,
which no fitting has ever seen.

---

## 3. Definitions, fixed before capture

### 3.1 Ground truth

`scripts/capture_scenario.sh` must additionally record `/gz/dynamic_poses`
(`tf2_msgs/TFMessage`, one `TransformStamped` per entity, published by
`gz_pose_bridge`). The projectile's true position at any time is its transform in
that stream; the drone's is the `iris_ar0234` transform.

A bag with no `/gz/dynamic_poses` messages, or with none for the projectile in a
positive scenario, is **void** — not a failure, not a success. It is re-captured.
A void bag is reported by ID in the artifact so the count of voids is visible.

### 3.2 A matched detection

A `/threat/centroid` message at sim time *t* is a **matched detection** iff

    || p_detected(t)  -  p_ball(t) ||  <=  R(z)

where `p_ball(t)` is the projectile's truth position interpolated to *t* from the
two nearest `/gz/dynamic_poses` samples (linear; the pose stream runs far faster
than the cloud), both expressed in the same fixed frame, and

    R(z)  =  max( 0.50 ,  3 * sigma(z) )  +  v_ball * 0.020      [metres]
    sigma(z)  =  0.30 * (z / 26)^2                               [metres]

with *z* the detection's own range from the camera.

Rationale, all of it fixed here rather than chosen later:

- **3 sigma** because the modelled error dominates at range and a 3-sigma gate
  admits ~99.7 % of true detections. At 26 m that is 0.90 m; at 5 m, where sigma
  collapses to a centimetre, the floor binds instead.
- **The 0.50 m floor** stops R collapsing to centimetres at short range, where
  voxelization (0.02 m), the ball's own 0.08 m extent and clustering
  quantisation dominate the modelled error.
- **v_ball · 0.020** is one 20 ms allowance for stamp skew between the cloud and
  the pose stream. At 20 m/s that is 0.40 m and it is not negligible.

`R` is a *matching* tolerance, not an accuracy claim. Centroid accuracy is
reported separately as the distribution of `|| p_detected - p_ball ||`, so a
system that scrapes in under R on every frame cannot hide behind a pass.

### 3.3 Recall (the primary metric)

A positive scenario is **recalled** iff, inside its strict detection window
(`score_bags_logic.strict_window_bounds`, unchanged), there are at least

    K = 3

matched detections.

**K = 3, not 1.** `evasion.yaml` sets `min_track_updates: 3` — three
confirmations before a dodge may fire. A single matched frame is not a detection
the rest of the system can act on, and a recall metric that counted it would
measure something the aircraft cannot use. K is therefore the downstream
requirement, taken from the shipped parameter rather than chosen to be
achievable.

    recall  =  (positives recalled) / (positives enumerated in §4)

reported with a Wilson 95 % interval. **The denominator is §4's enumeration, not
the count of bags that captured cleanly.** Void bags (§3.1) are re-captured; they
are never dropped from the denominator.

### 3.4 False positives (reported separately, never merged)

Recall and false-positive performance are different numbers over different data
and are never combined into a single score.

Over the **H-negatives** (§4.2), a scenario **fires** iff it contains >= K
`/threat/centroid` messages within its window that are *not* matched to any truth
entity — i.e. clutter, terrain, or the airframe's own standoffs promoted to a
threat. Reported as:

    false-positive rate  =  (negatives that fired) / (negatives enumerated)

plus, as a descriptive statistic over **all** scenarios including positives, the
count of unmatched `/threat/centroid` messages per scenario. A positive scenario
that is correctly recalled *and* emits spurious centroids is a real defect, and
this is the number that shows it.

### 3.5 What is deliberately not a metric

- **Precision over the pooled library.** It would let a good negative set paper
  over a missed positive.
- **Any per-frame recall.** The unit is the scenario. A scenario with 40 matched
  frames and one with 3 both count once.
- **Anything computed from `dodged`.** That is a dodge-battery quantity, from a
  different lane with a different scoring rule.

---

## 4. The held-out set, enumerated before capture

Enumerated here first, then mirrored into `config/scenario_matrix.yaml` as the
`heldout:` split with IDs `H01…`. If the two ever disagree, the yaml is wrong.

### 4.1 Positives — 18

Eighteen rather than the minimum seventeen, so that a single void or mis-thrown
bag can be re-captured without the set silently shrinking to exactly the
threshold. **All eighteen are in the denominator.**

Design axes, chosen disjoint from every value already used by S01–S12 and
T01–T14, so this set cannot be a re-run of tuning data:

| axis | H-set values | already used by S/N | already used by T |
|---|---|---|---|
| speed (m/s) | 5, 9, 13, 16, 18, 20 | 4, 8, 12, 14 | 6, 11, 17 |
| approach angle (deg) | 0, ±8, ±12, ±28 | 0, +5, −20, −25, ±30, 180 | 0, ±15, +22, ±35 |
| miss distance (m) | 0.0, 0.2, 0.7, 1.1 | 0.0, 0.1, 0.4, 0.5, 1.2, 1.5 | 0.0, 0.35, 0.9, 2.0 |

The speed axis reaches 20 m/s, which the earlier splits never did, because this
lane's whole purpose is that range. Every positive spawns outside
`roi_max_range_m` (28.00 m) by the offset rule in
`config/week7_rendered_battery.yaml`, so every bag contains the acquisition
transient rather than starting with the ball already in the gate.

### 4.2 Negatives — 6

At least six, keeping the library's >= 25 % negative budget (6/24 = 25 %):

1. clean straight-leg patrol, no spawn;
2. clean hover, no spawn;
3. patrol through a turn — egomotion-induced depth change, the FP class
   `detector.yaml` names as this library's dominant one;
4. ball spawned behind the camera (outside the ±13.5° sector);
5. ball passing far outside the sector but inside the range gate — the case the
   narrow AR0234 lens makes newly interesting;
6. ball thrown at a wide miss, well outside any hit radius, at 20 m/s.

Negative 6 is deliberately the hardest: the ball *is* real and *is* detected, so
it separates "fabricated a threat from clutter" from "detected a real object and
mis-assessed it". Those two are reported separately per §3.4.

### 4.3 Disjointness rules

- No H-scenario may share a full parameter triple (speed, angle, miss) with any
  S, N or T scenario.
- No H-bag may ever be replayed during tuning, threshold selection, debugging of
  a threshold, or any run whose output influences a parameter value. Debugging a
  *harness* fault (a dead node, a missing topic) is permitted and must be noted
  in the artifact.
- The H-set is scored **once** per frozen configuration.

---

## 5. Success criteria, fixed before running

Stated as a decision table so there is nothing left to decide afterwards:

| measured recall on H-positives | what may be claimed |
|---|---|
| 18/18 | "100 % recall on 18 held-out positive scenarios" |
| 17/18 | "94 % recall (17/18)". **The 100 % claim is dropped.** |
| < 17/18 | the measured figure, and an investigation |

The false-positive rate is reported alongside in every case, as its own fraction
over the six negatives, and never folded into the recall number.

**If the target is missed, the number reported is the number measured.** The
remedies available are: capture a *new* held-out set against a re-tuned and
re-frozen configuration (reporting both results, old and new, with their dates
and SHAs), or state the measured recall on the résumé. The remedies *not*
available are: re-defining K, R, the window, the denominator or the set
membership after seeing the result; dropping a scenario as "unrepresentative";
or reporting the best of several scoring passes.

If a scenario fails for a demonstrable harness fault — no `/clock`, a dead node,
a bag with no cloud messages — it is **void** per §3.1, re-captured, and both the
void and the re-capture appear in the artifact. "Harness fault" means a fault
demonstrable from the artifact independently of whether the detection succeeded;
"the detector didn't see it" is never a harness fault.

---

## 6. Artifact

One machine-readable record per scoring pass, committed to the repo, containing:

- the freeze SHAs and `git status --porcelain` of the frozen paths (§2.1);
- the world file, camera model, delivered cloud rate, sector, and detector params
  file actually used;
- per scenario: ID, label, matched-detection count, K, window bounds, void flag,
  unmatched-centroid count, and the centroid-error distribution;
- the two headline fractions, each with its own denominator;
- the date, and the machine it ran on.

Every number in `docs/VERIFICATION.md` that concerns perception recall points at
this artifact and at the exact command that produced it.

---

## 7. Order of operations

1. Commit this document. *(done — this commit)*
2. Add `/gz/dynamic_poses` to `capture_scenario.sh`.
3. Implement the truth-attributed scorer against §3, test-first, including a
   ball-free control that must score zero matched detections.
4. Add the `H01…` entries and the `heldout:` split to `scenario_matrix.yaml`.
5. Freeze (§2.1) and record the SHAs.
6. Capture the H-set on the Dell.
7. Score **once**. Write the artifact. Report both fractions.

Steps 2–4 may be revised freely; they are instrumentation, and any revision
happens before step 5. After step 5, this document is fixed.
