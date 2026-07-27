# Project Journal — HuitzilinReflex

Compacted log: one entry per closed week, keeping only decisions and facts that still
matter. Full narrative history lives in git (`git log -p docs/JOURNAL.md`).

## Week 1 (2026-06-15) — toolchain + first scripted flight ✔

- ArduPilot SITL + Gazebo Harmonic + ROS 2 Jazzy stood up; first pymavlink flight
  (arm → takeoff → hold) verified in QGC.
- Versions locked: ROS 2 Jazzy, Gazebo Harmonic 8.11.0, ArduPilot main, Python 3.12.3.
- WSL2 dev box renders Gazebo at ~24% real-time (no GPU passthrough) → headless
  operation + sim-time discipline ever since.

## Week 2 (2026-06-18) — autonomous patrol loop ✔

- `huitzilin_sim` flies arm → takeoff (2 m) → 5×5 m square → loop through the
  ROS 2 ↔ pymavlink bridge; one-command bring-up via `week2_sitl.launch.py`.
- Evidence: 43 laps, mean 29.51 s, stdev 0.93 s — `docs/week2_patrol_evidence.md`.
- Root-caused the silent no-lift takeoff bug (`FRAME_CLASS=0` on fresh EEPROM) plus the
  port-fanout, patrol-autostart, and `.parm` comment traps — all recorded as sharp edges
  in `CLAUDE.md`; don't re-derive them here.
- Deferred: W2-05 Pass-B airframe fidelity → Week 7–8. W2-18 fresh-checkout sign-off →
  effectively satisfied by the Week 3 Dell bring-up from a fresh checkout.

## Week 3 (closed 2026-07-15) — perception pipeline ✔

Detection node + scenario tooling written, tuned, and scored against a 17-bag labeled
library; held-out test split passed; live acceptance run confirmed on the Dell.

### Egomotion regression (2026-07-06) — why pre-b0eedd5 bags are invalid

Train recall had collapsed to 60%. Three independent defects:

1. **`compensate_egomotion` was declared but never implemented** — differencing ran in
   the moving camera frame, so patrol motion turned the whole cloud into "foreground"
   and the `fg_max_points` flood guard discarded the frames containing the ball.
   Fix: clouds re-expressed in `odom` before differencing (`fixed_frame` param); pure
   math extracted to `cloud_geometry.py` with unit tests.
2. **`/huitzilin/odom` carried no orientation** (all-zero quaternion) at only 10 Hz.
   Fix: `MavBridge.ned_rpy_to_enu_quat()` (unit-tested), cached telemetry in
   `get_state()`, `stream_rate_hz` 10 → 30. **Bags recorded before b0eedd5 lack
   attitude — the detector detects the invalid quaternion and falls back to
   camera-frame mode. Never score against pre-b0eedd5 bags.**
3. **`score_bags` booked a missing bag as a false positive** — missing bags are now
   harness errors that fail the run explicitly.

### Final results (2026-07-15)

- `detector.yaml` tuned on the train split (commits `40d79f2` → `454e312`):
  `diff_threshold_m` 0.10, `roi_half_angle_deg` 45, `cluster_min_points` 5,
  `roi_max_range_m` 5.0.
- Train (14 bags): recall 90% (FN: S08), precision 81.8% (FP: N02/N03).
- Held-out test (S11/S12/N05, never tuned against): **recall 100%, FP on N05,
  `run_regression.sh … test` exits 0 — PASS.** This is the Week 3 DoD number.
- Acceptance run (W3-21): live end-to-end on the Dell, S02 spawned live, centroid
  marker confirmed in RViz, bag at `/data/huitzilin_bags/week3_acceptance`.

### Open items carried into Week 4

- **S08 false negative** (14 m/s near-miss, train split) — never root-caused. Two
  blind threshold attempts (range cut, range restore) regressed other scenarios
  without fixing it. Any re-tune must start with a single-bag `debug_funnel:=true`
  trace on S08, not threshold guessing.
- **N02/N03/N05 false positives** (patrol-turn and background-clutter triggers) —
  hurt precision, don't fail the recall gate; uninvestigated.
- **`/huitzilin/cmd_vel` produced no observable motion** in one manual test —
  verify before Week 4 evasion commands depend on that path.

### Week 4 inherits

- `/threat/centroid` (`geometry_msgs/PointStamped`, reliable, `base_link`) — the only
  contract the Kalman/dodge node consumes; active in `docs/architecture.md`.
- `detector.yaml` @ `454e312` as the tuned operating point (imperfect, best measured).
- Bag library at `/data/huitzilin_bags` (17 scenario bags + acceptance bag);
  re-capture procedure in `docs/week3_capture_runbook.md`.

## Week 4 (in progress, live bring-up 2026-07-26) — Kalman + dodge trigger

First live bring-up on the Dell over SSH. Three root causes found and fixed; the
dodge mechanism is proven end-to-end but the battery is not yet passing.

### Fixed

- **Stray projectile aborts the physics engine** (`cbfda03`). A ball left in the
  world has link gravity off and no rolling resistance, so it rolls until its AABB
  no longer fits ODE's hash-space int quantization and Gazebo aborts the *whole
  world*: `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound
  < dMaxIntExact" failed in collide()`. Killed the world after ~9 idle hours, taking
  SITL's FDM link with it. Leftovers had been seen at x=-42 m and x=-207 m and
  dismissed as litter. `spawn_projectile` now self-removes after `lifetime_s`
  (default 20 s wall); `dodge_battery` already called `gz_remove`.
- **`/oak/points` starved the detector** (`cfd3f3a`). The cloud is a 7.37 MB sample
  (640×480 × 24 B) fragmented across thousands of UDP datagrams against a 208 KB
  `net.core.rmem_max`; under BEST_EFFORT one lost fragment discards the entire
  sample with no retransmit. Measured on the same topic, one variable changed:
  best_effort depth=1 → **3.99 Hz**, reliable depth=10 → **14.43 Hz**. The detector
  itself was down to **1.25 Hz**, so a ball crossing the frustum in ~0.75 s of sim
  time was seen once and no track could mature. Now RELIABLE depth=5 → **13.7 Hz**
  (11× more frames). `cloud_reliable: false` for a real OAK-D.
- **odom yaw froze at a value minutes old** (`8205e98`). `MavBridge.get_state()`
  used two type-filtered `recv_match` calls; `recv_match(type=X)` discards every
  message it scans past, so the position fetch threw away the queued ATTITUDE
  messages. Position stayed exact while yaw went stale — measured: odom position
  matched Gazebo truth to the millimetre while odom yaw read **180.0°** against a
  true **85.8°** (AP NED yaw 4.18°; `ned_rpy_to_enu_quat` was correct all along).
  Consequences: `spawn_projectile` threw the ball ~94° away from where the camera
  pointed, so the detector never saw it; and the detector rotated clouds by a dead
  attitude. Hid for a whole session because a stale yaw is still right just after
  takeoff — exactly when the one successful smoke throw ran. Now drains by type.

### Measured

- **Dodge works, hovering.** Ball tracked continuously 4.40→0.28 m with
  `ball_sized=1` every frame (extent 0.09–0.13 m, correct for the 80 mm ball),
  9 centroids, then `DODGE: miss=0.04 m tca=0.46 s latency=112 ms`.
- **Battery (patrolling), before the yaw fix: 0/18.** After: **2/18 (11%)**,
  0/2 false dodges, latency mean 213 ms / max 337 ms against a 150 ms budget.
  One run made budget: B03 r0 (14 m/s) dodged at 89 ms, miss 0.303 m.
  Results kept on the Dell: `/tmp/week4_battery{,_patrol,_hover,_fgmax}.csv`.
- With the duplicate stack gone the Dell runs near **real-time**, not the 0.33 RTF
  recorded earlier — that figure came from two stacks competing.

### Open blocker — detector is blind while the drone translates

81% of frames trip `fg_max_points` during patrol (floods of 17k–34k) while a
hovering drone gives `fg=0`. This is **not** the stale yaw (it persists with yaw
correct to 0.2°) and **not** the flood guard: raising `fg_max_points` 5000 → 45000
gave 1/18, no better than 2/18, and cost 2 Hz of frame rate. The leading
explanation is inherent to fixed-frame differencing — translating ~1 m across the
5-frame background window reveals ground never previously observed, and those
points have no near neighbour in the buffer regardless of transform accuracy.
Flooded frames often still hold a clean ball cluster next to a metre-scale blob
(`fg=223 clusters=2 (182, 0.84), (41, 0.09)`), so the ball is being discarded with
the noise. Alternative not yet excluded: attitude-dependent residual error.

**Settle this before tuning triggers.** The parameter sweep was deliberately NOT
run: at 11% detection its numbers would be detection noise, not
`dodge_speed_mps × trigger_horizon_s` signal.

### Still temporary

`params/detector.yaml` carries `debug_funnel: true` **and**
`debug_funnel_throttle_s: 0.0` (per-frame logging). Both must be reverted before
Week 4 closes — `run_regression.sh` uses the installed copy. `docs/WEEK4_PLAN.md`
stays until then.

### Correction + two new findings (same day, after the fixes above)

**The "detector is blind while translating" blocker was wrong.** Re-measured with
all three fixes in place, thresholds at their Week-3 tuned values, and the drone
patrolling normally: **7 floods / 275 frames (2.5%)**, and across a whole battery
**22 floods / 1499 frames (1.5%)** with 30 centroids published. The earlier 81%
figure was measured during battery activity and is not a property of patrol.

What is true is a sharp **rate dependence**, measured with `cmd_vel` (which does
work — 15 m in 12 s — contrary to the Week 3 open item):

| motion | max fg |
|---|---|
| hover | 0 |
| translate 1.5 m/s | ~0 |
| translate 4 m/s | 38631 |
| yaw 1.5 rad/s | 15504 |

Patrol was measured at **0.18–0.48 m/s**, well under the onset, which is why it looked
clean. **Superseded 2026-07-26 — that figure was an artifact of two competing stacks
running at ~0.33 RTF.** On a single clean stack near real-time, patrol translates at
**2.5–3.2 m/s** (measured from `/huitzilin/odom` twist across nine throws), i.e. right
at the flood onset rather than far below it. So "patrol never reaches onset" does not
hold; the clean 1.5–2.5% flood rate seen during patrol needs re-explaining, and the
rate-dependence numbers below should be redone against a single-stack baseline.
Note `patrol.yaml`'s `cruise_speed_ms: 1.5` is silently ignored in `mode: "position"`
(ArduPilot's WPNAV owns the speed there). The onset near 2–3 m/s is consistent with
odom stamp latency: `mav_bridge_node` stamps odom with `get_clock().now()`
(publication time, not measurement time), and at 4 m/s a 25 ms error equals 0.1 m —
exactly `diff_threshold_m`. Raising `diff_threshold_m` 0.10 → 0.25 cut max fg
38631 → 26137 but left 32/106 frames flooding, so timing is a contributor, not the
whole story. Reverted (it is a Week-3 tuned value). Fix the stamp before retuning.

**Finding 1 — the dodge fires, and the trigger works.** `B03 r1 dodged=True
lat=64 ms` (inside the 150 ms budget), `B03 r2 dodged=True lat=404 ms`. Both scored
✗ only on clearance: `min_dist` 0.25 m and 0.134 m, inside `hit_radius`. **The
parameter sweep is now unblocked and meaningful** — clearance is exactly what
`dodge_speed_mps × trigger_horizon_s` controls.

**Finding 2 (safety, blocking) — the dodge direction has no ground clearance
constraint.** Gravity-compensated throws arrive descending, so the perpendicular
escape vector points *down*: measured `dir_body=(+0.01,+0.68,-0.74)` and
`(+0.03,+0.56,-0.83)` in body FLU. From 2 m the drone dodges into the runway —
MAVProxy logged `Crash` then `Disarm`, and the battery's last 10 runs aborted on the
`MIN_SPAWN_Z` guard (`spawn z=0.03 < 0.3`) because the vehicle was on the ground.
The evasion node must clamp the downward component against altitude (prefer
horizontal/upward escape below some floor) before any hardware flight. This belongs
in `docs/SAFETY_CASE.md` too.

Battery results so far: patrol 0/18 (pre-yaw-fix) → 2/18 → v4 aborted at 10 runs by
the crash, but with dodges firing at 64 ms. CSVs on the Dell:
`/tmp/week4_battery{,_patrol,_hover,_fgmax,_v4}.csv`.

### Ground-clearance clamp, non-colliding ball, and the first complete battery

**Finding 2 fixed** (`adfe69f`). `kalman.clamp_dodge_to_clearance` caps the dodge's
downward component at the headroom above a new `dodge_floor_m` (1.0 m AGL) and
re-spends the freed budget along the same horizontal escape bearing, so escape speed
and pass side are preserved. Escape is never flipped *upward* — the projectile is
descending through that space. Measured effect: `dir_body` z went from −0.74/−0.83 to
**+0.12 and +0.24**. No ground strike since.

**Finding 3 — one marginal dodge used to destroy the whole battery.** With the
clearance fix in, the next battery still stopped at B03: `dodged=True min=0.267 m`
(a genuine miss-distance failure), then the ball *contacted* the airframe, ArduPilot
crash-checked at `AngErr=170>30` and disarmed, and all ~50 remaining runs aborted on
`MIN_SPAWN_Z` against the wreck. A flipped vehicle cannot be recovered in-harness:
ArduPilot refuses to arm inverted, so this always costs a full stack restart.
Fixed by giving the projectile `<collide_bitmask>0x00</collide_bitmask>` — it now
flies its parabola through the airframe and the runway alike. This is a measurement
decision, not a fidelity shortcut: `min_dist_m` is computed geometrically from
`/gz/dynamic_poses` against `hit_radius`, so the verdict is unchanged and better
resolved than "did the physics engine flip it". Restore contact only for a
deliberate impact-damage study.

**First complete battery (v6, 20/20 runs, no crash):** 3/18 dodges (17%),
**0/2 false dodges**, latency mean 201 ms / max 364 ms against the 150 ms budget.

**Finding 4 — the battery is currently measuring the throw harness, not the dodge.**
B01–B05 all specify `miss_distance_m: 0.0` (direct hit), yet runs where no dodge
fired measured min_dist of **2.6, 2.35, 2.0, 1.36, 2.75, 1.07, 1.22, 1.14 m**. Those
throws were never on target, so the trigger *correctly* ignored them (miss >
`threat_radius` 0.75) — they are scored ✗ only because `expect_dodge: true`. So most
of the 15 "failures" are aiming error, not trigger error. Aim is computed from the
latest odom in `_one_run`, but the ball needs ~0.75 s of flight plus the gz spawn
call, and `auto_resume_patrol` puts the drone back in motion between runs; that
accounts for a few tenths of a metre, not 2.7 m. Also seen: `B01 r0` reported
**latency −51 ms**, i.e. a centroid stamped in the future relative to the trigger,
which points at the same odom/centroid stamping question as the flood rate
dependence. **Fix aiming before reading anything into the sweep** — with 15 of 18
scenarios not actually threatening, a sweep would grid noise.

Latency is the other real gap: mean 201 ms is over the 150 ms budget, and the spread
(61 → 364 ms) is wide enough that it is a scheduling/stamping problem rather than
compute cost.

Dell helper: `/tmp/hz_restart.sh` does the full teardown+restart (world → SITL →
MAVProxy → ROS stack) that any crash requires. CSVs: `/tmp/week4_battery*.csv`,
logs `/tmp/battery_v{5,6}.log`.

### Throw aiming: root-caused, fixed, and what it uncovered (2026-07-26, later)

**Finding 4 resolved — root cause was a missing target lead, not the wrench, not
`aim_at_drone`, not stale config.** Ruled out in this order:

- **Config was fine.** `week4_battery.yaml` sets `aim_at_drone: true` and
  `compensate_gravity: true` in `defaults:`, and `_one_run` merges them
  (`{**defaults, **r}`), so both did reach `compute_spawn`.
- **The gravity wrench was fine.** A probe (`/tmp/hz_throw_probe.py`) logged the ball
  against `/gz/dynamic_poses` and compared measured z to both hypotheses: it tracked the
  predicted parabola to within **0.01 m** the whole flight. The arithmetic that made
  "dropped wrench" look attractive (flat-throw drop of 2.76 m at 8 m/s ≈ the observed
  2.75 m miss) was a coincidence.
- **The aim math was fine for a stationary target.** Same throw, drone hovering:
  measured closest approach **0.114 m** against a `miss_distance_m: 0.0` spec.
- **The drone was moving much faster than recorded.** `/huitzilin/odom` twist during
  patrol reads **2.5–3.2 m/s**, not the 0.18–0.48 m/s in the Week-3 notes (that figure
  came from two competing stacks at ~0.33 RTF). With patrol running, the identical
  throw missed by **2.191 m** ≈ `|v| * t_flight`. `yaw_rate` was 0.00 throughout, so
  the 6 m lever arm was never involved — it is pure translation lead.

Fix: `compute_spawn` gained `target_vel_enu` + `spawn_latency_s` and advances the aim
point by `v * (latency + offset_forward/speed)`. No iteration is needed because
`offset_forward` is measured *from the aim point*, so flight time is unchanged.
`spawn_latency_s` was **calibrated to 0.0** by sweeping it against measured closest
approach (0.0 → 0.32/0.35 m, 0.25 → 1.44/0.50/2.70, 0.5 → 1.27/1.34/1.57): the warm
wrench bridge launches effectively immediately, so the `gz` create call's documented
~0.5 s of sim time leaves no dead time to compensate. Note `twist.linear` is world
**ENU** here (`mav_bridge_node` publishes `ned_to_enu(vn,ve,vd)`), not body FLU as
REP-103 would imply for `child_frame_id: base_link`; the lead depends on that.

**Battery v8 (lead + steady-flight gate + honest scoring):**

| metric | v6/v7 | v8 |
|---|---|---|
| dodge success, all runs | 3/18, 2/18 | 3/18 (17%) |
| **dodge success, on-target runs only** | not measured | **3/7 (43%)** |
| false dodges | 0/2 (invalid, see below) | 0/1 (valid) |
| off-target throws | ~15/20 | 12/20 |
| aim error, no-dodge runs | ~2.6 m systematic | mean 1.36 m, max 3.93 m |
| latency | mean 378, max 455 ms | mean 210, max 320 ms |

**The old "0/2 false dodges" was false credit.** B07 specifies a 1.5 m wide miss but
v7 delivered it at 0.298 m and 0.717 m, so the trigger was being praised for ignoring
a near-hit. Rows now carry `spec_miss_m`, `aim_err_m`, `off_target`, `steady`, and the
report prints on-target-only rates plus the aim-error spread, so aiming error can
never again be read as trigger error.

**Finding 5 (now the top blocker) — the trigger does not fire on genuine threats.**
With throws finally on target, the failures moved: v8 has runs at **0.54, 0.58, 0.67,
0.74, 0.88, 0.97 m** with `no dodge fired`, and v7 had 0.136/0.258/0.328 m ignored
outright. This was invisible while the throws were 2 m wide. Suspects, in order:
detector not yielding `min_track_updates: 3` centroids inside the window; the KF's
predicted miss disagreeing with the geometric one; and the stamping question below.
Latency also still misses budget (mean 210 ms vs 150 ms).

**Finding 6 — the steady-flight gate helps less than expected, and the reason is
structural.** `steady_vel_gate_mps` only guarantees velocity is steady *at throw time*;
B01's 4 m/s throw flies for 1.5 s, and patrol waypoints are ~5 m apart at ~3 m/s, so
that flight nearly always spans a turn (B01 still missed by 3.1–3.9 m). Constant-velocity
extrapolation cannot fix that. **The clean instrument is a hover battery** — patrol off
and `auto_resume_patrol: false` on `/evasion` — where aim error is 0.11 m and the
dodge chain is still fully exercised. Keep patrol batteries for realism, but do not
read trigger rates off them at low projectile speed.

Dell helpers now live in `~/hz_tools/` (`hz_restart.sh`, `hz_env.sh`, `hz_env_sitl.sh`)
because `/tmp` is wiped by reboot; `hz_restart.sh` locates its own env via `$SCRIPT_DIR`.
Logs `/tmp/battery_v{7,8}.log`, probe `/tmp/hz_throw_probe.py`.

### The trigger blocker, root-caused: the detector goes blind while patrolling (2026-07-26, later still)

Finding 5 said the trigger "does not fire on genuine threats" and listed three suspects.
The first one is right, but not for the reason listed, and the measured chain is now
unambiguous. **The trigger policy and the Kalman filter are both fine. The detector
publishes no centroid at all while the drone translates, so the trigger never receives
a measurement.**

Instrument: `/tmp/hz_trigger_probe.py` — throws one ball via the production path
(`compute_spawn` + `WrenchThrower`) and logs every boundary of
`/oak/points -> detector -> /threat/centroid -> KF -> /threat/intercept ->
/threat/evade_event` against `/gz/dynamic_poses` ground truth. Non-dodge decisions are
published nowhere, so it reconstructs the predicted miss from `/threat/intercept`:
in hover the drone does not move, so `|intercept|` in base_link *is* the miss the
policy compared to `threat_radius_m`.

**Hover: the chain fires every time, at every geometry that failed in v8.**

| throw | ground-truth min_dist | centroids | predicted miss | dodge | latency |
|---|---|---|---|---|---|
| 8 m/s direct | 0.127 m | 8 | 0.045 m | yes | 375 ms |
| 4 m/s direct | 0.165 m | 3 | — | yes | 332 ms |
| 14 m/s direct | 0.159 m | 5 | 0.043 m | yes | 273 ms |
| 8 m/s, 0.5 m miss | 0.441 m | 9 | 0.439 m | yes | 271 ms |
| 8 m/s, 1.5 m miss | 1.500 m | 5 | **1.486 m** | no (correct) | — |
| 8 m/s, 2.5 m miss | 2.580 m | 3 | **2.473 m** | no (correct) | — |
| 8 m/s, 3.5 m miss | 3.501 m | **0** | — | no | — |

The KF predicts the miss to 0.014–0.107 m and the policy declines correctly outside
0.75 m. Nothing here needs tuning. The 3.5 m row brackets a **~3 m lateral FOV ceiling**
at a 6 m throw distance — beyond that the ball is simply never imaged.

**Patrol: same throws, zero centroids, even when the ball is on target.**

| throw | drone speed | min_dist | centroids | dodge | flood-skipped frames | max fg |
|---|---|---|---|---|---|---|
| 1 | 0.63 m/s | 4.53 m | 1 | no | 23 | 47746 |
| 2 | 1.41 m/s | 0.94 m | 0 | no | 17 | 47817 |
| 3 | 1.72 m/s | **0.74 m** | 1 | no | 21 | 48232 |
| 4 | 3.27 m/s | **0.45 m** | 1 | no | 20 | 43810 |

Throws 3 and 4 arrived *inside* `threat_radius_m` and still produced one centroid,
below the `min_track_updates: 3` gate. This is the v8 failure reproduced on demand, and
it is not aim contamination: a separate on-target patrol throw measured **0.484 m** with
**0 centroids** and a funnel showing the ball present but discarded —
`fg=303 clusters=2 ball_sized=1 (size,extent)=[(291, 1.42), (12, 0.18)]`. The 12-point
0.18 m cluster *is* the ball; the 1.42 m one is scene.

**Mechanism.** `fg` ramps and resets across a throw —
`48 -> 3475 -> 9727 -> 15212 -> 20691 -> 29234 -> 45023`, then `168 -> 3435 -> ...` —
i.e. the 5-frame rolling background buffer (oldest frame 4 cloud frames = **0.267 s** old)
cannot cover the scene the camera newly sees as patrol translates and yaws. Egomotion
compensation transforms correctly; it cannot invent background for regions never viewed.
The foreground therefore grows to tens of thousands of points, and at
`cluster_tolerance_m: 0.20` the ball is connected to that flood, absorbed into a giant
cluster, and dropped by `cluster_max_points: 500`.

**Two hypotheses tested and disproved, recorded so they are not re-run:**
- *Stale odom TF.* Odom is 30 Hz and one period at 3 m/s is 0.10 m = `diff_threshold_m`
  exactly, which looked damning. But the flood is 47746 points at **0.63 m/s**, where one
  period is 0.02 m. Magnitude kills it. (The `_lookup_matrix` docstring still claims
  "cm-level at patrol speed" — that was written against the disproved 0.18–0.48 m/s
  figure and is now wrong at 2.5–3.2 m/s, so it is worth fixing as a comment even though
  it is not the bug.)
- *`fg_max_points: 5000` converting a noisy frame into a blind frame.* Raised it to
  200000 on the Dell and re-ran: `flood_skips=0`, `published=0`, **still 0 centroids**,
  funnel `fg=35408 clusters=0`. Even `fg=2946` — under the old guard — gives
  `clusters=0`. The guard was masking the loss, not causing it. Setting restored.

**So the fix is the background model, not a threshold.** Options, in the order I would
try them: (1) replace the 5-frame rolling buffer with a persistent voxel-hashed
background map in odom, so already-seen regions stay known across a turn; (2) stop
discarding oversized clusters outright — a small compact object touching a large surface
must stay separable (extent-based splitting rather than `cluster_max_points`);
(3) failing both, a different detection principle that is egomotion-tolerant by
construction (per-pixel radial-velocity gating). None of these is a tuning change, and
(1)+(2) together is the recommendation.

Also measured, and unchanged by any of the above: **latency is 271–375 ms against a
150 ms budget**, and the dodge itself blinds the detector — `fg=52442` the moment the
maneuver starts. Hover throws remain the only trustworthy instrument for trigger work
(`auto_resume_patrol: false` on `/evasion`).

### The background-model fix, built and measured (2026-07-26, `af50089`)

Both recommended changes are in, param-gated, and measured live on the Dell.

**(1) `background_map.py` — `VoxelBackgroundMap` replaces the 5-frame deque.** Every
voxel the camera has observed in the fixed `odom` frame stays known, TTL- and
size-bounded (`bg_map_ttl_s: 20.0`, `bg_map_max_voxels: 400000`). Lookup is a
`searchsorted` over a sorted int64 key array, not a rebuilt cKDTree — a persistent
background is 10^5–10^6 points and the tree build alone would eat the 66 ms frame
budget. Background means "own voxel or any of the 26 neighbours occupied", so the
effective tolerance is `leaf .. leaf*sqrt(3)`; keep `bg_map_leaf_m` equal to
`diff_threshold_m`. Camera mode (pre-b0eedd5 bags) and `use_persistent_bg: false` both
fall back to the deque, so the Week-3 recall figures stay reproducible.

**(2) `cluster_and_split()` — oversized clusters are re-clustered, not discarded.**
`cluster_max_points` now applies only *after* splitting; applying it first is what threw
the ball away. **Recursive shrinking was implemented first and rejected on evidence:**
at a radius near a surface's own point spacing it shatters the surface into ball-sized
fragments that all pass the extent gate — 29 of them in
`test_recursive_shrinking_would_shatter_the_wall`. One pass at an explicit intermediate
`cluster_split_tol_m: 0.10` separates a detached object without shattering. That test is
kept precisely so nobody re-adds the recursion.

**Foreground under patrol, same stack, before → after:**

| | max fg | mean fg | flood-skipped frames |
|---|---|---|---|
| 5-frame deque | 43810–48232 | — | 23+ per throw |
| persistent map | **4961** | **581** | **0** in 400 frames |

The map converged to 1373 voxels hovering and 77517 patrolling — far under the cap.

**Centroids on a patrol throw went 0–1 → 6–11.** The detector is no longer blind while
translating. Hover is unregressed: 10 centroids, predicted miss 0.035 m vs truth
0.128 m, dodge fired.

**Battery v9 vs v8 (same 17-throw battery):**

| | v8 | v9 |
|---|---|---|
| on-target dodge success | 3/7 (43%) | **4/6 (67%)** |
| overall | 3/18 (17%) | 4/16 (25%) |
| false dodges | 0/1 | **0/1** |
| latency | mean 210 / max 320 ms | mean 179 / max 297 ms |

**The honest caveats, all measured:**

- **The map helps *revisited* scene by construction, not first-pass novel scene.** A
  region never looked at is still novel however long the map has run. Patrol is a closed
  loop with waypoints inside the 5 m ROI, so it re-sees most of its route — that is why
  the flood collapsed — but a genuinely new environment will still flood on entry.
- **New false-positive stream.** The pipeline now clusters foreground it previously
  skipped: **71 spurious `/threat/centroid` in 60 s of patrol with no ball thrown**
  (~30% of frames), where the old config published ~nothing. It caused **0 evade events**
  in that window and **0 false dodges** across the battery — the chi2 gate,
  `min_track_updates`, and `threat_radius_m` filter all of it. Layered defence working,
  but it is a real regression in detector precision and the margin is not measured.
  Likely mechanism (unverified): novel regions are now small, so their fragments pass the
  0.35 m extent gate where metre-scale blobs did not. The cheap discriminator to try is
  isolation — an airborne ball has no *non-foreground* points adjacent to it, a
  scene-entry fragment sits on the seen/unseen boundary and does.
- **Latency is still over budget** (mean 179, max 297 vs 150 ms), though better.
- **Aiming under patrol is now the dominant limit, not the trigger**: 11/17 throws landed
  off-target, aim error mean 1.50 m / max 3.79 m, because a throw spans a patrol turn.
  The battery cannot measure dodge performance well while two thirds of its throws miss
  by design. Fix the harness (hover instrument, or lead through the turn) before reading
  much into the overall rate. Genuine remaining trigger failures are just B03 r1/r2
  (0.591, 0.589 m); B06 r2 at 0.890 m is correctly ignored, and B05 r0 dodged at 80 ms
  latency but still took the hit at 0.173 m — late *detection*, not slow reaction.
- **The odom stamp fix is deliberately NOT done.** `mav_bridge_node` still stamps odom
  with `get_clock().now()`. It needs an offset estimate between the autopilot's
  `time_boot_ms` and Gazebo `/clock` (min-filter on `now - boot_ms`, plus a clamp so a
  stamp never lands in the future and breaks TF), and that is a clock-sync change to a
  working flight path whose payoff just shrank: it was a contributor to the *flood*, and
  the flood is now 10x smaller. It deserves its own measurement window, not a ride-along.
- 3/17 rows were harness spawn flakes (`create service did not confirm the spawn`),
  pre-existing.

### Fixing the throw harness: aiming under patrol (2026-07-27, `0f2a36c`…`8de8d3d`)

Week 4 inherited "aiming under patrol is the dominant limit, not the trigger". Fixed, in
three measured steps — each one wrong in an instructive way.

**Root cause.** `compute_spawn` leads the drone at constant velocity over the ball's
flight (`t = offset_forward_m / speed_mps`). `patrol_node` is a *corner* follower flown
in position mode, so ArduPilot decelerates into every waypoint and accelerates out along
a new heading. That is a discontinuity in the commanded path, not a smooth curve, so no
constant-velocity *or* constant-acceleration lead can predict across it. Fitting a better
motion model is the wrong fix; the right one is to only throw when the drone is on a
straight leg, at cruise, with enough leg left to cover the flight.

New `throw_window.py` is pure geometry: `leg = (dist_to_wp - accept_radius) / cruise`.
`patrol_node` now publishes `/huitzilin/patrol_state` (JSON String, same convention as
`/huitzilin/state`) with the current leg; control logic is untouched. A run with no
aimable window is **SKIPPED, not thrown**, and skipped rows are excluded from the dodge
and false-dodge denominators — an aiming limit must never be read as a trigger failure.

**v10 — the gate made things worse.** Gating on leg time alone, computed with the
*instantaneous* speed, gave 16/20 off-target (aim mean 1.60 m, max 4.96 m) and never
refused a single throw. The quotient is inverted near a corner: the speed collapses, so
the window inflates at exactly the moment the aim is least predictable.

| window_s | aim_err_m |
|---|---|
| 0.76 | **0.18** |
| 2.48 | **0.21** |
| 5.86 | 3.22 |
| 8.20 | 0.94 |

Fixed by evaluating the leg at a **cruise estimate** (rolling max odom speed) and
additionally requiring the drone to *be* at cruise. `cruise_mps` is a required kwarg so
the v10 usage is not expressible. Constants came from measurement, not guesswork: 45 s of
patrol, 1350 odom samples — median 2.09 m/s, p90 3.35, max 3.49, and only **30% of the
time above 80% of max**. Note `patrol.yaml`'s `cruise_speed_ms: 1.5` is not the flown
speed; position mode means `WPNAV_SPEED` governs.

**v11 — the gate worked, and measured that the loop was too small.** 8/20 skipped, B01
skipped all 3 times. Aim improved (mean 1.39 m, max 3.04 m) but 11/12 thrown runs were
still off-target, because a 5 m leg offers at most `(5.0-0.6)/3.4 = 1.29 s` and the 1.05 s
scenarios could only fire at the leg *start*, still accelerating.

**v12 — a 12 m loop collapsed the variance.** New `week4_patrol.yaml`, threaded via a
`patrol_params` launch argument through week4 → week3 → week2_sitl, so `patrol.yaml`
keeps the 5 m Week 2 demo square (`scripts/plot_telemetry.py` hardcodes it). Per-scenario
spread fell to ±0.04 m, which exposed the remainder as a *systematic* bias proportional
to flight time — ~0.9 m/s × `t_flight`:

| scenario | t_flight | aim error (3 reps) |
|---|---|---|
| B03 (14 m/s) | 0.43 s | 0.39, 0.36, 0.44 |
| B06 (8 m/s) | 0.75 s | 0.66, 0.66, 0.63 |
| B07 (8 m/s) | 0.75 s | 0.54, 0.54 |

That residual *is* the 20% of cruise the floor admitted. Bounding it under the 0.5 m
off-target tolerance at the worst flight time against the 12 m loop's measured 5.26 m/s
cruise needs `(1-frac) * 5.26 * 1.5 < 0.5`, i.e. `frac > 0.94` → **0.95**.

**v13 — the result.**

| | v9 | v13 |
|---|---|---|
| on-target dodge success | 3/7 (43%) | **4/5 (80%)** |
| false dodges | 0/1 | 0/2 |
| off-target throws | 11/17 | 10/17 |
| latency mean / max | 179 / 297 ms | 203 / 287 ms |

Caveats, all still open:
- **The 12 m loop costs a ~66 s foreground flood on entry** (fg to 41159, 73 skipped
  frames) while the persistent map learns the enlarged area, then fg returns to 0–388.
  This is the documented "map helps revisited scene, not first-pass" limitation, not a
  regression — but the battery must settle ≥75 s after patrol starts or its first run
  lands in the flood. v12's B01 r0 (5.39 m) did.
- **Oblique runs are still systematically off** (B04 1.04–5.00 m, B05 1.29–1.67 m) and do
  not scale with the cruise floor — that is a separate `aim_at_drone` geometry issue.
- **B01 remains unmeasurable** even on 12 m: needs 1.80 s, best leg seen 2.19 s, but the
  95% cruise floor rarely coincides with that much leg left. Shorten `offset_forward_m`
  for the slow scenario rather than lowering the margin.
- **Latency is still over budget** (203 mean vs 150 ms) and slightly worse than v9.
- v13 hit one `deque mutated during iteration` harness exception — `_cruise_est` racing
  `_odom_cb`. Fixed with a dedicated lock.

### Latency root-caused: the detector runs ~2x slower than its input (2026-07-27, `33680e6`)

The 203–233 ms dodge latency is measured from the **cloud's** `header.stamp` (the detector
stamps centroids with `msg.header.stamp`), so it spans the whole sensor-to-actuation chain.
There was no instrumentation to say which stage owned it. `_cloud_cb` already captured
`t0 = time.monotonic()` and never used it.

**Measured split** (`debug_funnel`, 15 samples under patrol):

| | range | note |
|---|---|---|
| transport (stamp → callback entry) | 53–348 ms sim, typically >300 | already over the whole 150 ms budget |
| compute (entry → centroid published) | 63–354 ms wall, mean ~160 | on raw=107k–215k points |

**RTF is 0.864, not the ~0.33 older notes assume.** So 15 Hz clouds arrive every **77 ms of
wall time**, and that is the real per-frame budget. Compute averages ~160 ms — the detector
is **~2x slower than its input rate**. With `cloud_queue_depth: 5` the queue backs up, and
that backlog *is* the transport figure. **Transport is the symptom; compute is the cause.**
Optimising transport (QoS, queue depth) would be treating the wrong stage.

**A measurement trap, recorded so it is not repeated:** compute measured in *sim* time reads
exactly 0 ms. The callback blocks the single-threaded executor, so no `/clock` is processed
while it runs and the sim clock cannot advance mid-callback. Sim-time compute is structurally
unmeasurable from inside the callback; the first version of this instrumentation reported
`compute=0 ms(sim)` and a derived `rtf=0.00`, both meaningless. Wall time only.

Not yet attempted, and deliberately so — making the detector 2–3x faster is real performance
work that can move the Week-3 recall numbers, so it needs per-stage profiling first rather
than a guess at which stage is hot. Two candidates worth measuring before touching anything:
`read_points_numpy` deserialising 640×480, and the fact that the `gz_flu` convention remap
and the finite-filter both allocate full ~200k×3 arrays *before* `roi_max_range_m: 5.0`
discards most of them. The range gate is invariant under that signed-axis permutation, so it
could run first — but profile before assuming that is where the time goes.

Also note raw point counts have grown (115k → 215k) since the 12 m loop; more of the scene
falls inside the ROI, which directly inflates compute.
