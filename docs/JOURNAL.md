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

> **Closed 2026-07-27.** Both keys reverted (`debug_funnel: false`, throttle
> `1.0`) in `f4bfc95`/`5073a01`, and `docs/WEEK4_PLAN.md` deleted — all nine of
> its tasks had shipped, only its checkboxes were never ticked.

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

---

## 2026-07-27 — Latency fixed at the hot stages, and the aim error re-diagnosed

Two threads closed today, and the second overturned an item that was on the Week-4 list.

### Per-stage profiling: two functions were the whole overrun

The previous entry ends by refusing to optimise the detector without knowing which stage was
hot. `stage_profiler.py` (`StageProfiler`, `profile_stages` param, default off) answers that:
wall-time totals per named stage, reported every 30 clouds, ranked by **total** rather than
per-call. Every arriving cloud counts as a frame, because a single-threaded executor is
delayed by a frame that bails at the range gate as much as by one that publishes.

First measurement under patrol, before any change:

| stage | per call | calls / 30 frames |
|---|---|---|
| `voxel` | **66–74 ms** | 5–25 |
| `cluster` | 27–70 ms | 2–14 |
| `bg_query` | 5–25 ms | 5–25 |
| `finite` | 6–7 ms | 30 |
| `convention` | 4 ms | 30 |
| `deserialize` | 2.5–3 ms | 30 |

Per-frame mean swung 17–88 ms against the 77 ms wall budget. **Both hot stages were single
numpy/scipy anti-patterns, not algorithmic cost:**

- `voxel_downsample` used `np.unique(keys, axis=0)` — which views each row as a void dtype and
  lexicographically sorts rows — plus `np.add.at`, the unbuffered scatter-add. Replaced with
  one packed int64 key (21 bits/axis, ±21 km at a 0.02 m leaf) and `np.bincount(weights=…)`.
  x occupies the high bits, so the lexicographic row order the old code produced is preserved
  exactly. Microbench 17.9→3.5 / 68.6→11.3 / 164.8→26.6 ms at 20k/60k/120k points (5.4–6.7×);
  **live 66–74 → 5–17 ms.**
- `cluster_all` issued one Python-level `tree.query_ball_point` **per point** inside a BFS.
  Single-linkage radius clustering *is* connected components of the radius graph, so it is now
  one `query_pairs` plus one `connected_components`. Microbench 3.2–5.3×; **live 12–71 → 2–20 ms.**

Per-frame mean is now **14–55 ms, every window under budget**. 103 tests pass, including the
pre-rewrite implementations kept as oracles and compared element-wise / partition-identical —
these are speed rewrites of code the detection numbers depend on, so equivalence is tested,
not argued.

Two corrections to the previous entry:

- **`debug_funnel` was not a meaningful part of the latency.** I flipped it off partly on the
  theory that its six full-cloud reductions mattered; measured as its own stage at throttle
  0.0 it costs **1.4–2.3 ms/frame, 4–5%**. The theory was wrong.
- **The ~160 ms figure was the cost of a *publishing* frame**, not the mean — that is the only
  path the old dbg timing log reported. It was `voxel` + `cluster`, and both are now fixed.

The pre-gate reorder floated last time (range-gate before the `gz_flu` remap and finite
filter) is still available and still valid — it would move `finite` + `convention` from ~200k
rows to survivors, ~10 ms/frame — but it was not needed to get under budget and is not done.

### The hover control: the spawn geometry is exact, so the aim error is all lead

The Week-4 list carried "oblique aim is systematically off — separate `aim_at_drone` geometry
issue in `compute_spawn`". **That item does not exist.** B04 (+30°) sat at 0.98–1.05 m and B05
(−30°) at 1.37–1.70 m against a 0.0 m spec, and B06 at 1.07–1.11 m against a 0.50 m spec,
consistently across three batteries — systematic enough to look like a geometry bug.

Patrol cannot distinguish a bad lead from bad geometry. Against a stationary target the lead is
exact by construction, so a hover control separates them. That needs a battery mode, not a
manual service call: **`evasion_node` resumes patrol when a dodge completes**, so stopping
patrol once holds only until the first successful dodge — measured, run 1 hovered and all 17
after it were back at cruise, and my first "hover" numbers were really patrol numbers.
`hover_mode` re-stops patrol before every throw *and* waits for the drone to be measurably
stationary, because stopping patrol only stops new setpoints while ArduPilot keeps flying to
its last position target — seconds of cruise away on a 12 m loop.

Hover control result — **off-target 0/19, aim error mean 0.10 m, max 0.20 m**:

| scenario | spec | hover | under patrol |
|---|---|---|---|
| B04 +30° | 0.00 m | 0.147 / 0.160 / 0.171 | 0.98–1.05 |
| B05 −30° | 0.00 m | 0.154 / 0.094 | 1.37–1.70 |
| B06 near miss | 0.50 m | 0.511 / 0.496 / 0.506 | 1.07–1.11 |
| B07 wide | 1.50 m | 1.501 / 1.496 | 1.78–1.99 |
| B01 4 m/s | 0.00 m | 0.088 / 0.374 / 0.202 | unmeasurable (skipped) |

The geometry is exact to a few centimetres at every angle and every specified miss. This also
explains the +30°/−30° asymmetry that made it look like geometry: a lead error points along
the drone's velocity, so its perpendicular (miss-distance) component depends on the angle
between approach ray and velocity, while the geometry itself is symmetric.

### Spawn dead time: measured, compensated, and NOT the cause

The obvious suspect was spawn dead time. `compute_spawn` leads over
`spawn_latency_s + t_flight`, `spawn_latency_s` was parameterised at **0.0**, and at 5.24 m/s
every 0.1 s of undeclared dead time is 0.5 m of aim error — the right order for 0.98–1.70 m.
So the battery now measures it directly (`spawn_dead_s` in the CSV): sim time of the odom
sample the plan was built from vs. sim time the ball first appears on `/gz/dynamic_poses`.

**Measured 0.216 s mean, 0.185–0.247 s over 18 throws** — tight, and entirely undeclared,
i.e. worth 1.14 m at cruise. That looked conclusive.

It was not. Re-running with `spawn_latency_s:=0.216` (report confirms undeclared −0.006 s, so
the dead time is now fully compensated) moved the aim only ~0.1–0.2 m:

| | v18, latency 0.0 | v19, latency 0.216 |
|---|---|---|
| aim error mean | 1.18 m | **0.96 m** |
| off-target | 13/18 | 14/17 |
| B04 +30° | 0.92 / 0.96 / 0.97 | 0.83 / 0.86 / 0.82 |
| B05 −30° | 1.63 / 1.61 / 1.66 | 1.47 / 1.47 / 1.45 |
| B06 (spec 0.50) | 1.07 / 1.08 / 1.05 | 1.02 / 1.07 / 1.08 |

So the dead time is real, is now declared, is worth ~0.2 m — and something else accounts for
the remaining ~0.8–1.5 m. (This also vindicates the 2026-07-26 sweep that calibrated it to 0.0:
that sweep was n=2–3 per point and predates the throw-window gate, so it was measuring corner
noise, but its conclusion that 0.25 was not a win happens to hold.)

**What the geometry says about where to look next.** `min_dist` is the *perpendicular* distance
from the ball's path to the drone. An aim-point error purely *along* the drone's velocity
produces a perpendicular miss of |e|·sin α, which is **zero for a head-on throw** — yet head-on
B02/B03 still measure 0.33–1.00 m under patrol against 0.11–0.17 m in hover. So the residual is
not a pure along-track timing error; there is a cross-track and/or vertical component, and the
+30°/−30° asymmetry (0.83 vs 1.46) confirms it is not symmetric about the flight path.

The battery cannot decompose this today: it records only the scalar `min_dist`. The concrete
next step is to record the **miss vector** at closest approach and split it into along-track,
cross-track and vertical in the ball's path frame. That turns one ambiguous scalar into three
diagnosable numbers, and it is worth doing before touching the lead again. Prime candidates it
would separate: stale odom position vs. velocity (the deferred `mav_bridge_node` odom-stamp
item), a lateral deviation the window gate is not catching, and gravity-compensation error in
the vertical.

### The finding that matters most: exact aim collapses the dodge rate

With the aim error removed, dodge success fell to **4/17**, and eleven runs read *"dodged but
min_dist 0.11–0.17 ≤ hit_radius"*. The trigger fires; the manoeuvre does not clear the 0.30 m
hit radius against a genuine intercept. Only B06 (a 0.5 m specified miss) and one B01 cleared it.

So the patrol batteries' "clean dodge" successes were partly **credit for aim error** — the
ball was already going to miss. The honest reading of Week 4 is that dodge *authority* (1.5 m/s
for 1.0 s) and end-to-end latency, not detection, are what stand between here and a real dodge.

**Latency did improve, and it is the one budget now being met.** Patrol batteries, evasion-node
end-to-end (budget 150 ms):

| battery | mean | max |
|---|---|---|
| v13 / v14 (before) | ~208 ms | 291 ms |
| v15 (voxel fixed) | 168 ms | 196 ms |
| **v18 (voxel + cluster fixed)** | **121 ms** | **175 ms** |

It is no longer detector compute. With clouds arriving every 77 ms and `min_track_updates: 3`,
the remaining floor is structurally ~2 cloud periods plus compute, so further gains have to come
from the track-confirmation count or the camera rate, not from the numpy.

Do not re-tune detection thresholds against the patrol dodge rate; it is measuring two things
at once. Use `hover_mode` for anything about the manoeuvre and patrol for anything about the lead.

---

## 2026-07-27 (later) — The aim error was a stale velocity sample

The miss decomposition promised at the end of the last entry got built, and it
found the Week-4 aim error in three measurements. **Off-target throws went from
12/18 to 2/18** and on-target dodge success from 2/5 to 8/14. The cause was not
physics, geometry, or a dead time: it was one stale variable.

### What the decomposition added

`min_dist` is the *perpendicular* distance from the ball's path to the drone, so a
mistimed lead, a sideways drift and a gravity error are indistinguishable in it.
`ballistics.miss_components` now resolves the closest-approach miss into the
ball's path frame, and the battery records two triples per run:

- **miss** — ball vs drone, in the ball's heading frame. `+along` = the ball went
  past, `+cross` = it passed to the drone's left, `+vert` = above. Vertical is
  *world* vertical, so gravity error stays in one component instead of smearing
  across two.
- **lead** — the aim point vs where the drone actually was, in the *drone's*
  heading frame. `+along` = over-led. Absent under `hover_mode`, where a
  stationary target has no heading and the lead is exact anyway.

`miss along` sits at ~0 by construction at a true closest approach, which is the
built-in sanity check that the frame maths is right; it measured -0.015 m.

### Three measurements, two of them wrong turns

First reading, v20: **`lead_along` median -1.90 m — the throws were UNDER-led**,
not over-led. That immediately killed the assumption that a mis-set spawn dead
time was the story.

1. **Odom position lag — real, 129 ms, declared.** Ground truth leads
   `/huitzilin/odom` by 0.561 m along-track at 4.36 m/s (n=1595, median 128 ms).
   The offset's *magnitude* equals its along-track component, i.e. pure transport
   lag with no lateral bias — a latency to declare, not a calibration to correct.
   Same root cause as the deferred `mav_bridge_node` odom-stamp item. Now
   `odom_lag_s`, summed with `spawn_latency_s` into the lead.
2. **A velocity measurement that was wrong, and how.** I derived truth speed from
   pose *arrival* time with a `dt > 0.02 s` filter, which kept only the
   late-arriving samples — a biased subset that inflates `dt` and deflates the
   derivative. It reported a 24.7% speed error that does not exist. Redone with
   header stamps over a 0.257 s baseline: **odom velocity is accurate to 0.4%**
   (n=1624), and odom altitude to 7 mm. Never take a derivative off that stream
   with arrival times.
3. **A prediction that failed.** I predicted declaring the 129 ms would move
   `lead_along` by 0.68 m and shrink `miss_vert`. Medians moved 0.18 m and
   `miss_vert` not at all, and I read that as the fix having failed. It had not:
   comparing `lead_along` across batteries is invalid because **aim error is only
   recorded for no-dodge runs**, so firing more dodges leaves the badly-aimed
   throws behind and inflates the surviving subset. A selection effect in my own
   metric. The flight-time excess below depends on neither speed nor subset.

### The root cause

Auditing the two inputs the lead trusts blindly settled it. Ball speed measured
3.90-14.16 m/s against 4/8/14 m/s specs — the flight is correct. But the ball
reached closest approach **0.217 s earlier than the assumed `offset_forward /
speed`**, and 0.217 s at the 5.26 m/s cruise is 1.14 m — the same number
`lead_along` was reporting, measured a second, independent way.

An accurate but *stale* velocity was all that was left, and it was in the code:

```
steady, vel = self._wait_steady_velocity()     # velocity sampled HERE
win_ok, ... = self._wait_throw_window(t_flight) # blocks up to 40 s
odom = self._latest_odom                        # position re-read HERE
```

The old comment claimed "re-read odom AFTER the waits so position matches that
velocity". The window wait sits between the two samples, so it never did. And the
bias has a sign, which is why it was systematic rather than noise: **that gate
waits until the speed reaches 95% of cruise**, so the pre-gate velocity is
reliably slower than the speed at launch. Every throw under-led.

Position and velocity now come from the same message after both waits;
`_wait_steady_velocity`'s return is dropped explicitly, since it is a check that
the velocity has stopped changing, not an aim input.

| | v22 (before) | v23 (after) |
|---|---|---|
| flight excess vs assumed | **-0.217 s** | **+0.021 s** |
| `lead along` | -1.40 m | **+0.13 m** |
| off-target throws | 12/18 | **2/18** |
| aim error, no-dodge | 0.82 m | 0.62 m |
| on-target dodge success | 2/5 | **8/14** |
| `miss vert` | +0.53 m | **+0.20 m** |
| latency mean | 214 ms | 131 ms |

`miss_vert` falling with the lead partly vindicates the coupling argument the
failed prediction could not test: a lofted ball is above its aim altitude
everywhere except at `t_flight`, so an under-lead makes closest approach happen
early, while the ball is still high. A +0.20 m residual remains.

### Dodge authority is a TIME problem, not a force problem

With the aim fixed, the remaining failures are all one shape: six runs read
*"dodged but min_dist 0.11-0.20 <= hit_radius"*. The reason is now measurable
across **every dodge ever recorded — `tca` at the moment of commit is 0.07-0.28 s**,
while latency over the same events ranges 51-271 ms. Latency is therefore *not*
the binding term: even the 51 ms dodge had only 0.19 s of warning. At
`dodge_speed_mps` 1.5 that window buys **0.10-0.42 m** against a 0.30 m hit
radius, which is exactly the range of the failures.

So the manoeuvre is not weak, it is starved. The budget for an 8 m/s ball:

| term | cost | set by |
|---|---|---|
| ball visible | 0.625 s | `roi_max_range_m: 5.0` at 8 m/s |
| confirm track | -0.20 s | `min_track_updates: 3` at 15 Hz |
| pipeline | -0.12 s | detector compute + transport |
| **left to move** | **~0.30 s** | |

Raising `dodge_speed_mps` treats the symptom. The lever is the 0.625 s, and
`roi_max_range_m` was capped at 5.0 in Week 3 **because far background dominated
the frame difference** — the exact root cause the persistent voxel background map
(af50089) fixed. That cap is a candidate for re-testing, but it must be justified
against the **bag-library recall**, not the dodge rate: the standing warning
against tuning detection thresholds on the dodge rate applies with full force
here, and the bags are at `/data/huitzilin_bags` (34 entries) via
`scripts/run_regression.sh`.

### Housekeeping

`docs/WEEK4_PLAN.md` deleted. All nine of its tasks had shipped — verified every
artifact and both entry points exist — and its gating item (the `debug_funnel`
revert) landed earlier the same day; only its checkboxes were never ticked. The
two config headers that pointed at it now point at the runbook.

---

## 2026-07-27 (night) — Two dodge-authority levers, both refuted

With the aim fixed, dodge authority became the only blocker, and the reframing
from the previous entry said it is a **time** problem: every dodge on record
commits with `tca` 0.07-0.47 s, and at 1.5 m/s that buys 0.11-0.70 m against a
0.30 m hit radius. Two candidate levers were predicted to buy time. **Both were
measured, and both bought none.**

### Lever 1: `roi_max_range_m` 5.0 -> 8.0 — refuted

The cap was set in Week 3 because far background dominated the frame difference,
which is the exact root cause the persistent voxel background map later fixed, so
it looked like a stale workaround worth lifting. A/B on a settled 12 m loop:

| | roi 5.0 | roi 8.0 |
|---|---|---|
| `tca` mean | 0.204 s | **0.201 s** |
| on-target dodge | 10/15 (67%) | 9/15 (60%) |
| latency mean | 128 ms | **169 ms** |
| aim error | 0.41 m | 0.21 m |

No time gained, 41 ms of latency lost. Reverted (build copy only; the repo value
never moved).

**Why, measured directly:** the battery now records the ball's *true* separation
at the first `/threat/centroid` that matches it. **First detection happens at mean
3.36 m, range 0.90-4.75 m — entirely inside the 5.0 m gate.** The gate was never
what decides when the ball becomes trackable; the ball's own detectability at
range is, because an 80 mm sphere projects few points at 6-8 m against
`cluster_min_points: 5`. Widening a gate cannot reveal points that were never
there.

### Lever 2: `min_track_updates` 3 -> 2 — refuted

At 15 Hz, confirming three updates spends 0.20 s of the ~0.42 s of total warning
that a 3.36 m first detection gives against an 8 m/s ball — nearly half the
budget, and an *evasion* parameter, so the sweep could grid it live with no
restarts. Prediction: dropping to 2 hands ~0.067 s back.

| combo | `tca` mean | on-target dodge |
|---|---|---|
| 1.5 m/s, mtu 2 | 0.172 s | 2/6 |
| 1.5 m/s, mtu 3 | 0.175 s | 4/6 |
| 2.5 m/s, mtu 2 | 0.177 s | 3/5 |
| 2.5 m/s, mtu 3 | 0.124 s | 2/6 |

`tca` moved **0.003 s**. The dodge rates scatter 33-67% with no coherent pattern —
mtu 3 at 1.5 m/s scored best and mtu 3 at 2.5 m/s worst, which is physically
impossible as an effect — so at n=5-6 per combo they are noise. Defaults kept
(`min_track_updates: 3`, `dodge_speed_mps: 1.5`); nothing measured beats them.

### What the two nulls together imply

`tca` is pinned near 0.12-0.27 s and is invariant to **both** when detection
starts and how many updates are required. So the trigger is not waiting on the
*existence* of a track — it is waiting on the **quality** of one. The policy fires
when the KF's predicted miss falls inside `threat_radius_m` (0.75 m) with
`tca <= trigger_horizon_s` (1.5 s, never binding at these values), so if an early
track's predicted miss is too noisy to be confidently inside 0.75 m, the commit
waits for the ball to close regardless of how early it was first seen.

The spread in first detection supports this: **0.21-5.48 m**, i.e. some throws are
not seen until essentially point-blank. Detection is not just limited in range, it
is *unreliable* at range.

**The measurement that would settle it** (not yet done): log `/threat/intercept`
through a single throw and find when `|intercept|` first crosses `threat_radius_m`,
against when the track started. That separates "the KF converges late" from "the
policy is conservative". Hover is the right instrument — in hover `|intercept|` in
base_link *is* the miss the policy compares, and the lead is exact so the geometry
is not a confound.

Do **not** reach for `dodge_speed_mps` or `dodge_duration_s` first: `tca * speed`
is the product that must clear the hit radius, and the sweep above shows the speed
half is already inside the noise floor. The time half is where the leverage is.

### Two instrument defects fixed along the way

Both were manufacturing wrong numbers, and both were found by inspecting a
battery per-run instead of trusting its means:

- **`first_det_range_m` was counting false positives.** The detector emits a ~1/s
  FP stream during patrol, so the first centroid in a 5 s window is more often
  clutter than the ball — it recorded 7.03 m and 6.39 m against a 5.0 m gate,
  which the ball cannot produce. A centroid must now *match* the ball: the
  centroid is in `base_link`, so its norm is the target's range, comparable to the
  true separation with no attitude needed, and the two must agree within
  `det_match_tol_m` (0.75 m).
- **A spawn flake was being scored as a run.** One throw's impulse wrench was
  dropped (`ball_speed 0.00 m/s`), and everything downstream was garbage:
  `min_dist` became the distance to a stationary ball at the spawn point and the
  "flight time" became **1312 s**. That single row moved the battery's reported aim
  error from 0.41 m to 1.44 m and the flight excess to +72.9 s, which reads exactly
  like an aim regression. A ball measured below `min_launch_speed_frac` (0.25) of
  its scenario speed is now an explicit harness error. Averaging a throw that never
  happened is worse than losing the row.

### Also worth knowing

- **The bag library cannot referee detector-threshold changes at this operating
  point.** Measured today on the train split: recall **100%** (TP=10, FN=0 — the
  background map fixed the old S08 false negative) and false positives on **4 of 4**
  negative bags. Saturated at both ends, so it can neither confirm nor refute.
- **`run_regression.sh` pkills any running detector**, including a live stack's.
  Run bag work before bringing the stack up, or restart T3 afterwards.
- Best latency recorded to date: **93 ms mean / 200 ms max** on a clean settled
  stack, comfortably inside the 150 ms mean budget.

### The trigger-timing probe: contaminated, and what it still showed

The measurement named above was attempted (`/tmp/trigger_timing.py` on the Dell:
log `|/threat/intercept|` — the policy's own decision variable, published on every
confirmed track update *before* the `should_dodge` check — against the ball's true
range, through hover throws). **It failed for the same reason the first-detection
instrument did: the false-positive stream.** It grouped 40 intercept messages into
31 "throws", and the ones it printed carry predicted misses of 1.79-6.46 m at ball
ranges of 12.65-16.54 m. Those cannot be the thrown ball. A redo needs the same
match discipline the battery now uses — key the track to the *active* ball's name
and require the intercept to be plausibly near it — not a bare
`child_frame_id.startswith("ball_")`.

**But it establishes one thing cleanly: the FP stream reaches CONFIRMED tracks.**
Those spurious intercepts are published, which means clutter survives
`min_track_updates: 3` and the chi2 gate, gets a Kalman track, and produces a dodge
plan. What stops it is `threat_radius_m` (0.75 m) — every spurious predicted miss
measured 1.79 m or wider. That independently explains the sweep's otherwise
surprising result that **`min_track_updates: 2` produced 0 false dodges**: the
update count was never what was filtering clutter, so lowering it costs no
precision. It also means the FP stream is a *latency* and instrumentation tax, not
a safety one, which is worth knowing before anyone spends effort on detector
precision.

Hover control alongside it: **0/6**, `tca` mean 0.205 s (0.040-0.410), first
detection 3.67 m, latency 144 ms mean / 315 ms max. Consistent with the earlier
hover figure — with the aim exact, the manoeuvre simply does not clear 0.30 m in
~0.2 s at 1.5 m/s.

---

## 2026-07-27 (late) — Dodge authority root-caused to KF velocity convergence

Three levers tried, all refuted, and the fourth measurement explained why all
three had to fail. **`tca` at dodge commit is gated by how fast the Kalman filter
learns the ball's velocity**, not by anything in the detector or the manoeuvre.

### The measurement

`/threat/intercept` publishes the policy's own decision variable on every
confirmed update. Logged against the ball's true range, on inbound balls inside
the range gate:

| track | predicted miss over updates | ball's true range |
|---|---|---|
| 2 | 3.90 → 3.26 → 2.36 → 2.14 → **1.12** | 3.88 → 3.40 → 2.84 → 2.49 → 2.21 |
| 1 | 1.75 → 1.46 → **1.23** | 2.18 → 1.66 → 1.30 |
| 4 | 5.11 → **4.97** | 5.04 → 4.98 |
| 5 | 1.98 → **1.54** | 2.76 → 1.94 |

**The predicted miss starts equal to the ball's current range.** For a ball on a
true collision course the intercept point should sit essentially on the drone and
the miss should be near zero, so this is diagnostic, not noise.

### Why, from the code

`predict_closest_approach` works on *relative* state, and
`v_rel = v_proj - v_drone`. The tracker initialises velocity to **zeros**
(`kalman.py:107`, with variance `init_vel_std**2`), so while `v_proj` is still
small, `v_rel ≈ -v_drone` — the ball appears to **recede**, the minimum of the
relative distance lands at `t ≈ 0`, and therefore `tca ≈ 0` and
`miss ≈ |p_rel| =` the current range. `should_dodge` requires
`miss < threat_radius_m` (0.75 m), so the trigger simply cannot commit until the
velocity estimate matures. The miss then falls *faster* than the range (3.90 →
1.12 while range only goes 3.88 → 2.21), which is the estimate converging.

This holds in hover too — with `v_drone = 0` a small `v_proj` still puts closest
approach at "now" — which is why the hover control measured the same `tca` (0.205 s)
as patrol and looked so puzzling.

**One causal chain therefore explains all three nulls:** detecting earlier
(`roi_max_range_m`) and confirming sooner (`min_track_updates`) both hand the
filter more *time*, but the filter still spends its first updates climbing out of
a zero-velocity prior, so the commit lands at the same place. And a bigger
`dodge_speed_mps` cannot help a manoeuvre that starts too late.

### Lever 3, also refuted: the manoeuvre is not sluggish

Measured the drone's true displacement from an evade event, capturing poses
online (the first attempt used a `maxlen` deque that wrapped and returned all-NaN):

| | measured | commanded |
|---|---|---|
| displacement by `tca` | **0.491 m** | 0.307 m (1.5 x 0.205) = **160%** |
| at 0.50 s | 1.067 m | 0.75 m |
| at 1.00 s | 1.690 m | 1.5 m |
| cleared 0.30 m within `tca` | **10/14** | |

The drone moves *more* than commanded, so a slow-ramp/authority explanation is
dead. **Caveat, stated because it matters:** the dodge fires while the drone is
patrolling at ~5.26 m/s, so part of this displacement is pre-existing cruise
momentum rather than dodge response, and only the component **perpendicular to the
ball's path** raises `min_dist`. The clean version needs the counterfactual — where
the drone would have been had it kept cruising. What the number does establish is
that gross motion is not the shortfall.

### Hypothesis, NOT yet measured

The remaining puzzle is why the velocity estimate is still immature at the third
update, when an `init_vel_std` of 15 m/s (variance 225) should let the *second*
measurement pin velocity to roughly `(z2-z1)/dt`. The most likely explanation is
that the ball's track is being **reseeded**: `process()` counts Mahalanobis
rejects and, at `_max_rejects`, calls `reset()` and re-initialises from the
outlier — putting velocity back to zero. With the detector emitting ~1
false-positive centroid per second interleaved with the ball's, a track can be
repeatedly reseeded and never accumulate a mature velocity.

If that is true it **reframes the false-positive stream**: harmless for false
dodges (`threat_radius` rejects every spurious plan, all measured >= 1.79 m wide)
but the direct cause of the dodge-authority failure. That would also make detector
precision the highest-value work in Week 4, not a nice-to-have.

**The test:** log `n_updates` and the reject/reseed count alongside each published
intercept, and check whether ball tracks reseed mid-flight. Do this before touching
`init_vel_std`, `dodge_speed_mps`, or any detector threshold — a tuning change
that happens to help would hide the mechanism.

### B01 is measurable again

`offset_forward_m: 4.0` for B01 only. At 4 m/s the 6.0 m default flies 1.50 s and
the window gate then needs 1.80 s of straight leg; the 12 m loop's best is 2.18 s
and rarely coincides with the 95%-cruise floor, so B01 skipped on **every** attempt
from v12 to v24. It now throws (v25: 1 of 3 thrown and dodged, `min_dist` 0.207 m).
Shortening the flight rather than lowering `throw_window_margin_s`, which is part
of the gate that fixed the aiming. Caveat in the config: 4 m starts the ball inside
the 5 m range gate, so B01's `first_det_range_m` is not comparable to the others.

### Where the battery stands (v24/v25, aim fixed, settled stack)

- on-target dodge success **10/13 (77%)** and **10/16 (62%)**
- off-target **2/17** and **2/19**; aim error 0.46-0.99 m
- latency **94-107 ms mean**, max 256-267 ms, against a 150 ms mean budget
- `tca` 0.192-0.205 s mean, and that is the number to move

---

## 2026-07-27 (night, cont.) — The dodge time budget closes, and consistency is the binding term

`track_age_s` and the reseed counters turned the dodge-authority question from
inference into arithmetic. **The budget now reconciles to within 0.02 s**, and the
term that is actually binding is not the one anyone would have guessed.

### The reseed hypothesis: refuted

The previous entry proposed that the detector's ~1/s false-positive stream was
reseeding the ball's track through the Mahalanobis reject path, keeping the
velocity estimate at zero. **It is not.** Measured over a full battery:

    track at commit: 3.0 updates (range 3-3), age 0.138 s (0.132-0.198)
                     => 14.5 Hz of accepted ball detections

The ball's own detections arrive at essentially the **full camera rate**, and the
track age at commit is exactly the time to accumulate 3 updates (2 gaps at
~14.5 Hz). The track that fires is healthy and was not restarted.

The reseeds are real and numerous — **209 in one battery (reject 60, timeout 149)**
— but they are lone false-positive centroids creating a track that expires 0.5 s
later *between* throws. **I had reported this badly** ("11/11 dodges followed a
restarted track"), which is true only in the sense that a lifetime counter was
non-zero, and it invited exactly the wrong conclusion. The report now leads with
track AGE and demotes the lifetime totals to a footnote telling the reader to
ignore them. Age against the ball's visibility is the measurement that separates a
restarted track from sparse detection; the totals cannot.

### The budget

| term | measured |
|---|---|
| warning available (3.64 m / 8 m/s) | 0.455 s |
| - track confirmation (3 updates @ 14.5 Hz) | 0.138 s |
| - pipeline latency | 0.082 s |
| **= predicted tca** | **0.235 s** |
| measured tca | **0.257 s** |

### Lever 4: `cluster_min_points` 5 -> 3 — improved the target term, still no tca

Detection range being the dominant term, and the ROI gate having been cleared,
the ball's detectability at range was the lever: an 80 mm sphere projects few
points at distance. Lowering the floor was justified by two things measured
tonight — false positives cause **no** false dodges (`threat_radius` rejects every
spurious plan, all >= 1.79 m wide) and, per above, do **not** poison the ball's
track. So the Week-3 reason for raising 3->5 no longer implies the same cost.

| | min_pts 5 | min_pts 3 |
|---|---|---|
| first detection | 3.64 m | **4.52 m (+24%)** |
| `tca` mean | 0.257 s | **0.218 s** |
| on-target dodge | 11/17 (65%) | 11/14 (79%) |
| false dodges | 0/2 | 0/2 |

**It moved first detection by 24% and bought no tca.** Reverted: shipping a change
whose only *measured* effect is more false positives is not justified by a dodge
rate that moved inside its own noise (n=14-17).

### Why range levers keep failing, and what the term really is

The clue is that track age at commit is **exactly 0.132 s on every dodge**
(0.132-0.132, perfectly uniform) — always precisely 3 consecutive frames. Working
backwards: the ball sits ~`tca * speed` = 1.74 m away at commit, and the track
began `speed * age` = 1.06 m earlier, so it started at **~2.8 m**. First detection
was at **4.52 m**.

**~1.7 m of range is detected and then discarded**, because those early detections
never become three *consecutive* frames: a lone centroid times out
(`track_timeout_s` 0.5 s) before the run completes. That is why widening
`roi_max_range_m` and lowering `cluster_min_points` both improved first detection
and neither improved `tca` — both bought RANGE, and the binding term is
**consistency**. The battery now derives and prints this gap directly.

**So the next work is detection consistency at range, not range.** Candidates, in
the order their cost is understood: the per-frame survivor counts on a single
throw with `debug_funnel` (does the ball fail `cluster_min_points`, the extent
gate, or the score gate on the frames it is missed on?); then whether the
persistent background map is absorbing a distant slow-parallax ball into
background; then `min_publish_score`. Do not touch `min_track_updates` or
`dodge_speed_mps` — both measured, both inside the noise.

### Where Week 4 stands

Best figures on a settled stack with the aim fixed:

- on-target dodge success **11/15 (73%)** and **11/14 (79%)**
- off-target **0/19** on the best run; aim error 0.36-0.46 m
- latency **75-82 ms mean**, max 136-182 ms, against a 150 ms budget
- `tca` 0.218-0.257 s — the number still to move
- B01 measurable again (`offset_forward_m: 4.0`), so all seven scenarios now throw

---

## 2026-07-27 (night, cont.) — The detection-consistency gap does not exist

The frame-by-frame trace that was supposed to name which detector gate drops the
ball instead **refuted the premise it was built on**. The detector does not drop
the ball. It publishes the true ball on an unbroken run of frames from ~4.7 m
inwards, and the frames the tracker never uses were never missing.

### The measurement

Two new instruments, joined offline on the sim clock:

- `debug_dump_dir` (detector.yaml, default off) writes one `.npz` per cloud
  frame: the post-voxel egomotion-compensated cloud, its foreground mask, every
  cluster the splitter produced, and the publish decision.
- `scripts/hz_truth_probe.py` records the ball's and the drone's true poses plus
  every published `/threat/centroid`.
- `scripts/hz_funnel_attribute.py` joins them and classifies each frame.

Three B02 throws (8 m/s head-on, the design point), inbound frames only:

| throw | frames with the true ball published | range span |
|---|---|---|
| r0 | **5 consecutive** | 4.49 -> 1.07 m |
| r1 | **6 consecutive** | 4.69 -> 0.73 m |
| r2 | **6 consecutive** | 4.92 -> 1.07 m |

Every one matched the ball's true range to **0.02-0.03 m**, at the full
15.2 Hz, with no gaps. Cluster sizes grew 29 -> 88 points as the ball closed and
extents stayed 0.08-0.16 m — a textbook ball signature on every frame.

**So there is no ~1.7 m of range "detected and then discarded".** The detector
delivers 5-6 consecutive true detections starting at ~4.7-4.9 m. Yet the track
that fires the dodge is still, invariably, exactly 3 updates and 0.132 s old.
Something between `/threat/centroid` and the dodge decision throws away the
first two to three true detections.

### Where that puts the dodge-authority problem

The binding term is in the **tracker**, not the detector. The budget entry from
earlier tonight charged 0.138 s to "track confirmation at 14.5 Hz" and treated
it as irreducible; it is not — the detections needed to confirm a track were
already in hand ~2 frames earlier.

The likeliest mechanism, and the next thing to test: the evasion node's tracker
is being churned by the detector's false-positive stream. Measured on this run:
**673 reseeds** over the battery (153 by Mahalanobis reject, 520 by timeout),
i.e. ~1.4 per second, continuing right through the throw windows. If the ball's
opening detections land on a live FP track, they are rejected until `_max_rejects`
forces a `reset()`, and the ball's track restarts from the outlier — costing
precisely the two frames observed.

Note this does **not** resurrect the reseed hypothesis as previously stated and
refuted. That version claimed the ball's detections were arriving sparsely; they
are not, and this trace confirms it directly. The claim now is narrower and
different: the detections arrive intact and the tracker discards them.

**The named test:** log, per centroid, which track it was associated with and
whether it was accepted or rejected, then read that across one throw. Do not
touch `min_track_updates`, `init_vel_std`, or any detector threshold first — a
tuning change that happens to help would hide the mechanism, exactly as it would
have earlier tonight.

### A real defect found on the way: two clocks in one stack

Measured directly, not inferred:

    /oak/points        631.951 s          Gazebo sim clock
    /gz/dynamic_poses  631.9xx s          Gazebo sim clock
    /huitzilin/odom    1785183152.876 s   WALL clock

`week2_sitl.launch.py` never sets `use_sim_time`, so **`mav_bridge`, `patrol` and
`telemetry_logger` run on the wall clock** while every Gazebo-sourced node runs
on sim time (`ros2 param get /mav_bridge use_sim_time` -> False).

Consequence in the detector: it inserts `odom -> base_link` into its TF buffer
stamped on the wall clock, then looks it up at cloud stamps on the sim clock.
The exact-stamp lookup can never succeed, so **every frame silently takes the
`Time()` "latest available" fallback** and egomotion compensation is never
time-matched.

Measured impact, and it is smaller than feared: on frames that published, the
cluster sat **0.02-0.19 m** from the ball's true position. So this is a genuine
defect worth fixing, but it is not what is costing the dodge. Left unchanged
tonight deliberately — it alters flight-node timing semantics and there was no
budget to re-validate patrol behaviour after it.

The two clocks also do **not** differ by a constant: sim runs at RTF ~0.87, so
they differ by a rate. Fitting one offset across a 50 s battery left an 11.6 m
residual. Anything joining these streams must handle that.

### Instrument lessons, dearly bought

- **The truth probe blinded the detector it was observing.** Recording every
  link in `/gz/dynamic_poses` (rotor_0..3, imu_link, camera_link, base_link) and
  flushing per row wrote ~1400 rows/s; with it running, the same B02 battery
  measured first detection at **1.83 m instead of 4.08 m** and dropped from 2/3
  dodges to 1/3. Filtered to the drone and `ball_*` and flushed on a 2 s timer,
  it costs nothing measurable. Any probe on this box needs this checked.
- **The dump itself costs ~40 ms/frame** (latency 128-172 ms with it on, against
  75-94 ms without). Fine for attribution, useless for timing. Never read a
  latency or `tca` number off a dumping run.
- **Reconstructing the ball in the odom frame is the weak link, not the cloud.**
  Frames the reconstruction called "unclustered" each held exactly one cluster of
  exactly the ball's size and extent, 0.5-1.0 m from where the reconstruction put
  it — attitude lag at range, since 6 deg at 4.5 m is 0.5 m. The reliable test
  needs no reconstruction: `|centroid_bl|` is a range in base_link, directly
  comparable to the ball's true range. When the two disagree, believe the range.

### Where Week 4 stands

Unchanged by this entry — no threshold was altered, and the stack is back on the
shipped `detector.yaml`:

- on-target dodge success 11/15 (73%) and 11/14 (79%) on the best settled runs
- latency 75-94 ms mean against a 150 ms budget
- `tca` 0.18-0.26 s — still the number to move, now with a named cause to test

---

## 2026-07-27 (night, cont.) — Dodge authority root-caused: a false positive poisons the ball's track

The trace above cleared the detector. The cause is in `ProjectileTracker`, it is
reproducible offline with no sim at all, and it is worth 0.267 s — more than any
detector threshold ever tried.

### The mechanism

`ProjectileTracker` is **single-target**: one `_x`, fed by every centroid the
detector publishes. The detector emits ~1.4 false positives per second (673
reseeds per battery: 153 reject, 520 timeout), so a stale FP track is usually
alive when a ball arrives. What happens then is not "the ball is ignored":

| frame | outcome | velocity error |
|---|---|---|
| f0 | reject | 8.0 m/s (seeded at zero) |
| f1 | **ACCEPTED** | **38.6 m/s** — fits a line from the FP to the ball |
| f2 | reject | 38.6 |
| f3 | reject | 38.6 |
| f4 | reseed from the ball, `n_updates` back to **1** | 8.0 |
| f5 | accept | **0.48** |

After a single rejection the incumbent's covariance has inflated enough
(`init_vel_std` 15 m/s, so one 66 ms prediction gives ~1 m of position sigma) to
**swallow the ball as a legitimate update**. The filter then fits a velocity
along the false-positive-to-ball line — 38 m/s in the wrong direction — and
spends the next two true detections contradicting the fiction it just built.

**The control, with no incumbent, reaches 0.48 m/s velocity error at f1.** The
ball needs six frames to reach what takes two: a **4-frame, 0.267 s penalty**.
`tca` at commit measures 0.18-0.26 s, so this is the entire missing warning.

Pinned in `test/test_kalman.py` (`test_ball_alone_pins_velocity_by_the_second_frame`,
`test_an_incumbent_false_positive_corrupts_the_balls_velocity_estimate`,
`test_the_incumbent_costs_four_frames_of_warning`). No behaviour was changed —
the tests describe today's tracker so the fix has something to beat.

### It resolves two results that never made sense

- **The invariant 0.132 s / 3-update track age at commit.** The track that fires
  is always the *reseeded* one, which starts at `n_updates = 1` by construction,
  so it is always exactly 2 gaps old when it reaches 3. Nothing about detection
  rate was ever going to move that.
- **Why `min_track_updates` 3->2 bought only 0.003 s.** The binding constraint
  is *when the reseed happens*, not the count that follows it. Lowering the
  count just shortens a wait that begins too late.

It also supersedes the earlier "KF velocity convergence" entry. That entry was
right that the trigger waits on velocity quality and right that
`predict_closest_approach` starts with `miss ~= current range`. It attributed
that to a zero-velocity prior converging slowly. The real cause is worse: the
prior is not merely uninformative, it is actively **wrong**, having been fitted
across a jump between two different objects.

### What to do about it, and what not to

The obvious knobs are traps. `max_consecutive_rejects` 3->1 would reseed sooner
but destroy a mature ball track on a single outlier. Raising the chi2 gate makes
the swallowing worse. Lowering `init_vel_std` slows genuine convergence.

The fix that matches the diagnosis is to stop forcing one filter to represent
whatever arrived last: keep a small set of candidate tracks, associate each
centroid to the best gate-passing one, start a new track for anything
unassociated, and let the trigger act on whichever *confirmed* track threatens.
The ball's opening detections then accumulate on their own hypothesis instead of
being spent correcting someone else's, and confirmation should arrive ~4 frames
sooner.

Predicted effect if that holds: `tca` 0.18-0.26 s -> ~0.45-0.53 s, i.e.
0.67-0.79 m of dodge travel at 1.5 m/s against a 0.30 m hit radius. That is the
number to check, and it is the first change all night with a mechanism behind it
rather than a threshold.

### 2026-07-27 — The multi-hypothesis tracker works, and it did not buy the dodge

Landed `MultiHypothesisTracker` (0c719ab) and measured it against the full
20-run battery. The prediction in the entry above — `tca` 0.18-0.26 s ->
0.45-0.53 s — **did not happen**. Recording that plainly, because the fix did
do what it was designed to do and the failure is informative.

What it fixed, measured:

| | before | after |
|---|---|---|
| lifetime reseeds in one stack run | 5377 (1123 reject / 4256 timeout) | concept gone |
| detection consistency gap | +1.7 m | **+0.31 m** |
| live hypotheses at commit | n/a (one filter) | 1.1 mean, max 2 |
| track age at commit | 0.132 s, invariant | 0.143 s (0.132-0.198) |

The gap closing to +0.31 m is the direct confirmation: the ball's track now
starts on its own first detection instead of ~1.7 m later, and `max 2` live
hypotheses says the associator is not being saturated by the false-positive
stream. The offline 4-frame penalty is real and it is gone.

What did not move: `tca` **0.218 s mean** (0.090-0.450), dodge success **13/18
(72%)**, false dodges **0/2**, latency 107 ms mean / 224 ms max. Compare the
best previous settled figures: `tca` 0.18-0.26 s, 11/15 and 11/14. Inside noise.

### Why closing a 1.7 m gap bought no warning

Because the gap was never spent as *warning* — it was spent before the ball was
ever detected. Track age at commit is still ~3 frames, which now means what it
says: the trigger fires 3 frames after the ball's FIRST detection, and first
detection sits at 3.2 m mean against a 5.0 m gate. The tracker was wasting
frames; removing that waste just moved the bottleneck back to where detection
begins. Two terms now bound `tca` and nothing else does:

    tca = (first_det_range - min_track_updates / 15 Hz * speed) / speed - pipeline

The failures are all one shape. B03 (14 m/s) is 0/3 with min_dist 0.155, 0.157,
0.163 — and the measured aim error on no-dodge runs is 0.15 m. Those dodges
contributed essentially nothing; the ball passed at its natural miss distance.
At 14 m/s, three confirmation frames cost 2.8 m of the 5 m gate outright.

### Two "refuted" levers are hereby un-refuted

This is the part worth carrying forward. `roi_max_range_m` 5->8 and
`cluster_min_points` 5->3 were both measured last night, both bought detection
RANGE, both moved `tca` by nothing, and both were written down as refuted. That
verdict was correct *about the tracker they were measured on*: extra range
handed the filter more opening frames, and the filter spent them arguing with a
false positive. With the ball now keeping its own hypothesis, range converts to
warning at ~1:1. Their refutations do not survive the fix that removed the
reason they failed.

`min_track_updates` 3->2 is the one lever that is genuinely tested under the new
tracker, and it is a real null: `tca` 0.218 -> 0.262 s (+0.044 s, about
two-thirds of a frame, exactly as arithmetic predicts) but success 13/18 ->
11/16 and latency mean 107 -> 127 ms. Reverted to 3. Buying two-thirds of a
frame by weakening confirmation is not a trade worth making; the frames have to
come from detecting the ball further out.

Next: `cluster_min_points` 5->3, which measured a 24% detection-range gain last
night, re-run under the new tracker. Its stated risk was false positives, and
false positives are precisely what the associator now absorbs.

### 2026-07-27 — Both un-refuted levers re-tested under the new tracker; both null

Ran the full 20-run battery three more times, one lever at a time, all on the
multi-hypothesis tracker with everything else at shipped values. Baseline for
comparison is the run above: `tca` 0.218 s, 13/18, latency 107/224 ms.

| lever | tca mean | on-target success | latency mean/max | live hyps |
|---|---|---|---|---|
| baseline (shipped) | 0.218 s | **13/18 (72%)** | 107 / 224 ms | 1.1 / 2 |
| `min_track_updates` 3->2 | 0.262 s | 11/16 (69%) | 127 / 331 ms | 1.2 / 2 |
| `cluster_min_points` 5->3 | 0.259 s | 11/17 (65%) | 122 / 346 ms | 1.1 / 2 |
| `roi_max_range_m` 5->8 | 0.199 s | 9/17 (53%) | 112 / 246 ms | **2.0 / 4** |

False dodges 0/2 in all four. Shipped config is the best of the four on every
column that matters, and is what the stack was left on.

The un-refutation argued above was wrong, and the reasoning is worth keeping
because it was wrong in an instructive way. Extra range does now reach a clean
hypothesis — the consistency gap confirms it, running -0.02 m at `roi 8` — but
it arrives with company. Live hypotheses at commit double to 2.0 mean / 4 max,
and `tca` gets *worse*, not better. What the wider gate buys is far-field
clutter that the associator has to entertain, and the ball's own confirmation
still costs the same three frames. Range was never the binding term; it was not
made binding by fixing the tracker either.

### What actually bounds tca, arithmetically

Every configuration lands at the same place because the same three terms set it:

    tca = first_det_range/speed  -  min_track_updates/15 Hz  -  pipeline

With the shipped values that is 4.5 m / 8 m/s - 0.20 s - 0.08 s = 0.28 s against
0.218 s measured. Track age at commit is 0.132-0.198 s across every battery run
tonight — pinned at the confirmation term, never above it. Four thresholds have
now been moved and the sum has not: each lever trades one term for another.

The term nobody has touched is **15 Hz**. Three confirmation frames cost 0.200 s
at 15 Hz and 0.100 s at 30 Hz — 0.1 s of `tca` for free, more than double what
any threshold delivered tonight, and without weakening confirmation the way
`min_track_updates` 2 does. The blocker is recorded in CLAUDE.md: `ros_gz_bridge`
PointCloudPacked->PointCloud2 cannot sustain 30 Hz at 640x480 on the Dell, which
is why `iris_depth` runs `update_rate=15`. That is a transport problem with
known shapes (lower resolution at the same rate, a compressed transport, or
detecting on the depth image instead of the point cloud) and it is the next
thing worth an evening — not another threshold.

### 2026-07-27 — Halving the confirmation term: measured, and reverted

The entry above named 15 Hz as the one untouched term. Tested it directly.

First, the cheap capacity question, with `profile_stages` on the live stack:
the detector runs **30 ms/frame mean, 11.6-48.8 ms range**. A 30 Hz cloud allows
33 ms sim = ~38 ms wall at RTF 0.87, so 640x480 @ 30 Hz is blocked by detector
COMPUTE before the bridge is even reached — and blocked on precisely the busy
frames (voxel 12-16 ms, bg_query 4.5-9.6, finite 5-7.6, convention 3-4), which
are the frames with foreground in them. Nearly all of that cost is per-point, so
it scales with pixel count: quarter the pixels, quarter the work, same bytes/s
through the bridge.

So: `iris_depth` to **320x240 @ 30 Hz**, full restart, full battery.

| | 640x480 @ 15 Hz (shipped) | 320x240 @ 30 Hz |
|---|---|---|
| accepted ball detections | 14.8 Hz | **27.1 Hz** |
| track age at commit | 0.143 s | **0.097 s** |
| latency mean / max | 107 / 224 ms | **65 / 175 ms** |
| first detection | 3.22 m | 4.40 m |
| `tca` | 0.218 s | 0.266 s |
| on-target success | **13/18 (72%)** | 11/16 (69%) |
| aim error (no-dodge) | 0.15 m | 0.56 m |

The mechanism did exactly what was predicted: confirmation halved, latency fell
40%. And `tca` moved **+0.048 s** — the same size as every other lever — while
success did not move at all. RTF fell 0.87 -> 0.758 and Gazebo delivered ~27 Hz
of the 30 requested; the consistency gap reopened to +1.41 m and live hypotheses
rose to 1.7/3, because a denser frame stream is also a denser false-positive
stream.

Reverted. It costs sensor fidelity against the real OAK-D Lite (which does
640x480 depth at 30 fps in its own hardware, so the sim resolution is the thing
that would be lying) and buys no success. The latency number is worth keeping in
mind for Weeks 5-6 though: at quarter resolution the whole pipeline runs at
65 ms mean, less than half the 150 ms budget, which is the margin a Pi-class
board will need.

### The pattern, after five levers

`min_track_updates`, `cluster_min_points`, `roi_max_range_m`, and now frame rate
have each been moved under the fixed tracker. Every one produced a `tca` change
of +-0.05 s and no improvement in dodge success, which has sat at 61-72% across
six full batteries tonight regardless of what `tca` did.

That is the finding. `tca` in the 0.20-0.27 s band is no longer what decides
whether a dodge succeeds — if it were, +0.05 s (a third more escape travel)
would show up somewhere in 18 runs, and it never does. The failures are bimodal:
successes land at 0.35-0.53 m and failures at ~0.155 m, which is the measured
aim error, i.e. the ball's natural miss distance with the dodge contributing
nothing. B03 (14 m/s) is 0/3 in every battery — at that speed the ball crosses
the entire 5 m gate in 0.36 s and no amount of the current budget helps.

The next question is therefore not "how do we get more warning" but "why does a
dodge that commits 0.27 s out, with a manoeuvre that over-delivers at 160% of
1.5 m/s, still leave the drone at 0.155 m". That is a question about the
manoeuvre and the geometry, not the trigger, and it should start by plotting
commanded vs achieved displacement for a single failing 8 m/s run.
