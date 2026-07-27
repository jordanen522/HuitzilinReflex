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
