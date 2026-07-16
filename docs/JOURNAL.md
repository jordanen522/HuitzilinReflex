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
