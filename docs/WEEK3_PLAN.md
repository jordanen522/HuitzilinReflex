# Week 3 Plan — CLOSED 2026-07-15

All tasks W3-01…W3-22 are done: perception package, simulated OAK-D depth sensor,
projectile spawner + 17-scenario matrix, detection node, offline scoring harness,
labeled bag library, tuning, held-out test pass (recall 100%, gate exits 0), and a
recorded live acceptance run. Results and open items (S08 FN, N02/N03/N05 FPs) are in
`docs/JOURNAL.md`; the task-by-task history is in this file's git history.

Facts from the plan that outlive it:

- **All depth rendering and bag capture happens on the native-Ubuntu Dell box** —
  WSL2/Iris Xe cannot render Gazebo depth at rate. SITL/flight logic runs on either box.
- Bags live outside git at `/data/huitzilin_bags` (hardcoded default in
  `score_bags.py` and `run_regression.sh`); no LFS in this repo.
- **Never tune against the test split** (S11, S12, N05).
- Camera TF offset in the launch file is a placeholder — measure the real mount in
  Phase B (`docs/frames.md`).
- `max_step_size` must stay `0.001` (1000 Hz) — `0.004` causes a SITL "Main loop slow"
  PreArm failure.
- Re-capture / re-tune procedure: `docs/week3_capture_runbook.md`.
