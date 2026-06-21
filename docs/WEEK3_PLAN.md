# HuitzilinReflex — Week 3 Reconciled Plan

*Generated 2026-06-21. This is the execution plan for Week 3, reconciling `HuitzilinReflex_Week3_Playbook.docx` (tasks W3-01…W3-22) against the **actual repo** and **actual Week 2 outcome**. Where the playbook assumes something the repo doesn't have yet, the resolution is stated here.*

---

## 1. Where we are

Week **3 of 9**, Phase A (Foundations & Simulation) — the first week the camera matters. Pure simulation: synthetic and recorded depth only. No real OAK-D (Week 6), no Kalman/dodge (Week 4).

> **Week 3 Definition of Done:** the detection node flags ≥95% of simulated incoming clusters, with a quantified false-positive rate, scored repeatably against a labeled rosbag library — demonstrated on a fresh checkout.

**The bright line:** detection only. If a task tempts you toward predicting an intercept or commanding a maneuver, it's misfiled — that's Week 4.

## 2. Reconciliation — playbook vs. actual repo

| # | Item | Playbook assumes | Actual repo state | Resolution for Week 3 |
|---|---|---|---|---|
| R1 | Week 2 carry-overs (W3-02) | Two open items to close: W2-18 (fresh-checkout sign-off), W2-05 (Pass-B model) | Per `docs/JOURNAL.md` (2026-06-18): W2-05 **explicitly deferred** to Wk 7-8; W2-18 was "in progress (teammate testing)" | Confirm W2-18 sign-off status before W3-04 (depth work shouldn't start on an unverified baseline). Re-affirm W2-05 stays deferred — don't re-open it to "make depth nicer." |
| R2 | Perception package (W3-03) | New `huitzilin_perception` ament_python package, sibling to `huitzilin_sim` | `src/` currently has only `huitzilin_sim` and the superseded `mavlink_bridge` — no perception package exists yet | Create `src/huitzilin_perception/` as a sibling package. Keep depth processing **out of** `mav_bridge`; the only seam to flight is the Week 4 `/cmd/evade` path. |
| R3 | Topic contracts (`/oak/points`, `/oak/depth`, `/threat/centroid`) | Provisional contracts already drafted | Confirmed present in `docs/architecture.md` §"Future weeks (Wk3-6)" — best-effort/keep-last-1, frame `camera_optical_frame` for `/oak/*`; reliable, `base_link` for `/threat/centroid` | Adopt as written. W3-19 promotes `/oak/*` and `/threat/centroid` from provisional to **active**; `/cmd/evade` and `/payload/alarm` stay provisional (Wk4/Wk6). |
| R4 | GPU rendering (W3-04 onward) | A box with a real GPU does the sensor rendering | Primary dev box is WSL2 + Intel Iris Xe — no GPU passthrough, Gazebo already runs at ~24% real-time headless. Native-Ubuntu Dell Inspiron 3670 (i5-8400 / UHD630) is the only box with a real GPU path | **All depth-sensor rendering and scenario capture happens on the Dell Inspiron.** SITL/flight logic can run on either box. This is the single fact that shapes the whole week (playbook §3) — extend the existing sim-time discipline (see `CLAUDE.md` sharp edges) to frame rate too. |
| R5 | Frame mount (`base_link → camera_link → camera_optical_frame`) | Exact mount offset measured in "Phase B" | Not yet measured — no camera exists on the model today | Use a placeholder static offset for W3-06, note it explicitly as provisional in `docs/frames.md`, and flag the real measurement as a Phase-B follow-up (don't block W3 on hardware that doesn't exist yet). |
| R6 | Bag storage (W3-10) | Versioned rosbag library, large blobs kept out of git proper | No LFS or external-blob convention exists in this repo yet (`.gitattributes` only normalizes line endings) | Track an index + checksums in git; store the actual `.mcap` blobs outside git (path noted in `docs/SETUP.md`) rather than introducing LFS mid-project. |

## 3. Task order (from the playbook, W3-01 → W3-22)

Work top to bottom — later tasks assume earlier ones are done. Six work areas:

1. **Inherited State & Setup** — W3-01 (regression gate) → W3-02 (close Wk2 carry-overs) → W3-03 (scaffold `huitzilin_perception`)
2. **Simulated Depth Sensor (Gazebo)** — W3-04 (add sensor to SDF) → W3-05 (bridge to ROS 2) → W3-06 (camera TF) → W3-07 (verify in RViz)
3. **Synthetic Threat Scenarios** — W3-08 (projectile spawner) → W3-09 (scenario matrix + labels) → W3-10 (labeled rosbag library)
4. **Detection Node** — W3-11 (skeleton) → W3-12 (ROI/filter) → W3-13 (differential clustering) → W3-14 (centroid + publish) → W3-15 (tune thresholds)
5. **Rosbag Library & Regression** — W3-16 (offline harness) → W3-17 (quantify TP/FP) → W3-18 (regression-proof)
6. **Integration, Reproducibility & DoD** — W3-19 (update contracts) → W3-20 (reproducible launch) → W3-21 (recorded acceptance run) → W3-22 (retro + Wk4 handoff)

## 4. Immediate next actions (W3-01…W3-03)

- [ ] **W3-01** — From a fresh shell, fly one clean patrol lap (mean ~29.5s, matching Week 2 evidence) before touching perception code.
- [ ] **W3-02** — Resolve the W2-18 sign-off in `JOURNAL.md` (or log the remaining gap explicitly); re-affirm W2-05 stays deferred.
- [ ] **W3-03** — Scaffold `src/huitzilin_perception/` (ament_python; deps: rclpy, sensor_msgs, geometry_msgs, visualization_msgs, tf2_ros, ros_gz_image/ros_gz_bridge, numpy); register `data_files` in `setup.py` so launch/param files don't vanish post-install (the Week 2 lesson).

## 5. What stays out of scope this week

- No real OAK-D hardware (Week 6).
- No Kalman filter / intercept prediction / dodge command (Week 4) — `/threat/intercept` and `/cmd/evade` stay provisional.
- No Pass-B airframe fidelity work (Week 7-8) — depth hangs on the stock `iris_with_standoffs` model as-is.

## 6. Evidence & doc trail

- Numbers go in `docs/week3_detection_evidence.md` (new, mirroring `docs/week2_patrol_evidence.md`).
- Contract updates (`/oak/*` provisional → active) land in `docs/architecture.md` in the **same commit** as the code that implements them.
- TF additions get verified against `docs/frames.md`.
- Retro + Week 4 handoff appended to `docs/JOURNAL.md` at W3-22.
