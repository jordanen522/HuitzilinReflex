# HuitzilinReflex — Week 3 Reconciled Plan

*Generated 2026-06-21. This is the execution plan for Week 3, reconciling `HuitzilinReflex_Week3_Playbook.docx` (tasks W3-01…W3-22) against the **actual repo** and **actual Week 2 outcome**. Where the playbook assumes something the repo doesn't have yet, the resolution is stated here.*

*Status updated 2026-06-21 — all work that can be done without a live Gazebo/GPU is committed; what remains is the on-Dell capture, tuning, and acceptance pass (see §4).*

---

## 1. Where we are

Week **3 of 9**, Phase A (Foundations & Simulation) — the first week the camera matters. Pure simulation: synthetic and recorded depth only. No real OAK-D (Week 6), no Kalman/dodge (Week 4).

> **Week 3 Definition of Done:** the detection node flags ≥95% of simulated incoming clusters, with a quantified false-positive rate, scored repeatably against a labeled rosbag library — demonstrated on a fresh checkout.

**The bright line:** detection only. If a task tempts you toward predicting an intercept or commanding a maneuver, it's misfiled — that's Week 4.

**Where the split falls right now:** the entire perception code path — package, simulated depth sensor, TF, projectile spawner, scenario matrix, detector pipeline, scoring/regression harness, and the unified launch — is written and committed. None of it has been exercised against a live depth stream yet, because depth rendering requires the GPU on the Dell. So everything left is hardware-bound capture and tuning, not code.

## 2. Reconciliation — playbook vs. actual repo

| # | Item | Playbook assumes | Actual repo state | Resolution for Week 3 |
|---|---|---|---|---|
| R1 | Week 2 carry-overs (W3-02) | Two open items to close: W2-18 (fresh-checkout sign-off), W2-05 (Pass-B model) | Per `docs/JOURNAL.md` (2026-06-18): W2-05 **explicitly deferred** to Wk 7-8; W2-18 was "in progress (teammate testing)" | **Done** — JOURNAL.md now logs the W2-18 gap, confirms W2-05 stays deferred, and records the Week 3 kickoff. |
| R2 | Perception package (W3-03) | New `huitzilin_perception` ament_python package, sibling to `huitzilin_sim` | `src/` currently has only `huitzilin_sim` and the superseded `mavlink_bridge` — no perception package exists yet | **Done** — `src/huitzilin_perception/` created as a sibling package (package.xml, setup.py, setup.cfg). Depth processing kept out of `mav_bridge`. |
| R3 | Topic contracts (`/oak/points`, `/oak/depth`, `/threat/centroid`) | Provisional contracts already drafted | Confirmed present in `docs/architecture.md` §"Future weeks (Wk3-6)" — best-effort/keep-last-1, frame `camera_optical_frame` for `/oak/*`; reliable, `base_link` for `/threat/centroid` | Adopt as written. W3-19 promotes `/oak/*` and `/threat/centroid` from provisional to **active**; `/cmd/evade` and `/payload/alarm` stay provisional (Wk4/Wk6). |
| R4 | GPU rendering (W3-04 onward) | A box with a real GPU does the sensor rendering | Primary dev box is WSL2 + Intel Iris Xe — no GPU passthrough, Gazebo already runs at ~24% real-time headless. Native-Ubuntu Dell Inspiron 3670 (i5-8400 / UHD630) is the only box with a real GPU path | **All depth-sensor rendering and scenario capture happens on the Dell Inspiron.** SITL/flight logic can run on either box. This is the single fact that shapes the whole week (playbook §3) — extend the existing sim-time discipline (see `CLAUDE.md` sharp edges) to frame rate too. |
| R5 | Frame mount (`base_link → camera_link → camera_optical_frame`) | Exact mount offset measured in "Phase B" | Not yet measured — no camera exists on the model today | **Done (provisional)** — placeholder static offset baked into the launch file for W3-06; flagged in `docs/frames.md` as a Phase-B follow-up. |
| R6 | Bag storage (W3-10) | Versioned rosbag library, large blobs kept out of git proper | No LFS or external-blob convention exists in this repo yet (`.gitattributes` only normalizes line endings) | Track an index + checksums in git; store the actual `.mcap` blobs outside git (path noted in `docs/SETUP.md`) rather than introducing LFS mid-project. |

## 3. Task order (from the playbook, W3-01 → W3-22)

Work top to bottom — later tasks assume earlier ones are done. Six work areas. Status legend: `[x]` done · `[~]` partial (code done, live tuning pending) · `[ ]` pending (Dell box).

1. **Inherited State & Setup** — `[ ]` W3-01 (regression gate, Dell) → `[x]` W3-02 (close Wk2 carry-overs) → `[x]` W3-03 (scaffold `huitzilin_perception`)
2. **Simulated Depth Sensor (Gazebo)** — `[x]` W3-04 (add sensor to SDF + world) → `[x]` W3-05 (bridge to ROS 2) → `[x]` W3-06 (camera TF) → `[ ]` W3-07 (verify in RViz, Dell)
3. **Synthetic Threat Scenarios** — `[x]` W3-08 (projectile spawner) → `[x]` W3-09 (scenario matrix + labels) → `[ ]` W3-10 (record labeled rosbag library, Dell)
4. **Detection Node** — `[x]` W3-11 (skeleton) → `[x]` W3-12 (ROI/filter) → `[x]` W3-13 (differential clustering) → `[x]` W3-14 (centroid + publish) → `[~]` W3-15 (tune thresholds — params exposed, operating point pending sweep)
5. **Rosbag Library & Regression** — `[x]` W3-16 (offline harness) → `[ ]` W3-17 (quantify TP/FP on held-out set, Dell) → `[~]` W3-18 (regression-proof — harness committed, library wiring pending bags)
6. **Integration, Reproducibility & DoD** — `[ ]` W3-19 (update contracts) → `[x]` W3-20 (reproducible launch) → `[ ]` W3-21 (recorded acceptance run, Dell) → `[ ]` W3-22 (retro + Wk4 handoff)

## 4. Status snapshot (2026-06-21)

### Done — codeable without Gazebo (committed)

- **W3-02** — JOURNAL.md: W2-18 gap logged, W2-05 deferral confirmed, Week 3 kickoff written.
- **W3-03** — `huitzilin_perception` ament_python package (package.xml, setup.py, setup.cfg).
- **W3-04** — `models/iris_depth/model.sdf`: `iris_with_standoffs` extended with an OAK-D Lite–spec depth sensor (640×480, 73° DFOV, 30 fps, 0.2–19 m, 1 cm Gaussian noise); `worlds/huitzilin_runway.sdf` world that uses it.
- **W3-05** — `params/perception_bridge.yaml`: ros_gz_bridge config for `/oak/depth`, `/oak/points`, `/oak/camera_info`.
- **W3-06** — Static TF baked into the launch file: `base_link → camera_link → camera_optical_frame`.
- **W3-08** — `models/projectile/model.sdf` (80 mm sphere, 150 g) + `spawn_projectile.py`: spawns relative to drone pose via odom, deterministic per scenario ID.
- **W3-09** — `config/scenario_matrix.yaml`: 12 positives + 5 negatives with explicit train/test split; head-on / oblique / miss-distance axes covered.
- **W3-11–14** — `detector_node.py`: full pipeline — ROI range gate → voxel downsample → frame differencing → Euclidean clustering → centroid → `/threat/centroid` (PointStamped, `base_link`) + RViz marker.
- **W3-15 (code)** — `params/detector.yaml`: every threshold a ROS param, no magic numbers in code. *(Operating point still to be chosen — see remaining list.)*
- **W3-16** — `score_bags.py`: offline harness — replay bags, score TP/FP/FN, print table, exit 1 if recall < 95%.
- **W3-18 (code)** — `scripts/run_regression.sh`: one-command regression. *(Library not yet wired to recorded bags — see remaining list.)*
- **W3-20** — `launch/week3_perception.launch.py`: depth bridge + TF + detector in one command.

### Remaining — requires live Gazebo on the native Dell box

> **Run first:** `colcon build --symlink-install` from `~/huitzilin_ws` — the new `huitzilin_perception` package must be picked up before any launch works.
>
> **Full step-by-step procedure:** see `docs/week3_capture_runbook.md` — covers every row below plus the per-scenario `spawn_projectile` params and `.label.yaml` sidecar format that `score_bags.py` requires (not previously documented).

| Task | What |
|------|------|
| **W3-01** | `colcon build && ros2 launch huitzilin_sim week2_sitl.launch.py` — confirm a clean lap before touching perception. |
| **W3-04 verify** | Export `GZ_SIM_RESOURCE_PATH` to include the `models/` install path, launch with `huitzilin_runway.sdf`. |
| **W3-07** | RViz: add PointCloud2 on `/oak/points` (best-effort QoS), fly a patrol lap, confirm depth stream stable. |
| **W3-10** | Record labeled bags per scenario matrix: `ros2 bag record -s mcap /oak/depth /oak/points /clock /huitzilin/odom`. |
| **W3-15 (tune)** | Sweep `detector.yaml` thresholds against train-set bags; commit the chosen operating point. |
| **W3-17** | Run `./scripts/run_regression.sh /data/huitzilin_bags test` on held-out scenarios. |
| **W3-18 (wire)** | Point the harness at the recorded bag library so a detection-hurting change fails CI automatically. |
| **W3-21** | Clean-shell end-to-end acceptance run; record RViz + bag as evidence. |
| **W3-22** | Retro + journal update. |

## 5. What stays out of scope this week

- No real OAK-D hardware (Week 6).
- No Kalman filter / intercept prediction / dodge command (Week 4) — `/threat/intercept` and `/cmd/evade` stay provisional.
- No Pass-B airframe fidelity work (Week 7-8) — depth hangs on the stock `iris_with_standoffs` model as-is.

## 6. Evidence & doc trail

- Numbers go in `docs/week3_detection_evidence.md` (new, mirroring `docs/week2_patrol_evidence.md`).
- Contract updates (`/oak/*` provisional → active) land in `docs/architecture.md` in the **same commit** as the code that implements them (W3-19, pending).
- TF additions get verified against `docs/frames.md`.
- Retro + Week 4 handoff appended to `docs/JOURNAL.md` at W3-22.
