# Week 4 Dodge Battery Runbook (Dell)

Bring-up + scoring procedure for the Week 4 closed loop. Everything here
runs on the **native-Ubuntu Dell** (live Gazebo depth). World/SITL bring-up
is identical to Week 3 — see `docs/week3_capture_runbook.md` for the depth
world commands; only the launch + battery steps differ.

## 1. Bring-up (4 terminals)

```bash
# T1 — depth world (exact command per docs/week3_capture_runbook.md)
./scripts/week3_world.sh

# T2 — SITL (same fan-out as always)
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --add-param-file=$HOME/huitzilin_ws/src/huitzilin_sim/params/sitl_frame.parm \
  --out udp:127.0.0.1:14551 --out udp:127.0.0.1:14552 --out udp:127.0.0.1:14553

# T3 — Week 4 stack (Week 3 stack + evasion + ground-truth pose bridge)
ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true

# T4 — arm, take off, start patrol (service types matter — see CLAUDE.md)
ros2 service call /huitzilin/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /huitzilin/takeoff std_srvs/srv/Trigger
ros2 service call /huitzilin/start_patrol std_srvs/srv/SetBool '{data: true}'
```

Sanity checks before scoring:
- `ros2 topic hz /threat/centroid` shows traffic when something crosses the ROI.
- `ros2 topic echo /gz/dynamic_poses --once` lists the drone model. If its
  `child_frame_id` is not `iris_depth`, pass the real name to the battery:
  `EXTRA_ARGS="-p drone_model:=<name>" ./scripts/run_dodge_battery.sh`.
- `ros2 topic info /cmd/evade` shows 1 pub (evasion) + 1 sub (mav_bridge).

## 2. Single-shot smoke test

```bash
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=SMOKE -p speed_mps:=8.0 -p compensate_gravity:=true
```

Expected within ~1 s (sim): evasion logs `DODGE: miss=... tca=... latency=...`
and the drone visibly jinks sideways in Gazebo. The full cycle is **~2.3 s of
sim time**, three sequential phases (defaults from `params/evasion.yaml`):

| Phase | Param | Default | What you see |
|---|---|---|---|
| EVADING | `dodge_duration_s` | 1.0 s | velocity spike streams on `/cmd/evade`; `/payload/alarm` true |
| RECOVERING | `recover_hold_s` | 0.5 s | zero-velocity settle; alarm drops to false as this phase starts |
| HANDOFF | `patrol_handoff_s` | 0.8 s | `/cmd/evade` silent, then `dodge complete -> TRACKING (patrol resumed)` |

So `/payload/alarm` pulses true→false at ~1.0 s, but patrol only resumes at
~2.3 s. Judge both in **sim time** — at RTF ~0.33 that is ~7 s of wall clock.

If the drone does not move, check `ros2 topic echo /cmd/evade` first — if
commands stream but nothing moves, the bridge priority path is at fault; if no
commands, the trigger never fired (check `/threat/evade_event`).

## 3. Full battery

```bash
./scripts/run_dodge_battery.sh
```

- ~20 spawns (B01–B07 × repeats), each: spawn → 5 s sim watch window →
  ball removed → 6 s sim settle. Budget ≥ 15 min wall at ~0.33 RTF.
- Output: table + per-combo aggregates → `/tmp/week4_battery.txt`,
  rows → `/tmp/week4_battery.csv`.
- **No hard success gate** (2026-07-16 decision): exit ≠ 0 only for harness
  errors (no odom / no pose stream / spawn failures).

## 4. Parameter sweep

```bash
./scripts/run_dodge_battery.sh sweep
```

Grids `dodge_speed_mps × trigger_horizon_s` (3×2) over B02/B03/B06 via
`ros2 param set` on the live evasion node — no restarts. Results →
`/tmp/week4_sweep.{txt,csv}`. Pick the winning combo, write it into
`params/evasion.yaml`, and re-run the full battery once to confirm.
The battery restores the baseline evasion params after the sweep; if a
sweep aborts partway, restart T3 before running the confirmation battery.

## 5. What "done" looks like (Week 4 DoD)

- Battery report shows dodges firing on hit-intent runs with measured
  latency vs the 150 ms budget, and no false dodge on B07.
- `min_dist_m` on successful dodges exceeds `hit_radius_m` (0.30 m).
- Record the numbers + chosen params in `docs/JOURNAL.md` (Week 4 entry),
  then delete `docs/WEEK4_PLAN.md` (closed-plan convention).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Dodge fires, drone doesn't move | patrol still fighting the spike → check evasion log called `/huitzilin/start_patrol false`; verify bridge evade priority |
| No dodge on obvious hits | `/threat/centroid` silent (detector issue — Week 3 runbook) or `min_track_updates` unreached for fast balls at 15 Hz → try `trigger_horizon_s` 2.0 |
| `ball ... never seen on /gz/dynamic_poses` | wrong `world_name`/`drone_model` param, or pose bridge disabled (`gz_pose_bridge:=false`) |
| Balls pile up on the runway | `gz_remove` failing — check T3 log; remove manually: `gz service -s /world/huitzilin_runway/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "<ball>", type: MODEL'` |
| Ball spawns but just drops (no throw) | `gz service --req` parses protobuf **text** format only — a JSON body is rejected with empty stdout and exit code 0 (silent success). Also: `gz.msgs.EntityFactory` has **no velocity field** in Harmonic, so the throw is a separate one-physics-step `gz.msgs.EntityWrench` on `/world/<world>/wrench`, applied by `gz-sim-apply-link-wrench-system` (the world must load it). **Do NOT pause the world to bridge the gap** — with SITL flying, ArduPilot lurches on resume and the frames after it flood the detector's egomotion diff (measured 2026-07-26: `fg=34881` > `fg_max_points`, then `raw` 115k→153k as the tilted frustum fills with ground). Instead the ball's link ships with `<gravity>false</gravity>` so it hangs motionless until thrown, and the impulse plus a persistent `-mass*g` gravity-restore wrench are published together from warm ROS publishers (`spawn_projectile.WrenchThrower` via `wrench_bridge` in `week4_evasion.launch.py`) so both land on the same physics step. A throw that flies dead straight means the gravity wrench was dropped; one that dives at ~2g means a **duplicate** wrench bridge is delivering it twice. `spawn_projectile.gz_spawn` does all of this; to reproduce by hand (SITL down only, since this pauses): `gz service -s /world/huitzilin_runway/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'`, then `gz service -s /world/huitzilin_runway/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf_filename: "projectile", name: "<ball>", pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {x: 0, y: 0, z: 0, w: 1}}'`, then `gz topic -t /world/huitzilin_runway/wrench -m gz.msgs.EntityWrench -p 'entity: {name: "<ball>::link", type: LINK}, wrench: {force: {x: 0.000000, y: 0.000000, z: 1200.000000}, torque: {x: 0, y: 0, z: 0}}'` (force = mass 0.150 kg × v ÷ step 0.001 s), then unpause with `--req 'pause: false'`. |
| Gazebo aborts: `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide()` | A **leftover projectile rolled out of ODE's hash space** and took the world with it. The ball has link gravity off and no rolling resistance, so an un-removed one rolls until its AABB no longer fits ODE's int quantization (leftovers measured at x=-42 m and x=-207 m; the 2026-07-26 world died this way after ~9 idle hours). The battery calls `gz_remove` after every run and `spawn_projectile` now self-removes after `lifetime_s` (default 20 s wall); if you throw by hand, remove the ball yourself. Recovery is a full world restart — SITL loses its FDM link too. Check for strays with `gz model --list \| grep projectile` before leaving the sim idle. |
| Everything slow / timing weird | judging by wall clock — all windows are sim-time; RTF ~0.33 is expected |
