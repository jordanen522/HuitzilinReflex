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

Expected within ~1 s (sim): evasion logs `DODGE: miss=... tca=... latency=...`,
the drone visibly jinks sideways in Gazebo, patrol pauses and resumes
~1.5 s later, and `/payload/alarm` pulses true→false. If the drone does not
move, check `ros2 topic echo /cmd/evade` first — if commands stream but
nothing moves, the bridge priority path is at fault; if no commands, the
trigger never fired (check `/threat/evade_event`).

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
| Balls pile up on the runway | `gz_remove` failing — check T3 log; remove manually: `gz service -s /world/huitzilin_runway/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req '{"name": "<ball>", "type": 2}'` |
| Everything slow / timing weird | judging by wall clock — all windows are sim-time; RTF ~0.33 is expected |
