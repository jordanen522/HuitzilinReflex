# Dodge battery runbook (Dell)

Owns: bring-up, smoke test, battery, and sweep procedure for the evasion loop, plus how
to read the miss decomposition. Native-Ubuntu Dell only (live Gazebo depth).
Results and the capability envelope live in `CLAUDE.md`.

## 1. Bring-up (4 terminals)

```bash
# T1 — depth world
./scripts/week3_world.sh
# T2 — SITL: the standard fan-out, see CLAUDE.md
# T3 — evasion stack (perception + evasion + ground-truth pose bridge)
ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true
# T4 — arm → takeoff → patrol, per CLAUDE.md
```

Sanity checks before scoring:
- `ros2 topic hz /threat/centroid` shows traffic when something crosses the ROI.
- `ros2 topic echo /gz/dynamic_poses --once` lists the drone model. If its
  `child_frame_id` is not `iris_depth`, pass the real name:
  `EXTRA_ARGS="-p drone_model:=<name>" ./scripts/run_dodge_battery.sh`.
- `ros2 topic info /cmd/evade` shows 1 pub (evasion) + 1 sub (mav_bridge).

## 2. Single-shot smoke test

```bash
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=SMOKE -p speed_mps:=8.0 -p compensate_gravity:=true
```

Expected within ~1 s sim: evasion logs `DODGE: miss=... tca=... latency=...` and the
drone visibly jinks. The full cycle is **~2.3 s sim**, three sequential phases (defaults
from `params/evasion.yaml`):

| Phase | Param | Default | What you see |
|---|---|---|---|
| EVADING | `dodge_duration_s` | 1.0 s | velocity spike on `/cmd/evade`; `/payload/alarm` true |
| RECOVERING | `recover_hold_s` | 0.5 s | zero-velocity settle; alarm drops false as this phase starts |
| HANDOFF | `patrol_handoff_s` | 0.8 s | `/cmd/evade` silent, then `dodge complete -> TRACKING (patrol resumed)` |

So `/payload/alarm` pulses true→false at ~1.0 s but patrol only resumes at ~2.3 s. Judge
both in sim time — at RTF ~0.33 that is ~7 s wall.

If the drone does not move, check `ros2 topic echo /cmd/evade`: commands streaming but no
motion means the bridge priority path; no commands means the trigger never fired (check
`/threat/evade_event`).

## 3. Full battery

```bash
./scripts/run_dodge_battery.sh
```

- ~20 spawns (B01–B07 × repeats), each: spawn → 5 s sim watch → ball removed → 6 s sim
  settle. Budget ≥ 15 min wall at ~0.33 RTF.
- Output: table + per-combo aggregates → `/tmp/week4_battery.txt`, rows → `.csv`.
  **The file is written only at the very end** — check its mtime and
  `pgrep -fc "dodge_batt[e]ry"` before summarising, or you will read the previous run.
- **No hard success gate**: exit ≠ 0 only for harness errors (no odom / no pose stream /
  spawn failures).
- The report prints a blended on-target rate. Do not quote it — split by ball speed
  (`CLAUDE.md`); the blend hides both halves of the envelope.

## 4. Parameter sweep

```bash
./scripts/run_dodge_battery.sh sweep
```

Grids `min_track_updates × dodge_speed_mps` (2×2, read from `config/week4_sweep.yaml` —
edit that file, not this sentence, to change the grid) over B02/B03/B06 via
`ros2 param set`
on the live evasion node — no restarts. Results → `/tmp/week4_sweep.{txt,csv}`. Write the
winning combo into `params/evasion.yaml` and re-run the full battery to confirm.

**Both shipped axes are already measured nulls.** Running the grid as it stands
re-derives a known answer — the baseline won on every column. It is kept as the
worked example of the sweep format. Change the axes before spending a session on it,
and check the null list in `CLAUDE.md` first. The
battery restores baseline evasion params afterwards; if a sweep aborts partway, restart
T3 before the confirmation battery.

## 5. Reading the miss decomposition

The battery reports two signed triples per combo, over **no-dodge runs only** (a dodge
moves the drone, so the miss stops measuring the aim). Use these instead of `min_dist_m`
when diagnosing *why* a throw missed — the scalar is a perpendicular distance and cannot
tell a mistimed lead from a sideways drift.

| line | frame | + means | a bias here points at |
|---|---|---|---|
| `miss along` | ball's heading | ball went past the drone | lead timing / target-speed estimate |
| `miss cross` | ball's left | ball passed left of the drone | heading error — target turned, or lead direction wrong |
| `miss vert` | world z | ball passed above the drone | gravity compensation (`compensate_gravity`, `offset_vertical_m`) |
| `lead along` | drone's heading | aimed ahead of where it got to | over-led: too much lead time (`spawn_latency_s`) or over-estimated speed |
| `lead cross` | drone's left | aimed left of its own track | drone turned inside the flight — a throw-window gate escape |

`lead` is absent in `hover_mode`: a stationary target has no heading frame and the lead is
exact by construction. `miss along` is near zero at a true closest approach by definition,
so a large value means the pose samples bracket the approach coarsely — not that the lead
is healthy.

Two selection biases to respect: `aim_err` is biased toward no-dodge runs, and
`first_det_range_m` is contaminated by the false-positive stream (~1.4/s).

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Dodge fires, drone doesn't move | patrol still fighting the spike → check the evasion log called `/huitzilin/start_patrol false`; verify bridge evade priority |
| No dodge on obvious hits | `/threat/centroid` silent (detector — see `bag_capture_runbook.md`) or `min_track_updates` unreached for fast balls at 15 Hz |
| `ball ... never seen on /gz/dynamic_poses` | wrong `world_name`/`drone_model`, or pose bridge disabled (`gz_pose_bridge:=false`) |
| Balls pile up on the runway | `gz_remove` failing — check T3 log; remove manually: `gz service -s /world/huitzilin_runway/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "<ball>", type: MODEL'` |
| Ball spawns but just drops (no throw) | see **Throw mechanism** below |
| Gazebo aborts with an ODE `aabbBound` assertion | see **ODE abort** below |
| Everything slow / timing weird | judging by wall clock — all windows are sim-time; RTF ~0.33 is expected |

**Throw mechanism.** `gz service --req` parses protobuf **text** only — a JSON body is
rejected with empty stdout and exit code 0, which reads as success. `gz.msgs.EntityFactory`
has no velocity field in Harmonic, so the throw is a separate one-physics-step
`gz.msgs.EntityWrench` on `/world/<world>/wrench` (needs `gz-sim-apply-link-wrench-system`
loaded in the world). **Never pause the world to bridge the gap** — with SITL flying,
ArduPilot lurches on resume and the frames after it flood the detector's egomotion diff.
Instead the ball's link ships `<gravity>false</gravity>` so it hangs motionless until
thrown, and the impulse plus a persistent `-mass*g` gravity-restore wrench are published
together from warm ROS publishers (`spawn_projectile.WrenchThrower` via `wrench_bridge` in
`week4_evasion.launch.py`) so both land on the same physics step. A throw that flies dead
straight means the gravity wrench was dropped; one that dives at ~2 g means a **duplicate**
wrench bridge is delivering it twice. `spawn_projectile.gz_spawn` does all of this.

**ODE abort.** A leftover projectile rolled out of ODE's hash space and took the world
with it: the ball has link gravity off and no rolling resistance, so an un-removed one
rolls until its AABB no longer fits ODE's int quantization. The battery calls `gz_remove`
after every run and `spawn_projectile` self-removes after `lifetime_s` (default 20 s
wall); if you throw by hand, remove the ball yourself. Recovery is a full world restart —
SITL loses its FDM link too. Check for strays with `gz model --list | grep projectile`
before leaving the sim idle.
