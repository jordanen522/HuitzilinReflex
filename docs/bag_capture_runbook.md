# Bag capture & regression runbook (Dell)

Owns: how to (re)capture the labeled scenario library and run the detector regression.
Native-Ubuntu Dell only. All timing in **sim time** (`/clock`, message stamps).

## Bring-up

4 terminals, each sourced (`source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash`):

```bash
cd ~/huitzilin_ws && colcon build --symlink-install   # once per checkout

# T1 — perception world (exports GZ_SIM_RESOURCE_PATH itself)
./scripts/week3_world.sh
# T2 — SITL: the standard fan-out, see CLAUDE.md
# T3 — perception stack
ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true
# T4 — arm → takeoff → patrol (odom-polled climb)
./scripts/week3_flyup.sh
```

Sanity gate before recording: `/oak/points` stable at 15 Hz sim (66 ms stamp spacing, no
dropouts) at rest and in flight, and the detector logs
`differencing frame -> fixed (odom TF available)`. A fallback to camera-frame mode means
the odom attitude is broken — fix that before capturing anything.

## Capture loop — one bag per scenario

Scenario IDs and parameters come from `src/huitzilin_perception/config/scenario_matrix.yaml`
(positives S01–S12, negatives N01–N05; **S11/S12/N05 are the held-out test split**).

```bash
./scripts/capture_scenario.sh <ID>   # Ctrl-C ~8–10 s sim after spawn
```

Manual equivalent:

```bash
ros2 bag record -s mcap -o /data/huitzilin_bags/week3_<ID> \
  /oak/depth /oak/points /clock /huitzilin/odom /threat/centroid &
BAG_PID=$!
ros2 run huitzilin_perception spawn_projectile --ros-args \
  -p scenario_id:=<ID> -p speed_mps:=… -p approach_angle_deg:=… \
  -p miss_distance_m:=… -p offset_forward_m:=…      # skip for N01/N02/N05
# wait time_to_closest_s + 4.0 s sim, then:
kill -SIGINT $BAG_PID
```

Special cases:
- **N02** — no spawn; time the window to span a patrol waypoint turn.
- **N04** — needs ~10 m *vertical* miss. `spawn_projectile.py` has
  `offset_vertical_m` (the matrix row sets `-10.0`) and `capture_scenario.sh` passes it
  automatically — no manual `gz service … EntityFactory` spawn, and no editing the
  parameter out. What you *do* still have to do by hand is fly high enough: set
  `takeoff_alt_m: 12.0` in `bridge.yaml` and restore 2.0 afterwards, or the spawn lands
  underground and `capture_scenario.sh` aborts.
- **N05** — 60 s of clean patrol, no spawn.

Every bag needs a `/data/huitzilin_bags/week3_<ID>.label.yaml` sidecar.
**`capture_scenario.sh` writes it for you** — hand-write one only when you captured
the bag manually with the block above. Fields map 1:1 from the matrix row:

```yaml
scenario_id: S02
label: positive            # positive | negative
closest_approach_m: 0.0
time_to_closest_s: 0.75
detection_window_s: 4.0    # widen for slow scenarios
```

There is no `bag_start_sim_t` field. Nothing writes one and nothing reads one; a
sidecar carrying it is not more complete, just inconsistent with the tooling.

All 17 sidecars must exist. `score_bags.py` fails explicitly on a missing *bag*, but a
missing *sidecar* silently drops the scenario from the recall math.

## Capturing the T-set (T01–T14, tuning split)

Same bring-up as above; scenario IDs come from `scenario_matrix.yaml`'s T01–T14 rows (see
the SPLITS block at the bottom of that file). All 14 were captured non-interactively over
ssh (no TTY) — the four points below are load-bearing for anyone repeating this without a
controlling terminal.

### The SIGINT-over-ssh pattern (and the two ways it silently breaks)

`capture_scenario.sh` ends in `exec ros2 bag record`, so the PID you background *becomes*
the recorder. Signal that PID directly with `kill -INT`, never a pattern match. Two failure
modes were found and fixed this session, both silent (no error, no crash — just a recorder
that never receives the signal, or a signal sent to the wrong PID):

1. **A non-interactive, job-control-off bash sets SIGINT/SIGQUIT to `SIG_IGN` for `&`
   background jobs** (POSIX/bash rule for asynchronous commands), and that ignored
   disposition survives `exec()` into `ros2 bag record`. `kill -INT $PID` then does
   nothing — confirmed via `/proc/$PID/status`'s `SigIgn` mask. `nohup CMD &` does **not**
   fix this; it only adds its own `SIGHUP`-ignore on top, SIGINT/SIGQUIT stay ignored.
   Neither does `trap - INT` inside the child — bash refuses to un-ignore a signal that was
   already ignored at shell entry. **Fix: `set -m` (job control) before backgrounding.**
   With job control active, bash does not apply the auto-ignore rule to async commands.
2. **`setsid CMD` forks internally whenever the caller is already a process group
   leader** (required, because `setsid()` fails otherwise) — and `set -m` makes every
   backgrounded job its own group leader. Stacking `setsid` on top of `set -m` therefore
   decouples `$!` (setsid's own short-lived PID) from the PID `ros2 bag record` actually
   ends up running as; `kill -INT $!` then signals a process that's already gone.
   **Fix: don't use `setsid`.** `set -m` already gives the job its own process group; a
   synchronous, foreground ssh capture doesn't need a new *session* on top of that.

Net: `set -m`, no `setsid`, no `nohup` — a plain `CMD > log 2>&1 &` is enough once job
control is on, and `capture_scenario.sh`'s own `exec ros2 bag record` preserves `$!` as the
recorder's real PID. To verify live: `kill -INT $PID; sleep 1; grep SigIgn /proc/$PID/status`
should show the SIGINT bit clear before you rely on it for anything.

### The wrench-bridge gap — `week3_perception.launch.py` needs it added by hand

`spawn_projectile.py`'s fast throw path (`WrenchThrower`: one-step launch impulse + gravity
restore, landed on the same physics step) needs `/world/<world>/wrench` and
`/world/<world>/wrench/persistent` bridged ROS→gz. **Only `week4_evasion.launch.py` brings
that bridge up** (its `_wrench_bridge` OpaqueFunction) — `week3_perception.launch.py`, the
launch file this runbook and the T-set capture brief both prescribe, does not. Without it,
every throw falls back to the `gz` CLI path, which:

- costs **~15–20 s of wall time** waiting for a wrench-bridge connection that will never
  arrive, then a further ~2 s for the CLI throw itself (logged as `wrench bridge never
  connected`, then `Spawn OK` from the CLI fallback);
- is not intermittent — it happened on **11/11** spawn scenarios captured before this was
  diagnosed, with the CLI throw's actual impulse landing, in every case, **after** a
  fixed-duration recording window had already stopped (`Recording stopped` preceded `Spawn
  OK` by 7–10 s in all 11 logs, checked directly). Every one of those first-pass bags
  contained a ball sitting motionless at its spawn point for the whole recording — not the
  scenario it was supposed to capture, and for the T-set negatives specifically, a static
  new object sitting in frame while patrol flies past it is a capture-artifact false
  positive, not a measurement of whether the detector suppresses the labeled scenario.

**Fix, applied for this capture session:** bring up the same bridge `week4_evasion` uses,
as a standalone extra node alongside `week3_perception.launch.py` — no launch-file edits,
no detector/matrix changes:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/huitzilin_runway/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench" \
  "/world/huitzilin_runway/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench"
```

Once it's up, `spawn_projectile.py`'s own `WrenchThrower.wait_for_bridge()` finds it and
uses the fast path automatically (logged as `thrown via warm bridge`) — no argument or env
var to pass. **Bring this up before the first spawn scenario**, not after discovering the
slow path the hard way.

Even with the fast path, do not assume `SPAWN_LEAD` (3 s, fixed inside
`capture_scenario.sh`) is when the ball actually moves. **Poll the capture log for the
literal string `Spawn OK` before starting any post-throw countdown**, rather than trusting
a fixed formula — the fast path normally lands a couple of seconds past `SPAWN_LEAD`, but
was seen to miss a generous 40 s poll window entirely (T09, first retry) and need a second
retry at 60 s. A scenario that needs multiple tries at this stage is a capture-timing
problem, not a scenario problem — don't read anything into it about the geometry.

### T13 needs altitude, and it must come back down

T13 (`offset_vertical_m: -7.5`) needs the drone flying ≥ 8.0 m before the throw, or
`capture_scenario.sh` aborts (`spawn would be at z=... < 0.5 m`). Same shape as N04's
existing procedure, with three things worth spelling out because they cost real time here:

- **`mav_bridge_node` reads `takeoff_alt_m` once at construction**, into a plain attribute
  — there is no parameter callback, so `ros2 param set .../takeoff_alt_m` is accepted and
  silently ignored by the running node. Changing it requires restarting the node (here: the
  whole T3 stack — `mav_bridge` and `patrol` are both inside `week3_perception.launch.py`'s
  `with_patrol:=true` include, and there's no way to restart one without the other).
  Gazebo + SITL + the wrench bridge stay up across this; only
  `ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true` needs to
  come down and back up.
- **`/huitzilin/takeoff` calls `MAV_CMD_NAV_TAKEOFF`, which ArduCopter only accepts when
  landed.** Don't try to "climb to 9 m" by re-triggering takeoff while already airborne —
  land first (`ros2 param set /mav_bridge mode LAND` + `/huitzilin/set_mode`), *then* edit
  `bridge.yaml`, restart T3, re-arm, and take off into the new altitude.
- **ArduCopter refuses to arm while the current mode is LAND** (`Arm: Land mode not
  armable` — visible in the SITL/mavproxy console log, *not* in the ROS service response,
  which just times out after 10 s with a generic "not confirmed" message that gives no
  hint the real cause is the flight mode). Switch back to GUIDED
  (`ros2 param set /mav_bridge mode GUIDED` + `/huitzilin/set_mode`) before the next arm
  call.
- Do not start patrol between takeoff and the T13 capture — patrol's waypoints are fixed at
  `d: -2.0` NED and will fly the drone straight back down to 2 m the moment it starts.
- **Restore `takeoff_alt_m: 2.0` in the same session**, then repeat land → edit → restart
  T3 → arm → takeoff → start-patrol before capturing anything else.
  `git diff -- src/huitzilin_sim/params/bridge.yaml` must be empty before committing.

### T11 needs a commanded climb, not just an idle hover

T11 (vertical-egomotion negative) has no spawn — its notes call for "a window spanning a
commanded ~2 m climb and descent." Patrol's own MAVLink connection sends its own position
setpoints at 10 Hz and will fight anything else back to its fixed altitude, so **pause
patrol first** (`/huitzilin/start_patrol` with `data: false` — ArduCopter then just holds
GUIDED position; it does not fail-safe on its own). With patrol paused, `/huitzilin/cmd_vel`
(FLU, so `linear.z` positive is up) is free to command the climb and descent directly:

```bash
ros2 topic pub -r 10 /huitzilin/cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'   # climb, ~2 s
ros2 topic pub -r 10 /huitzilin/cmd_vel geometry_msgs/msg/Twist '{linear: {z: -1.0}}'  # descend, ~2 s
```

`cmd_timeout_s` (0.7 s in `bridge.yaml`) zero-holds automatically once the `topic pub`
process ends — no explicit stop command needed. Resume patrol (`data: true`) once the bag
is closed.

### Hz gate — judge every bag, not just the throw

`/oak/points` under 15 Hz sim is normal on this Dell under depth rendering (see CLAUDE.md),
but a bag needs enough density to prove the scenario, not just to exist. Compute the actual
rate from `ros2 bag info` (`/oak/points` message count ÷ bag duration) for every capture —
a live `ros2 topic hz` sample taken before or after the recording can read very differently
from what the bag itself contains, and isn't a substitute. T01–T14 all landed at
10.2–13.4 Hz this session; treat anything under ~10 Hz as a failed capture and re-fly it,
most urgently the short-`ttc` scenarios (T03/T07/T08), which have the least frame budget
to lose.

## Tuning + regression

```bash
./scripts/run_regression.sh /data/huitzilin_bags train   # tune against this
./scripts/run_regression.sh /data/huitzilin_bags test    # score once; never tune on it
```

Knobs in `src/huitzilin_perception/params/detector.yaml`:

| Symptom | Knob |
|---|---|
| Missing fast/oblique positives | lower `diff_threshold_m`, widen `roi_half_angle_deg` |
| Missing wide-miss positives | raise `roi_max_range_m`, lower `cluster_min_points` |
| False triggers on patrol turns | confirm egomotion compensation active, raise `diff_threshold_m` slightly |
| False triggers on clutter | raise `min_publish_score` or `cluster_min_points` |

The current operating point is tuned and the held-out test split passes at recall 100%.
**The library is saturated** — it cannot referee further threshold changes, and every
trigger-side lever has already been measured as a null (see `CLAUDE.md`). Commit
`detector.yaml` changes with a message naming the FN/FP they fixed.

## Open detector items

- **S08 false negative** (14 m/s near-miss, train split) — never root-caused. Two blind
  threshold attempts regressed other scenarios without fixing it. Any re-tune must start
  from a single-bag `debug_funnel:=true` trace on S08, not threshold guessing.
- **N02 / N03 / N05 false positives** (patrol-turn and background-clutter triggers) —
  they hurt precision but don't fail the recall gate. Uninvestigated.
