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
ssh (no TTY) — the points below are load-bearing for anyone repeating this without a
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

### Spawn-OK log-polling is what makes a throw capture land — not a fixed wait

Do not derive the post-spawn wait from a fixed formula (e.g. `3 + time_to_closest_s + 9`
sim-seconds off `SPAWN_LEAD`). **On this launch file, every throw goes through
`spawn_projectile.py`'s `gz` CLI path** (see the next section for why that's correct, not a
bug), which costs on the order of 15–20 s of *wall* time between the spawn command firing
and the projectile actually getting its impulse. That shows up in the log as
`wrench bridge never connected — falling back to the CLI throw`, followed a few seconds
later by `Spawn OK`. **That warning line is expected and benign on `week3_perception.launch.py`
— it fires on essentially every throw captured through this launch file, including every
correctly-captured bag in the original 17-bag library, and is not itself a symptom of
anything broken.** The bug is trusting a fixed timer across that gap, not the gap existing.

The fix — and the one that actually rescued T01–T10 of this batch — is to poll the capture
log for the literal string `Spawn OK` before starting any post-throw countdown:

```bash
until grep -q 'Spawn OK' "$LOG"; do sleep 1; done
sleep "$((TTC_S + 9))"        # THEN start the post-throw buffer
kill -INT "$PID"
```

Checked directly against every first-pass log before this fix was applied: `Recording
stopped` preceded `Spawn OK` by 7–10 s in **11 of 11** spawn scenarios captured under a
fixed-timing wait — every one of them recorded a ball sitting motionless at its spawn point
for the whole bag, not the scenario it was supposed to capture, and (for the T-set
negatives specifically) a static new object sitting in frame while patrol flies past it is
a capture-artifact false positive, not a measurement of whether the detector suppresses the
labeled scenario. Re-capturing with the poll above — same CLI throw path, just an honest
wait — is what fixed all 11; **no bridge was involved in that fix** (see next section).

T09 needed at least one re-capture for this same spawn-timing reason during this batch.
The specific claim that its first retry missed a 40 s poll window and a second retry
succeeded at 60 s **cannot be verified after the fact** — the driver logs to a fixed
per-scenario path (`/tmp/t4_capture_T09.log`) and each retry overwrites the previous
attempt's log, so the evidence for that exact two-step timeline no longer exists. Treat T09
as "needed at least one re-capture for a spawn-timing reason," not as a confirmed sequence,
and budget a generous poll timeout (60 s+) up front rather than tuning it per-scenario.

### The CLI throw path is canonical for this bag library — do not bridge wrench topics into `week3_perception.launch.py`

It's tempting to read `wrench bridge never connected` as a missing feature and wire
`week4_evasion.launch.py`'s wrench bridge (`ros_gz_bridge parameter_bridge` on
`/world/<world>/wrench` and `/world/<world>/wrench/persistent`) into
`week3_perception.launch.py` so throws use the fast, same-physics-step `WrenchThrower` path
instead of the CLI fallback. **Don't — this was considered and ruled against.** The
original 17 bags (S01–S12, N01–N05) were captured 2026-07-15, **eleven days before
`WrenchThrower` existed** (`37e0d34`, 2026-07-26); they went through the CLI-only path by
construction, not by omission. T01–T10 of this batch went through the same CLI path (see
above). **The CLI throw is what the entire 31-bag library currently shares, and that shared
mechanism is exactly what keeps the tune and test splits comparable to each other and to
the historical train/test split.** Bridging wrench topics into the capture launch file
would make every *future* capture use a different throw mechanism (a true same-step impulse
vs. a two-call CLI sequence that restores gravity ~0.25 s late and flattens the trajectory
— see `spawn_projectile.py`'s own docstrings) than every bag currently in the library,
trading one documented, benign log line for a fresh, silent comparability hazard.
`week3_perception.launch.py` deliberately does not bridge the wrench topics — leave it that
way, and read `wrench bridge never connected` as confirmation the canonical path is being
used, not as something to fix.

**Exception, disclosed:** T12 and T13 in this batch *did* go through the fast bridge path
(`thrown via warm bridge` in their logs). The bridge was brought up standalone — outside
the launch file, as a bare extra `ros2 run` process, never wired into
`week3_perception.launch.py` itself — partway through this session, to shorten a long
pre-throw static-object window that was inflating T12's false-positive count (see below).
It was left running for T13. **T12 and T13 are therefore the only 2 of 31 bags in the whole
library captured via a different throw mechanism than every other bag**, including the
other 12 in this same batch. Both are otherwise valid captures and were not re-flown
CLI-only, since re-flying T12 costs a live patrol-motion window and T13 costs a full
altitude excursion — but Task 5 should treat T12/T13's throw kinematics (gravity-restore
timing in particular) as not directly comparable to any other single bag in the library,
and should not extend this exception to future captures.

### T13 needs altitude, and it must come back down

T13 (`offset_vertical_m: -7.5`) needs the drone flying ≥ 8.0 m before the throw, or
`capture_scenario.sh` aborts (`spawn would be at z=... < 0.5 m`). Same shape as N04's
existing procedure, with four things worth spelling out because they cost real time here:

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

T13's own post-throw window in this batch came out short (~2.2 s observed, against a ~10 s
buffer the capture driver was asked for) because it **skipped the Spawn-OK-anchored polling
described above.** T13's extra pre-recording step (the vertical-offset/altitude check) adds
latency before `capture_scenario.sh` writes its `spawn=yes` label line; the driver's short,
fixed pre-poll delay raced that write and lost, so the driver's branch check found no
`spawn=yes` yet, fell through to a flat un-anchored sleep instead of polling for `Spawn OK`,
and sent SIGINT on that flat timer instead. The throw itself is still confirmed genuine
(ball spawned at the expected z = drone altitude − 7.5 m, thrown via the warm bridge, well
before the recording stopped, zero false triggers), so the bag is usable — but the short
window is a capture-driver race condition, not an unexplained anomaly. Give the pre-poll
delay enough margin for scenario-specific startup latency (T13's altitude check in
particular), or check for the label line rather than sleeping a fixed amount before the
first `Spawn OK` check.

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

### `/threat/centroid` counts are not a detection signal — read this before Task 5 retunes

Every T-bag in this batch shows substantial `/threat/centroid` traffic, including the
negatives — several rows show dozens to ~100 messages, not the near-zero expected of a
working detector on a held-out negative or during a positive's dead time. On direct
cross-reference against the point clouds (checked on T03 and T08), the large majority of
this traffic — on the order of 98% — is ambient noise, not the labeled throw, and this
pattern is present across **every** T-bag, not only the highest-count rows. **A raw
`/threat/centroid` message count is not a detection indicator by itself and must not be
read as ground truth** for whether a scenario was "detected" — score against the labeled
window and actual cluster geometry, not the message count.

More importantly: **the false-positive rate correlates with patrol motion, across the whole
tune set, not as an isolated fluke.** T14 (~100 messages, patrol running throughout, zero
spawns, 45 s) against T11 (1 message, patrol *paused* for the whole capture, zero spawns,
~14 s) is the cleanest read of this — same "nothing thrown" condition, the only structural
difference is whether the drone is patrolling. **This is a structural, patrol-motion-
correlated false-positive property of the detector as currently tuned, and it needs to be
root-caused *before* Task 5 fits new thresholds against these bags — not discovered
after.** Heavy, motion-correlated FPs on the negatives will pull any recall/precision
tradeoff in a specific, hard-to-diagnose-after-the-fact direction if a retune is fit blind
to the cause. This is a handoff for Task 5, not a fix made here — `detector.yaml` was
explicitly out of scope for the capture task.

### Hz gate — judge every bag, not just the throw

`/oak/points` under 15 Hz sim is normal on this Dell under depth rendering (see CLAUDE.md),
but a bag needs enough density to prove the scenario, not just to exist. Compute the actual
rate from `ros2 bag info` (`/oak/points` message count ÷ bag duration) for every capture —
a live `ros2 topic hz` sample taken before or after the recording can read very differently
from what the bag itself contains, and isn't a substitute. T01–T14 all landed at
11.5–13.4 Hz this session; treat anything under ~10 Hz as a failed capture and re-fly it,
most urgently the short-`ttc` scenarios (T03/T07/T08), which have the least frame budget to
lose. Hz measures whether a bag has enough data to *contain* the scenario — it says nothing
about `/threat/centroid` quality; see the note above before drawing any detection
conclusion from a bag that passes this gate.

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
