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
the SPLITS block at the bottom of that file). The T-set was captured non-interactively over
ssh with no TTY; the notes below matter to anyone repeating that without a controlling
terminal.

### Stopping the recorder over ssh

`capture_scenario.sh` ends in `exec ros2 bag record`, so the PID you background *becomes*
the recorder. Signal that PID directly with `kill -INT`, never a pattern match, and enable
job control with `set -m` before backgrounding. Without job control, a non-interactive bash
sets SIGINT/SIGQUIT to `SIG_IGN` for `&` jobs (the POSIX rule for asynchronous commands),
and that disposition survives `exec()` into `ros2 bag record`, so `kill -INT $PID` does
nothing. `nohup` does not fix it — it only adds a `SIGHUP` ignore on top — and neither does
`trap - INT` in the child, because bash will not un-ignore a signal that was already
ignored at shell entry.

Do not add `setsid` on top. It forks internally whenever the caller is already a process
group leader, which `set -m` guarantees, so `$!` becomes setsid's own short-lived PID
rather than the recorder's and `kill -INT $!` signals a process that is already gone.

A plain `CMD > log 2>&1 &` is enough once job control is on. To verify a recorder can
actually be signalled: `kill -INT $PID; sleep 1; grep SigIgn /proc/$PID/status` — the
SIGINT bit must be clear.

### Anchor the post-spawn wait on `Spawn OK`, not a fixed timer

Every throw on `week3_perception.launch.py` goes through `spawn_projectile.py`'s `gz` CLI
path (see the next section for why that is correct rather than a bug), which costs 15–20 s
of *wall* time between the spawn command firing and the projectile getting its impulse. The
log shows `wrench bridge never connected — falling back to the CLI throw`, then `Spawn OK`
a few seconds later. That warning is expected and benign on this launch file: it fires on
essentially every throw captured through it, including every correctly-captured bag in the
original 17-bag library.

A fixed post-spawn wait (for example `3 + time_to_closest_s + 9` sim-seconds off
`SPAWN_LEAD`) races that gap and loses. Under fixed timing, `Recording stopped` preceded
`Spawn OK` by 7–10 s in **11 of 11** spawn scenarios, each recording a ball sitting
motionless at its spawn point instead of the labeled scenario; on a negative, a static new
object in frame while patrol flies past is a capture artifact, not detector behaviour. Poll
the capture log instead:

```bash
until grep -q 'Spawn OK' "$LOG"; do sleep 1; done
sleep "$((TTC_S + 9))"        # THEN start the post-throw buffer
kill -INT "$PID"
```

Budget a generous poll timeout (60 s+) up front rather than tuning it per scenario. T09
needed at least one re-capture for this spawn-timing reason.

### The CLI throw path is canonical for this bag library

Do not wire `week4_evasion.launch.py`'s wrench bridge (`ros_gz_bridge parameter_bridge` on
`/world/<world>/wrench` and `/world/<world>/wrench/persistent`) into
`week3_perception.launch.py` to get the fast, same-physics-step `WrenchThrower` path. The
original 17 bags (S01–S12, N01–N05) were captured 2026-07-15, eleven days before
`WrenchThrower` existed (`37e0d34`, 2026-07-26), so they use the CLI path by construction;
T01–T10 use it too. The CLI throw is what the whole 31-bag library shares, and that shared
mechanism is what keeps the tune and test splits comparable to each other and to the
historical train/test split. Bridging the wrench topics would give every future capture a
different throw mechanism — a true same-step impulse instead of a two-call CLI sequence
that restores gravity ~0.25 s late and flattens the trajectory (see `spawn_projectile.py`'s
docstrings). Read `wrench bridge never connected` as confirmation the canonical path is in
use.

T12 and T13 are the exception: both went through the fast bridge (`thrown via warm bridge`
in their logs), brought up standalone as a bare `ros2 run` process outside the launch file
to shorten a long pre-throw static-object window that was inflating T12's false-positive
count. They are the only 2 of 31 bags in the library captured through a different throw
mechanism. Both are otherwise valid and were not re-flown CLI-only, since re-flying T12
costs a live patrol-motion window and T13 a full altitude excursion. Treat their throw
kinematics — gravity-restore timing in particular — as not directly comparable to any other
bag, and do not extend the exception to new captures.

### T13 needs altitude, and it must come back down

T13 (`offset_vertical_m: -7.5`) needs the drone flying ≥ 8.0 m before the throw, or
`capture_scenario.sh` aborts (`spawn would be at z=... < 0.5 m`). Same shape as N04's
procedure, with five constraints worth spelling out:

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
- **Restore `takeoff_alt_m: 2.0` immediately afterwards**, repeating land → edit → restart
  T3 → arm → takeoff → start-patrol before capturing anything else.
  `git diff -- src/huitzilin_sim/params/bridge.yaml` must be empty before committing.

T13's altitude check adds latency before `capture_scenario.sh` writes its `spawn=yes` label
line. A capture driver with a short fixed pre-poll delay races that write, finds no
`spawn=yes`, and falls through to a flat un-anchored sleep instead of polling for
`Spawn OK` — which is how the shipped T13 bag ended up with a ~2.2 s post-throw window
against the ~10 s requested. The throw itself is genuine (ball at the expected
z = drone altitude − 7.5 m, thrown via the warm bridge well before recording stopped, zero
false triggers) and the bag is usable. Give the pre-poll delay margin for scenario-specific
startup latency, or check for the label line rather than sleeping a fixed amount before the
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

### `/threat/centroid` message counts are not a detection signal

Every T-bag shows substantial `/threat/centroid` traffic, including the negatives — several
rows carry dozens to ~100 messages, not the near-zero a working detector would produce on a
negative or during a positive's dead time. Cross-referenced against the point clouds (on
T03 and T08), the large majority of that traffic — on the order of 98% — is ambient noise
rather than the labeled throw, and the pattern holds across every T-bag, not only the
highest-count rows. Score against the labeled window and actual cluster geometry; a raw
message count is not evidence that a scenario was detected.

The false-positive rate also correlates with patrol motion across the whole tune set. T14
(~100 messages, patrol running throughout, zero spawns, 45 s) against T11 (1 message, patrol
paused for the whole capture, zero spawns, ~14 s) isolates it: same "nothing thrown"
condition, and the only structural difference is whether the drone is patrolling. This is a
property of the detector as currently tuned. Root-cause it before fitting new thresholds
against these bags — heavy motion-correlated false positives on the negatives pull the
recall/precision tradeoff in a direction that is hard to diagnose afterwards.

### Hz gate — judge every bag, not just the throw

`/oak/points` under 15 Hz sim is normal on this Dell under depth rendering (see CLAUDE.md),
but a bag needs enough density to prove the scenario, not just to exist. Compute the actual
rate from `ros2 bag info` (`/oak/points` message count ÷ bag duration) for every capture. A
live `ros2 topic hz` sample taken before or after the recording can read very differently
from what the bag itself contains and is not a substitute. T01–T14 all landed at
11.5–13.4 Hz; treat anything under ~10 Hz as a failed capture and re-fly it, most urgently
the short-`ttc` scenarios (T03/T07/T08), which have the least frame budget to lose. This
gate measures whether a bag contains enough data to hold the scenario and says nothing
about `/threat/centroid` quality — see the note above.

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
The library is saturated: it cannot referee further threshold changes, and every
trigger-side lever has already been measured as a null (`docs/RESULTS.md` §7). Commit
`detector.yaml` changes with a message naming the FN/FP they fixed.

## Scoring a split against ground truth

Two different scorers exist and answer different questions — don't mix them up.

- **`score_bags.py` / `score_bags_logic.attribute_closing_ball`** — a range-closure-rate
  heuristic. It asks "did a run of detections move like a ball" (in-band closing rate,
  floored above the airframe's own max ground speed). It needs no ground truth, so it works
  on any bag, but it can only say "something closed at a ball-like rate," never "that was
  the ball."
- **`truth_score_heldout.py` / `truth_attribution.py`** — matches detections to
  `/gz/dynamic_poses` (the projectile's true position), so it can say whether a detection
  was actually of the ball. Only usable on bags that record `/gz/dynamic_poses`
  (`capture_scenario.sh` since the line noted above); a positive bag missing that topic is
  VOID for this scorer, not a scoreable miss.

Ground-truth scoring rules:

- **Score each scenario in its own detector process, not one shared process for a whole
  split.** `params/rendered_detector.yaml` sets `use_persistent_bg:true` by design (real
  operating behavior); scored across many scenarios in one process, that lets a later
  scenario inherit background state from an earlier one that shares corridor geometry.
  `run_heldout_eval.sh` restarts the detector per scenario for this reason — use it, or the
  same pattern, rather than a shared-process score.
- **Never tune against a held-out split**, including to debug a threshold or just to look.
  A split's value as a recall measurement is that the detector never saw it during tuning,
  and one look ends that. Use the `tune` / `tune_rendered` splits for iteration.
- **A match is >= K detections within `match_radius_m` of the ball's true position inside
  the scenario's detection window** (`truth_attribution.score_scenario`); false positives on
  negatives are counted on their own denominator and never merged into the recall fraction.

Dodge-side scoring rules (counterfactual, speed splits, hover vs patrol) live in
`docs/RESULTS.md` §10.

## Open detector items

- **S08 false negative** (14 m/s near-miss, train split) — never root-caused. Two blind
  threshold attempts regressed other scenarios without fixing it. Any re-tune must start
  from a single-bag `debug_funnel:=true` trace on S08, not threshold guessing.
- **N02 / N03 / N05 false positives** (patrol-turn and background-clutter triggers) — they
  hurt precision but do not fail the recall gate. Uninvestigated.
- Rendered-lane recall and the spent held-out bags H01/H02/H03/H16: `docs/KNOWN_ISSUES.md`.
