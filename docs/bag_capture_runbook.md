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

## Capturing new bags over ssh

Same bring-up as above; scenario IDs come from `scenario_matrix.yaml` (see its SPLITS
block). The T-set (T01-T14) was captured this way with no TTY, and these are the traps
that cost re-flights.

### Stopping the recorder

`capture_scenario.sh` ends in `exec ros2 bag record`, so the PID you background *becomes*
the recorder. Enable job control with `set -m` before backgrounding, then signal that PID
directly -- never a pattern match:

```bash
set -m
CMD > "$LOG" 2>&1 &
PID=$!
kill -INT "$PID"
```

Without `set -m`, a non-interactive bash sets SIGINT to `SIG_IGN` for `&` jobs and that
disposition survives `exec()`, so `kill -INT` does nothing. `nohup` does not fix it, and
neither does `trap - INT` in the child. Do not add `setsid` on top: it forks whenever the
caller is already a process group leader, which `set -m` guarantees, so `$!` becomes
setsid's own short-lived PID. To verify a recorder can be signalled:
`kill -INT $PID; sleep 1; grep SigIgn /proc/$PID/status` -- the SIGINT bit must be clear.

### Anchor the post-spawn wait on `Spawn OK`, never a fixed timer

The CLI throw path costs 15-20 s of *wall* time between the spawn command and the impulse.
Under fixed timing, `Recording stopped` preceded `Spawn OK` by 7-10 s in **11 of 11** spawn
scenarios -- each recording a ball sitting motionless at its spawn point instead of the
labelled scenario. Poll the log:

```bash
until grep -q 'Spawn OK' "$LOG"; do sleep 1; done
sleep "$((TTC_S + 9))"        # THEN start the post-throw buffer
kill -INT "$PID"
```

Budget a generous poll timeout (60 s+) rather than tuning per scenario, and give the
*pre*-poll delay margin too: a scenario with startup latency (T13's altitude check) can
race `capture_scenario.sh`'s own label write, and a driver that sleeps a fixed amount
first falls through to an un-anchored sleep. `wrench bridge never connected -- falling
back to the CLI throw` is expected and benign on `week3_perception.launch.py`.

### The CLI throw path is canonical for this library

Do not wire `week4_evasion.launch.py`'s wrench bridge into `week3_perception.launch.py` to
get the faster same-physics-step `WrenchThrower`. The original 17 bags predate
`WrenchThrower` (`37e0d34`) and use the CLI path by construction; the shared mechanism is
what keeps the splits comparable. The CLI throw restores gravity ~0.25 s late and flattens
the trajectory (see `spawn_projectile.py`).

T12 and T13 are the only exception in the 31-bag library -- both went through the warm
bridge (`thrown via warm bridge` in their logs). Both are valid, but treat their throw
kinematics as not comparable to any other bag, and do not extend the exception.

### A scenario needing altitude (T13 shape)

`mav_bridge_node` reads `takeoff_alt_m` **once at construction** with no parameter
callback, so `ros2 param set` is accepted and silently ignored -- the launch must restart.
And `/huitzilin/takeoff` calls `MAV_CMD_NAV_TAKEOFF`, which ArduCopter accepts only when
landed, so the sequence is land -> edit `bridge.yaml` -> restart -> arm -> take off.

ArduCopter also refuses to arm while the mode is LAND (`Arm: Land mode not armable`,
visible only in the SITL console -- the ROS service just times out with a generic message).
Switch back to GUIDED first. Do not start patrol between takeoff and the capture: patrol's
waypoints are pinned at `d: -2.0` NED and fly the drone straight back down.

Restore `takeoff_alt_m: 2.0` afterwards --
`git diff -- src/huitzilin_sim/params/bridge.yaml` must be empty before committing.

### A scenario needing a commanded climb (T11 shape)

Patrol sends its own position setpoints at 10 Hz and will fight anything else back to its
fixed altitude, so **pause patrol first** (`/huitzilin/start_patrol` with `data: false`;
ArduCopter then just holds GUIDED position). Then `/huitzilin/cmd_vel` is free (FLU, so
`linear.z` positive is up):

```bash
ros2 topic pub -r 10 /huitzilin/cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'   # climb, ~2 s
ros2 topic pub -r 10 /huitzilin/cmd_vel geometry_msgs/msg/Twist '{linear: {z: -1.0}}'  # descend, ~2 s
```

`cmd_timeout_s` (0.7 s in `bridge.yaml`) zero-holds once `topic pub` ends. Resume patrol
once the bag is closed.

### `/threat/centroid` counts are not a detection signal

Every T-bag shows substantial centroid traffic including the negatives -- dozens to ~100
messages, not the near-zero a working detector would produce. That is a property of the
detector as currently tuned, not evidence the capture worked. Root-cause it before fitting
new thresholds against these bags.

### Hz gate -- judge every bag, not just the throw

Compute the actual rate from `ros2 bag info` (`/oak/points` count / bag duration) for every
capture. A live `ros2 topic hz` sample can read very differently from what the bag contains
and is not a substitute. T01-T14 all landed at 11.5-13.4 Hz; treat anything under ~10 Hz as
a failed capture and re-fly it, most urgently the short-`ttc` scenarios (T03/T07/T08).


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
