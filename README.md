# HuitzilinReflex

A 3.5″ ducted micro-quadrotor that patrols a fixed loop, signals with light and
sound, and reflexively dodges thrown projectiles, using onboard depth sensing,
trajectory prediction and automated flight control. The project simulates first and
flies last; [`HuitzilinReflex_v2.md`](HuitzilinReflex_v2.md) has the full project
description and roadmap.

ROS 2 Jazzy, Gazebo Harmonic, ArduPilot Copter 4.5+ SITL, pymavlink, Python 3.12,
Ubuntu 24.04.

## Status

Active, simulation phase. The patrol loop, a detection pipeline scored against a
labelled bag library, and a Kalman filter plus dodge trigger are done and measured
end to end (see [Results](#results)). The rendered long-range sensor lane is in
progress. Hardware bring-up has not started.

Open problems are tracked in
[`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md).

## Goals

The requirements are REQ-01 through REQ-16 in
[`docs/requirements.md`](docs/requirements.md): autonomous patrol, threat detection via
the OAK-D Lite, intercept prediction, an evasive maneuver within 150 ms, and the alarm
payload. None of them names a specific dodge speed or recall percentage. All are met in
simulation at the envelope measured below.

Two things are open engineering work rather than requirements:

- Long-range detection on a rendered, AR0234-class sensor is unreliable. Recall on
  the tune split is 2/6 with 2/4 false-fire; see
  [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md).
- Dodging faster projectiles needs more sensor reach than the aircraft carries.
  [`docs/RESULTS.md`](docs/RESULTS.md) derives the reach a higher speed would need,
  and [`docs/optics_probe.md`](docs/optics_probe.md) studies a candidate sensor
  upgrade (AR0234, $89, 13.5 g). That upgrade is a documented path, not a
  description of the aircraft as built.

## Results

There are two measurement lanes. They use a different sensor, a different flight mode
and a different scoring rule, so they are reported separately and not compared
row-for-row.

### Real detector, patrol

The OAK-D Lite depth pipeline at its measured ~3.4 m reach, flying patrol: 95
hit-intent throws plus 12 clear-miss controls over five batteries, scored on `dodged`.

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

This is the only measurement of the detection pipeline itself.

### Oracle sensor, hover

Replacing the detector with a synthetic sensor whose reach is a settable input makes
the sensor requirement measurable. Over 310 on-course hover throws, the probability of
a save is a sigmoid in time-to-closest-approach with an LD50 of 0.798 s, and adding
ball speed as a covariate contributes nothing across nine speeds from 14 to 29 m/s.
Only the range needed to buy that time scales with speed:

| Quantity | Value |
|---|---|
| Required reach at 20 m/s, P = 0.90 | 21.1 m |
| Saves at 26 m reach, 20 m/s, head-on | 28/29 |
| False dodges in clear-miss cells | 0 in 31 |
| Maximum ball speed as built (~3.4 m OAK-D Lite) | ~3.2 m/s |

The binding constraint is sensor reach and rate. Every maneuver-side lever was measured
and came back null. Closing the gap to 20 m/s needs an AR0234 global-shutter mono on a
10 mm M12 lens (See3CAM_20CUG, $89, 13.5 g, against the 61 g OAK-D Lite it replaces):
the deficit is angular resolution, not headline range. The same lens narrows the
defended sector to roughly ±10° usable, because reach times half-angle is fixed by
pixel count.

Fit coefficients, dead-time budget, the sensor spec table, the measured nulls and the
scoring rules are in [`docs/RESULTS.md`](docs/RESULTS.md).

## Layout

```
src/huitzilin_sim/          flight bridge, patrol, supervisor, clock guard
src/huitzilin_perception/   detector, Kalman/evasion, payload, scoring harnesses
scripts/                    test runner, preflight, regression, capture
docs/                       results, architecture, frames, safety case, runbooks
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/huitzilin_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

Full install from scratch: [`docs/SETUP.md`](docs/SETUP.md).

## Test

```bash
./scripts/run_tests.sh            # whole unit suite, both packages
./scripts/run_tests.sh -k clock   # pytest args are forwarded
```

28 `test_*.py` modules across `huitzilin_sim` and `huitzilin_perception`.
`.github/workflows/tests.yml` runs the ROS-free subset on every push; the modules
that import `rclpy` run locally only, on the Dell.

```bash
./scripts/preflight_check.sh      # SITL environment
./scripts/run_regression.sh       # detection regression against the bag library
```

### Detection scoring

A 17-bag labelled library: 12 positive throw scenarios (S01–S12) crossing speed,
approach angle and miss distance, and 5 negatives (N01–N05), covering clean-patrol
baselines plus probes of egomotion, field-of-view and range-gate false positives.

S11, S12 and N05 are held out and never tuned against; the OAK-D-lane detector recalls
2/2 on that split. Recall on the train split is 90%. S08, a 14 m/s near-miss, is a
known false negative that was never root-caused, putting full-library positive recall
at 11/12. The regression gate is a 95% recall floor only: there is no precision gate,
and N02, N03 and N05 carry uninvestigated false positives. A separate, larger held-out
split (H01–H18, HN01–HN06) covers the rendered long-range sensor lane and does not yet
pass reliably; see [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md). Procedure:
[`docs/bag_capture_runbook.md`](docs/bag_capture_runbook.md).

### Projectile trials

Two denominators, reported separately. 310 on-course hover throws under the synthetic
oracle across 30 cells at swept ranges, counterfactual-scored; 26 m is the design point
derived in `docs/RESULTS.md` §4, not the range every cell was flown at. And 95
hit-intent patrol throws against the real ~3.4 m detector, fire-scored. Several hundred
further throws were flown and excluded by the per-throw gates in `docs/RESULTS.md` §10:
off-course, high fit residual, or from cells whose implied update rate exceeded the
launched oracle rate.

## Run

Bring-up is three terminals (Gazebo, SITL, ROS launch) followed by three service calls
in a fixed order. The commands and the exact service types are in
[`CLAUDE.md`](CLAUDE.md); [`docs/SETUP.md`](docs/SETUP.md) covers installing from
scratch. Read the sharp-edges list before the first run:
several failure modes are silent, notably `FRAME_CLASS=0`, where the aircraft arms,
accepts takeoff and produces no lift.

## Documentation

| Doc | Contents |
|---|---|
| [`docs/RESULTS.md`](docs/RESULTS.md) | Measured envelopes, the tca law, the sensor-reach requirement |
| [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md) | Open problems in the rendered long-range sensor lane |
| [`CLAUDE.md`](CLAUDE.md) | Bring-up commands and sharp edges |
| [`HuitzilinReflex_v2.md`](HuitzilinReflex_v2.md) | Objectives, BOM, roadmap |
| [`docs/architecture.md`](docs/architecture.md) | Node graph, message and service contracts |
| [`docs/frames.md`](docs/frames.md) | Coordinate frames and TF tree |
| [`docs/state_machine.md`](docs/state_machine.md) | Supervisor states and transitions |
| [`docs/requirements.md`](docs/requirements.md) | REQ-01 … REQ-16 and non-goals |
| [`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md) | FMEA, geofence/RTL, kill-switch, operating rules |
| [`docs/SETUP.md`](docs/SETUP.md) | Install from scratch |
| [`docs/optics_probe.md`](docs/optics_probe.md) | Rendered-camera reach probe; the AR0234 and depth-noise measurements |
| [`docs/bag_capture_runbook.md`](docs/bag_capture_runbook.md) | Bag capture and detection regression (Dell) |
| [`docs/dodge_battery_runbook.md`](docs/dodge_battery_runbook.md) | Dodge battery and sweep procedure (Dell) |

## Safety

The payload is a signal only. It is never used to follow or harass a person. Flight is
netted or tethered, inside a 10 m geofence with a 5 m ceiling, with a dedicated
kill-switch in hand for every powered test. These rules are binding; see
[`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md).

The evasive maneuver is for projectiles only. No fault condition can produce one, which
is asserted in tests over every combination of state, fault and armed status rather
than argued by inspection.
