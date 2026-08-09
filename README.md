# HuitzilinReflex

A 3.5″ ducted micro-quadrotor that patrols a fixed loop, signals with light and
sound, and reflexively dodges thrown projectiles.

ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink ·
Python 3.12 · Ubuntu 24.04.

## Status

**Week 5 — hardware bring-up (FC swap).** The Week 5 *software* lane is complete;
everything still open is gated on physically swapping the flight controller.
Weeks 1–4 closed: patrol loop, detection pipeline scored against a labelled bag
library, Kalman filter and dodge trigger.

## What Week 4 actually measured

The result is a **capability envelope, not a success rate**. Report it split by
ball speed — the blended number hides both halves of the finding:

| Ball speed | Scenarios | Dodges |
|---|---|---|
| ≤ 8 m/s | B01, B02, B04, B05, B06 | 78/78 |
| 14 m/s | B03 | 0/17 |
| false dodges | B07 | 0/12 |

95 scored throws over five batteries. **14 m/s is the edge of the reaction
envelope, not a defect**, and the binding constraint is *sensing* — detection
range and frame rate — not latency, not thresholds, not dodge authority. Latency
averages 95–115 ms against a 150 ms budget, and the ~25% of dodges that exceed it
cost no outcomes, because time-to-closest-approach at commit is 0.18–0.29 s.

Those conditions matter and travel with the numbers: **patrol** flight, the real
depth detector at its measured ~3.4 m reach, scored on `dodged`.

## What Week 6 answered

That distinction named its own next experiment, and the sim lane ran it ahead of the
hardware — with a synthetic sensor whose reach was a settable *input*, not a result.
The answer is a law, not a rate:

**A save is a threshold in time-to-closest-approach at ~0.83 s, and the threshold
does not depend on ball speed.** Only the range needed to buy that time does:

    range = speed × (tca + 0.177)        # 0.177 s = pipeline + commit, hover

Measured over 60 throws in hover at two ball speeds, the second pre-registered with
its own negative control. Every maneuver-side lever is refuted — the vehicle already
over-delivers on the velocity step it is commanded, and 4x the dodge command buys
1.095x the escape.

So the honest ceiling of the aircraft as built is **~3.5 m/s**, not 14. A 20 m/s ball
needs ~21 m of reach on a 7 cm target, and the part that buys it is an **AR0234
global-shutter mono with a 10 mm M12 lens** — $99, 26 g, *lighter* than the 61 g
OAK-D Lite it replaces. The deficit was angular resolution, never headline range.

Full derivation, every refuted lever, and the rejected alternatives:
[`docs/week6_result.md`](docs/week6_result.md).

## Layout

```
src/huitzilin_sim/          flight bridge, patrol, supervisor, clock guard
src/huitzilin_perception/   detector, Kalman/evasion, payload, scoring harnesses
scripts/                    test runner, preflight, regression, capture, probes
docs/                       architecture, frames, safety case, runbooks, plans
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

`.github/workflows/tests.yml` runs the ROS-free subset on every push; the modules
that import `rclpy` run locally only.

```bash
./scripts/preflight_check.sh      # SITL environment
./scripts/preflight_hw.sh         # hardware; warns rather than fails
./scripts/run_regression.sh       # detection regression against the bag library
```

## Run

Bring-up is three terminals (Gazebo, SITL, ROS launch) and three service calls in
a fixed order. The exact commands, the service *types* — which are not
interchangeable — and the traps that bite live in
[`CLAUDE.md`](CLAUDE.md). Read its sharp edges before the first run; several of
them fail silently rather than loudly, `FRAME_CLASS=0` most notoriously (the
aircraft arms, accepts takeoff, and produces no lift).

## Documentation

| Doc | Contents |
|---|---|
| [`CLAUDE.md`](CLAUDE.md) | Bring-up commands, sharp edges, measured nulls |
| [`HuitzilinReflex_v2.md`](HuitzilinReflex_v2.md) | Objectives, BOM, 9-week roadmap |
| [`docs/architecture.md`](docs/architecture.md) | Node graph, message and service contracts |
| [`docs/frames.md`](docs/frames.md) | Coordinate frames and TF tree |
| [`docs/state_machine.md`](docs/state_machine.md) | Supervisor states and transitions |
| [`docs/requirements.md`](docs/requirements.md) | REQ-01 … REQ-16 and non-goals |
| [`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md) | FMEA, geofence/RTL, kill-switch, operating rules |
| [`docs/week6_result.md`](docs/week6_result.md) | **The 20 m/s answer**: the tca threshold law, every refuted lever, the sensor that closes the gap |
| [`docs/weeks_5_9_plan.md`](docs/weeks_5_9_plan.md) | Weeks 5–9, hardware and software lanes |

## Safety

The payload is a **signal only** — it is never used to follow or harass a person.
Flight is netted or tethered, inside a 10 m geofence with a 5 m ceiling, with a
dedicated kill-switch in hand for every powered test. The rules are binding, not
aspirational: see [`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md).

The evasive maneuver is for projectiles only. No fault condition can produce one —
that is asserted over every state × fault × armed combination, not argued by
inspection.
