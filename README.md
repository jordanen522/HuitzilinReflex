# HuitzilinReflex

A 3.5″ ducted micro-quadrotor that patrols a fixed loop, signals with light and
sound, and reflexively dodges thrown projectiles.

It detects and dodges projectiles travelling up to **20 m/s in simulation** — **2.5×
the 8 m/s baseline** its real depth detector reaches — using **depth sensing,
trajectory prediction, and automated flight control**. Those two speeds come from two
*different* experiments and are not interchangeable: the 8 m/s baseline is the real
OAK-D detector on patrol, the 20 m/s figure is a derived 26 m / 60 Hz sensor in
hover. [The result](#the-result) reports them side by side and never pools them —
[`docs/RESULTS.md`](docs/RESULTS.md) §8 forbids it.

ROS 2 **Jazzy** · Gazebo **Harmonic** · ArduPilot Copter 4.5+ **SITL** · pymavlink ·
Python 3.12 · Ubuntu 24.04.

## Status

**Complete — simulation.** Closed 2026-08-10. Weeks 1–6 are done: patrol loop,
detection pipeline scored against a labelled bag library, Kalman filter and dodge
trigger, and the 20 m/s answer. Hardware bring-up is out of scope.

## The result

Two envelopes. They use a different sensor, a different flight mode and a different
scoring rule, so they are reported one after the other and never merged into a single
table — that rule is [`docs/RESULTS.md`](docs/RESULTS.md) §8, not a stylistic choice.

### The baseline — the aircraft as built

The real OAK-D Lite depth detector at its measured ~3.4 m reach, flying **patrol**,
over five batteries — **95 hit-intent throws** plus 12 clear-miss controls:
**78/78 dodges at ≤ 8 m/s** (B01, B02, B04, B05, B06), **0/17 at 14 m/s** (B03), and
**0 false dodges** in the 12 controls (B07). Scored on `dodged` — a *fire* count, not
the counterfactual save rule the 20 m/s lane below uses, which is one of the reasons
the two lanes are not comparable. This is the project's **only** real-detector
measurement, and it is the 8 m/s baseline the headline figure is measured against.

### The 20 m/s answer — a derived sensor

Everything below replaces that detector with a synthetic oracle whose reach is a
settable *input*, flying **hover**.

**A save is a sigmoid in time-to-closest-approach, and the threshold does not
depend on ball speed.** Over 310 on-course hover throws from 30 cells:

    logit P(save) = -22.271 + 27.910 × tca      LD50 0.798 s (CI 0.779–0.817)

P = 0.5 at 0.80 s, 0.9 at 0.88 s, and 155/155 above 1.00 s. Adding ball speed as a
second covariate across nine speeds from 14 to 29 m/s contributes nothing
(z = +0.93, not significant). Only the range needed to *buy* that time scales with
speed:

    range = speed × (tca_required + t_dead)     t_dead = 0.178 s at 60 Hz

At 20 m/s that is **21.1 m** of reach, and a 26 m sensor scores **28/29** head-on in
hover. False dodges were measured in *separate* clear-miss cells, not in that one:
**0 in 31**, while the 0.5 m near-miss control still fired 6/6. So the binding
constraint is **sensor reach and rate** — every maneuver-side lever was measured and
refuted, and the vehicle already over-delivers on the velocity step it is commanded.

The aircraft **as built** carries a ~3.4 m OAK-D Lite, which caps it at ~3.2 m/s. The
part that closes the gap is an **AR0234 global-shutter mono on a 10 mm M12 lens**
(See3CAM_20CUG, $89, 13.5 g — *lighter* than the 61 g OAK-D Lite it replaces): the
deficit was angular resolution, never headline range. The same lens narrows the
defended sector to about ±10° usable, which is an architectural consequence rather
than a bug — reach × half-angle is fixed by pixel count.

Everything in *this* section — the 20 m/s lane, not the baseline above it — is
measured against a *synthetic* sensor whose reach is a settable **input**, so it is a
claim about the tracker, trigger and airframe given such a sensor, not evidence that
one exists. Full write-up and every caveat: [`docs/RESULTS.md`](docs/RESULTS.md).

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

**511 passing unit tests** — 512 collected, one environment-conditional skip, zero
failures, run green on the Dell at this commit:

```bash
./scripts/run_tests.sh            # whole unit suite, both packages
./scripts/run_tests.sh -k clock   # pytest args are forwarded
```

That is 387 test functions expanded by parametrisation — `huitzilin_sim` 169
collected, `huitzilin_perception` 343. The Week-5 roadmap row in
[`HuitzilinReflex_v2.md`](HuitzilinReflex_v2.md) records **253** collected at
2026-07-31; the suite has grown since.

`.github/workflows/tests.yml` runs the ROS-free subset on every push (482 collected);
the modules that import `rclpy` run locally only.

```bash
./scripts/preflight_check.sh      # SITL environment
./scripts/run_regression.sh       # detection regression against the bag library
```

### Detection scoring

A **17-bag labelled library**: 12 positive throw scenarios (S01–S12) crossing speed,
approach angle and miss distance, and 5 negatives (N01–N05) — clean-patrol baselines
plus dedicated probes of egomotion, field-of-view and range-gate false positives.

**S11, S12 and N05 are held out and never tuned against — recall on that held-out
split is 100%.** Recall on the train split is **90%**: **S08**, a 14 m/s near-miss,
is a known and never-root-caused false negative, which puts full-library positive
recall at 11/12. The regression gate is a 95% recall floor only — there is no
precision gate, and N02/N03/N05 carry uninvestigated false positives. Details and
the open items: [`docs/bag_capture_runbook.md`](docs/bag_capture_runbook.md).

### Projectile trials

Reported as two denominators, never pooled: **310 on-course hover throws** under the
synthetic oracle across 30 cells at swept ranges (26 m is the *derived design point*
of §4, not the range they were all flown at), counterfactual-scored — the sigmoid
above — and **95 hit-intent patrol throws** against the real ~3.4 m detector,
fire-scored. Several hundred further throws were flown and excluded by rule —
off-course, high fit residual, or from cells quarantined for stack contamination.

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
| [`docs/RESULTS.md`](docs/RESULTS.md) | **The 20 m/s answer**: the tca law, the sensor requirement, every closed question |
| [`CLAUDE.md`](CLAUDE.md) | Bring-up commands and sharp edges |
| [`HuitzilinReflex_v2.md`](HuitzilinReflex_v2.md) | Objectives, BOM, roadmap |
| [`docs/architecture.md`](docs/architecture.md) | Node graph, message and service contracts |
| [`docs/frames.md`](docs/frames.md) | Coordinate frames and TF tree |
| [`docs/state_machine.md`](docs/state_machine.md) | Supervisor states and transitions |
| [`docs/requirements.md`](docs/requirements.md) | REQ-01 … REQ-16 and non-goals |
| [`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md) | FMEA, geofence/RTL, kill-switch, operating rules |
| [`docs/bag_capture_runbook.md`](docs/bag_capture_runbook.md) | Bag capture and detection regression (Dell) |
| [`docs/dodge_battery_runbook.md`](docs/dodge_battery_runbook.md) | Dodge battery and sweep procedure (Dell) |

## Safety

The payload is a **signal only** — it is never used to follow or harass a person.
Flight is netted or tethered, inside a 10 m geofence with a 5 m ceiling, with a
dedicated kill-switch in hand for every powered test. The rules are binding, not
aspirational: see [`docs/SAFETY_CASE.md`](docs/SAFETY_CASE.md).

The evasive maneuver is for projectiles only. No fault condition can produce one —
that is asserted over every state × fault × armed combination, not argued by
inspection.
