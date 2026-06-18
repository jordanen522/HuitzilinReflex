# Week 2 — Patrol Loop Evidence (W2-13 / W2-15 / W2-17)

Autonomous closed patrol loop, flown entirely through the `huitzilin_sim`
ROS 2 ↔ pymavlink bridge in SITL (Gazebo Harmonic, ArduCopter, gazebo-iris
airframe with `FRAME_CLASS=1`/`FRAME_TYPE=1`).

## Run summary

Sequence: `ros2 launch huitzilin_sim week2_sitl.launch.py` → `/huitzilin/arm` →
`/huitzilin/takeoff` (2.0 m) → `/huitzilin/start_patrol`. Patrol in **position**
mode over the 5 m × 5 m NED square at 2 m altitude (`d = -2.0`).

| Metric | Value |
|---|---|
| Full laps captured | **43** (continuous) |
| Total flight time | ~21.2 min, no restart |
| Mean lap time | **29.51 s** |
| Lap-time stdev | **0.93 s** (over 43 laps — no drift) |
| Lap min / max | 28.20 s / 32.78 s |

Per-leg timing (each leg = 5 m, representative lap):

| Leg | Time | Effective speed |
|---|---|---|
| WP0→WP1 | 8.28 s | 0.60 m/s |
| WP1→WP2 | 9.10 s | 0.55 m/s |
| WP2→WP3 | 8.50 s | 0.59 m/s |
| WP3→WP0 | 6.90 s | 0.72 m/s |

Effective speed < `cruise_speed_ms` (1.5) is expected: in position mode the
follower decelerates into each `accept_radius_m` (0.6 m) corner.

## Interpretation

The <1 s lap-time deviation across 43 laps demonstrates a stable, repeatable
closed loop — satisfies W2-13 ("flies corner-to-corner, logs each WP, loops
≥ 2 laps") far beyond the 2-lap gate. CSV (`week2_telemetry_*.csv`) plotted via
`scripts/plot_telemetry.py` provides the ground-track figure (W2-15).

Source: patrol node `reached WP n` timestamps, 2026-06-17 session.
