# Week 2 — Patrol Loop Evidence (W2-13 / W2-15 / W2-17)

Autonomous closed patrol loop through the `huitzilin_sim` ROS 2 ↔ pymavlink bridge in
SITL: `week2_sitl.launch.py` → arm → takeoff (2 m) → start_patrol, position mode over
the 5×5 m NED square.

| Metric | Value |
|---|---|
| Full laps (continuous) | **43** (~21.2 min, no restart) |
| Mean lap time | **29.51 s** |
| Lap-time stdev | **0.93 s** (no drift) |
| Lap min / max | 28.20 s / 32.78 s |

Per-leg effective speed ~0.55–0.72 m/s over 5 m — below `cruise_speed_ms` (1.5) as
expected: position mode decelerates into each 0.6 m accept radius.

The <1 s deviation across 43 laps satisfies W2-13 (≥2 clean laps) with margin. CSV
(`week2_telemetry_*.csv`) via `scripts/plot_telemetry.py` gives the ground track (W2-15).

Source: patrol node `reached WP n` timestamps, 2026-06-17 session.
