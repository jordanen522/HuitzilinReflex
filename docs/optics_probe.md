# Optics probe — can a *rendered* depth camera see the ball at 26 m?

**Run 2026-08-21 on the Dell (native Ubuntu 24.04.4, Intel UHD 630, no discrete
GPU) at commit `3525f72`.** Raw results: `lab/probe_out/*.result.txt`.
Harness: `scripts/optics_probe/` (`run_probe.sh`, `count_ball_points.py`,
`probe_world.sdf.in`).

## Why this exists

Every 20 m/s result in this repo up to now was taken through either the oracle
(centroid *asserted* from Gazebo truth) or the synthetic-depth lane (cloud
*fabricated* from Gazebo truth, then fed to the real detector). Both were built
on the same stated blocker: an 80 mm ball at 26 m subtends 0.176°, which is
sub-pixel at the depth world's 640×480 over 67.3° (0.105°/px), so no rendered
camera in this simulation could produce a cluster to detect.

That blocker is real **for the camera the aircraft carries**. It is not a
property of rendering. This probe measures whether a *different* rendered
camera clears it.

## Method

A bare world (`probe_world.sdf.in`) containing exactly two things: the camera
under test, and one static 80 mm sphere (radius 0.040, matching
`models/projectile`) on the optical axis at a settable range.

**There is deliberately no ground plane.** Every finite point in the cloud is
therefore a ball return, and the count needs no spatial gate — a spatial gate
would beg the question the probe exists to answer. Counting is vectorised;
a per-point Python loop throttles the subscriber below the sensor rate and
makes `delivered_hz` a property of the script rather than of the camera.

`delivered_hz` is computed from **message stamps (sim time)**, not arrival
times. `rtf` is sim-seconds per wall-second.

All 26 m rows were run with the far clip opened to 30 m. The shipped
`iris_depth` clips at **19.0 m**, which would otherwise have manufactured a
zero that had nothing to do with optics.

## Positive control

| Config | Range | Points/frame | delivered_hz | rtf |
|---|---|---|---|---|
| OAK-D as shipped: 640×480, 67.3°, 15 Hz | 3.0 m | **124** | 15.2 | 0.998 |

The harness counts. A zero elsewhere is therefore a measurement, not a broken
bridge. (`count_ball_points.py` also exits non-zero if no cloud ever arrives.)

## Result — reach

All at 26.0 m, far clip 30 m, 15 Hz:

| Config | Points/frame | delivered_hz | rtf |
|---|---|---|---|
| OAK-D as shipped: 640×480, 67.3° | **4** | 13.9 | 0.998 |
| AR0234 proposed: 800×650, 27.0° | **24** | 15.2 | 0.997 |
| AR0234 proposed: 1600×1300, 27.0° | **80** | 9.8 | 0.345 |

Two things follow.

1. **The stated blocker is confirmed, and it is an optics limit, not a
   rendering limit.** The shipped camera returns 4 points at 26 m — just under
   `cluster_min_points: 5` in `detector.yaml`. That is the mechanism behind the
   ~3.4 m practical reach, measured rather than argued.

2. **The proposed optics clear it with margin, and rendering is affordable.**
   800×650 over 27.0° returns 24 points at essentially free RTF (0.997).

**The synthetic model is independently validated by this.**
`synthetic_depth.py` predicts 85 returns for an 80 mm ball at 26 m through
1600×1300 over ±13.5°. The rendered camera returns **80** — 6% apart, from
completely independent geometry. The synthetic lane's *geometry* was right.

## Result — rate

Rate is the third sensor axis and it sets `t_dead`. Fitting the two measured
points (`t_dead` 0.178 s @ 60 Hz, 0.270 s @ 12 Hz) to `t_dead = a + b/f` gives
`a = 0.155 s`, `b = 1.380`, and required reach follows from
`range = speed × (tca + t_dead)` at the P=0.90 threshold `tca = 0.88 s`:

| Config | Points | delivered_hz | rtf | t_dead | Reach needed @20 m/s |
|---|---|---|---|---|---|
| 800×650 @15 Hz | 24 | 15.2 | 0.997 | 0.247 s | 22.5 m |
| **800×650 @30 Hz** | **24** | **23.0** | **0.926** | **0.215 s** | **21.9 m** |
| 800×650 @60 Hz | 24 | 44.0 | 0.493 | 0.186 s | 21.3 m |
| 560×455 @60 Hz | 8 | 51.6 | 0.953 | 0.182 s | 21.2 m |
| 1600×1300 @8 Hz | 80 | 5.4 | 0.682 | 0.328 s | 24.2 m |

No configuration delivers its *requested* rate: the ceiling is
`ros_gz_bridge` PointCloudPacked→PointCloud2 bandwidth, the same limit already
recorded for 30 Hz at 640×480. 1600×1300 is 2.08 M points ≈ 33 MB/cloud and is
bandwidth-bound at 5.4 Hz however it is configured. **Always quote the
*delivered* rate, never the requested one.**

Every row needs at most 24.2 m of reach against 26 m available, so reach is not
the binding constraint for any of them. **800×650 @ 30 Hz is the recommended
design point**: 24 points (≈5× the cluster minimum), 23 Hz delivered, and
RTF 0.926, which keeps battery throughput close to the current lane.

## What this probe does NOT establish

- **It is static.** Stationary camera, stationary ball, no flight dynamics, no
  motion blur, no attitude change during the throw. It measures optics only.
- **It is noise-free.** A Gazebo depth camera returns exact geometric depth. A
  real stereo pair does not: `synthetic_depth.py` models σ = 0.30 m at 26 m,
  growing as z², and *common-mode* across the ball, so the error lands on the
  centroid. A rendered lane without that noise is **better than the real
  sensor** and must not be scored as if it were the real one.
- **It says nothing about detection, tracking, or dodging** — only that enough
  points exist for clustering to be possible.
- **It is not evidence about the physical part.** That a simulated 27.0° camera
  at 800×650 resolves the ball at 26 m is not evidence that a $89 AR0234 with a
  10 mm M12 lens detects an 80 mm ball at 26 m in daylight.
