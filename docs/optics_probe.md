# Optics probe — can a *rendered* depth camera see the ball at 26 m?

**Run 2026-08-21 on the Dell (native Ubuntu 24.04.4, Intel UHD 630, no discrete
GPU) at commit `3525f72`.** Raw results: `lab/probe_out/*.result.txt`.
Harness: `scripts/optics_probe/` (`run_probe.sh`, `count_ball_points.py`,
`probe_world.sdf.in`).

## Scope

Every 20 m/s result elsewhere in the repo comes from the oracle (centroid
*asserted* from Gazebo truth) or the synthetic-depth lane (cloud *fabricated*
from Gazebo truth, then fed to the real detector), because an 80 mm ball at 26 m
subtends 0.176° — sub-pixel at the depth world's 640×480 over 67.3° (0.105°/px),
so no cluster forms. That is a limit of the camera the aircraft carries, not of
rendering. This probe measures whether a different rendered camera clears it.

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

## What this probe does not establish

- **It is static.** Stationary camera, stationary ball, no flight dynamics, no
  motion blur, no attitude change during the throw. It measures optics only.
- **It is noise-free**, so the counts are an upper bound. No sensor noise is
  declared in the probe world, which is correct for a reach probe. The shipped
  `iris_depth` is barely different: a 1 cm per-pixel gaussian, ~30× too small at
  26 m and independent where the real error is correlated. A real stereo pair
  has σ = 0.30 m at 26 m, growing as z² and *common-mode* across the ball, so
  the error lands on the centroid. A rendered lane without that is better than
  the real sensor and must not be scored as if it were. `depth_noise.py` and
  `iris_ar0234` close that gap.
- **It says nothing about detection, tracking, or dodging** — only that enough
  points exist for clustering to be possible.
- **It is not evidence about the physical part.** A simulated 27.0° camera at
  800×650 resolving the ball at 26 m says nothing about whether a $89 AR0234
  with a 10 mm M12 lens detects an 80 mm ball at 26 m in daylight.

---

## Noise-stage validation (2026-08-21)

The probe above answers the *reach* question. It leaves the sensor better than
the real part, because the rendered cloud carries essentially no depth error.
`depth_noise_node.py` closes that gap, and this is the measurement that it
works — end to end, through the bridge, not against a synthetic array.

Rig: `scripts/optics_probe/run_noise_probe.sh`, pinned to the `iris_ar0234`
optics (800×650, 27.0° HFOV, far 35 m), ball parked at a known range, 120
frames. Reported quantity is the spread of the ball centroid's **range**.

| Run | Ball | σ_ref | Points/frame | Centroid range mean | **std** |
|---|---|---|---|---|---|
| `noise_off_26m` | 26 m | 0.0 | 24 | 25.9784 m | **0.0000 m** |
| `noise_on_26m` | 26 m | 0.30 | 24 | 25.9724 m | **0.2859 m** |
| `noise_on_13m` | 13 m | 0.30 | 80 | 12.9714 m | **0.0665 m** |

Three things this establishes:

1. **The noise is applied and lands on the centroid.** 0.2859 m against the
   0.30 m spec, inside the ~6.5 % standard error of a std from 120 samples.
2. **The control is a true control.** `sigma_ref_m: 0.0` gives std and
   peak-to-peak of *exactly* zero — no randomness consumed, so an A/B differs
   only in the noise and not in the seed.
3. **The z² law holds in situ.** 0.2859 / 0.0665 = 4.30 against the ideal 4.0.
   The 13 m arm reads ~11 % below its predicted 0.075 m, in the predicted
   direction: the ball spans ~10 px there (80 returns) against a 7 px
   correlation cell, so the centroid partially averages. At 26 m it spans ~5 px
   and gets a single disparity solution, which is the regime that matters.

### Depth axis convention

Gazebo's `PointCloudPacked` stays in the sensor **body** frame (X-forward). An
earlier `depth_noise_node` read depth from column 2, so near boresight the
body-frame Z was ~0, σ(z) collapsed, every return moved by ~0.0001 m, and the
stage published a noiseless sensor while reporting a clean run — well-formed
cloud, correct point count, std 0.0000 m on the noised arm. The node now selects
the axis from `cloud_convention` (`detector.yaml`'s existing `gz_flu | optical`
vocabulary) and logs an ERROR naming the parameter if the first cloud comes back
unchanged.
