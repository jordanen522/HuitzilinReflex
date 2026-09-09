# Action escalation

A privacy-preserving subsystem that recognises **aggressive body motion** and, on
confirmation, asks for the warning lights and siren. It is architecturally separate from
the projectile-evasion pipeline and cannot command flight.

**Status: no real input path exists.** There is no pose estimator in this project, on the
camera or on the Pi. Everything below has been exercised only against hand-authored
synthetic keypoint sequences. No number in this document is a measured detection rate,
because none has ever been measured.

## What it does

```
/action/keypoints ---> action_recognizer ---> /action/alert_request ---> alert_signal
   (body joints)        window / score            (Bool)                     |
                        confirm / cooldown                                   v
                              |                                    /action/alert_state
                              |---> /action/escalation_event
                              |---> /action/status
```

| Node | Role |
|---|---|
| `action_recognizer` | Scores joint frames, applies confirmation and cooldown, requests the alert |
| `alert_signal` | Drives the light/siren sink, holds its own dead-man |
| `scenario_player` | Replays a synthetic sequence so the chain runs with no camera |

## Categories, and the non-goals

**Aggressive motion only. Three categories: `LUNGE`, `STRIKE`, `SHOVE`.**

There is **no benign category**. Benign motion is not labelled; it is simply the case
where nothing crossed a threshold. The system never builds a description of what a person
is doing, only whether an aggression threshold tripped. That is a privacy property as much
as a scope one: it keeps this an alarm rather than an activity log.

Two categories were considered and cut:

- **`GRAPPLE_PROXIMITY`**: proximity is not aggression. Two bodies close together and
  moving quickly is also an embrace, a handshake, or helping someone up. It generates
  false alarms while wearing a threat label.
- **`RAISED_OBJECT`**: **not detectable from this input at all.** The whitelist is twelve
  body joints with no object detection, so a raised hand holding a weapon and a raised
  hand hailing a taxi are the identical signal. Deferred rather than dropped: it needs a
  real object detector, a separate capability with its own compute budget. Object
  detection would not breach the privacy contract, since an object is not an identity,
  but it is not free.

Permanent non-goals, for this subsystem and the project: face detection, facial
recognition, person identification, re-identification, persistent person tracking,
biometric embeddings, gait or soft-biometric matching, and raw-video retention.

## Privacy contract

Enforced by `pose_frame.py`, in code, at the boundary, rather than by a policy that
downstream code is trusted to honour.

- **`ALLOWED_JOINTS` is COCO-17 minus keypoints 0-4**: nose, both eyes, both ears. Those
  five are the face. Twelve remain: shoulders, elbows, wrists, hips, knees, ankles. No
  face can be reconstructed from what this subsystem holds.
- **Parsing is strict.** An unknown top-level key, an unknown joint name, or a
  string-valued `subject` rejects the **whole frame**. A lenient parser that ignored
  fields it did not recognise is exactly how an `image`, a `crop` or an `embedding` field
  arrives and then gets copied along by everything downstream.
- **`subject` is an integer index within one frame.** It is not a track id and is not
  stable between frames. A string subject is refused because a stable identifier is the
  shape identity smuggles itself in as.
- **Bounded memory.** A `FeatureWindow` holds `window_s` (0.6 s) of frames and nothing
  survives it. There is no history a person could be tracked through.
- **No recording.** `record_video` defaults false and the node refuses to start if it is
  set true. `publish_observations` defaults false.
- **Events say what was decided, never what was seen.** No identifier, no bounding box, no
  image reference, no joint positions. That is the difference between an event log and a
  surveillance record.
- **`frames_rejected` is published on `/action/status`.** Non-zero means something
  upstream is sending fields this subsystem refuses, visible from `ros2 topic echo` rather
  than only in a log.

## Expected input

`/action/keypoints`, `std_msgs/String` carrying JSON. Camera optical frame: +Z depth away
from the camera, +Y down, metres.

```json
{"schema": 1, "stamp_s": 12.345, "frame_id": "camera_optical_frame",
 "source": "scenario_player", "subject": 0,
 "joints": {"shoulder_l": [-0.20, -0.50, 4.00, 0.95],
            "wrist_r": [0.28, 0.05, 3.96, 0.88]}}
```

Joint values are `[x, y, z, conf]`. `std_msgs/String` rather than a typed message because
the privacy contract rests on **rejecting unknown fields**, and a typed message has no
rejection hook. `sensor_msgs/JointState` is the closest stock alternative and was not
used: it is one scalar per name, carries no confidence, and imports robot-joint semantics.

## Confidence and temporal confirmation

The published field is **`score`, not `confidence`**: it is a normalised threshold score,
not a probability. Every event carries `"provenance": "UNVALIDATED_HEURISTIC"`.

**Scoring is conjunctive.** Each category takes the **minimum** over its required evidence
terms, never the maximum or a sum:

| Category | Requires, simultaneously |
|---|---|
| `LUNGE` | closing speed, whole-body translation, and torso pitch rate |
| `STRIKE` | hand speed, and shoulder-to-wrist opening rate |
| `SHOVE` | **both** arms driving forward, and closing speed |

Disjunctive scoring fires on any single term, so a person walking briskly toward the
aircraft would read as a lunge on approach speed alone. The cost of a false positive here
is a siren pointed at a bystander.

The shipped `benign_wave` scenario demonstrates this: it reaches a **higher** peak wrist
speed than the `strike` scenario does, and still scores STRIKE 0.13, because a wave moves
an already-extended arm instead of opening it.

Then, in order:

1. **Confirmation**: `min_confirm_frames` (4) above `enter_score` (0.70) within
   `confirm_window_s` (0.6 s). A single spike does nothing.
2. **Hysteresis**: staying confirmed only needs `exit_score` (0.45). Without the gap a
   score hovering at the threshold chatters the siren.
3. **`min_alert_s`** (2.0 s): a lone confirmation still produces a perceptible signal.
4. **`max_alert_s`** (10 s) dead-man: a stuck-high scorer cannot latch the alert on.
5. **`cooldown_s`** (15 s): one incident is one alert, not a burst.
6. **`stale_input_s`** (2 s): no *new* frame forces `DEGRADED` and drops the alert. A
   frame whose stamp has not advanced does not count as new, so a producer republishing an
   identical frame reads as stale rather than as a permanent alarm.

`alert_signal` holds an independent dead-man (`max_on_s` 12 s), longer than the
recogniser's cap so the two do not race.

## Lights and siren: why there is no GPIO backend

`payload_node` (huitzilin_perception) already owns the WS2812B data line (GPIO 18) and the
siren line (gpiochip0 line 17). **Linux GPIO line requests are exclusive.** If
`alert_signal` started first and took line 17, `payload_node` could not acquire it, so a
discretionary body-motion warning would have silently disabled the **projectile** threat
annunciator, which is the safety-critical one. That priority inversion is unacceptable in
either direction of timing luck.

So `backend: hardware` is **refused**: it returns a null sink and logs a reason naming the
conflict. It does not raise, because `payload.select_backend` documents a never-raises
contract and `SAFETY_CASE.md` section 1 rates a payload/GPIO fault log-and-continue.

**The deferred fix, not built here:** `payload_node` grows a second, lower-priority input,
so one process continues to own the line and the projectile alarm always wins arbitration.
Implementing it from this package would put escalation logic inside the projectile package
and destroy the isolation this subsystem exists to keep.

## Compute and latency on a Pi 5 / OAK-D Lite

**What may be claimed:** the recogniser's own arithmetic is a few dozen floating-point
operations over a bounded window (`window_s` x `rate_hz` frames, at most 12 joints) and is
negligible against everything else on the companion computer. That is arithmetic that can
be counted, not a measurement.

**What may NOT be claimed, and is not claimed anywhere in this repository:**

- Any pose-estimation frame rate, latency or accuracy. No estimator exists here.
- Any detection, false-positive or false-negative rate for `LUNGE`, `STRIKE` or `SHOVE`.
  There is no labelled data and no ground truth. Passing self-authored scenarios is the
  same saturation `CLAUDE.md` already records for the projectile bag library.
- That this runs alongside the projectile pipeline without degrading it. Untested.
- Any end-to-end escalation latency. Under `scenario_player`, `input_latency_s` reads
  about zero because the stamp and the clock come from the same place. It measures the
  player, not a camera.
- Any power figure. `SAFETY_CASE.md` section 1 already carries an inference-power-spike
  (5 V/5 A) to companion-reset to FAILSAFE row, written before this feature existed. A
  pose estimator is exactly that spike.

Running an estimator on the OAK-D Lite's MyriadX would contend with the on-chip stereo
depth that feeds the **safety-critical** projectile detector. That direction of contention
must be stated rather than discovered.

## Known risks

- **False positives.** Thresholds are engineering starting points, not fitted values.
  Movement alone cannot establish intent: a person stumbling, catching a falling object,
  playing, or reacting to a startle can produce the same kinematics as aggression. An
  alert means "motion consistent with an aggressive action", never "an assault occurred".
- **Bias.** Kinematic thresholds in absolute metres per second are not neutral across body
  size, age, mobility-aid use, or gait. A child, a person using crutches, or someone
  dancing may score differently for reasons unrelated to aggression, in either direction.
  None of this has been characterised.
- **Occlusion.** Partial bodies contribute no lean or bilateral-reach evidence, so a
  partly occluded person scores lower. Conjunctive scoring biases toward missing an event
  rather than inventing one, which is the correct direction here, but it is still a miss.
- **Viewpoint.** All geometry is relative to the camera optical frame. Motion across the
  frame is measured well; motion along the optical axis relies on depth, the noisiest axis
  on a stereo camera. A drone that is itself moving adds ego-motion this subsystem does
  not compensate for.
- **Single-window memory.** No action longer than `window_s` is representable.
- **Self-authored evaluation.** The scenarios were written by the same hand that set the
  thresholds. Only the negative controls can fail informatively.

## Running it

```bash
ros2 launch huitzilin_action_escalation action_escalation.launch.py scenario:=lunge.yaml
```

Scenarios: `lunge.yaml`, `strike.yaml`, `shove.yaml` (expect an alert),
`benign_wave.yaml`, `ambiguous_approach.yaml` (must **not** alert).

`use_sim_time` defaults to **false** here, unlike every other launch file in this
workspace: there is no Gazebo world behind this subsystem, so nothing publishes `/clock`,
and defaulting it true would take every node down on the clock guard.

```bash
ros2 topic echo /action/escalation_event
ros2 topic echo /action/alert_state
ros2 topic echo /action/status
```

Tests run as part of `./scripts/run_tests.sh`.
