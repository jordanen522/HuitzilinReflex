# Guard alarm

A security alarm carried by the drone. It flies the perimeter of a rectangle and, while
armed, turns on lights and a siren when a person is inside that rectangle.

It works like a house alarm panel: somebody arms it, somebody disarms it, and in between it
watches. Nothing about it is personal. It reports **that** a person is in the area, never who
they are or what they are doing, which is what makes it usable for something like a curfew on
a city block without becoming a judgement about the individual crossing it.

**No camera has ever been connected to this system.** No detection rate, false-positive rate
or latency has been measured. Everything below describes what the code does, not how well it
works in the world. See [Limits](#limits).

---

## The box

The box is a rectangle in **ENU metres** (x east, y north), measured from the point where the
aircraft was armed — that is the origin `/huitzilin/odom` reports against. Park the drone at
one corner of the area you want guarded and the four numbers are distances from there.

```
        (min_x,max_y) +-----------------+ (max_x,max_y)
                      |                 |
                      |   guarded area  |   flight path = the outline
                      |                 |   alarm region = the whole rectangle
        (min_x,min_y) +-----------------+ (max_x,min_y)
                        start, then CCW
```

The drone flies the **perimeter only**, counter-clockwise from `(min_x, min_y)`. It never
crosses the interior. Rectangles are supported as well as squares, so a block that is 80 m by
25 m is expressed directly.

`contains_xy` is **two-dimensional on purpose** and never looks at height. Range to a person
is a monocular scale estimate, so the height it implies is the least trustworthy number
available; testing it would drop a tall person, a crouching one, or one at a badly estimated
range out of a box they are standing in. Edges count as inside.

### Changing it

Edit **one file**, `src/huitzilin_guard/params/guard.yaml`. The five `box_*` values appear
under both `patrol:` and `guard:` and **must be identical** — `test_guard_params.py` fails the
build if they are not. Two boxes that quietly disagree mean the drone patrols one area and
guards another, and nothing looks wrong: it flies, the detector detects, the siren stays quiet.

### Before you enlarge it

**The geofence is a circle and the box is a rectangle, so it is the corners that breach.**
`hw_frame.parm` sets `FENCE_RADIUS 10` with `FENCE_ACTION 1` (RTL). `patrol_node` checks
`box.max_corner_radius_m()` against `fence_radius_m` at construction, before the MAVLink
connect, and refuses to start naming the parameter to change. Without that check the failure
appears in the air: the drone flies, reaches a corner, breaches, and returns to launch in the
middle of its patrol, which reads as a flight-controller fault rather than a configuration one.

SITL loads no fence parameters at all, so this check is the only thing standing between a
too-large box and a surprise on the first real flight.

The shipped 5 x 5 m box reaches **7.07 m** at its far corner, inside the 10 m fence. A 9 x 9 m
box has every *edge* inside 10 m and a corner at 12.7 m, which is why the check is on the
corner.

---

## Pipeline

```
  box corners --+--> patrol path (fly the perimeter)
                +--> alarm region (is a person inside?)

  camera --> pose_detector --> /guard/detections --> guard --> /guard/alarm_request
                                                       ^                |
                          /huitzilin/odom -------------+                v
                                  armed?  <-- /guard/arm           alert_signal
                                                                   siren + lights
```

| Interface | Type | Meaning |
|---|---|---|
| `/guard/detections` | `std_msgs/String` (JSON) | body joints from `pose_detector` |
| `/huitzilin/odom` | `nav_msgs/Odometry` | the drone's own pose, ENU. Telemetry in, nothing out |
| `/guard/alarm_request` | `std_msgs/Bool` | the guard's decision, republished continuously |
| `/guard/alarm_state` | `std_msgs/Bool` | what the sink actually did |
| `/guard/status` | `std_msgs/String` (JSON, 1 Hz) | the heartbeat, below |
| `/guard/arm` | `std_srvs/SetBool` | **the** operator interface |

`/guard/arm` is **not** the motor-arming service. That one belongs to a different subsystem.
Arming the alarm on a disarmed aircraft is normal: that is a guard sitting on the bench,
watching.

`/guard/alarm_request` carries a **level**, republished on every evaluation rather than only
on edges. `alert_signal` holds its own dead-man and clears the siren when the stream stops,
which is what silences a siren if the guard node dies mid-alarm. An edge-only publisher is
indistinguishable from a dead one.

### Running it

```bash
ros2 launch huitzilin_guard guard.launch.py
ros2 service call /guard/arm std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /guard/status --full-length
```

`--full-length` matters: `ros2 topic echo` truncates long strings, and a truncated JSON
payload reads as a malformed one.

With `with_pose_detector:=false` (the default) the graph is complete except for the camera, so
a synthetic detection published by hand on `/guard/detections` exercises the whole decision
path. The detector needs a model that is not tracked in this repository; fetch it with
`scripts/fetch_pose_model.sh` and run with `PYTHONPATH=~/.local/ros-deps`.

---

## When it sounds, and when it stops

The asymmetry is deliberate in both directions.

| Setting | Default | Why |
|---|---|---|
| `confirm_frames` | 3 | One frame is a detector artefact often enough that a single-frame trigger would make the siren untrustworthy, and an alarm nobody believes is worse than none |
| `clear_after_s` | 3 s | Of an **empty box**, not one empty frame. A person standing still is what the detector misses most |
| `min_alert_s` | 3 s | A brief intrusion must still produce a signal somebody can perceive |
| `max_alert_s` | 300 s | Stuck-detector guard, long on purpose: a real intruder standing still must not be able to outwait the alarm |
| `stale_input_s` | 5 s | Dead-man on the frames feeding a **sounding** alarm |

**There is no cooldown.** Someone still in the box is still an intruder, and a quiet period
after a clear is a window in which the alarm deliberately does not do its job.

### Silence is not an all-clear

`pose_detector` publishes only when it **sees** somebody, so an empty box and a dead detector
both look like silence. Two consequences, both load-bearing:

- Silence alone is **never** treated as a fault. Doing so would leave the system permanently
  faulted on any quiet night, which is exactly how a real fault gets trained out of an
  operator's attention.
- An alarm only stops with the reason *"box clear"* when a frame has actually **arrived** and
  reported nobody there. If the detector dies mid-alarm, no frame ever observes an empty box,
  so the dead-man stops it instead and says so. Elapsed time is not evidence.

`alert_signal`'s `max_on_s` (310 s) must exceed the guard's `max_alert_s` (300 s). If the sink
expired first it would silence the siren while the guard still believed it was sounding.

---

## While disarmed

The node still parses detections and still reports `person_in_box` in its status. It just
fires no alarm. Reporting an empty box while somebody stands in it would be a lie told by the
only instrument an operator has.

Arming **resets the confirm window**, so somebody already standing in the box when the panel is
armed is confirmed over the next three frames rather than sirened instantly.

### Status heartbeat

```json
{"armed": false, "alarm": false, "person_in_box": true, "frames_seen": 25972,
 "frames_rejected": 0, "frames_unplaceable": 2, "consecutive_in_box": 0,
 "secs_since_detection": 0.03, "have_odom": true, "recording": false,
 "last_stop_reason": "box clear for 3 s", "box": [0.0, 5.0, 0.0, 5.0]}
```

- `frames_rejected` is the **privacy telemetry**: non-zero means something upstream is sending
  fields this subsystem refuses.
- `frames_unplaceable` counts detections that arrived with no odometry or an unconverged
  attitude. Those are dropped, never guessed at: calling them "not in the box" would be a
  guess in the unsafe direction.
- `secs_since_detection` is tracked armed **or not**, so it never reads as a dead detector
  while `frames_seen` climbs.
- `recording` is stated in the heartbeat so the property is checkable at runtime by anyone
  with a terminal, rather than only promised in this document.

---

## Privacy

The guarantee is not a policy this code is trusted to honour; it is a parser that rejects
anything it was not promised, plus tests that assert it against the source.

- `pose_frame.py` accepts **COCO-17 minus keypoints 0-4** — nose, both eyes, both ears are
  absent, so twelve limb and torso joints remain and no face can be reconstructed. A frame
  carrying a face joint, an image, a crop, an embedding or a string `subject` is discarded
  **whole**, not partially used.
- No recording path exists in any node. `record_video: true` makes `pose_detector` refuse to
  start.
- Only `pose_detector_node.py` may import `cv2` or `onnxruntime`; a second file doing so fails
  `test_privacy_invariants.py`. It is the only place that ever holds an image.
- The published status carries no identifying field, not even the position of the person or
  the subject index the parser accepted. An operator learns the area is occupied. That is all
  a presence alarm is entitled to tell them.

No database, no SQL, no network notification, no stored video. The alarm is entirely on the
aircraft.

## The alarm never flies the aircraft

`package.xml` declares no `geometry_msgs`, so no `Twist` can be constructed anywhere in the
package. `test_isolation.py` additionally asserts, against the source:

- no file names a flight or projectile topic in a string constant (docstrings excluded, so the
  reasoning can be written down without the ban deleting the explanation);
- no file calls `create_client` — this subsystem answers requests, it does not make them;
- only `guard_node.py` imports `std_srvs`, and only to **serve** the arm switch;
- no file imports `mav_bridge`, which owns the MAVLink connection and the setpoint senders.

`std_srvs` used to be banned outright. The arm switch needs it, so the guarantee moved to the
narrower rules above rather than being dropped.

### No GPIO backend, deliberately

`payload_node` already owns the WS2812B data line (SPI0, GPIO 10) and the siren line (GPIO
17), and **Linux GPIO line requests are exclusive**. A second process taking those lines
would silently stop the *projectile* alarm from firing — a discretionary intrusion warning
disabling the safety-critical annunciator. That is a priority inversion and is not acceptable
in either direction of timing luck.

So `backend: hardware` **logs a refusal and runs inert** rather than raising: a node killed by
a params typo is worse than one running loudly degraded.

The correct hardware design, deferred and deliberately not built here: `payload_node` grows a
second, lower-priority input, so one process keeps owning the line and the projectile alarm
always wins arbitration. Building that from this package would put guard logic inside the
projectile package and destroy the isolation this subsystem exists to keep.

---

## Limits

State these; do not paper over them.

- **Coverage is partial.** A forward-facing camera on a perimeter loop does not see the middle
  of a large box. It reliably catches somebody crossing or near the boundary; for a city block,
  most of the interior is unobserved most of the time. The alarm region is the whole rectangle,
  but the *observed* area is a moving wedge along the edge.
- **Range is a monocular scale estimate** from bounding-box height against an assumed standing
  adult (1.70 m). A crouching, seated or partly framed person gets a proportionally wrong range
  and can land on the wrong side of a box edge. A `depth` mode exists in the detector and **has
  never been run**.
- **No camera has ever been connected.** No detection rate, false-positive rate, or latency
  figure exists. Claim none.
- **The camera mount offset is not modelled.** A detection is placed from the body origin, so a
  lens mounted a few centimetres forward contributes an error far smaller than the range
  estimate above, but not zero.
- **One person per frame.** `pose_detector.decode` returns only the highest-confidence person.
  A second person in the same frame is not separately reported, which does not affect the
  alarm: the question is only whether *anybody* is inside.
- **The box origin is the arming point.** A box written against any other origin is not wrong
  at construction and cannot be detected in software; it simply guards the wrong patch of
  ground.
