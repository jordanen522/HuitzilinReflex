# Safety Case — Project HuitzilinReflex

Owns every binding safety, legal, and privacy rule for the project.

## 1. Failure Mode & Effects Analysis (FMEA)

| Failure Mode | Cause | Effect | Detection | Safe Response | Severity |
|---|---|---|---|---|---|
| Sensor dropout | OAK-D Lite disconnect | No threat detection | Topic timeout | → FAILSAFE | High |
| RC/link loss | Radio interference | No manual override | Heartbeat timeout | → FAILSAFE → RTL | Critical |
| Low battery | Insufficient charge | Motor cutout mid-flight | Voltage monitor | → RTL/LAND | Critical |
| FC failsafe | ArduPilot internal fault | Loss of flight control | FC status message | → FAILSAFE | Critical |
| Uncommanded flyaway | Software bug / bad command | Uncontrolled flight | Geofence breach | Kill-switch / RTL | Critical |
| GUIDED setpoint stream loss | patrol/evasion node stalls or `cmd_vel` stops | Stale velocity setpoint → risk of coast/lunge | Bridge watchdog (`cmd_timeout_s`) | → zero-velocity hold; then mode → LOITER/RTL | High |
| Dodge into the ground | Descending threat → geometric escape perpendicular points down (measured `dir_body` z = −0.83 at 2 m AGL) | Drone flies itself into the surface; observed `Crash`/`Disarm` in SITL | Pre-command: altitude vs planned descent | `clamp_dodge_to_clearance` caps descent to the headroom above `dodge_floor_m` (1.0 m) and re-aims horizontally; never escapes upward into a descending threat | Critical |
| GPIO/payload fault | Wiring fault | LED/buzzer fails | Node error | Log + continue | Low |
| Pi power brownout | Inference power spike (5V/5A) | Companion computer resets | Heartbeat loss | → FAILSAFE | High |

## 2. Geofence & RTL Behavior

### Geofence
- Shape: cylinder around test area
- Max radius: 10 m from launch point
- Max altitude: 5 m AGL
- ArduPilot params: `FENCE_ENABLE=1`, `FENCE_TYPE=3` (circle + altitude), `FENCE_RADIUS=10`, `FENCE_ALT_MAX=5`

All of these ship in `src/huitzilin_sim/params/hw_frame.parm` and are asserted by
`test_hw_config.py`. Load that file; do not type them in by hand.

### RTL Triggers
| Trigger | ArduPilot Param |
|---|---|
| Link loss | `FS_THR_ENABLE=1` |
| Low battery | `BATT_FS_LOW_ACT=2` (RTL) |
| Fence breach | `FENCE_ACTION=1` (RTL) |

### RTL Sequence
1. Climb to `RTL_ALT` = **4 m** (`RTL_ALT 400`; the parameter is in centimetres)
2. Fly back to launch point
3. Descend and land
4. Disarm

**`RTL_ALT` must stay below `FENCE_ALT_MAX`.** ArduPilot's default is 1500 cm = 15 m,
three times the 5 m fence ceiling above, so an RTL triggered by a fence breach would
climb straight through the fence it is responding to. 4 m leaves 1 m of ceiling margin
and still clears the 2 m takeoff altitude. `param_audit.check_fence_consistency`
asserts `RTL_ALT / 100 < FENCE_ALT_MAX` — note the conversion, since a plausible-looking
`RTL_ALT` of `4` would mean 4 cm.

### Fail-Safe Default
Any undefined fault → FAILSAFE (calm geofenced hover) → RTL → land. **Never** an evasive
maneuver on a fault — the reflex is for projectiles only. All safe states trace to
`docs/state_machine.md`.

## 3. Kill-Switch Design

### Concept
A dedicated RC channel mapped to ArduPilot's motor emergency stop.
Cuts all motors immediately regardless of flight mode.

### ArduPilot Mapping
- `RC7_OPTION=31` (or any free channel) → Motor Emergency Stop
- Activating the switch disarms motors instantly
- Must be re-armed manually after activation

### Operating Rule
- Kill-switch must be in hand for all powered tests, no exceptions
- Operator's thumb stays on the switch during any armed state
- The switch must be wired and bench-tested before any powered flight. It has not been
  built yet — the project is still in simulation, and nothing here has flown.

## 4. Test Enclosure & Operating Rules

### Operating Safety Rules
- Projectile evasion testing only inside netting with soft projectiles
- Props-off rule enforced on the bench at all times
- Kill-switch within reach of operator during all armed tests
- Visual line-of-sight maintained at all times
- Defined abort criteria: any unexpected behavior → kill-switch immediately

### Legal & Battery Rules
- **FAA:** register, broadcast Remote ID, maintain visual line of sight, no autonomous
  flight over people or vehicles without a waiver, stay clear of controlled airspace.
- **LiPo:** charge on the ISDT inside the fireproof bag, never unattended; store at
  storage charge; 18650s cased, never loose.

### Privacy Rules
- The drone camera records depth data only, not RGB video by default
- Any footage captured stays on local storage, never uploaded
- Retention: footage deleted after 30 days unless needed for analysis
- Camera never pointed at anyone who has not explicitly consented
- The warning payload (buzzer + LED) is a safety signal only — never used to harass or follow a person

### Abort Criteria
| Situation | Action |
|---|---|
| Unexpected flight path | Kill-switch immediately |
| Any person enters test area | Land immediately |
| Payload malfunction | Land, props off, inspect |
| Loss of visual line-of-sight | Kill-switch immediately |

## 5. Requirements Traceability

| Safety Claim | REQ ID |
|---|---|
| System responds to link loss safely | REQ-14 |
| Kill-switch cuts motors immediately | REQ-15 |
| All faults land in a safe state | REQ-16 |
