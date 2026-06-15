# Safety Case — Project HuitzilinReflex

## 1. Failure Mode & Effects Analysis (FMEA)

| Failure Mode | Cause | Effect | Detection | Safe Response | Severity |
|---|---|---|---|---|---|
| Sensor dropout | OAK-D Lite disconnect | No threat detection | Topic timeout | → FAILSAFE | High |
| RC/link loss | Radio interference | No manual override | Heartbeat timeout | → FAILSAFE → RTL | Critical |
| Low battery | Insufficient charge | Motor cutout mid-flight | Voltage monitor | → RTL/LAND | Critical |
| FC failsafe | ArduPilot internal fault | Loss of flight control | FC status message | → FAILSAFE | Critical |
| Uncommanded flyaway | Software bug / bad command | Uncontrolled flight | Geofence breach | Kill-switch / RTL | Critical |
| GPIO/payload fault | Wiring fault | LED/buzzer fails | Node error | Log + continue | Low |
| Pi power brownout | Inference power spike (5V/5A) | Companion computer resets | Heartbeat loss | → FAILSAFE | High |

## 2. Geofence & RTL Behavior

### Geofence
- Shape: cylinder around test area
- Max radius: 10 m from launch point
- Max altitude: 5 m AGL
- ArduPilot params: `FENCE_ENABLE=1`, `FENCE_TYPE=3` (circle + altitude), `FENCE_RADIUS=10`, `FENCE_ALT_MAX=5`

### RTL Triggers
| Trigger | ArduPilot Param |
|---|---|
| Link loss | `FS_THR_ENABLE=1` |
| Low battery | `BATT_FS_LOW_ACT=2` (RTL) |
| Fence breach | `FENCE_ACTION=1` (RTL) |

### RTL Sequence
1. Climb to `RTL_ALT` (default 15 m)
2. Fly back to launch point
3. Descend and land
4. Disarm

### Fail-Safe Default
FAILSAFE state = calm hover → RTL → land. Never an evasive maneuver on a fault.

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
- Switch is wired and tested in Phase B before any real flights

## 4. Test Enclosure & Operating Rules

### Operating Safety Rules
- Projectile evasion testing only inside netting with soft projectiles
- Props-off rule enforced on the bench at all times
- Kill-switch within reach of operator during all armed tests
- Visual line-of-sight maintained at all times
- Defined abort criteria: any unexpected behavior → kill-switch immediately

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

## 5. Fail-Safe Defaults

- Default on any undefined fault: → FAILSAFE (calm hover) → RTL → land
- FAILSAFE never triggers an evasive maneuver — reflex is for projectiles only
- All safe states trace to the state machine in `docs/state_machine.md`

## Requirements Traceability

| Safety Claim | REQ ID |
|---|---|
| Drone stays within geofence | REQ-X |
| System responds to link loss safely | REQ-X |
| Kill-switch cuts motors immediately | REQ-X |
| Camera privacy rules enforced | REQ-X |
| All faults land in a safe state | REQ-X |