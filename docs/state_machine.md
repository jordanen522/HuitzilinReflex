# Flight/Behavior State Machine — Project HuitzilinReflex

## States

| State | Description |
|---|---|
| DISARMED | Motors off, system idle |
| ARMING | Pre-arm checks running |
| TAKEOFF | Climbing to patrol altitude |
| PATROL | Autonomous path following, payloads active |
| EVADE | Dodge maneuver executing + alarm firing |
| RTL/LAND | Returning to launch and landing |
| FAILSAFE | Emergency hold/land on fault |

## Transitions

| From | Trigger | To |
|---|---|---|
| DISARMED | Arm command received, pre-arm checks pass | ARMING |
| ARMING | Armed successfully | TAKEOFF |
| TAKEOFF | Target altitude reached | PATROL |
| PATROL | Threat detected | EVADE |
| EVADE | Dodge complete | PATROL |
| PATROL/EVADE | Link loss / low battery / sensor dropout | FAILSAFE |
| PATROL/EVADE | Fence breach | RTL/LAND |
| FAILSAFE | Stable hover achieved | RTL/LAND |
| RTL/LAND | Landed | DISARMED |

## Fault Defaults
- Any undefined condition → FAILSAFE
- FAILSAFE behavior: calm hover → RTL → land (never an evasive maneuver)