# Flight/Behavior State Machine — Project HuitzilinReflex

State and fault names below are the `State` / `Fault` enum members in
`src/huitzilin_sim/huitzilin_sim/supervisor.py`, spelled exactly so they grep.

## States

| State | Description |
|---|---|
| DISARMED | Motors off, system idle |
| ARMING | Pre-arm checks running |
| TAKEOFF | Climbing to patrol altitude |
| PATROL | Autonomous path following, payloads active |
| EVADE | Dodge maneuver executing + alarm firing |
| RTL_LAND | Returning to launch and landing |
| FAILSAFE | Emergency hold/land on fault |

## Transitions

| From | Trigger | To |
|---|---|---|
| DISARMED | Arm command received, pre-arm checks pass | ARMING |
| ARMING | Armed successfully | TAKEOFF |
| TAKEOFF | Target altitude reached | PATROL |
| PATROL | Threat detected | EVADE |
| EVADE | Dodge complete | PATROL |
| PATROL/EVADE | LOW_BATTERY or FENCE_BREACH | RTL_LAND |
| PATROL/EVADE | Any other fault | FAILSAFE |
| FAILSAFE | Stable hover achieved | RTL_LAND |
| RTL_LAND | Landed | DISARMED |

## Fault routing

`FAULT_RESPONSE` (`supervisor.py`) maps `LOW_BATTERY` and `FENCE_BREACH` to `RTL_LAND`;
`fault_response()` sends every other fault — `SENSOR_DROPOUT`, `LINK_LOSS`,
`COMPANION_LOSS`, `FC_FAILSAFE`, `SETPOINT_STALL`, `UNKNOWN` — to `FAILSAFE`.

`detect_faults()` returns nothing while disarmed: on the bench most watched topics are
legitimately quiet. A watch whose timeout is `0.0` is skipped entirely, which is the only
way to say "this configuration does not publish that topic".

Which fault each row answers, and why no fault path reaches EVADE: `docs/SAFETY_CASE.md` §2.
