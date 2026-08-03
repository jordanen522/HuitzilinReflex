# Node Graph — Project HuitzilinReflex

Active namespace is `/huitzilin/*`. Every contract below is **active** — the
Week 5 software lane landed `payload_node` and `supervisor_node`, so nothing here
is provisional any more. The evasion path (`/cmd/evade`) commands through the
same `mav_bridge` velocity path and preempts `/huitzilin/cmd_vel` while fresh.

## Nodes

| Node | Responsibility | Inputs | Outputs | Rate | Phase | Runs on |
|---|---|---|---|---|---|---|
| mav_bridge | ROS 2 ↔ ArduPilot bridge (pymavlink); single NED↔ENU conversion point | `/huitzilin/cmd_vel`; `/cmd/evade` (priority); services `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` | `/huitzilin/odom`, `/huitzilin/state`; ArduPilot FC | 30 Hz odom | **active (Wk2)** | Pi / dev PC |
| patrol_node | Autonomous patrol path-following | `/huitzilin/odom`; service `/huitzilin/start_patrol` | position targets to FC; `/huitzilin/mission_marker` | 10 Hz | **active (Wk2)** | Pi / dev PC |
| camera_driver | Depth + point cloud (sim: Gazebo bridge; real: OAK-D Lite) | sensor | `/oak/points`, `/oak/depth` | 15 Hz sim (30 Hz real target) | **active, sim (Wk3)** | Pi (real, Wk6) / Dell (sim) |
| detector_node | ROI gate, egomotion-compensated differencing, clustering, centroid | `/oak/points`, `/huitzilin/odom` (TF) | `/threat/centroid` + RViz marker | per cloud | **active, sim (Wk3)** | Pi / dev PC |
| evasion_node | Kalman filter + dodge trigger + patrol pause/resume | `/threat/centroid`, `/huitzilin/odom` | `/threat/intercept`, `/cmd/evade`, `/payload/alarm` (mock), `/threat/evade_event` | per centroid; 20 Hz while evading | **active, sim (Wk4)** | Pi / dev PC |
| payload_node | LED strip + siren via GPIO; every backend degrades to a logging no-op if the library is missing | `/payload/alarm` | GPIO (WS2812B via level shifter; siren via transistor) | on-event, 10 Hz dead-man tick | **active (Wk5)** | Pi (real output) / anywhere (no-op) |
| supervisor_node | 7-state machine + FMEA fault monitor; faults resolve before the threat branch, so no fault can produce EVADE | message ages on `/huitzilin/odom`, `/huitzilin/state`, `/oak/points`, `/huitzilin/patrol_state`, `/huitzilin/cmd_vel`, `/payload/alarm` | `/huitzilin/set_mode`, `/huitzilin/start_patrol` (edge-triggered) | 1 Hz | **active (Wk5)**, opt-in via `with_supervisor:=true` | Pi / dev PC |

## Diagram

```
[ active ]
supervisor_node → mode/start → patrol_node → position targets → mav_bridge → ArduPilot (pymavlink)
                               mav_bridge → /huitzilin/odom, /huitzilin/state → all
camera_driver → /oak/points, /oak/depth → detector_node → /threat/centroid → RViz marker
detector_node → /threat/centroid → evasion_node → /threat/intercept, /cmd/evade → mav_bridge
                                   evasion_node → /payload/alarm → payload_node → LED + siren (GPIO)
                                   evasion_node → /huitzilin/start_patrol (pause/resume)
supervisor_node watches every topic above for staleness → /huitzilin/set_mode, /huitzilin/start_patrol
```

A watch whose timeout is `0.0` in `supervisor.yaml` is disabled, and one of them
ships disabled: `cmd_vel_timeout_s`, because position-mode patrol drives MAVLink
directly and publishes no `/huitzilin/cmd_vel`. See `docs/state_machine.md`.

## Message & Service Contracts

### Active — flight (`/huitzilin/*`, Wk2)

| Interface | Type | Direction | QoS | Frame |
|---|---|---|---|---|
| `/huitzilin/cmd_vel` | `geometry_msgs/Twist` | patrol → bridge | Reliable, keep-last 10 | body **FLU** |
| `/huitzilin/odom` | `nav_msgs/Odometry` | bridge → all | Reliable, keep-last 10 | `odom` (ENU) |
| `/huitzilin/state` | `std_msgs/String` (JSON) | bridge → all | Reliable | N/A |
| `/huitzilin/patrol_state` | `std_msgs/String` (JSON) | patrol → supervisor, dodge_battery | Reliable, keep-last 10 | N/A |
| `/huitzilin/mission_marker` | `visualization_msgs/MarkerArray` | patrol → RViz | Reliable, keep-last 1 | `odom` (ENU) |
| `/huitzilin/arm` | `std_srvs/SetBool` | → bridge | service | N/A |
| `/huitzilin/takeoff` | `std_srvs/Trigger` | → bridge | service | N/A |
| `/huitzilin/set_mode` | `std_srvs/Trigger` (+ `mode` param) | → bridge | service | N/A |
| `/huitzilin/start_patrol` | `std_srvs/SetBool` | → patrol | service | N/A |

The four service types are exact and are not interchangeable — `arm` and
`start_patrol` are `SetBool`, `takeoff` and `set_mode` are `Trigger`. `set_mode`
takes the mode as the bridge's `mode` **parameter**, set before the trigger.

`/huitzilin/state` JSON keys: `n`, `e`, `alt`, `yaw` (position/attitude), plus `armed`,
`mode`, `batt_v`, `batt_pct`, `fc_failsafe` from `HEARTBEAT` and `SYS_STATUS`. **A key
may be `null`**: absent telemetry stays absent rather than defaulting, so a consumer can
distinguish "never reported" from "reported false". `batt_v` is `null` rather than 65.5
when ArduPilot reports its 65535 mV unknown sentinel.

`/huitzilin/patrol_state` is published at 10 Hz by `patrol_node`. It has two
consumers and both treat its absence as fatal rather than as a default:
`supervisor_node` watches its age (`patrol_state_timeout_s: 2.0` → `COMPANION_LOSS`),
and `throw_window.py` refuses to score a throw without it rather than assuming the
drone was patrolling. It gates the Week 4 throw window.

**Frame rule:** ArduPilot speaks NED; all ROS topics are ENU/FLU; the **only** NED↔ENU
conversion lives in `mav_bridge` (see `docs/frames.md`). Velocity setpoints use
`MAV_FRAME_BODY_OFFSET_NED`; absolute position setpoints use `MAV_FRAME_LOCAL_NED`.

### Active — perception (sim, promoted W3-19)

| Topic | Type | QoS | Frame |
|---|---|---|---|
| `/oak/points` | `sensor_msgs/PointCloud2` | Best-effort, keep-last 1 | `camera_optical_frame` |
| `/oak/depth` | `sensor_msgs/Image` | Best-effort, keep-last 1 | `camera_optical_frame` |
| `/threat/centroid` | `geometry_msgs/PointStamped` | Reliable | `base_link` |
| `/threat/marker` | `visualization_msgs/Marker` | Best-effort, keep-last 1 | `base_link` |

### Active — evasion (sim, promoted W4)

| Interface | Type | QoS | Frame |
|---|---|---|---|
| `/threat/intercept` | `geometry_msgs/PointStamped` | Reliable | `base_link` |
| `/cmd/evade` | `geometry_msgs/Twist` | Reliable | body **FLU** (bridge priority over `/huitzilin/cmd_vel`) |
| `/threat/evade_event` | `std_msgs/String` (JSON) | Reliable | N/A |
| `/threat/intercept_marker` | `visualization_msgs/Marker` | Reliable | `base_link` |
| `/payload/alarm` | `std_msgs/Bool` | Reliable | N/A (consumer: `payload_node`) |

Both marker topics are RViz-only: nothing subscribes to them in flight, and the
detector publishes its own best-effort rather than reliable, so a slow RViz cannot
back-pressure the detection path.
