# Node Graph — Project HuitzilinReflex

Active namespace is `/huitzilin/*`. Weeks 1–3 contracts are **active**; Week 4–6
contracts are provisional until their nodes land. In Week 4 the evasion path
(`/cmd/evade`) commands through the same `mav_bridge` velocity path.

## Nodes

| Node | Responsibility | Inputs | Outputs | Rate | Phase | Runs on |
|---|---|---|---|---|---|---|
| mav_bridge | ROS 2 ↔ ArduPilot bridge (pymavlink); single NED↔ENU conversion point | `/huitzilin/cmd_vel`; services `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` | `/huitzilin/odom`, `/huitzilin/state`; ArduPilot FC | 30 Hz odom | **active (Wk2)** | Pi / dev PC |
| patrol_node | Autonomous patrol path-following | `/huitzilin/odom`; service `/huitzilin/start_patrol` | position targets to FC; `/huitzilin/mission_marker` | 10 Hz | **active (Wk2)** | Pi / dev PC |
| camera_driver | Depth + point cloud (sim: Gazebo bridge; real: OAK-D Lite) | sensor | `/oak/points`, `/oak/depth` | 15 Hz sim (30 Hz real target) | **active, sim (Wk3)** | Pi (real, Wk6) / Dell (sim) |
| detector_node | ROI gate, egomotion-compensated differencing, clustering, centroid | `/oak/points`, `/huitzilin/odom` (TF) | `/threat/centroid` + RViz marker | per cloud | **active, sim (Wk3)** | Pi / dev PC |
| evasion_node | Kalman filter + dodge trigger | `/threat/centroid` | `/threat/intercept`, `/cmd/evade` | 30 Hz | Wk4 | Pi / dev PC |
| payload_node | LED strip + buzzer via GPIO | `/payload/alarm` | GPIO | on-event | Wk6 | Pi only |
| supervisor_node | State machine, fault monitoring | node statuses | `/huitzilin/set_mode`, `/huitzilin/start_patrol` | 1 Hz | Wk2+ | Pi / dev PC |

## Diagram

```
[ active ]
supervisor_node → mode/start → patrol_node → position targets → mav_bridge → ArduPilot (pymavlink)
                               mav_bridge → /huitzilin/odom, /huitzilin/state → all
camera_driver → /oak/points, /oak/depth → detector_node → /threat/centroid → RViz marker

[ Wk4–6 future ]
detector_node → /threat/centroid → evasion_node → /threat/intercept, /cmd/evade → mav_bridge
                                   evasion_node → /payload/alarm → payload_node
```

## Message & Service Contracts

### Active — flight (`/huitzilin/*`, Wk2)

| Interface | Type | Direction | QoS | Frame |
|---|---|---|---|---|
| `/huitzilin/cmd_vel` | `geometry_msgs/Twist` | patrol → bridge | Reliable, keep-last 10 | body **FLU** |
| `/huitzilin/odom` | `nav_msgs/Odometry` | bridge → all | Reliable, keep-last 10 | `odom` (ENU) |
| `/huitzilin/state` | `std_msgs/String` (JSON) | bridge → all | Reliable | N/A |
| `/huitzilin/mission_marker` | `visualization_msgs/MarkerArray` | patrol → RViz | Reliable, keep-last 1 | `odom` (ENU) |
| `/huitzilin/arm` | `std_srvs/SetBool` | → bridge | service | N/A |
| `/huitzilin/takeoff` | `std_srvs/Trigger` | → bridge | service | N/A |
| `/huitzilin/set_mode` | `std_srvs/Trigger` (+ `mode` param) | → bridge | service | N/A |
| `/huitzilin/start_patrol` | `std_srvs/SetBool` | → patrol | service | N/A |

**Frame rule:** ArduPilot speaks NED; all ROS topics are ENU/FLU; the **only** NED↔ENU
conversion lives in `mav_bridge` (see `docs/frames.md`). Velocity setpoints use
`MAV_FRAME_BODY_OFFSET_NED`; absolute position setpoints use `MAV_FRAME_LOCAL_NED`.

### Active — perception (sim, promoted W3-19)

| Topic | Type | QoS | Frame |
|---|---|---|---|
| `/oak/points` | `sensor_msgs/PointCloud2` | Best-effort, keep-last 1 | `camera_optical_frame` |
| `/oak/depth` | `sensor_msgs/Image` | Best-effort, keep-last 1 | `camera_optical_frame` |
| `/threat/centroid` | `geometry_msgs/PointStamped` | Reliable | `base_link` |

### Provisional (Wk4–6)

| Topic | Type | QoS | Frame | Phase |
|---|---|---|---|---|
| `/threat/intercept` | `geometry_msgs/PointStamped` | Reliable | `base_link` | Wk4 |
| `/cmd/evade` | `geometry_msgs/Twist` | Reliable | `base_link` (FLU) | Wk4 (folds into the bridge cmd path) |
| `/payload/alarm` | `std_msgs/Bool` | Reliable | N/A | Wk6 |
