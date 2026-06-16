# Node Graph — Project HuitzilinReflex

> Topic/service contracts reconciled to the **Week 2 decision** (2026-06-15): the active flight-control path uses the `/huitzilin/*` namespace (playbook §3). Perception, evasion, and payload nodes are future-week (Wk3–6); their contracts are marked provisional. See `docs/WEEK2_PLAN.md`.

## Nodes

| Node | Responsibility | Inputs | Outputs | Rate | Phase | Runs on |
|---|---|---|---|---|---|---|
| mav_bridge | ROS 2 ↔ ArduPilot bridge (pymavlink); single NED↔ENU conversion point | `/huitzilin/cmd_vel`; services `/huitzilin/arm`, `/huitzilin/takeoff`, `/huitzilin/set_mode` | `/huitzilin/odom`, `/huitzilin/state`; ArduPilot FC (pymavlink) | 10 Hz | **Wk2 (active)** | Pi / dev PC |
| patrol_node | Autonomous patrol path-following | `/huitzilin/odom`; service `/huitzilin/start_patrol` | `/huitzilin/cmd_vel` (velocity mode) **or** position targets direct to FC; `/huitzilin/mission_marker` | 10 Hz | **Wk2 (active)** | Pi / dev PC |
| camera_driver | Publishes depth + point cloud from OAK-D Lite | Hardware / sim sensor | `/oak/points`, `/oak/depth` | 30 Hz | Wk3 | Pi (real) / dev PC (sim) |
| evasion_node | Detection + Kalman filter + dodge trigger | `/oak/points`, `/oak/depth` | `/threat/centroid`, `/threat/intercept`, `/cmd/evade` | 30 Hz | Wk4 | Pi / dev PC |
| payload_node | Controls LED strip + buzzer via GPIO | `/payload/alarm` | GPIO hardware | on-event | Wk6 | Pi only |
| supervisor_node | State machine, fault monitoring | All node statuses | Mode commands (`/huitzilin/set_mode`, `/huitzilin/start_patrol`) | 1 Hz | Wk2+ | Pi / dev PC |

> Week 1's `mavlink_bridge` node (subscribed `/cmd/evade`) is **superseded by `mav_bridge` in the `huitzilin_sim` package**; retire `mavlink_bridge` once `huitzilin_sim` is green. In Week 4 the evasion path (`/cmd/evade`) will command through the same `mav_bridge` velocity path — its final contract is set in Week 4.

## Diagram

```
[ Wk2 active ]
supervisor_node → mode/start → patrol_node → /huitzilin/cmd_vel → mav_bridge → ArduPilot (pymavlink)
                                             mav_bridge → /huitzilin/odom, /huitzilin/state → patrol_node / all
                                             patrol_node → /huitzilin/mission_marker → RViz

[ Wk3–6 future ]
camera_driver → /oak/points, /oak/depth → evasion_node → /cmd/evade → mav_bridge → ArduPilot
                                          evasion_node → /payload/alarm → payload_node
```

## Message & Service Contracts

### Week 2 — active (`/huitzilin/*`)

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

**Frame rule:** ArduPilot speaks NED; all ROS topics are ENU/FLU; the **only** NED↔ENU conversion lives in `mav_bridge` (see `docs/frames.md`). Velocity setpoints to ArduPilot use `MAV_FRAME_BODY_OFFSET_NED`; absolute position setpoints use `MAV_FRAME_LOCAL_NED`.

### Future weeks (Wk3–6) — contracts provisional

| Topic | Type | QoS | Frame | Phase |
|---|---|---|---|---|
| `/oak/points` | `sensor_msgs/PointCloud2` | Best-effort, keep-last 1 | `camera_optical_frame` | Wk3 |
| `/oak/depth` | `sensor_msgs/Image` | Best-effort, keep-last 1 | `camera_optical_frame` | Wk3 |
| `/threat/centroid` | `geometry_msgs/PointStamped` | Reliable | `base_link` | Wk4 |
| `/threat/intercept` | `geometry_msgs/PointStamped` | Reliable | `base_link` | Wk4 |
| `/cmd/evade` | `geometry_msgs/Twist` | Reliable | `base_link` (FLU) | Wk4 (folds into the bridge cmd path) |
| `/payload/alarm` | `std_msgs/Bool` | Reliable | N/A | Wk6 |
