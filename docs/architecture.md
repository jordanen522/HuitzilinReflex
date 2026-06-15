# Node Graph — Project HuitzilinReflex

## Nodes

| Node | Responsibility | Inputs | Outputs | Rate | Runs on |
|---|---|---|---|---|---|
| camera_driver | Publishes depth + point cloud from OAK-D Lite | Hardware | `/oak/points`, `/oak/depth` | 30 Hz | Pi (real) / dev PC (sim) |
| evasion_node | Detection + Kalman filter + dodge trigger | `/oak/points`, `/oak/depth` | `/threat/centroid`, `/threat/intercept`, `/cmd/evade` | 30 Hz | Pi / dev PC |
| patrol_node | Autonomous path following | State machine mode | `/patrol/setpoint` | 10 Hz | Pi / dev PC |
| mavlink_bridge | Translates ROS commands to ArduPilot MAVLink | `/cmd/evade`, `/patrol/setpoint` | ArduPilot FC | 10 Hz | Pi / dev PC |
| payload_node | Controls LED strip + buzzer via GPIO | `/payload/alarm` | GPIO hardware | on-event | Pi only |
| supervisor_node | State machine, fault monitoring | All node statuses | Mode commands | 1 Hz | Pi / dev PC |

## Diagram

```
camera_driver → /oak/points, /oak/depth → evasion_node → /cmd/evade → mavlink_bridge → ArduPilot
                                           evasion_node → /payload/alarm → payload_node
supervisor_node → mode commands → patrol_node → /patrol/setpoint → mavlink_bridge
```

## Message Contracts

| Topic | Type | QoS | Frame | Rate |
|---|---|---|---|---|
| `/oak/points` | `sensor_msgs/PointCloud2` | Best-effort, keep-last 1 | `camera_optical_frame` | 30 Hz |
| `/oak/depth` | `sensor_msgs/Image` | Best-effort, keep-last 1 | `camera_optical_frame` | 30 Hz |
| `/threat/centroid` | `geometry_msgs/PointStamped` | Reliable | `base_link` | 30 Hz |
| `/threat/intercept` | `geometry_msgs/PointStamped` | Reliable | `base_link` | 30 Hz |
| `/cmd/evade` | `geometry_msgs/Twist` | Reliable | `base_link` | 10 Hz |
| `/patrol/setpoint` | `geometry_msgs/PoseStamped` | Reliable | `map` | 10 Hz |
| `/payload/alarm` | `std_msgs/Bool` | Reliable | N/A | on-event |