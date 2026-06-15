# Coordinate Frames & TF Tree — Project HuitzilinReflex

## Frames

| Frame | Convention | Description |
|---|---|---|
| `map` | ENU | Fixed world frame, origin at launch point |
| `odom` | ENU | Odometry frame, drifts over time |
| `base_link` | ENU | Drone body center |
| `camera_link` | ENU | OAK-D Lite mount point on body |
| `camera_optical_frame` | ROS optical (Z forward) | OAK-D Lite optical axis |

## NED vs ENU

- **ArduPilot** uses NED (North-East-Down)
- **ROS 2** uses ENU (East-North-Up) per REP-103
- **Conversion happens in the MAVLink bridge node only** — all other nodes work exclusively in ENU

## Static Transforms

- `base_link` → `camera_link`: fixed offset based on physical mount position (to be measured in Phase B)
- `camera_link` → `camera_optical_frame`: standard ROS optical frame rotation

## TF Tree

```
map
└── odom
    └── base_link
        └── camera_link
            └── camera_optical_frame
```