# Coordinate Frames & TF Tree — Project HuitzilinReflex

## Frames

`odom` is the root. There is **no `map` frame** in this project: nothing publishes
one and nothing consumes one. `detector.yaml` sets `fixed_frame: "odom"`, and
`mav_bridge_node` stamps odometry `odom` → `base_link`. Do not add `map` to a
launch file or an RViz config expecting it to be filled in.

| Frame | Convention | Description |
|---|---|---|
| `odom` | ENU (world-fixed) | Odometry frame, origin at launch point, drifts over time |
| `base_link` | FLU (body) | Drone body center — X forward, Y left, Z up |
| `camera_link` | FLU (body) | OAK-D Lite mount point on body |
| `camera_optical_frame` | ROS optical (Z forward, X right, Y down) | OAK-D Lite optical axis |

**ENU is world-fixed; body frames are FLU.** REP-103 gives world frames ENU and
body frames FLU, and they are not the same convention — labelling `base_link` ENU
is the mistake that makes a body velocity command come out rotated.

## NED vs ENU

- **ArduPilot** uses NED (North-East-Down)
- **ROS 2** uses ENU (East-North-Up) for world frames, FLU for body frames, per REP-103
- **Conversion happens in the MAVLink bridge node only** —
  `MavBridge.ned_to_enu` / `enu_to_ned` in `mav_bridge.py`. All other nodes work
  exclusively in ENU/FLU. Mirrored RViz markers mean a bridge conversion bug, not
  a marker bug.

## Static Transforms

Both are published by `week3_perception.launch.py` (and duplicated in
`run_regression.sh`, which builds the same two `static_transform_publisher` calls).

| Parent → child | Translation (m) | Rotation (rpy, rad) |
|---|---|---|
| `base_link` → `camera_link` | x=0.10, y=0.0, z=0.02 | 0, 0, 0 |
| `camera_link` → `camera_optical_frame` | 0, 0, 0 | roll=−1.5707963, pitch=0, yaw=−1.5707963 |

The mount offset is exposed as the launch arguments `camera_link_x`,
`camera_link_y`, `camera_link_z` — override them rather than editing the launch
file. It is the **nominal** offset from the CAD mount, not a measured one, and has
never been verified against a real airframe. The optical rotation is the standard
REP-103 body→optical rotation and is not a tunable.

## TF Tree

```
odom
└── base_link
    └── camera_link
        └── camera_optical_frame
```
