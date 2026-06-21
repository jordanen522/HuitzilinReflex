"""
week3_perception.launch.py — HuitzilinReflex Week 3, W3-20.

One-command bring-up of the full Week 3 perception stack:
  1. ros_gz_image bridge   — /gz/oak/depth → /oak/depth (sensor_msgs/Image)
  2. ros_gz_bridge         — /gz/oak/depth/points → /oak/points (PointCloud2)
                           — /gz/oak/depth/camera_info → /oak/camera_info
  3. static_transform_publisher  base_link → camera_link
  4. static_transform_publisher  camera_link → camera_optical_frame
  5. detector_node         — subscribes /oak/points, publishes /threat/centroid

USAGE
-----
  # Full stack (depth bridge + TF + detector):
  ros2 launch huitzilin_perception week3_perception.launch.py

  # With optional patrol (add drone flight):
  ros2 launch huitzilin_perception week3_perception.launch.py with_patrol:=true

  # Offline scoring only (detector + scorer, no Gazebo bridge):
  ros2 launch huitzilin_perception week3_perception.launch.py mode:=score \
      bag_dir:=/data/huitzilin_bags split:=test

MACHINE NOTE (from CLAUDE.md environment section)
--------------------------------------------------
Gazebo depth rendering requires the native Dell Inspiron (UHD 630).
The WSL2/Iris Xe laptop cannot render depth frames at rate.
Run this launch file on the native-Ubuntu box for any scenario that
requires live Gazebo depth — bag replay / scoring can run anywhere.

COORDINATE FRAMES
-----------------
  base_link
    └── camera_link            x=+0.10 y=0 z=+0.02 (10 cm forward, 2 cm up)
          └── camera_optical_frame   roll=-π/2, yaw=-π/2 (standard optical)

These values are provisional (exact offset measured in Phase B / Week 6).
Update camera_link_x, camera_link_z params if the physical mount changes.
Document the change in docs/frames.md in the same commit.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_perception = get_package_share_directory("huitzilin_perception")
    pkg_sim = get_package_share_directory("huitzilin_sim")

    # ── Arguments ─────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("mode", default_value="live",
                              description="live | score"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 patrol stack"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Camera mount offset (provisional; update when physically measured)
        DeclareLaunchArgument("camera_link_x", default_value="0.10"),
        DeclareLaunchArgument("camera_link_y", default_value="0.0"),
        DeclareLaunchArgument("camera_link_z", default_value="0.02"),

        # Detector param file
        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg_perception, "params", "detector.yaml"),
        ),

        # Scorer params (only used in score mode)
        DeclareLaunchArgument("bag_dir", default_value="/data/huitzilin_bags"),
        DeclareLaunchArgument("split", default_value="test"),
        DeclareLaunchArgument(
            "scenario_matrix",
            default_value=os.path.join(pkg_perception, "config",
                                       "scenario_matrix.yaml"),
        ),
        DeclareLaunchArgument("recall_floor", default_value="0.95"),
        DeclareLaunchArgument("score_output",
                              default_value="/tmp/week3_regression.txt"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    mode         = LaunchConfiguration("mode")
    with_patrol  = LaunchConfiguration("with_patrol")

    # ── 1. ros_gz_image bridge — depth image ──────────────────────────────────
    depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="depth_image_bridge",
        output="screen",
        arguments=["/gz/oak/depth"],
        remappings=[("/gz/oak/depth", "/oak/depth")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            # Only in live mode — scoring replays bags that already have /oak topics
            _eq_condition(mode, "live")
        ),
    )

    # ── 2. ros_gz_bridge — point cloud + camera_info ──────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="perception_gz_bridge",
        output="screen",
        arguments=[
            # PointCloud2
            "/gz/oak/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # CameraInfo
            "/gz/oak/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/gz/oak/depth/points",       "/oak/points"),
            ("/gz/oak/depth/camera_info",  "/oak/camera_info"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(_eq_condition(mode, "live")),
    )

    # ── 3+4. Static TF: base_link → camera_link → camera_optical_frame ────────
    #
    # camera_link: forward + up of base_link (provisional mount offset)
    # camera_optical_frame: standard optical rotation (REP-103)
    #   optical = body rotated: roll = -90°, then yaw = -90°
    #   → quaternion: x=-0.5, y=0.5, z=-0.5, w=0.5
    #
    tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_camera_link",
        arguments=[
            LaunchConfiguration("camera_link_x"),
            LaunchConfiguration("camera_link_y"),
            LaunchConfiguration("camera_link_z"),
            "0", "0", "0",                          # roll pitch yaw
            "base_link", "camera_link",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    tf_camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera_link_to_optical",
        arguments=[
            "0", "0", "0",                          # no translation
            "-1.5707963", "0", "-1.5707963",        # roll=-π/2, yaw=-π/2
            "camera_link", "camera_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 5. Detector node ──────────────────────────────────────────────────────
    detector = Node(
        package="huitzilin_perception",
        executable="detector",
        name="detector",
        output="screen",
        parameters=[
            LaunchConfiguration("detector_params"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── Optional: score_bags node (score mode) ────────────────────────────────
    scorer = Node(
        package="huitzilin_perception",
        executable="score_bags",
        name="score_bags",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "bag_dir":          LaunchConfiguration("bag_dir"),
            "scenario_matrix":  LaunchConfiguration("scenario_matrix"),
            "split":            LaunchConfiguration("split"),
            "recall_floor":     LaunchConfiguration("recall_floor"),
            "output_file":      LaunchConfiguration("score_output"),
        }],
        condition=IfCondition(_eq_condition(mode, "score")),
    )

    # ── Optional: Week 2 patrol stack ─────────────────────────────────────────
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "week2_sitl.launch.py")
        ),
        condition=IfCondition(with_patrol),
    )

    return LaunchDescription(
        args + [
            depth_image_bridge,
            gz_bridge,
            tf_base_to_camera,
            tf_camera_to_optical,
            detector,
            scorer,
            patrol_launch,
        ]
    )


# ── Helper: string equality condition ─────────────────────────────────────────
# LaunchConfiguration comparison not natively available as a simple bool.
# Use PythonExpression as a workaround.

from launch.conditions import LaunchConfigurationEquals  # noqa: E402


def _eq_condition(lc: LaunchConfiguration, value: str):
    return LaunchConfigurationEquals(lc, value)
