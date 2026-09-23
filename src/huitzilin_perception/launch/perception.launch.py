"""
perception.launch.py — live perception stack on Gazebo depth.

One-command bring-up of the perception stack:
  1. ros_gz_image bridge   — /gz/oak/depth → /oak/depth (sensor_msgs/Image)
  2. ros_gz_bridge         — /gz/oak/depth/points → /oak/points (PointCloud2)
                           — /gz/oak/depth/camera_info → /oak/camera_info
  3. static_transform_publisher  base_link → camera_link
  4. static_transform_publisher  camera_link → camera_optical_frame
  5. detector_node         — subscribes /oak/points, publishes /threat/centroid

USAGE
-----
  # Full stack (depth bridge + TF + detector):
  ros2 launch huitzilin_perception perception.launch.py

  # With optional patrol (add drone flight):
  ros2 launch huitzilin_perception perception.launch.py with_patrol:=true

Bag scoring does not go through this file: use scripts/run_regression.sh,
which carries the /clock warm-up the detector needs (CLAUDE.md).

MACHINE NOTE
------------
Gazebo depth rendering requires the native Dell Inspiron (UHD 630).
The WSL2/Iris Xe laptop cannot render depth frames at rate.

COORDINATE FRAMES
-----------------
  base_link
    └── camera_link            x=+0.10 y=0 z=+0.02 (10 cm forward, 2 cm up)
          └── camera_optical_frame   roll=-π/2, yaw=-π/2 (standard optical)

These values are nominal, from the CAD mount (measured against the real airframe
in Week 6). Override with the camera_link_x/y/z launch arguments.
Update camera_link_x, camera_link_z params if the physical mount changes.
Document the change in docs/frames.md in the same commit.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_perception = get_package_share_directory("huitzilin_perception")
    pkg_sim = get_package_share_directory("huitzilin_sim")

    args = [
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 patrol stack"),
        # Passed straight through to sitl.launch.py so Week 4 can fly a longer loop
        # (see huitzilin_sim/params/week4_patrol.yaml). Defaults to the 5 m
        # Week 2 demo square, so week3 on its own is unchanged.
        DeclareLaunchArgument(
            "patrol_params",
            default_value=os.path.join(pkg_sim, "params", "patrol.yaml")),
        # Forwarded to sitl.launch.py. The supervisor watches /oak/points, which
        # only this launch file publishes -- starting it from a bare sitl.launch.py
        # gives a permanent SENSOR_DROPOUT. Needs with_patrol:=true, because
        # the supervisor node lives inside the sitl.launch.py include.
        DeclareLaunchArgument("with_supervisor", default_value="false",
                              description="run supervisor_node (requires "
                                          "with_patrol:=true)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Camera mount offset (provisional; update when physically measured)
        DeclareLaunchArgument("camera_link_x", default_value="0.10"),
        DeclareLaunchArgument("camera_link_y", default_value="0.0"),
        DeclareLaunchArgument("camera_link_z", default_value="0.02"),

        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg_perception, "params", "detector.yaml"),
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    with_patrol  = LaunchConfiguration("with_patrol")

    # 1. ros_gz_image bridge — depth image
    depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="depth_image_bridge",
        output="screen",
        arguments=["/gz/oak/depth"],
        remappings=[("/gz/oak/depth", "/oak/depth")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. ros_gz_bridge — point cloud + camera_info
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="perception_gz_bridge",
        output="screen",
        arguments=[
            "/gz/oak/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/gz/oak/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/gz/oak/depth/points",       "/oak/points"),
            ("/gz/oak/depth/camera_info",  "/oak/camera_info"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2b. Clock bridge — gz /clock → ROS /clock (sim time source)
    # Without this, every use_sim_time node (both bridges, both TF publishers,
    # detector) blocks on a /clock that never advances (frozen at t=0), so the
    # detector never fires /threat/centroid and /clock is absent from recorded
    # bags (breaks score_bags sim-time math). No use_sim_time here on purpose:
    # a clock *source* must run on wall time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # 3+4. Static TF: base_link → camera_link → camera_optical_frame
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

    # 5. Detector node
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

    # Optional: Week 2 patrol stack
    patrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, "launch", "sitl.launch.py")
        ),
        launch_arguments={
            "patrol_params": LaunchConfiguration("patrol_params"),
            "with_supervisor": LaunchConfiguration("with_supervisor"),
            # Must be forwarded explicitly. Without it the Week 2 flight nodes
            # fell back to their own default and ran on the wall clock while
            # everything else here ran on sim time, so stamps could not be
            # joined across the two. sitl.launch.py now defaults to true on its
            # own, but this keeps use_sim_time:=false actually reaching them.
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(with_patrol),
    )

    return LaunchDescription(
        args + [
            depth_image_bridge,
            gz_bridge,
            clock_bridge,
            tf_base_to_camera,
            tf_camera_to_optical,
            detector,
            patrol_launch,
        ]
    )
