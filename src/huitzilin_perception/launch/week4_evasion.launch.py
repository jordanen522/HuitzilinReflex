"""
week4_evasion.launch.py — HuitzilinReflex Week 4.

One-command bring-up of the Week 4 evasion stack:
  1. Week 3 live perception stack (bridges, TF, detector) — included
  2. evasion node (Kalman + dodge trigger)
  3. Gazebo dynamic-pose bridge -> /gz/dynamic_poses (battery ground truth)

USAGE (Dell, after world + SITL are up — docs/week4_dodge_runbook.md)
---------------------------------------------------------------------
  ros2 launch huitzilin_perception week4_evasion.launch.py with_patrol:=true

  # then, in another terminal:
  ./scripts/run_dodge_battery.sh          # battery
  ./scripts/run_dodge_battery.sh sweep    # parameter sweep
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_perception")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 flight stack"),
        DeclareLaunchArgument("world_name", default_value="huitzilin_runway"),
        DeclareLaunchArgument("gz_pose_bridge", default_value="true",
                              description="Bridge Gazebo dynamic poses (battery ground truth)"),
        DeclareLaunchArgument(
            "evasion_params",
            default_value=os.path.join(pkg, "params", "evasion.yaml"),
        ),
    ]

    week3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "week3_perception.launch.py")),
        launch_arguments={
            "mode": "live",
            "with_patrol": LaunchConfiguration("with_patrol"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    evasion = Node(
        package="huitzilin_perception",
        executable="evasion",
        name="evasion",
        output="screen",
        parameters=[
            LaunchConfiguration("evasion_params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(args + [week3, evasion,
                                     OpaqueFunction(function=_pose_bridge)])


def _pose_bridge(context):
    """Ground-truth pose bridge; the world name must be resolved to build
    the gz topic string, hence OpaqueFunction instead of a plain Node."""
    if context.launch_configurations.get("gz_pose_bridge", "true").lower() != "true":
        return []
    world = context.launch_configurations["world_name"]
    use_sim_time = (context.launch_configurations.get("use_sim_time", "true")
                    .lower() == "true")
    gz_topic = f"/world/{world}/dynamic_pose/info"
    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_pose_bridge",
        output="screen",
        arguments=[f"{gz_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[(gz_topic, "/gz/dynamic_poses")],
        parameters=[{"use_sim_time": use_sim_time}],
    )]
