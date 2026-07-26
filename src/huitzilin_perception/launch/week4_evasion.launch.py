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
                                     OpaqueFunction(function=_pose_bridge),
                                     OpaqueFunction(function=_wrench_bridge)])


def _wrench_bridge(context):
    """ROS->gz bridge for the two ApplyLinkWrench topics.

    spawn_projectile throws the ball by publishing a one-step impulse and a
    persistent -mass*g wrench (the model spawns with gravity off). Both have
    to land on the same physics step, which the `gz` CLI cannot do — each call
    costs ~0.5 s of sim time. Without this bridge the throw silently degrades
    to the slower CLI path and the trajectory flattens.
    """
    world = context.launch_configurations["world_name"]
    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wrench_bridge",
        output="screen",
        arguments=[
            f"/world/{world}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
            f"/world/{world}/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench",
        ],
    )]


def _pose_bridge(context):
    """Ground-truth pose bridge; the world name must be resolved to build
    the gz topic string, hence OpaqueFunction instead of a plain Node.

    Uses huitzilin_perception's own gz_pose_bridge node (shells out to the
    `gz` CLI, same pattern as spawn_projectile.py) rather than
    ros_gz_bridge's parameter_bridge — the Pose_V -> TFMessage factory isn't
    registered in this Harmonic/ros_gz_bridge build, so parameter_bridge
    starts but /gz/dynamic_poses never produces output."""
    if context.launch_configurations.get("gz_pose_bridge", "true").lower() != "true":
        return []
    world = context.launch_configurations["world_name"]
    use_sim_time = (context.launch_configurations.get("use_sim_time", "true")
                    .lower() == "true")
    return [Node(
        package="huitzilin_perception",
        executable="gz_pose_bridge",
        name="gz_pose_bridge",
        output="screen",
        parameters=[{"world_name": world, "use_sim_time": use_sim_time}],
    )]
