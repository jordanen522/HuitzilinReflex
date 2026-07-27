#!/usr/bin/env python3
"""
Week 2 launch: starts mav_bridge, patrol, and telemetry_logger together.

Usage:
  Terminal 1 (sim):  ros2 launch ardupilot_gz_bringup iris_runway.launch.py
  Terminal 2 (ours): ros2 launch huitzilin_sim week2_sitl.launch.py

Keeping the sim in its own terminal makes failures easier to read.
Uncomment the IncludeLaunchDescription block below to start everything at once.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("huitzilin_sim")
    bridge_params = os.path.join(pkg, "params", "bridge.yaml")
    # Overridable so Week 4 can fly a longer loop without changing the Week 2
    # demo geometry (scripts/plot_telemetry.py hardcodes the 5 m square).
    default_patrol_params = os.path.join(pkg, "params", "patrol.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("patrol_params",
                              default_value=default_patrol_params,
                              description="patrol node params yaml"),
        # --- optional: start SITL+Gazebo in this launch instead of a separate terminal ---
        # from launch.actions import IncludeLaunchDescription
        # from launch.launch_description_sources import PythonLaunchDescriptionSource
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        #     get_package_share_directory("ardupilot_gz_bringup"),
        #     "launch", "iris_runway.launch.py"))),

        Node(
            package="huitzilin_sim",
            executable="mav_bridge",
            name="mav_bridge",
            output="screen",
            parameters=[bridge_params],
        ),
        Node(
            package="huitzilin_sim",
            executable="patrol",
            name="patrol",
            output="screen",
            parameters=[LaunchConfiguration("patrol_params")],
        ),
        Node(
            package="huitzilin_sim",
            executable="telemetry_logger",
            name="telemetry_logger",
            output="screen",
        ),
    ])
