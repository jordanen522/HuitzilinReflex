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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("huitzilin_sim")
    bridge_params = os.path.join(pkg, "params", "bridge.yaml")
    patrol_params = os.path.join(pkg, "params", "patrol.yaml")

    return LaunchDescription([
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
            parameters=[patrol_params],
        ),
        Node(
            package="huitzilin_sim",
            executable="telemetry_logger",
            name="telemetry_logger",
            output="screen",
        ),
    ])
