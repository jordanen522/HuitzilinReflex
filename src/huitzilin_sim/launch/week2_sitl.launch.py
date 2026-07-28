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

    # These three nodes previously ran on the wall clock while every
    # Gazebo-sourced node ran on sim time, so header stamps could not be joined
    # across the boundary (the two clocks differ by a *rate*, RTF, not an
    # offset). Gazebo publishes /clock, and ArduPilot SITL is lockstepped to
    # Gazebo, so sim time is the correct timebase for the whole flight stack.
    # Overridable for the rare case of running the bridge against a real
    # vehicle, where no /clock exists and a sim-time node would freeze.
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("patrol_params",
                              default_value=default_patrol_params,
                              description="patrol node params yaml"),
        DeclareLaunchArgument("use_sim_time",
                              default_value="true",
                              description="follow Gazebo /clock; set false only "
                                          "when flying real hardware"),
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
            parameters=[bridge_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="huitzilin_sim",
            executable="patrol",
            name="patrol",
            output="screen",
            parameters=[LaunchConfiguration("patrol_params"),
                        {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="huitzilin_sim",
            executable="telemetry_logger",
            name="telemetry_logger",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
