#!/usr/bin/env python3
"""
Week 2 launch: starts mav_bridge, patrol, and telemetry_logger together.

Usage:
  Terminal 1 (sim):  ros2 launch ardupilot_gz_bringup iris_runway.launch.py
  Terminal 2 (ours): ros2 launch huitzilin_sim week2_sitl.launch.py

Keeping the sim in its own terminal makes failures easier to read.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("huitzilin_sim")
    bridge_params = os.path.join(pkg, "params", "bridge.yaml")
    # Overridable so Week 4 can fly a longer loop without changing the Week 2
    # demo geometry, which is the 5 m square.
    default_patrol_params = os.path.join(pkg, "params", "patrol.yaml")
    supervisor_params = os.path.join(pkg, "params", "supervisor.yaml")

    # The whole flight stack must share Gazebo's clock: the wall and sim clocks
    # differ by a *rate* (RTF), not an offset, so header stamps cannot be joined
    # across the boundary. ArduPilot SITL is lockstepped to Gazebo.
    # Overridable for the rare case of running the bridge against a real
    # vehicle, where no /clock exists and a sim-time node would freeze.
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("patrol_params",
                              default_value=default_patrol_params,
                              description="patrol node params yaml"),
        # Off by default: the Week 3/4 regression path and every recorded
        # battery ran without a supervisor, and a fault monitor that starts
        # commanding modes mid-battery would change what those numbers mean.
        # Opt in explicitly for HITL and flight work.
        #
        # NOTE: supervisor.yaml watches /oak/points, which THIS launch file
        # never publishes -- nothing here starts perception. Enabling the
        # supervisor from a bare week2_sitl therefore gives a permanent
        # SENSOR_DROPOUT the moment the aircraft arms. Launch it through
        # week3_perception / week4_evasion instead (with_patrol:=true
        # with_supervisor:=true); both forward this argument and do publish the
        # cloud. Bare week2_sitl + supervisor is only valid with
        # sensor_timeout_s: 0.0.
        DeclareLaunchArgument("with_supervisor",
                              default_value="false",
                              description="run supervisor_node (state machine "
                                          "+ fault monitor; needs perception "
                                          "for /oak/points)"),
        DeclareLaunchArgument("use_sim_time",
                              default_value="true",
                              description="follow Gazebo /clock; set false only "
                                          "when flying real hardware"),
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
        Node(
            package="huitzilin_sim",
            executable="supervisor",
            name="supervisor",
            output="screen",
            condition=IfCondition(LaunchConfiguration("with_supervisor")),
            parameters=[supervisor_params, {"use_sim_time": use_sim_time}],
        ),
    ])
