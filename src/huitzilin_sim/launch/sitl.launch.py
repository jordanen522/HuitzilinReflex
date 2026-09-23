#!/usr/bin/env python3
"""
SITL flight stack: mav_bridge, patrol, telemetry_logger, optional supervisor.

Usage (the full three-terminal bring-up, with the sim_vehicle.py line, is in
CLAUDE.md):
  Terminal 1 (sim):  gz sim -s -r ~/ardupilot_gazebo/worlds/iris_runway.sdf
  Terminal 3 (ours): ros2 launch huitzilin_sim sitl.launch.py

Keeping the sim in its own terminal makes failures easier to read.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("huitzilin_sim")
    bridge_params = os.path.join(pkg, "params", "bridge.yaml")
    # Overridable so Week 4 can fly a longer loop without changing the Week 2
    # demo geometry, which is the 5 m square.
    default_patrol_params = os.path.join(pkg, "params", "patrol.yaml")
    supervisor_params = os.path.join(pkg, "params", "supervisor.yaml")
    with open(supervisor_params) as fh:
        shipped_sensor_timeout = yaml.safe_load(fh)["supervisor"][
            "ros__parameters"]["sensor_timeout_s"]

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
        # NOTE: the supervisor watches /oak/points, which THIS launch file
        # never publishes -- nothing here starts perception. Enabling the
        # supervisor from a bare sitl.launch.py therefore gives a permanent
        # SENSOR_DROPOUT the moment the aircraft arms. Launch it through
        # perception.launch.py / evasion.launch.py instead (with_patrol:=true
        # with_supervisor:=true); both forward this argument and do publish
        # it. Bare sitl.launch.py + supervisor is only valid with
        # sensor_timeout_s: 0.0.
        DeclareLaunchArgument("with_supervisor",
                              default_value="false",
                              description="run supervisor_node (state machine "
                                          "+ fault monitor; needs perception "
                                          "for /oak/points)"),
        # The supervisor's camera watch is sized for the real OAK-D. On the
        # Dell under the full stack, Gazebo delivers depth frames with gaps of
        # up to ~3 s sim time, so the shipped 1.0 s faults every takeoff.
        # Widen it here for a sim run; never edit supervisor.yaml for it.
        DeclareLaunchArgument("sensor_timeout_s",
                              default_value=str(shipped_sensor_timeout),
                              description="supervisor camera watch, sim "
                                          "seconds (0.0 disables it)"),
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
            parameters=[supervisor_params, {
                "use_sim_time": use_sim_time,
                # Forced to float: "5" would otherwise arrive as an integer
                # and be rejected by the float-declared parameter.
                "sensor_timeout_s": ParameterValue(
                    LaunchConfiguration("sensor_timeout_s"), value_type=float),
            }],
        ),
    ])
