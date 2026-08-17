"""
week6_oracle.launch.py — the evasion stack with a synthetic far-range sensor.

Same chain as week4_evasion.launch.py with ONE substitution: oracle_detector
replaces detector. Everything after /threat/centroid is identical, which is
the point — it isolates "how far can we see" from "what do we do about it".

WHY IT IS A SEPARATE FILE. Two publishers on /threat/centroid would feed the
tracker two uncorrelated views of the same ball, so the real detector and the
oracle must never run together. Making that a separate launch file rather than
a boolean on week4_evasion means the mistake cannot be made halfway: this file
never includes week3_perception, so there is no detector to disable. It also
starts none of the depth machinery (image bridge, cloud bridge, camera TF),
because nothing here consumes a point cloud.

USAGE (Dell, after the world + SITL are up — docs/dodge_battery_runbook.md)
--------------------------------------------------------------------------
  ros2 launch huitzilin_perception week6_oracle.launch.py with_patrol:=true

  # then, in another terminal:
  ./scripts/run_dodge_battery.sh week6        # the 20 m/s battery

  # fidelity gate FIRST — pin the oracle to the depth detector's real SENSOR
  # (reach AND sector AND rate, not reach alone) and confirm the known results
  # reproduce before believing any new one. Reach-only under-specifies: a gate
  # pinned to 3.4 m still failed once because it flew the proposed optics and
  # rate against a reference flown on different ones. `oracle_rate_hz` and
  # `fov_half_angle_deg` (params/oracle_detector.yaml) default silently.
  ros2 launch huitzilin_perception week6_oracle.launch.py \
      with_patrol:=true detection_range_m:=3.4 oracle_rate_hz:=14.5

THE SUPERVISOR IS PINNED OFF, not merely defaulted off — see the include
below. supervisor.yaml watches /oak/points, which this file never publishes,
and a watch on a topic nothing publishes is a permanent SENSOR_DROPOUT the
moment the aircraft arms (CLAUDE.md).

This file carries its OWN clock bridge: week2_sitl.launch.py has none — in the
Week 3/4 path week3_perception owns it — and without one every use_sim_time
node here dies on the clock guard after its 5 s grace.
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_perception")
    sim_pkg = get_package_share_directory("huitzilin_sim")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 flight stack"),
        DeclareLaunchArgument("world_name", default_value="huitzilin_runway"),
        DeclareLaunchArgument("gz_pose_bridge", default_value="true",
                              description="Bridge Gazebo dynamic poses — the "
                                          "oracle's input AND the battery's "
                                          "ground truth; never disable it here"),
        # Promoted to a launch argument because it is the variable this whole
        # harness exists to sweep, and editing the shipped yaml to change it
        # would dirty the repo (--symlink-install points the installed copy
        # back into src/).
        DeclareLaunchArgument("detection_range_m", default_value="12.0",
                              description="oracle sensor reach, metres — an "
                                          "INPUT to every result, not a result"),
        DeclareLaunchArgument("oracle_rate_hz", default_value="14.5",
                              description="oracle detection rate; 14.5 mimics "
                                          "the measured depth cadence"),
        DeclareLaunchArgument("oracle_delay_s", default_value="0.0",
                              description="sensor pipeline latency (exposure "
                                          "-> usable centroid). 0.0 = the "
                                          "zero-latency oracle every recorded "
                                          "result was flown against"),
        DeclareLaunchArgument(
            "oracle_params",
            default_value=os.path.join(pkg, "params", "oracle_detector.yaml"),
        ),
        DeclareLaunchArgument(
            "evasion_params",
            default_value=os.path.join(pkg, "params", "evasion.yaml"),
        ),
        DeclareLaunchArgument(
            "patrol_params",
            default_value=os.path.join(sim_pkg, "params", "week4_patrol.yaml"),
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # A clock SOURCE must run on wall time — no use_sim_time here on purpose.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    flight_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, "launch", "week2_sitl.launch.py")),
        launch_arguments={
            "patrol_params": LaunchConfiguration("patrol_params"),
            "use_sim_time": use_sim_time,
            # Pinned off, not merely defaulted off. week2_sitl declares
            # with_supervisor, so leaving it unset would let it be passed
            # through from this file's command line -- and supervisor.yaml
            # watches /oak/points, which nothing here publishes, so it would
            # fault into permanent SENSOR_DROPOUT the moment the aircraft
            # arms. There is no correct value but false under this launch file.
            "with_supervisor": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_patrol")),
    )

    oracle = Node(
        package="huitzilin_perception",
        executable="oracle_detector",
        name="oracle_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("oracle_params"),
            {
                "use_sim_time": use_sim_time,
                # value_type=float is load-bearing, not tidiness. Without it a
                # command line of `detection_range_m:=5` is inferred as INTEGER
                # against a DOUBLE declaration and oracle_detector dies at
                # startup with InvalidParameterTypeException. A sweep over
                # "3.4 5 7 9 12" therefore ran one cell and skipped four, and
                # the only reason that was noticed at all is that the harness
                # re-read the range out of the oracle's own startup log.
                "detection_range_m": ParameterValue(
                    LaunchConfiguration("detection_range_m"), value_type=float),
                "rate_hz": ParameterValue(
                    LaunchConfiguration("oracle_rate_hz"), value_type=float),
                # A launch argument rather than yet another near-duplicate
                # oracle yaml: the sweep is over one scalar, and every extra
                # copied params file is another place the sensor model can
                # drift from the one the cell thinks it is flying.
                "detection_delay_s": ParameterValue(
                    LaunchConfiguration("oracle_delay_s"), value_type=float),
            },
        ],
    )

    evasion = Node(
        package="huitzilin_perception",
        executable="evasion",
        name="evasion",
        output="screen",
        parameters=[
            LaunchConfiguration("evasion_params"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        args + [clock_bridge, flight_stack, oracle, evasion,
                OpaqueFunction(function=_pose_bridge),
                OpaqueFunction(function=_wrench_bridge)])


def _wrench_bridge(context):
    """ROS->gz bridge for the two ApplyLinkWrench topics.

    spawn_projectile throws the ball with a one-step impulse plus a persistent
    -mass*g wrench (the model spawns with gravity off). Both must land on the
    same physics step, which the `gz` CLI cannot do — each call costs ~0.5 s of
    sim time. Without this bridge the throw silently degrades to the slower CLI
    path and the trajectory flattens, which at 20 m/s would quietly turn every
    scenario into a different one.
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
    """Ground-truth pose bridge — here it feeds the oracle as well as the
    battery, so disabling it leaves the oracle with no input at all.

    The world name must be resolved to build the gz topic string, hence
    OpaqueFunction rather than a plain Node. Uses our own gz_pose_bridge (which
    shells out to the `gz` CLI) because the Pose_V -> TFMessage factory is not
    registered in this Harmonic/ros_gz_bridge build: parameter_bridge starts
    but /gz/dynamic_poses never produces output.
    """
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
