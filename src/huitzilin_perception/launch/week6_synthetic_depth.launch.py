"""week6_synthetic_depth.launch.py — the evasion stack with the REAL detector
fed by a synthetic long-range cloud.

Same chain as week6_oracle.launch.py with ONE substitution: instead of
oracle_detector asserting /threat/centroid from Gazebo truth, this file starts
synthetic_depth_publisher (truth -> /oak/points) plus the real, unmodified
detector (cloud -> /threat/centroid). Everything after /threat/centroid is
identical to both other lanes, which is the point — it isolates "did the
clustering pipeline actually produce this centroid" from everything else.

**IT IS A SYNTHETIC CLOUD THROUGH A REAL PIPELINE, NOT A RENDERED SENSOR.** An
80 mm ball at 26 m subtends 0.176 deg, which is sub-pixel at 640x480 over
67.3 deg, so no rendered depth camera in this world can produce that return —
that is precisely why the oracle exists. What changes here is who COMPUTES the
detection, not where the geometry comes from. detection_range_m is still an
INPUT, and no number from this lane is evidence that a real camera sees 26 m.

WHY IT IS A SEPARATE FILE, and what it must never include
---------------------------------------------------------
Two publishers on /threat/centroid feed the tracker two uncorrelated views of
one ball. This file therefore never includes week3_perception (which would
start a SECOND detector and the ros_gz depth bridges alongside it) and never
starts oracle_detector. Making that structural rather than a boolean means the
mistake cannot be made halfway. Never launch this alongside
week6_oracle.launch.py either.

It does start the two static TF publishers week3_perception owns
(base_link -> camera_link -> camera_optical_frame), because the detector needs
that chain twice: to re-express the cloud in the fixed odom frame before
differencing, and to transform the centroid back into base_link before
publishing. Without it the detector silently drops to camera-frame differencing
and then fails the centroid TF. camera_link_x/y/z drive BOTH the static TF and
the publisher's camera_offset_xyz_m, so the two cannot drift apart.

THE SUPERVISOR IS PINNED OFF, not merely defaulted off. Unlike the oracle lane
this file does publish /oak/points, so the supervisor's cloud watch would in
fact be satisfied — but it is pinned anyway so this lane and the oracle lane
differ ONLY in the detection path. An extra state machine that can command
LOITER/RTL is a confound in a save-rate measurement, not a feature of one.

This file carries its OWN clock bridge: week2_sitl.launch.py has none — in the
Week 3/4 path week3_perception owns it — and without one every use_sim_time
node here dies on the clock guard after its 5 s grace.

USAGE (Dell, after the world + SITL are up — docs/dodge_battery_runbook.md)
--------------------------------------------------------------------------
  ros2 launch huitzilin_perception week6_synthetic_depth.launch.py \
      with_patrol:=true detection_range_m:=26.0

  # Any escape or save measurement: HOVER. Patrol cannot deliver a hit at range.
  EXTRA_ARGS="-p hover_mode:=true" ./scripts/run_dodge_battery.sh week6

  # Fidelity gate FIRST — pin the modelled sensor to the depth detector's real
  # reach and confirm the known results reproduce before believing any new one:
  ros2 launch huitzilin_perception week6_synthetic_depth.launch.py \
      with_patrol:=true detection_range_m:=3.4

Raising detection_range_m above ~26.7 m ALSO requires raising
roi_max_range_m in params/synthetic_depth_detector.yaml — the ROI gate runs on
the noised cloud and must clear the reach by 4 * sigma_depth(reach), which grows
as the square of range. That file's header carries the rule.
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
                                          "cloud publisher's input AND the "
                                          "battery's ground truth; never "
                                          "disable it here"),
        # Promoted to a launch argument because it is the variable this harness
        # exists to sweep, and editing the shipped yaml to change it would
        # dirty the repo (--symlink-install points the installed copy back into
        # src/).
        DeclareLaunchArgument("detection_range_m", default_value="26.0",
                              description="modelled sensor reach, metres — an "
                                          "INPUT to every result, not a result"),
        DeclareLaunchArgument("sensor_rate_hz", default_value="60.0",
                              description="requested cloud rate; delivered as "
                                          "60/ceil(60/requested) — quote the "
                                          "delivered rate the node logs"),
        DeclareLaunchArgument("sensor_delay_s", default_value="0.0",
                              description="sensor pipeline latency (exposure "
                                          "-> publishable cloud). 0.0 matches "
                                          "the zero-latency oracle every "
                                          "recorded result was flown against"),
        # Camera mount offset. Feeds the static TF AND the publisher's
        # camera_offset_xyz_m from one place, so they cannot disagree.
        DeclareLaunchArgument("camera_link_x", default_value="0.10"),
        DeclareLaunchArgument("camera_link_y", default_value="0.0"),
        DeclareLaunchArgument("camera_link_z", default_value="0.02"),
        DeclareLaunchArgument(
            "sensor_params",
            default_value=os.path.join(pkg, "params", "synthetic_depth.yaml"),
        ),
        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg, "params",
                                       "synthetic_depth_detector.yaml"),
            description="NOT detector.yaml — its 5 m ROI rejects a 26 m ball "
                        "before clustering. See that file's header.",
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
            # Pinned off, not merely defaulted off — see the module docstring.
            # week2_sitl declares with_supervisor, so leaving it unset would let
            # it be passed through from this file's command line.
            "with_supervisor": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_patrol")),
    )

    return LaunchDescription(
        args + _camera_tf() + [
            clock_bridge,
            flight_stack,
            OpaqueFunction(function=_sensor_node),
            _detector_node(use_sim_time),
            _evasion_node(use_sim_time),
            OpaqueFunction(function=_pose_bridge),
            OpaqueFunction(function=_wrench_bridge),
        ])


def _camera_tf() -> list:
    """base_link -> camera_link -> camera_optical_frame, as week3_perception.

    The detector needs this chain to difference in the fixed odom frame and to
    transform its centroid back into base_link. Same values and same optical
    rotation (roll=-pi/2, yaw=-pi/2) as week3_perception.launch.py — this lane
    must not invent a second camera pose.
    """
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_camera_link",
            arguments=[
                LaunchConfiguration("camera_link_x"),
                LaunchConfiguration("camera_link_y"),
                LaunchConfiguration("camera_link_z"),
                "0", "0", "0",
                "base_link", "camera_link",
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_camera_link_to_optical",
            arguments=[
                "0", "0", "0",
                "-1.5707963", "0", "-1.5707963",
                "camera_link", "camera_optical_frame",
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def _sensor_node(context):
    """The synthetic cloud publisher.

    An OpaqueFunction rather than a plain Node so camera_offset_xyz_m can be
    built as a real list of three floats. A DOUBLE_ARRAY parameter cannot be
    assembled from a python list of ParameterValue substitutions — launch_ros
    normalises list values elementwise and a substitution inside one is not the
    supported form — and getting it wrong there fails at launch, after the
    world and SITL are already up.

    value_type=float on the three swept scalars IS load-bearing, not tidiness:
    without it a command line of `detection_range_m:=5` is inferred as INTEGER
    against a DOUBLE declaration and the node dies at startup with
    InvalidParameterTypeException. A sweep over "3.4 5 7 9 12" then runs one
    cell and skips four (CLAUDE.md).
    """
    lc = context.launch_configurations
    offset = [float(lc.get(f"camera_link_{axis}", default))
              for axis, default in (("x", "0.10"), ("y", "0.0"), ("z", "0.02"))]
    return [Node(
        package="huitzilin_perception",
        executable="synthetic_depth_publisher",
        name="synthetic_depth_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("sensor_params"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "detection_range_m": ParameterValue(
                    LaunchConfiguration("detection_range_m"), value_type=float),
                "rate_hz": ParameterValue(
                    LaunchConfiguration("sensor_rate_hz"), value_type=float),
                "detection_delay_s": ParameterValue(
                    LaunchConfiguration("sensor_delay_s"), value_type=float),
                # Kept equal to the static TF above by construction: both read
                # the same camera_link_x/y/z launch arguments, so the mount
                # offset the publisher subtracts is always the one the detector
                # adds back.
                "camera_offset_xyz_m": offset,
            },
        ],
    )]


def _detector_node(use_sim_time) -> Node:
    """The REAL detector, unmodified, on synthetic_depth_detector.yaml."""
    return Node(
        package="huitzilin_perception",
        executable="detector",
        name="detector",
        output="screen",
        parameters=[
            LaunchConfiguration("detector_params"),
            {"use_sim_time": use_sim_time},
        ],
    )


def _evasion_node(use_sim_time) -> Node:
    return Node(
        package="huitzilin_perception",
        executable="evasion",
        name="evasion",
        output="screen",
        parameters=[
            LaunchConfiguration("evasion_params"),
            {"use_sim_time": use_sim_time},
        ],
    )


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
    """Ground-truth pose bridge — here it feeds the cloud publisher as well as
    the battery, so disabling it leaves the sensor with no input at all.

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
