"""week7_rendered.launch.py — the evasion stack on a RENDERED long-range camera.

    gz depth camera -> ros_gz_bridge -> depth_noise -> detector -> evasion

This is the only lane in the repo where every return is something the renderer
actually saw. The other three each cut the chain somewhere:

    week3/week4     rendered OAK-D, but its 67.3 deg lens puts a sub-pixel
                    0.176 deg ball at 26 m -- 4 returns, one under
                    cluster_min_points -- which is the mechanism behind its
                    ~3.4 m practical reach.
    week6_oracle    /threat/centroid ASSERTED from Gazebo truth. No perception.
    week6_synthetic FABRICATED ball-only cloud into the real detector. Real
                    clustering, but no scene and no rendered geometry, so it
                    can say nothing about false positives.

What changes here is the OPTICS, not the physics: models/iris_ar0234 is
models/iris_depth with only the sensor block edited (800x650 over 27.0 deg,
far 35 m), and test_camera_models.py holds it to that -- byte-identical flight
plugins, identical inertials, identical camera mount pose. 0.0338 deg/px puts
~24 returns on the 80 mm ball at 26 m, measured in
lab/probe_out/ar0234_800_26m.result.txt.

WHY depth_noise_node IS IN THE CHAIN AND IS NOT OPTIONAL
-------------------------------------------------------
A Gazebo depth camera returns near-exact geometric depth -- iris_depth declares
a 1 cm per-pixel gaussian, which is both ~30x too small at 26 m and
structurally the wrong shape. iris_ar0234 therefore ships NO <noise> tag at
all, so without this node the lane would fly a sensor with millimetre depth
accuracy at 26 m and every number it produced would be worthless in the
optimistic direction. depth_noise_node applies the modelled stereo error
instead: sigma(z) = 0.30 m * (z/26)^2, along the ray so bearing is preserved,
and spatially correlated over ~7 px so the ball gets ONE disparity solution
rather than averaging its own error down by sqrt(N).

Validated live before this file existed (docs/optics_probe.md): ball at 26 m,
120 frames, centroid range std 0.2859 m against the 0.30 m spec, 0.0000 m
exactly on the sigma_ref_m:=0.0 control arm, and 0.0665 m at 13 m for a z^2
ratio of 4.30 against 4.0 ideal.

sigma_ref_m:=0.0 IS THE CONTROL ARM, not a way to turn off an inconvenience.
It consumes no randomness, so an A/B differs only in the noise rather than in
the seed. Any save rate quoted from a sigma_ref_m:=0.0 run is a NOISELESS-SENSOR
number and must be labelled as one.

THE FIDELITY GATE COMES FIRST
-----------------------------
Nothing this lane reports is believable until it reproduces a result that was
measured another way. The reference is the Week 4 envelope -- rendered OAK-D on
patrol, 78/78 dodges at <= 8 m/s, 0/17 at 14 m/s, 0 false dodges in 12 -- and
this file can fly exactly that by pointing at the baseline world and camera:

  # Terminal 1 -- BASELINE world (iris_depth), not the ar0234 one:
  gz sim -s -r $(ros2 pkg prefix huitzilin_perception)/share/\\
huitzilin_perception/worlds/huitzilin_runway.sdf

  # Terminal 3:
  ros2 launch huitzilin_perception week7_rendered.launch.py \\
      with_patrol:=true \\
      gz_cloud_topic:=/gz/oak/depth/points \\
      sigma_ref_m:=0.0 \\
      detector_params:=$(ros2 pkg prefix huitzilin_perception)/share/\\
huitzilin_perception/params/detector.yaml

  DRONE_MODEL=iris_depth ./scripts/run_dodge_battery.sh week4

That arm changes ONE thing against Week 4: the cloud takes a detour through
depth_noise with the noise disabled. It must reproduce 78/78 and 0/17. If it
does not, the plumbing is wrong and no ar0234 number means anything yet.

CROSSING TO THE LONG-RANGE ARM CHANGES FOUR THINGS AT ONCE -- world, optics,
noise and ROI -- so it is a separate measurement, never a continuation:

  # Terminal 1 -- the ar0234 world:
  gz sim -s -r $(ros2 pkg prefix huitzilin_perception)/share/\\
huitzilin_perception/worlds/huitzilin_runway_ar0234.sdf

  # Terminal 3 (all defaults are already the long-range arm):
  ros2 launch huitzilin_perception week7_rendered.launch.py with_patrol:=true

  # Any escape or save measurement: HOVER. Patrol cannot deliver a hit at range.
  DRONE_MODEL=iris_ar0234 EXTRA_ARGS="-p hover_mode:=true" \\
      ./scripts/run_dodge_battery.sh week7

DRONE_MODEL IS NOT OPTIONAL ON THE ar0234 ARM. That world names its entity
iris_ar0234, while dodge_battery, hz_counterfactual and the oracle/synthetic
publishers all default drone_model to "iris_depth". Get it wrong and the
battery finds no drone; it does say so (dodge_battery.py prints "check
drone_model:=..."), but it says so at the end of a long setup.

WHAT THIS FILE PINS, AND WHAT IT DOES NOT
-----------------------------------------
A SENSOR IS REACH, SECTOR AND RATE. Two of those three live in the WORLD file
here, not in any launch argument: iris_ar0234 fixes the sector (+/-13.5 x
11.0 deg) and requests 30 Hz (~23 Hz delivered -- the ros_gz_bridge
PointCloudPacked hop is the ceiling, and t_dead = 0.155 + 1.380/f must be
evaluated at the delivered rate). Reach is not an input at all in this lane and
that is the whole point: unlike detection_range_m in the oracle and synthetic
lanes, nothing here declares how far the sensor sees. It sees as far as the
renderer, the optics and the noise let it. Quote the world file beside every
number from this lane, because the launch command alone does not identify the
instrument.

THE SUPERVISOR IS PINNED OFF, not merely defaulted off, for the same reason as
the other two week6 lanes: an extra state machine that can command LOITER/RTL
is a confound in a save-rate measurement. This lane does publish /oak/points,
so its cloud watch would in fact be satisfied -- it is pinned anyway so the
lanes differ only where they are meant to.

This file carries its OWN clock bridge: week2_sitl.launch.py has none, and
without one every use_sim_time node here dies on the clock guard after its 5 s
grace.
"""

from __future__ import annotations

import os

import yaml
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

# The gate arithmetic lives in a pure module, not here. A launch file imports
# `launch` and `launch_ros`, so CI's ROS-free subset cannot import it, and a
# guard no test can reach is a comment that raises. See rendered_lane.py.
from huitzilin_perception.rendered_lane import (
    assert_gates_clear_the_rendered_range,
)


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_perception")
    sim_pkg = get_package_share_directory("huitzilin_sim")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="Also launch the Week 2 flight stack"),
        # Unchanged from the baseline on purpose: huitzilin_runway_ar0234.sdf
        # keeps <world name>huitzilin_runway so the wrench and pose topics stay
        # where spawn_projectile and gz_pose_bridge already address them.
        DeclareLaunchArgument("world_name", default_value="huitzilin_runway"),
        DeclareLaunchArgument(
            "gz_cloud_topic", default_value="/gz/oak_lr/depth/points",
            description="iris_ar0234 publishes /gz/oak_lr/depth/points; the "
                        "baseline iris_depth publishes /gz/oak/depth/points. "
                        "Must match the world actually running -- a wrong "
                        "value bridges a topic nothing serves and the lane "
                        "goes silent with no error anywhere."),
        DeclareLaunchArgument(
            "rendered_cloud_topic", default_value="/oak/points_rendered",
            description="bridge output = noise stage input. Deliberately NOT "
                        "/oak/points: the detector must never be able to "
                        "subscribe to the noiseless cloud by accident."),
        DeclareLaunchArgument("cloud_topic", default_value="/oak/points",
                              description="noise stage output = detector input"),
        # -- The noise model --------------------------------------------------
        DeclareLaunchArgument(
            "sigma_ref_m", default_value="0.30",
            description="stereo depth error at ref_range_m, metres. 0.0 is the "
                        "NOISELESS CONTROL ARM and must be labelled as such on "
                        "any number taken from it."),
        DeclareLaunchArgument("ref_range_m", default_value="26.0"),
        DeclareLaunchArgument(
            "correlation_px", default_value="7.0",
            description="image-space correlation length of the error field. A "
                        "MODELLING ASSUMPTION, not a measurement: centroid "
                        "scatter is strongly sensitive to it (0.068/0.050/"
                        "0.029/0.016 m at 5/7/12/20 px). Quote it."),
        DeclareLaunchArgument("noise_seed", default_value="0"),
        DeclareLaunchArgument("camera_link_x", default_value="0.10"),
        DeclareLaunchArgument("camera_link_y", default_value="0.0"),
        DeclareLaunchArgument("camera_link_z", default_value="0.02"),
        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg, "params", "rendered_detector.yaml"),
            description="NOT detector.yaml -- its 5 m ROI rejects a 26 m ball "
                        "before clustering and its 0.35 m extent gate rejects "
                        "~20 % of ball frames at that range. Point it at "
                        "detector.yaml only for the OAK-D fidelity gate.",
        ),
        DeclareLaunchArgument(
            "evasion_params",
            default_value=os.path.join(pkg, "params", "evasion.yaml"),
        ),
        DeclareLaunchArgument(
            "patrol_params",
            default_value=os.path.join(sim_pkg, "params", "week4_patrol.yaml"),
        ),
        DeclareLaunchArgument("gz_pose_bridge", default_value="true",
                              description="ground truth for the battery; the "
                                          "sensor does not depend on it here"),
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
            "with_supervisor": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_patrol")),
    )

    return LaunchDescription(
        args + _camera_tf() + [
            clock_bridge,
            flight_stack,
            OpaqueFunction(function=_cloud_bridge),
            OpaqueFunction(function=_noise_node),
            OpaqueFunction(function=_detector_node),
            _evasion_node(use_sim_time),
            OpaqueFunction(function=_pose_bridge),
            OpaqueFunction(function=_wrench_bridge),
        ])


def _camera_tf() -> list:
    """base_link -> camera_link -> camera_optical_frame, as week3_perception.

    The detector needs this chain twice: to re-express the cloud in the fixed
    odom frame before differencing, and to transform the centroid back into
    base_link before publishing. Without it the detector silently drops to
    camera-frame differencing, which floods under patrol motion — the cause of
    the Week 3 60 %-recall regression. Same values and same optical rotation as
    week3_perception.launch.py: this lane must not invent a second camera pose.
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


def _cloud_bridge(context):
    """gz PointCloudPacked -> ROS PointCloud2, into the RENDERED topic.

    OpaqueFunction because the gz topic is a launch argument and the bridge
    spec string has to be built from it.

    Only the cloud is bridged. week3_perception also bridges the depth image
    and camera_info; neither is read by anything in this chain, and at 800x650
    they are pure load on a box that renders depth at ~0.33 RTF.
    """
    lc = context.launch_configurations
    gz_topic = lc["gz_cloud_topic"]
    return [Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="rendered_cloud_bridge",
        output="screen",
        arguments=[
            f"{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[(gz_topic, LaunchConfiguration("rendered_cloud_topic"))],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )]


def _noise_node(context):
    """The stereo error model. See the module docstring for why it is required.

    cloud_convention is passed explicitly rather than left to default, and it
    must equal the detector's. The two disagreeing is not a loud failure: it
    republishes a well-formed cloud of the right point count that has been
    scaled by essentially 1.0, i.e. a NOISELESS sensor reported as a clean run.
    That bug cost a full A/B before the node grew its first-cloud self-check
    (docs/optics_probe.md).

    cloud_reliable comes from the detector file too. BEST_EFFORT on a multi-MB
    cloud loses ~75 % of its frames to UDP fragmentation (measured, see
    params/detector.yaml), and a mismatch in the other direction leaves the
    subscription unconnected while the topic still lists a publisher.
    """
    lc = context.launch_configurations
    detector = _ros_params(lc["detector_params"], "detector")
    return [Node(
        package="huitzilin_perception",
        executable="depth_noise",
        name="depth_noise",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "cloud_in_topic": LaunchConfiguration("rendered_cloud_topic"),
            "cloud_out_topic": LaunchConfiguration("cloud_topic"),
            # value_type=float is load-bearing, not tidiness: sigma_ref_m:=0
            # is inferred as INTEGER against a DOUBLE declaration and the node
            # dies at startup with InvalidParameterTypeException — and 0 is
            # exactly what the control arm gets written as.
            "sigma_ref_m": ParameterValue(
                LaunchConfiguration("sigma_ref_m"), value_type=float),
            "ref_range_m": ParameterValue(
                LaunchConfiguration("ref_range_m"), value_type=float),
            "correlation_px": ParameterValue(
                LaunchConfiguration("correlation_px"), value_type=float),
            "seed": ParameterValue(
                LaunchConfiguration("noise_seed"), value_type=int),
            # Taken from the detector file actually being launched, so the two
            # ends of /oak/points cannot disagree about which axis is depth or
            # about how the cloud is transported.
            "cloud_convention": str(detector["cloud_convention"]),
            "cloud_reliable": bool(detector["cloud_reliable"]),
            "cloud_queue_depth": int(detector["cloud_queue_depth"]),
        }],
    )]


def _detector_node(context):
    """The REAL detector, unmodified, on whichever params file was launched."""
    assert_gates_clear_the_rendered_range(
        _ros_params(context.launch_configurations["detector_params"],
                    "detector"))
    return [Node(
        package="huitzilin_perception",
        executable="detector",
        name="detector",
        output="screen",
        parameters=[
            LaunchConfiguration("detector_params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )]


def _ros_params(path: str, node: str) -> dict:
    """The ros__parameters block of a shipped params file."""
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)[node]["ros__parameters"]


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
    """Ground-truth pose bridge — the battery's truth source.

    Unlike the synthetic lane, nothing in the SENSOR path depends on it here:
    the cloud comes from the renderer. It is still on by default because every
    scored result needs the ball's true track to compute counterfactual_min_m.

    Uses our own gz_pose_bridge (which shells out to the `gz` CLI) because the
    Pose_V -> TFMessage factory is not registered in this Harmonic/ros_gz_bridge
    build: parameter_bridge starts but /gz/dynamic_poses never produces output.
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
