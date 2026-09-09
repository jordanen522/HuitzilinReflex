"""The guard alarm, standalone.

    ros2 launch huitzilin_guard guard.launch.py
    ros2 service call /guard/arm std_srvs/srv/SetBool '{data: true}'

Includes NOTHING from huitzilin_sim or huitzilin_perception, and that is
structural rather than incidental: this graph and the projectile graph share
no topic, no service and no node, so there is no flight stack here to
interfere with and none to depend on. Flying the box is a separate launch
(week4_patrol with guard.yaml); the alarm does not care whether anything is
flying, which is what lets it be tested on a bench.

use_sim_time defaults to FALSE, unlike every other launch file in this
workspace. There is no Gazebo world behind this subsystem, so nothing
publishes /clock, and defaulting it true would take every node down on the
clock guard after the five-second grace window.

with_pose_detector defaults FALSE because the model is not tracked in this
repository and the node refuses to start without it. Set it true on a machine
that has run scripts/fetch_pose_model.sh:

    ros2 launch huitzilin_guard guard.launch.py with_pose_detector:=true

With it false the graph is complete except for the camera, and a synthetic
detection published by hand on /guard/detections exercises the whole
decision path.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_guard")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_pose_detector", default_value="false"),
        DeclareLaunchArgument(
            "guard_params",
            default_value=os.path.join(pkg, "params", "guard.yaml")),
        DeclareLaunchArgument(
            "alert_params",
            default_value=os.path.join(pkg, "params", "alert_signal.yaml")),
        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg, "params", "pose_detector.yaml")),
    ]

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    guard = Node(
        package="huitzilin_guard",
        executable="guard",
        name="guard",
        output="screen",
        parameters=[LaunchConfiguration("guard_params"), use_sim_time],
    )

    alert = Node(
        package="huitzilin_guard",
        executable="alert_signal",
        name="alert_signal",
        output="screen",
        parameters=[LaunchConfiguration("alert_params"), use_sim_time],
    )

    detector = Node(
        package="huitzilin_guard",
        executable="pose_detector",
        name="pose_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_pose_detector")),
        parameters=[LaunchConfiguration("detector_params"), use_sim_time],
    )

    return LaunchDescription(args + [guard, alert, detector])
