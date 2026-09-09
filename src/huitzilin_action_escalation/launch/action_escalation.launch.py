"""The escalation subsystem, standalone.

Includes NOTHING from huitzilin_sim or huitzilin_perception, and that is
structural rather than incidental: the two graphs share no topic, no service
and no node, so there is no projectile stack here to interfere with and none
to depend on.

use_sim_time defaults to FALSE, unlike every other launch file in this
workspace. There is no Gazebo world behind this subsystem, so nothing
publishes /clock, and defaulting it true would take every node down on the
clock guard after the five-second grace window.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("huitzilin_action_escalation")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_scenario", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="lunge.yaml"),
        # The real detection stage. Mutually exclusive with the scenario
        # player: both publish /action/keypoints, so running them together
        # would interleave a synthetic body and a real one into a single
        # feature window, exactly as oracle_detector and detector must never
        # both publish /threat/centroid. Set with_scenario:=false when using
        # this.
        DeclareLaunchArgument("with_pose_detector", default_value="false"),
        DeclareLaunchArgument(
            "recognizer_params",
            default_value=os.path.join(pkg, "params",
                                       "action_recognizer.yaml")),
        DeclareLaunchArgument(
            "alert_params",
            default_value=os.path.join(pkg, "params", "alert_signal.yaml")),
        DeclareLaunchArgument(
            "player_params",
            default_value=os.path.join(pkg, "params",
                                       "scenario_player.yaml")),
        DeclareLaunchArgument(
            "detector_params",
            default_value=os.path.join(pkg, "params", "pose_detector.yaml")),
    ]

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    recognizer = Node(
        package="huitzilin_action_escalation",
        executable="action_recognizer",
        name="action_recognizer",
        output="screen",
        parameters=[LaunchConfiguration("recognizer_params"), use_sim_time],
    )

    alert = Node(
        package="huitzilin_action_escalation",
        executable="alert_signal",
        name="alert_signal",
        output="screen",
        parameters=[LaunchConfiguration("alert_params"), use_sim_time],
    )

    player = Node(
        package="huitzilin_action_escalation",
        executable="scenario_player",
        name="scenario_player",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_scenario")),
        parameters=[
            LaunchConfiguration("player_params"),
            {"scenario": LaunchConfiguration("scenario")},
            use_sim_time,
        ],
    )

    detector = Node(
        package="huitzilin_action_escalation",
        executable="pose_detector",
        name="pose_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_pose_detector")),
        parameters=[LaunchConfiguration("detector_params"), use_sim_time],
    )

    return LaunchDescription(args + [recognizer, alert, player, detector])
