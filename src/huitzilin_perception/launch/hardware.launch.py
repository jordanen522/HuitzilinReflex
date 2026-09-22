"""Real-hardware stack on the Raspberry Pi 5. No Gazebo, no SITL, wall clock.

The flight controller sits behind mavlink-router (hardware/mavlink-router.conf),
which fans the serial link out to the same UDP ports SITL's `--out` used, so
mav_bridge and patrol run with the ports they already have. Each node loads its
sim config first and its hw_* overlay second; the later file wins key by key.

Stages (docs/HARDWARE.md), PROPS OFF until that doc says otherwise:

  # FC link, supervisor, payload, telemetry log
  ros2 launch huitzilin_perception hardware.launch.py

  # + OAK-D and the detector
  ros2 launch huitzilin_perception hardware.launch.py with_camera:=true

  # + evasion (needs the camera)
  ros2 launch huitzilin_perception hardware.launch.py \
    with_camera:=true with_evasion:=true

  # + patrol -- needs a position source on the FC; none is fitted yet
  ros2 launch huitzilin_perception hardware.launch.py with_patrol:=true

The supervisor switches the flight mode once the FC reports armed. For manual
RC flying, pass with_supervisor:=false.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Nothing publishes /clock on the aircraft. use_sim_time is pinned, not an
# argument: true here would kill every node on the clock guard after 5 s.
WALL_CLOCK = {"use_sim_time": False}


def _flag(name, want=True):
    return ["('", LaunchConfiguration(name), "'.lower() == 'true') == ",
            "True" if want else "False"]


def _when(*flags):
    """IfCondition over several boolean launch arguments, all of which must
    match: _when(("with_evasion", True), ("with_camera", True))."""
    expr = []
    for i, (name, want) in enumerate(flags):
        expr += ([" and "] if i else []) + _flag(name, want)
    return IfCondition(PythonExpression(expr))


def _watch_if(timeout_s, name):
    """A supervisor watch timeout that is 0.0 (off) unless `name` is launched.

    A watch on a topic nothing publishes is a permanent fault the moment the
    aircraft arms: no camera means no /oak/points, no patrol means no
    patrol_state.
    """
    return PythonExpression(["%r if " % float(timeout_s)] + _flag(name)
                            + [" else 0.0"])


def generate_launch_description():
    sim = get_package_share_directory("huitzilin_sim")
    perception = get_package_share_directory("huitzilin_perception")

    def sim_params(name):
        return os.path.join(sim, "params", name)

    def perception_params(name):
        return os.path.join(perception, "params", name)

    with open(sim_params("supervisor.yaml")) as fh:
        watches = yaml.safe_load(fh)["supervisor"]["ros__parameters"]

    args = [
        DeclareLaunchArgument("with_camera", default_value="false",
                              description="start the OAK-D driver and the detector"),
        DeclareLaunchArgument("with_evasion", default_value="false",
                              description="start evasion_node (needs with_camera)"),
        DeclareLaunchArgument("with_patrol", default_value="false",
                              description="start patrol (needs a position source)"),
        DeclareLaunchArgument("with_supervisor", default_value="true",
                              description="state machine + fault monitor; false "
                                          "for manual RC flying"),
        DeclareLaunchArgument("patrol_params",
                              default_value=sim_params("patrol.yaml")),
        DeclareLaunchArgument("telemetry_csv",
                              default_value=os.path.expanduser(
                                  "~/huitzilin_logs/telemetry.csv"),
                              description="where telemetry_logger writes"),
        # Nominal mount from docs/frames.md, not yet measured on the airframe.
        DeclareLaunchArgument("camera_x", default_value="0.10"),
        DeclareLaunchArgument("camera_z", default_value="0.02"),
    ]

    flight = [
        Node(package="huitzilin_sim", executable="mav_bridge", name="mav_bridge",
             output="screen",
             parameters=[sim_params("bridge.yaml"), sim_params("hw_bridge.yaml"),
                         WALL_CLOCK]),
        Node(package="huitzilin_sim", executable="supervisor", name="supervisor",
             output="screen",
             condition=IfCondition(LaunchConfiguration("with_supervisor")),
             parameters=[sim_params("supervisor.yaml"), WALL_CLOCK, {
                 "sensor_timeout_s": _watch_if(watches["sensor_timeout_s"],
                                               "with_camera"),
                 "patrol_state_timeout_s": _watch_if(
                     watches["patrol_state_timeout_s"], "with_patrol"),
             }]),
        Node(package="huitzilin_sim", executable="patrol", name="patrol",
             output="screen",
             condition=IfCondition(LaunchConfiguration("with_patrol")),
             parameters=[LaunchConfiguration("patrol_params"), WALL_CLOCK]),
        Node(package="huitzilin_sim", executable="telemetry_logger",
             name="telemetry_logger", output="screen",
             parameters=[{"csv_path": LaunchConfiguration("telemetry_csv")},
                         WALL_CLOCK]),
        Node(package="huitzilin_perception", executable="payload", name="payload",
             output="screen",
             parameters=[perception_params("payload.yaml"), WALL_CLOCK]),
    ]

    # depthai-ros publishes base_link -> oak -> optical frames itself when its
    # parent is base_link, so no static TF is needed here, and its point cloud
    # comes out on /oak/points, the topic the detector reads. The argument
    # names are depthai_ros_driver's camera.launch.py ones; confirm them with
    # `ros2 launch depthai_ros_driver camera.launch.py --show-args` on the Pi.
    camera = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("depthai_ros_driver"), "launch",
                 "camera.launch.py"])),
            condition=IfCondition(LaunchConfiguration("with_camera")),
            launch_arguments={
                "name": "oak",
                "camera_model": "OAK-D-LITE",
                "parent_frame": "base_link",
                "cam_pos_x": LaunchConfiguration("camera_x"),
                "cam_pos_z": LaunchConfiguration("camera_z"),
                "pointcloud.enable": "true",
            }.items()),
        Node(package="huitzilin_perception", executable="detector", name="detector",
             output="screen",
             condition=IfCondition(LaunchConfiguration("with_camera")),
             parameters=[perception_params("detector.yaml"),
                         perception_params("hw_detector.yaml"), WALL_CLOCK]),
        Node(package="huitzilin_perception", executable="evasion", name="evasion",
             output="screen",
             condition=_when(("with_evasion", True), ("with_camera", True)),
             parameters=[perception_params("evasion.yaml"),
                         perception_params("hw_evasion.yaml"), WALL_CLOCK]),
    ]

    return LaunchDescription(args + flight + camera)
