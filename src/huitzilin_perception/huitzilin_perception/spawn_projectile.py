"""
spawn_projectile.py — HuitzilinReflex Week 3, W3-08.

Spawns a projectile in Gazebo and applies an initial velocity impulse so it
flies a repeatable path at (or past) the drone.

USAGE (CLI):
  ros2 run huitzilin_perception spawn_projectile \
      --ros-args \
      -p scenario_id:=S01 \
      -p speed_mps:=8.0 \
      -p approach_angle_deg:=0.0 \
      -p miss_distance_m:=0.0 \
      -p offset_forward_m:=6.0

USAGE (from scenario_matrix.yaml via launch file):
  See week3_perception.launch.py — the scenario runner iterates the matrix
  and calls this node for each positive scenario.

DETERMINISM
-----------
Spawn position is computed relative to the drone's current ENU position
(from /huitzilin/odom), so the scenario tracks the patrol loop correctly.
A fixed random seed per scenario_id ensures identical replays.

DESIGN NOTES
------------
- Uses gz service /world/<world>/create (Gazebo Harmonic API) to spawn
  the projectile model, then gz topic to apply a velocity.
- The Gazebo "apply_link_wrench" or initial velocity set is done by spawning
  the model with a non-zero linear_velocity field in the EntityFactory proto.
  This is cleaner than a post-spawn force impulse in Harmonic.
- All gz service calls are made via subprocess (gz CLI) — the Python
  gz bindings are not stable across Harmonic patch versions.
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class SpawnProjectileNode(Node):
    """
    One-shot node: subscribe /huitzilin/odom, wait for a fix, spawn the
    projectile at the configured offset, then shut down.
    """

    def __init__(self) -> None:
        super().__init__("spawn_projectile")

        # ── Params ────────────────────────────────────────────────────────────
        self.declare_parameter("scenario_id", "S00")
        self.declare_parameter("speed_mps", 8.0)
        self.declare_parameter("approach_angle_deg", 0.0)   # 0° = head-on from front
        self.declare_parameter("miss_distance_m", 0.0)      # 0 = direct hit
        self.declare_parameter("offset_forward_m", 6.0)     # spawn distance ahead
        self.declare_parameter("world_name", "huitzilin_runway")
        self.declare_parameter("model_uri", "model://projectile")

        self._scenario_id  = self.get_parameter("scenario_id").value
        self._speed        = self.get_parameter("speed_mps").value
        self._angle_deg    = self.get_parameter("approach_angle_deg").value
        self._miss_dist    = self.get_parameter("miss_distance_m").value
        self._offset_fwd   = self.get_parameter("offset_forward_m").value
        self._world        = self.get_parameter("world_name").value
        self._model_uri    = self.get_parameter("model_uri").value

        self._latest_odom: Optional[Odometry] = None
        self._spawned = False

        self._odom_sub = self.create_subscription(
            Odometry,
            "/huitzilin/odom",
            self._odom_cb,
            RELIABLE_QOS,
        )
        self.get_logger().info(
            f"spawn_projectile ready — scenario {self._scenario_id} "
            f"speed={self._speed} m/s angle={self._angle_deg}° miss={self._miss_dist} m"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        if self._spawned:
            return
        self._latest_odom = msg
        self._do_spawn()

    def _do_spawn(self) -> None:
        if self._latest_odom is None or self._spawned:
            return
        self._spawned = True

        odom = self._latest_odom
        # Drone ENU position
        dx = odom.pose.pose.position.x
        dy = odom.pose.pose.position.y
        dz = odom.pose.pose.position.z

        # Drone yaw from quaternion (ENU)
        q = odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # Spawn point: forward of drone by offset_forward_m, laterally by miss_distance
        angle_rad = math.radians(self._angle_deg)
        spawn_x = dx + self._offset_fwd * math.cos(yaw) + self._miss_dist * math.sin(yaw)
        spawn_y = dy + self._offset_fwd * math.sin(yaw) - self._miss_dist * math.cos(yaw)
        spawn_z = dz  # same altitude as drone

        # Velocity: toward the drone (opposite of spawn direction)
        # approach_angle tilts the trajectory off head-on
        vel_dir_x = -math.cos(yaw + angle_rad)
        vel_dir_y = -math.sin(yaw + angle_rad)
        vel_dir_z = 0.0  # horizontal throw; adjust for arc if needed
        vx = self._speed * vel_dir_x
        vy = self._speed * vel_dir_y
        vz = self._speed * vel_dir_z

        model_name = f"projectile_{self._scenario_id}_{int(time.time())}"

        # Build the gz EntityFactory proto as JSON string
        factory_json = json.dumps({
            "sdf_filename": self._model_uri.replace("model://", ""),
            "name": model_name,
            "pose": {
                "position": {"x": spawn_x, "y": spawn_y, "z": spawn_z},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
            },
            "initial_linear_velocity": {"x": vx, "y": vy, "z": vz},
        })

        cmd = [
            "gz", "service",
            "-s", f"/world/{self._world}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", factory_json,
        ]

        self.get_logger().info(
            f"Spawning '{model_name}' at ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f}) "
            f"vel=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s"
        )
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode != 0:
                self.get_logger().error(f"gz service failed: {result.stderr}")
            else:
                self.get_logger().info(f"Spawn OK: {result.stdout.strip()}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("gz service timed out")
        except FileNotFoundError:
            self.get_logger().error("'gz' command not found — is Gazebo Harmonic installed?")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpawnProjectileNode()
    # Spin until spawned, then exit
    while rclpy.ok() and not node._spawned:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
