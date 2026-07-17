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
      -p offset_forward_m:=6.0 \
      -p offset_vertical_m:=0.0

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
- Spawn geometry (position + velocity) is computed by
  `ballistics.compute_spawn`, shared with the Week 4 dodge battery so both
  use the exact same math. `compensate_gravity:=true` lofts the throw so
  the parabola arrives at true drone altitude (a real hit trajectory)
  instead of the Week 3 flat throw, which passes below the target.
- `gz_spawn`/`gz_remove`/`_run_gz` are module-level (not node methods) so
  dodge_battery.py can reuse them without needing a live rclpy node.
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

from huitzilin_perception.ballistics import compute_spawn

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Never spawn below this world z (m) — keeps the ball out of the ground plane.
# Vertical-offset scenarios (N04) therefore require the drone to fly high
# enough that drone_z + offset_vertical_m >= MIN_SPAWN_Z.
MIN_SPAWN_Z = 0.3


def gz_spawn(world: str, model_uri: str, name: str, position, velocity,
             timeout_s: float = 5.0) -> tuple[bool, str]:
    """Spawn `model_uri` as `name` at ENU `position` with initial `velocity`
    via the Gazebo Harmonic EntityFactory service. Returns (ok, message).
    Module-level so dodge_battery.py can reuse it without a node."""
    factory_json = json.dumps({
        "sdf_filename": model_uri.replace("model://", ""),
        "name": name,
        "pose": {
            "position": {"x": float(position[0]), "y": float(position[1]),
                         "z": float(position[2])},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        },
        "initial_linear_velocity": {"x": float(velocity[0]),
                                    "y": float(velocity[1]),
                                    "z": float(velocity[2])},
    })
    cmd = [
        "gz", "service", "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", factory_json,
    ]
    return _run_gz(cmd, timeout_s)


def gz_remove(world: str, name: str, timeout_s: float = 5.0) -> tuple[bool, str]:
    """Remove model `name` from the world (gz.msgs.Entity type 2 = MODEL).
    The dodge battery calls this between runs so spent balls don't litter
    the ground plane and confuse the depth background model."""
    req = json.dumps({"name": name, "type": 2})
    cmd = [
        "gz", "service", "-s", f"/world/{world}/remove",
        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    return _run_gz(cmd, timeout_s)


def _run_gz(cmd: list[str], timeout_s: float) -> tuple[bool, str]:
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, "gz service timed out"
    except FileNotFoundError:
        return False, "'gz' command not found — is Gazebo Harmonic installed?"
    if result.returncode != 0:
        return False, f"gz service failed: {result.stderr.strip()}"
    return True, result.stdout.strip()


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
        self.declare_parameter("offset_vertical_m", 0.0)    # relative to drone; negative = below (N04)
        self.declare_parameter("world_name", "huitzilin_runway")
        self.declare_parameter("model_uri", "model://projectile")
        self.declare_parameter("compensate_gravity", False)  # loft to arrive at drone altitude (Week 4 battery)

        self._scenario_id  = self.get_parameter("scenario_id").value
        self._speed        = self.get_parameter("speed_mps").value
        self._angle_deg    = self.get_parameter("approach_angle_deg").value
        self._miss_dist    = self.get_parameter("miss_distance_m").value
        self._offset_fwd   = self.get_parameter("offset_forward_m").value
        self._offset_vert  = self.get_parameter("offset_vertical_m").value
        self._world        = self.get_parameter("world_name").value
        self._model_uri    = self.get_parameter("model_uri").value
        self._comp_gravity  = self.get_parameter("compensate_gravity").value

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

        plan = compute_spawn(
            (dx, dy, dz), yaw,
            speed_mps=self._speed,
            approach_angle_deg=self._angle_deg,
            miss_distance_m=self._miss_dist,
            offset_forward_m=self._offset_fwd,
            offset_vertical_m=self._offset_vert,
            compensate_gravity=self._comp_gravity,
        )
        spawn_x, spawn_y, spawn_z = plan.position

        if spawn_z < MIN_SPAWN_Z:
            self.get_logger().error(
                f"NOT spawning: z={spawn_z:.2f} m is below ground clearance "
                f"({MIN_SPAWN_Z} m) — drone z={dz:.2f}, "
                f"offset_vertical_m={self._offset_vert}. Fly higher first "
                f"(need altitude >= {MIN_SPAWN_Z - self._offset_vert:.1f} m, "
                f"e.g. raise takeoff_alt_m) and rerun."
            )
            return

        model_name = f"projectile_{self._scenario_id}_{int(time.time())}"
        vx, vy, vz = plan.velocity
        self.get_logger().info(
            f"Spawning '{model_name}' at ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f}) "
            f"vel=({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s "
            f"(gravity compensation {'ON' if self._comp_gravity else 'off'})"
        )
        ok, msg = gz_spawn(self._world, self._model_uri, model_name,
                           plan.position, plan.velocity)
        if ok:
            self.get_logger().info(f"Spawn OK: {msg}")
        else:
            self.get_logger().error(msg)


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
