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
  the projectile model, then a /world/<world>/wrench impulse to throw it.
  EntityFactory has NO velocity field in Harmonic, so the throw cannot be
  part of the spawn — see gz_impulse. The world must load
  gz-sim-apply-link-wrench-system or the ball spawns and simply drops.
- gz requests are protobuf TEXT format, not JSON. `gz service --req` parses
  text format only; a JSON body is rejected with an empty reply and exit
  code 0, which reads as success unless the reply is checked (_run_gz).
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

# Must match models/projectile/model.sdf <mass> and the world's
# <max_step_size>; gz_impulse converts a target velocity into a one-step
# force with them, so a change on either side rescales every throw.
PROJECTILE_MASS_KG = 0.150
WORLD_STEP_S = 0.001


def gz_spawn(world: str, model_uri: str, name: str, position, velocity,
             timeout_s: float = 5.0) -> tuple[bool, str]:
    """Spawn `model_uri` as `name` at ENU `position`, then kick it to
    `velocity` with a one-step impulse. Returns (ok, message).
    Module-level so dodge_battery.py can reuse it without a node."""
    req = (
        f'sdf_filename: "{model_uri.replace("model://", "")}", '
        f'name: "{name}", '
        f'pose: {{position: {_tf_vec3(position)}, '
        f'orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'
    )
    cmd = [
        "gz", "service", "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    # Pause across create+impulse. Each gz CLI call costs ~0.2 s of wall
    # clock, so on a running world the ball free-falls ~0.4 s before its
    # wrench lands — measured 2026-07-26: it reached the ground first and
    # ground contact ate over half the kick (3.8 m/s delivered of 8.0).
    # Paused, both messages apply on the same resume step: an 8 m/s vertical
    # test throw from z=5.0 apexed at 8.261 m vs 8.26 predicted.
    paused, _ = gz_world_control(world, "pause: true", timeout_s)
    try:
        ok, msg = _run_gz(cmd, timeout_s)
        if not ok:
            return False, msg
        if "data: true" not in msg:
            return False, (f"create service did not confirm the spawn "
                           f"(reply {msg!r}) — is model {model_uri} on the "
                           f"server's GZ_SIM_RESOURCE_PATH?")
        ok, msg = gz_impulse(world, name, velocity, timeout_s=timeout_s)
    finally:
        if paused:
            gz_world_control(world, "pause: false", timeout_s)
    return ok, msg


def gz_world_control(world: str, body: str,
                     timeout_s: float = 5.0) -> tuple[bool, str]:
    """gz.msgs.WorldControl request, e.g. body='pause: true'."""
    ok, msg = _run_gz([
        "gz", "service", "-s", f"/world/{world}/control",
        "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", body,
    ], timeout_s)
    return (ok and "data: true" in msg), msg


def gz_impulse(world: str, model_name: str, velocity, link_name: str = "link",
               mass_kg: float = PROJECTILE_MASS_KG,
               step_s: float = WORLD_STEP_S,
               timeout_s: float = 5.0) -> tuple[bool, str]:
    """Accelerate a just-spawned model from rest to `velocity` (world ENU).

    EntityFactory carries no velocity field — verified against
    `gz msg --info gz.msgs.EntityFactory` on this Harmonic build, whose
    oneof is sdf/sdf_filename/model/light/clone_name only — so the throw has
    to be a force applied after creation. A message on /world/<world>/wrench
    is applied by the ApplyLinkWrench system for exactly one physics
    iteration, so the delivered impulse is F*step_s and the resulting speed
    is F*step_s/mass — hence F = mass*v/step_s. The ball then flies a true
    drag-free parabola under gravity, which is what ballistics.py and the
    Week 4 Kalman filter both assume.

    Coupling to watch: `mass_kg` must match models/projectile/model.sdf and
    `step_s` must match the world's <max_step_size> (0.001 — see CLAUDE.md);
    changing either silently rescales every throw.
    """
    force = [mass_kg * float(v) / step_s for v in velocity]
    msg = (f'entity: {{name: "{model_name}::{link_name}", type: LINK}}, '
           f'wrench: {{force: {_tf_vec3(force)}, '
           f'torque: {{x: 0, y: 0, z: 0}}}}')
    cmd = [
        "gz", "topic", "-t", f"/world/{world}/wrench",
        "-m", "gz.msgs.EntityWrench", "-p", msg,
    ]
    ok, out = _run_gz(cmd, timeout_s)
    if not ok:
        return False, f"spawned but impulse failed: {out}"
    return True, (f"spawned + impulse "
                  f"({force[0]:.0f}, {force[1]:.0f}, {force[2]:.0f}) N "
                  f"for one {step_s * 1e3:.0f} ms step")


def gz_remove(world: str, name: str, timeout_s: float = 5.0) -> tuple[bool, str]:
    """Remove model `name` from the world (gz.msgs.Entity.Type MODEL).
    The dodge battery calls this between runs so spent balls don't litter
    the ground plane and confuse the depth background model."""
    req = f'name: "{name}", type: MODEL'
    cmd = [
        "gz", "service", "-s", f"/world/{world}/remove",
        "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000", "--req", req,
    ]
    return _run_gz(cmd, timeout_s)


def _tf_vec3(v) -> str:
    """gz.msgs.Vector3d in protobuf text format."""
    return f"{{x: {float(v[0]):.6f}, y: {float(v[1]):.6f}, z: {float(v[2]):.6f}}}"


def _run_gz(cmd: list[str], timeout_s: float) -> tuple[bool, str]:
    """Run a `gz` CLI call. (ok, stdout) on success, (False, reason) otherwise.

    Exit code 0 is NOT proof of success: a request the server rejects still
    exits 0 with empty stdout, and a request `gz` itself cannot parse writes
    a protobuf error to stderr and also exits 0. Both are caught here so no
    caller can read silence as success — that hid every failed spawn of the
    2026-07-26 bring-up session.
    """
    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, "gz call timed out"
    except FileNotFoundError:
        return False, "'gz' command not found — is Gazebo Harmonic installed?"
    if result.returncode != 0:
        return False, f"gz call failed: {result.stderr.strip()}"
    stderr = result.stderr.strip()
    if "Error parsing" in stderr or "Unable to create request" in stderr:
        return False, f"gz rejected the request: {stderr}"
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
