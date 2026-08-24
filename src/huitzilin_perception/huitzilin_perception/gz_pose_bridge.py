"""
gz_pose_bridge.py — HuitzilinReflex Week 4.

Replaces the `ros_gz_bridge parameter_bridge` conversion for
`/world/<world>/dynamic_pose/info` (gz.msgs.Pose_V -> tf2_msgs/TFMessage),
which is not registered in this Harmonic/ros_gz_bridge build (bridge starts
but never produces a factory match, so /gz/dynamic_poses stays silent).

Same workaround as spawn_projectile.py: shell out to the `gz` CLI instead of
depending on Python gz bindings or the ros_gz_bridge factory table. Here we
stream `gz topic -e -t <topic>`, which prints each message as protobuf text
format, and hand-parse it into TFMessage — matching the exact shape
dodge_battery.py already expects on /gz/dynamic_poses (TransformStamped per
pose, child_frame_id = model name).

Message format (captured via `gz topic -e -t .../dynamic_pose/info`):
  header {
    stamp { sec: <int> nsec: <int> }
  }
  pose {
    name: "<model>"
    id: <int>
    position { x: <f> y: <f> z: <f> }
    orientation { x: <f> y: <f> z: <f> w: <f> }
  }
  pose { ... }
  header { ... }      # next message begins — no separator line between them
  pose { ... }
  ...

Message framing (corrected on the Dell): this build emits **no
blank-line separators** — top-level blocks stream back to back. An earlier
version split messages on blank lines, so the buffer never flushed and
/gz/dynamic_poses stayed silent while the source streamed fine. Boundaries are
therefore inferred: a new top-level `header {`, or a `pose` whose model name
already appeared in the message being built (covers messages that omit an
unchanged `header`). `header` is optional, so the last-seen stamp is reused
when it's missing.

Grouping matters: dodge_battery.py measures drone-vs-ball distance only when
both appear in the SAME TFMessage, so each published message must be a whole
Pose_V snapshot — never one pose per message.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
import threading
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

_BLOCK_OPEN_RE = re.compile(r"^(\w+)\s*\{$")
_SCALAR_RE = re.compile(r"^\s*(\w+)\s*:\s*(-?[0-9.eE+-]+)")
_NAME_RE = re.compile(r'^\s*name\s*:\s*"(.*)"')


def split_top_level_blocks(lines: list[str]) -> list[tuple[str, list[str]]]:
    """Split protobuf-text lines into (key, inner_lines) for each top-level
    `key { ... }` block, tracking brace depth so nested blocks (position,
    orientation, stamp) stay intact inside their parent's inner_lines."""
    blocks: list[tuple[str, list[str]]] = []
    i, n = 0, len(lines)
    while i < n:
        m = _BLOCK_OPEN_RE.match(lines[i].strip())
        if not m:
            i += 1
            continue
        key = m.group(1)
        depth = 1
        inner: list[str] = []
        j = i + 1
        while j < n and depth > 0:
            depth += lines[j].count("{") - lines[j].count("}")
            if depth > 0:
                inner.append(lines[j])
            j += 1
        blocks.append((key, inner))
        i = j
    return blocks


def parse_scalar_block(lines: list[str]) -> dict[str, float]:
    """Parse flat `field: <number>` lines (position/orientation/stamp)."""
    out: dict[str, float] = {}
    for line in lines:
        m = _SCALAR_RE.match(line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def parse_pose_block(lines: list[str]) -> Optional[tuple[str, dict, dict]]:
    """Parse one `pose { ... }` block into (name, position, orientation)."""
    name = None
    for line in lines:
        m = _NAME_RE.match(line)
        if m:
            name = m.group(1)
            break
    if name is None:
        return None
    position, orientation = {}, {}
    for key, inner in split_top_level_blocks(lines):
        if key == "position":
            position = parse_scalar_block(inner)
        elif key == "orientation":
            orientation = parse_scalar_block(inner)
    return name, position, orientation


def parse_message(lines: list[str]) -> tuple[Optional[Time], list[tuple[str, dict, dict]]]:
    """Parse one blank-line-delimited Pose_V message into (stamp, poses).
    stamp is None when this message carried no `header` block."""
    stamp = None
    poses: list[tuple[str, dict, dict]] = []
    for key, inner in split_top_level_blocks(lines):
        if key == "header":
            for hkey, hinner in split_top_level_blocks(inner):
                if hkey == "stamp":
                    s = parse_scalar_block(hinner)
                    if "sec" in s:
                        stamp = Time(sec=int(s["sec"]), nanosec=int(s.get("nsec", 0)))
        elif key == "pose":
            parsed = parse_pose_block(inner)
            if parsed is not None:
                poses.append(parsed)
    return stamp, poses


class GzPoseBridgeNode(Node):
    """Streams `gz topic -e -t /world/<world>/dynamic_pose/info` and
    republishes it as tf2_msgs/TFMessage on `/gz/dynamic_poses`, matching
    what dodge_battery.py expects (child_frame_id = model name)."""

    def __init__(self) -> None:
        super().__init__("gz_pose_bridge")

        self.declare_parameter("world_name", "huitzilin_runway")
        self.declare_parameter("output_topic", "/gz/dynamic_poses")
        self.declare_parameter("frame_id", "world")

        world = self.get_parameter("world_name").value
        self._output_topic = self.get_parameter("output_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._gz_topic = f"/world/{world}/dynamic_pose/info"
        self._last_stamp: Optional[Time] = None

        self._pub = self.create_publisher(TFMessage, self._output_topic, SENSOR_QOS)

        # `gz topic -e` writes to std::cout, which is BLOCK-buffered (~4 KB)
        # when stdout is a pipe rather than a TTY. Without line buffering the
        # poses arrive in bursts — or appear to never arrive at all at low pose
        # rates — which looks exactly like a dead bridge. `stdbuf -oL` forces
        # line buffering. (spawn_projectile.py never needed this: its gz calls
        # are one-shot and flush at process exit.)
        cmd = ["gz", "topic", "-e", "-t", self._gz_topic]
        if shutil.which("stdbuf"):
            cmd = ["stdbuf", "-oL"] + cmd
        else:
            self.get_logger().warn(
                "stdbuf not found — gz stdout stays block-buffered, so poses "
                "may arrive in bursts"
            )

        # stderr must NOT be an undrained PIPE: gz can be chatty, and a full
        # ~64 KB pipe buffer blocks the child on write, stalling stdout with no
        # diagnostic. Route it to a temp file we can read on exit instead.
        self._stderr_file = tempfile.TemporaryFile(mode="w+")
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=self._stderr_file, text=True,
        )
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"gz_pose_bridge streaming '{self._gz_topic}' -> '{self._output_topic}'"
        )

    def _read_loop(self) -> None:
        """Accumulate whole top-level blocks by brace depth and flush one
        Pose_V snapshot per inferred message boundary (see module docstring —
        this stream has no blank-line separators to split on)."""
        assert self._proc.stdout is not None
        depth = 0
        block: list[str] = []      # lines of the top-level block being read
        message: list[str] = []    # blocks accumulated for the current message
        seen_names: set[str] = set()

        for line in self._proc.stdout:
            if self._stop.is_set():
                return
            stripped = line.strip()
            if not stripped:
                continue
            if depth == 0 and not _BLOCK_OPEN_RE.match(stripped):
                continue  # stray line between blocks (banner, warning)

            block.append(line)
            depth += line.count("{") - line.count("}")
            if depth > 0:
                continue
            depth = 0  # clamp: a stray '}' must not desync the reader

            key = _BLOCK_OPEN_RE.match(block[0].strip()).group(1)
            name = self._block_model_name(block) if key == "pose" else None

            # A message ends when the next one starts: either a fresh header,
            # or a model we've already recorded in this snapshot.
            if message and (key == "header" or (name is not None and name in seen_names)):
                self._publish_or_warn(message)
                message, seen_names = [], set()

            message.extend(block)
            if name is not None:
                seen_names.add(name)
            block = []

        if message:
            self._publish_or_warn(message)
        if not self._stop.is_set():
            self.get_logger().error(
                f"gz topic stream on '{self._gz_topic}' ended unexpectedly: "
                f"{self._read_stderr()}"
            )

    @staticmethod
    def _block_model_name(block: list[str]) -> Optional[str]:
        """Model name from a `pose { ... }` block, used for boundary detection."""
        for line in block:
            m = _NAME_RE.match(line)
            if m:
                return m.group(1)
        return None

    def _publish_or_warn(self, lines: list[str]) -> None:
        """Never let one malformed message kill the reader thread: a stray
        `nan`/`inf` or a line torn at a buffer boundary would raise inside
        float(), and this thread is the only thing feeding the topic — it
        would go permanently silent with nothing logged."""
        try:
            self._publish_message(lines)
        except Exception as exc:  # noqa: BLE001 — must keep streaming
            self.get_logger().warn(
                f"skipped unparseable pose message ({exc.__class__.__name__}: {exc})",
                throttle_duration_sec=5.0,
            )

    def _read_stderr(self) -> str:
        """Drain the child's stderr temp file (see __init__ for why it isn't a pipe)."""
        try:
            self._stderr_file.seek(0)
            return self._stderr_file.read().strip()
        except Exception:  # noqa: BLE001 — diagnostics must not raise
            return "<stderr unavailable>"

    def _publish_message(self, lines: list[str]) -> None:
        stamp, poses = parse_message(lines)
        if stamp is not None:
            self._last_stamp = stamp
        if not poses:
            return
        tf_msg = TFMessage()
        for name, position, orientation in poses:
            t = TransformStamped()
            t.header.stamp = self._last_stamp if self._last_stamp is not None \
                else self.get_clock().now().to_msg()
            t.header.frame_id = self._frame_id
            t.child_frame_id = name
            t.transform.translation.x = position.get("x", 0.0)
            t.transform.translation.y = position.get("y", 0.0)
            t.transform.translation.z = position.get("z", 0.0)
            t.transform.rotation.x = orientation.get("x", 0.0)
            t.transform.rotation.y = orientation.get("y", 0.0)
            t.transform.rotation.z = orientation.get("z", 0.0)
            t.transform.rotation.w = orientation.get("w", 1.0)
            tf_msg.transforms.append(t)
        self._pub.publish(tf_msg)

    def destroy_node(self) -> bool:
        """Stop the reader thread and reap the `gz topic` child process.

        Ordering matters: the stop flag is set before the child is
        terminated, so the reader sees the pipe close as a shutdown rather
        than as a read error. The child is killed if it ignores SIGTERM,
        because a surviving `gz topic` holds the transport subscription and
        the next run sees no poses at all.
        """
        self._stop.set()
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        # Close the stdout pipe and join the reader. The thread is a daemon, so
        # the process could exit without either -- but only at exit. Every
        # destroy_node() before that leaked a pipe fd and left a thread blocked
        # in `for line in self._proc.stdout`, which matters because the dodge
        # battery constructs and tears down this bridge once per scenario.
        # Ordering: the child is already dead, so the read hits EOF and the
        # loop ends on its own; closing first would race it into a ValueError.
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        try:
            if self._proc.stdout is not None:
                self._proc.stdout.close()
        except Exception:  # noqa: BLE001 — shutdown must not raise
            pass
        try:
            self._stderr_file.close()
        except Exception:  # noqa: BLE001 — shutdown must not raise
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzPoseBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM/SIGINT already shut the context down; a second shutdown
        # raises RCLError (seen as a traceback after every launch teardown).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
