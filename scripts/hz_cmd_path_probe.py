#!/usr/bin/env python3
"""hz_cmd_path_probe.py — where does a dodge command die?

Attributes a dodge that produced no airframe movement to one boundary on the
command path. READ-ONLY: publishes nothing, touches no flight node; run it
alongside a normal battery.

    evasion_node                mav_bridge_node              ArduPilot
    /cmd/evade  --20 Hz-->  send_velocity_body() --:14552-->   ^
                                                               | fights
    patrol_node --10 Hz send_position_ned() -------:14553------+

Three failure modes:
  (a) stream died          -- n_evade_cmds far below rate*duration
  (b) patrol never yielded -- patrol_state.running stayed True through the
                              dodge, so its position setpoints kept flying to
                              the waypoint on a SECOND MAVLink connection the
                              bridge cannot preempt
  (c) commanded but inert  -- stream fine, patrol stopped, odom velocity flat
                              => fault is below the bridge (ArduPilot)

TIMEBASE: every message is keyed by RECEIPT time in the probe's own clock,
never by msg.header.stamp, and the probe must run on sim time so those receipts
share a timebase with the evasion node's t_trigger_s.

    python3 scripts/hz_cmd_path_probe.py --out /tmp/cmd_path.jsonl \
        --ros-args -p use_sim_time:=true
    python3 scripts/hz_cmd_path_probe.py --summarise /tmp/cmd_path.jsonl

Result: the command path is NOT the bottleneck — this probe returned ok 16/16.
Do not re-run it to explain a missed dodge (`CLAUDE.md`, measured nulls).
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import deque

# Window kept around each trigger. pre covers the approach (was patrol already
# stopping?), post covers dodge_duration_s + recover_hold_s with margin.
PRE_S = 1.0
POST_S = 2.5
# How long to wait past the trigger before emitting the record, so the whole
# post-window has actually arrived.
REPORT_DELAY_S = POST_S + 0.3
BUFFER_S = PRE_S + POST_S + 2.0
# Offsets (s since trigger) at which the odom response is sampled.
PROBE_OFFSETS = (-0.20, 0.0, 0.05, 0.10, 0.20, 0.40, 0.60, 1.00, 1.50)
# A sample further than this from the requested instant is reported as missing
# rather than pretended to have been measured there.
SAMPLE_TOL_S = 0.15
# Verdict thresholds. A dodge streams dodge_duration_s * evade_cmd_rate_hz
# = 1.0 * 20 commands; half that is unambiguously a dead stream.
MIN_HEALTHY_CMDS = 10
# tca is 0.09-0.45 s, so patrol still commanding position 0.30 s in has had
# the vehicle for most of the time that mattered.
LATE_YIELD_S = 0.30
# Speed change over the whole dodge below which the airframe did not respond.
INERT_SPEED_DELTA = 0.15


def _nearest(samples, t):
    """Sample nearest in time to t, or None if the buffer does not span it."""
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s[0] - t))
    return best if abs(best[0] - t) <= SAMPLE_TOL_S else None


def _verdict(rec: dict) -> str:
    """Attribute one dodge to the boundary that failed."""
    if rec["n_evade_cmds"] < MIN_HEALTHY_CMDS:
        return "STREAM DIED"
    stop = rec["patrol_stop_offset_s"]
    if stop is None:
        return "PATROL NEVER YIELDED"
    if stop > LATE_YIELD_S:
        return f"PATROL LATE (>{LATE_YIELD_S:g}s)"
    dv = rec["odom_speed_delta"].get("1")
    if dv is not None and dv < INERT_SPEED_DELTA:
        return "COMMANDED BUT INERT"
    return "ok"


def summarise(path: str) -> int:
    """Print a per-dodge verdict table from a recorded jsonl file."""
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    if not rows:
        print(f"{path}: no dodge records", file=sys.stderr)
        return 1

    print("  idx  tca    cmds   rate  patrol_stop  |dv|@0.2  |dv|@1.0  verdict")
    print("  " + "-" * 74)
    verdicts: dict[str, int] = {}
    for i, r in enumerate(rows, 1):
        v = _verdict(r)
        verdicts[v] = verdicts.get(v, 0) + 1
        stop = r["patrol_stop_offset_s"]
        s = "  --  " if stop is None else f"{stop:+.3f}"
        d = r["odom_speed_delta"]
        fmt = lambda x: "  --  " if x is None else f"{x:.3f}"
        print(f"  {i:3d}  {r['tca_s']:.3f}  {r['n_evade_cmds']:4d}  "
              f"{r['evade_rate_hz']:5.1f}  {s:>11}  "
              f"{fmt(d.get('0.2')):>8}  {fmt(d.get('1')):>8}  {v}")

    print("\n  Verdicts:")
    for v, c in sorted(verdicts.items(), key=lambda kv: -kv[1]):
        print(f"    {c:3d}/{len(rows)}  {v}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="/tmp/cmd_path.jsonl",
                    help="where to append one JSON record per dodge")
    ap.add_argument("--summarise", metavar="JSONL",
                    help="analyse a recorded file instead of recording")
    args, _ = ap.parse_known_args(argv)

    if args.summarise:
        return summarise(args.summarise)

    # Imported late so --summarise works on a machine without ROS.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class CmdPathProbe(Node):
        def __init__(self) -> None:
            super().__init__("hz_cmd_path_probe")
            self._out = open(args.out, "a", buffering=1)
            self._evade: deque = deque()    # (t_recv, (x, y, z))
            self._patrol: deque = deque()   # (t_recv, running)
            self._odom: deque = deque()     # (t_recv, (vx, vy, vz))
            self._pending: list = []        # (t_report, event dict)
            self._n = 0

            self.create_subscription(String, "/threat/evade_event",
                                     self._on_event, 10)
            self.create_subscription(Twist, "/cmd/evade", self._on_evade, 10)
            self.create_subscription(String, "/huitzilin/patrol_state",
                                     self._on_patrol, 10)
            self.create_subscription(Odometry, "/huitzilin/odom",
                                     self._on_odom, 10)
            self.create_timer(0.05, self._tick)
            self.get_logger().info(
                f"cmd-path probe up (read-only) -> {args.out}")

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _trim(self, buf, now) -> None:
            while buf and buf[0][0] < now - BUFFER_S:
                buf.popleft()

        # -- Boundary taps. Receipt time only: see TIMEBASE TRAP above. ----
        def _on_evade(self, msg: Twist) -> None:
            t = self._now()
            self._evade.append((t, (msg.linear.x, msg.linear.y, msg.linear.z)))
            self._trim(self._evade, t)

        def _on_patrol(self, msg: String) -> None:
            t = self._now()
            try:
                running = bool(json.loads(msg.data).get("running"))
            except (ValueError, AttributeError):
                return
            self._patrol.append((t, running))
            self._trim(self._patrol, t)

        def _on_odom(self, msg: Odometry) -> None:
            t = self._now()
            v = msg.twist.twist.linear
            self._odom.append((t, (v.x, v.y, v.z)))
            self._trim(self._odom, t)

        def _on_event(self, msg: String) -> None:
            try:
                ev = json.loads(msg.data)
            except ValueError:
                return
            # t_trigger_s is the evasion node's SIM clock, which is this
            # probe's clock too when launched with use_sim_time:=true.
            self._pending.append((ev["t_trigger_s"] + REPORT_DELAY_S, ev))

        def _tick(self) -> None:
            now = self._now()
            ready = [p for p in self._pending if p[0] <= now]
            self._pending = [p for p in self._pending if p[0] > now]
            for _, ev in ready:
                self._emit(ev)

        def _emit(self, ev: dict) -> None:
            t0 = ev["t_trigger_s"]
            win = [(t, v) for t, v in self._evade if t0 <= t <= t0 + POST_S]
            span = (win[-1][0] - win[0][0]) if len(win) > 1 else 0.0
            rate = (len(win) - 1) / span if span > 1e-6 else 0.0

            # First moment patrol reports itself stopped after the trigger.
            # None means it never did within the window -- its 10 Hz absolute
            # position setpoints were still flying the drone to the waypoint
            # on a connection the bridge has no way to preempt.
            stop = None
            for t, running in self._patrol:
                if t >= t0 and not running:
                    stop = t - t0
                    break
            pre = _nearest(self._patrol, t0 - 0.1)

            base = _nearest(self._odom, t0)
            speed_delta: dict = {}
            vel: dict = {}
            for off in PROBE_OFFSETS:
                s = _nearest(self._odom, t0 + off)
                key = f"{off:g}"
                if s is None or base is None:
                    speed_delta[key] = None
                    vel[key] = None
                    continue
                vel[key] = list(s[1])
                speed_delta[key] = sum(
                    (a - b) ** 2 for a, b in zip(s[1], base[1])) ** 0.5

            self._n += 1
            rec = {
                "idx": self._n,
                "t_trigger_s": t0,
                "tca_s": ev.get("tca_s"),
                "miss_m": ev.get("miss_m"),
                "latency_s": ev.get("latency_s"),
                "dodge_body": ev.get("dodge_body"),
                "dodge_enu": ev.get("dodge_enu"),
                "n_evade_cmds": len(win),
                "evade_rate_hz": rate,
                "evade_first_offset_s": (win[0][0] - t0) if win else None,
                "evade_last_offset_s": (win[-1][0] - t0) if win else None,
                # Constant through the dodge if the node is behaving.
                "evade_cmd_first": list(win[0][1]) if win else None,
                "evade_cmd_last": list(win[-1][1]) if win else None,
                "patrol_running_before": None if pre is None else pre[1],
                "patrol_stop_offset_s": stop,
                "odom_speed_delta": speed_delta,
                "odom_vel": vel,
            }
            self._out.write(json.dumps(rec) + "\n")
            self.get_logger().warn(
                f"dodge {self._n}: cmds={len(win)} @{rate:.1f} Hz  "
                f"patrol_stop="
                f"{'NEVER' if stop is None else format(stop, '+.3f') + 's'}  "
                f"verdict={_verdict(rec)}")

    rclpy.init()
    node = CmdPathProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
