#!/usr/bin/env python3
"""hz_maneuver_probe.py — what does the vehicle DO when told to dodge?

There is no ball. That is the entire point.

Every acceleration figure quoted so far came from a counterfactual delta on a
scored throw, and two confounds sit on top of it: d_req varies with how well
that particular throw was aimed, and the counterfactual's extrapolation horizon
grows with detection range (~0.04 s at 3.4 m, ~0.35 s at 12 m), so cf_min is a
weaker estimate exactly where the interesting cells are. Neither confound is
removable by scoring more throws. Both vanish if the maneuver is commanded
directly from a known state and measured against nothing but the clock.

WHAT IS HELD CONSTANT
  * initial state — settled hover, |v_horiz| under V_SETTLE, at takeoff alt
  * command form  — dodge_speed_mps along one body axis, the exact command
                    evasion_node builds from a hover (v_drone ~ 0, so
                    dodge_velocity_command's cruise term is zero and the
                    command reduces to dodge_speed * direction)
  * command shape — EVADING dodge_duration_s -> RECOVERING recover_hold_s of
                    zeros -> HANDOFF patrol_handoff_s of silence, streamed at
                    evade_cmd_rate_hz. Same envelope the FC sees in a real
                    dodge, including the handback zero the cmd_router emits.

WHAT IS NOT THE SAME AS A SCORED DODGE, and must be said out loud:
  the sweep dodged from a PATROL CRUISE of a few m/s; this dodges from rest.
  dodge_velocity_command adds escape to cruise, so from hover the commanded
  vector is smaller and points purely along the escape. If the response curve
  measured here differs from the sweep's, cruise is a candidate reason and
  this probe cannot rule it in or out. Run --cruise later to test it.

WHY /cmd/evade AND NOT A PRIVATE TOPIC
  So that the thing under test is the real command path: same Twist, same
  mav_bridge._on_evade immediate send, same cmd_router.route arbitration, same
  MAV_FRAME_BODY_OFFSET_NED setpoint, same flight controller. The ONLY
  difference from a scored dodge is what decided to fire. Anything published
  on a private topic would measure a path nothing flies.

DIRECTIONS ALTERNATE (+Y, -Y, ...) for a mundane reason: in GUIDED with no
patrol, the vehicle holds wherever a dodge leaves it. Twelve dodges the same
way walk it 6-18 m downrange and into the fence. Alternating returns it near
the start every pair, and pays for itself as an axis-symmetry check --
analyse the signs separately before pooling them.

TIMEBASE: sim seconds from /clock, never wall-clock (CLAUDE.md). The vehicle's
own high-rate answer comes from the dataflash (SIM2 at 400 Hz); what this file
records is the COMMAND side plus a 30 Hz odom cross-check.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from enum import Enum

import rclpy
from geometry_msgs.msg import Accel, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Defaults mirror params/evasion.yaml exactly. They are NOT tuned here: this
# probe measures the response to the command the system already flies.
DODGE_SPEED_MPS = 1.5
DODGE_DURATION_S = 1.0
RECOVER_HOLD_S = 0.5
PATROL_HANDOFF_S = 0.8
EVADE_CMD_RATE_HZ = 20.0

V_SETTLE = 0.08          # m/s horizontal; above this the vehicle is not at rest
DRIFT_SETTLE = 0.06      # m of horizontal travel allowed across the window
SETTLE_WINDOW_S = 1.5    # how long those must hold before a trial may commit
SETTLE_TIMEOUT_S = 45.0  # give up on a trial rather than hang the run


class Phase(Enum):
    SETTLE = "settle"      # waiting for a quiet, repeatable initial state
    EVADE = "evade"        # streaming the dodge
    ZEROS = "zeros"        # RECOVERING: zero velocity AND zero accel
    SILENT = "silent"      # HANDOFF: publish nothing, let the bridge hand back
    REST = "rest"          # let the controller finish settling before re-arming
    DONE = "done"


def parse_dirs(spec: str):
    """'0,1,0;0,-1,0' -> [('+Y',(0,1,0)), ('-Y',(0,-1,0))], body FLU."""
    out = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [float(v) for v in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError(f"direction needs 3 components: {chunk!r}")
        n = math.sqrt(sum(v * v for v in parts))
        if n < 1e-9:
            raise ValueError(f"zero-length direction: {chunk!r}")
        unit = tuple(v / n for v in parts)
        axis = max(range(3), key=lambda i: abs(unit[i]))
        label = f"{'+' if unit[axis] >= 0 else '-'}{'XYZ'[axis]}"
        out.append((label, unit))
    if not out:
        raise ValueError("no directions parsed")
    return out


class ManeuverProbe(Node):
    def __init__(self, args) -> None:
        super().__init__("hz_maneuver_probe")
        self.a = args
        self._send_accel = (args.accel != "none")
        self.dirs = parse_dirs(args.dirs)

        self._evade_pub = self.create_publisher(Twist, "/cmd/evade", RELIABLE_QOS)
        self._accel_pub = self.create_publisher(Accel, "/cmd/evade_accel", RELIABLE_QOS)
        self.create_subscription(Odometry, "/huitzilin/odom",
                                 self._odom_cb, RELIABLE_QOS)

        self._odom_f = open(f"{args.out}_odom.csv", "w", newline="")
        self._odom_w = csv.writer(self._odom_f)
        self._odom_w.writerow(["t_s", "x", "y", "z", "vx", "vy", "vz",
                               "qx", "qy", "qz", "qw"])
        self._ev_f = open(f"{args.out}_events.csv", "w", newline="")
        self._ev_w = csv.writer(self._ev_f)
        self._ev_w.writerow(["trial", "dir_label", "t_commit_s",
                             "cmd_flu_x", "cmd_flu_y", "cmd_flu_z", "yaw_rad",
                             "x", "y", "z", "vx", "vy", "vz"])

        self._hist = []            # rolling (t, x, y, z, vh) for the settle test
        self._last = None
        self._phase = Phase.SETTLE
        self._t_phase = None       # sim s when the current phase began
        self._trial = 0
        self._cmd = (0.0, 0.0, 0.0)
        self._label = ""
        self._settle_start = None

        self.create_timer(1.0 / float(args.rate), self._tick)
        self.get_logger().info(
            f"maneuver probe: {args.trials} trials, speed {args.speed} m/s, "
            f"dirs {[d[0] for d in self.dirs]}, hold {args.hold}s, "
            f"stream {args.rate} Hz  -> {args.out}_*.csv")

    # ── clock ────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── telemetry ────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        t = self._now()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self._last = (t, p, q, v)
        self._odom_w.writerow([f"{t:.6f}", f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}",
                               f"{v.x:.5f}", f"{v.y:.5f}", f"{v.z:.5f}",
                               f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}"])
        vh = math.hypot(v.x, v.y)
        self._hist.append((t, p.x, p.y, p.z, vh))
        cutoff = t - SETTLE_WINDOW_S
        while len(self._hist) > 2 and self._hist[0][0] < cutoff:
            self._hist.pop(0)

    def _is_settled(self) -> bool:
        """Quiet AND stationary across a full window, not merely slow right now.

        A single sample under the threshold is satisfied at the top of an
        overshoot, which is the least repeatable instant there is.
        """
        if not self._hist or self._hist[-1][0] - self._hist[0][0] < SETTLE_WINDOW_S:
            return False
        if max(h[4] for h in self._hist) >= V_SETTLE:
            return False
        xs = [h[1] for h in self._hist]
        ys = [h[2] for h in self._hist]
        drift = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return drift < DRIFT_SETTLE

    # ── command ──────────────────────────────────────────────────────────
    def _publish(self, vel, accel=(0.0, 0.0, 0.0)) -> None:
        """Accel first, then velocity — the order evasion_node uses.

        The bridge only attaches a fresh acceleration to a velocity setpoint,
        so publishing it second would hang the previous tick's feedforward on
        the current command. Zero accel by default: evade_accel_ff_mps2 ships
        0.0, so that is the command every recorded result flew under.
        """
        if self._send_accel:
            a = Accel()
            a.linear.x, a.linear.y, a.linear.z = (float(v) for v in accel)
            self._accel_pub.publish(a)
        c = Twist()
        c.linear.x, c.linear.y, c.linear.z = (float(v) for v in vel)
        self._evade_pub.publish(c)

    @staticmethod
    def _yaw_of(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _commit(self) -> None:
        label, unit = self.dirs[self._trial % len(self.dirs)]
        self._cmd = tuple(u * float(self.a.speed) for u in unit)
        self._label = label
        t, p, q, v = self._last
        t_commit = self._now()
        self._publish(self._cmd)                       # send BEFORE logging
        self._ev_w.writerow([self._trial, label, f"{t_commit:.6f}",
                             f"{self._cmd[0]:.4f}", f"{self._cmd[1]:.4f}",
                             f"{self._cmd[2]:.4f}", f"{self._yaw_of(q):.6f}",
                             f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}",
                             f"{v.x:.5f}", f"{v.y:.5f}", f"{v.z:.5f}"])
        self._ev_f.flush()
        self.get_logger().info(
            f"trial {self._trial} {label}: commit t={t_commit:.3f} "
            f"cmd_flu=({self._cmd[0]:+.2f},{self._cmd[1]:+.2f},{self._cmd[2]:+.2f}) "
            f"at ({p.x:+.2f},{p.y:+.2f},{p.z:.2f})")

    def _enter(self, phase: Phase) -> None:
        self._phase = phase
        self._t_phase = self._now()

    # ── state machine ────────────────────────────────────────────────────
    def _tick(self) -> None:
        now = self._now()
        if now <= 0.0 or self._last is None:
            return                                     # no /clock or no odom yet
        if self._t_phase is None:
            self._t_phase = now
        held = now - self._t_phase

        if self._phase is Phase.SETTLE:
            if self._settle_start is None:
                self._settle_start = now
            if self._is_settled():
                self._settle_start = None
                self._commit()
                self._enter(Phase.EVADE)
            elif now - self._settle_start > SETTLE_TIMEOUT_S:
                self.get_logger().error(
                    f"trial {self._trial}: never settled in {SETTLE_TIMEOUT_S}s "
                    "— ABORTING the run rather than committing from an "
                    "unknown state (that is the one thing this probe controls)")
                self._enter(Phase.DONE)

        elif self._phase is Phase.EVADE:
            if held < float(self.a.hold):
                self._publish(self._cmd)
            else:
                self._publish((0.0, 0.0, 0.0))
                self._enter(Phase.ZEROS)

        elif self._phase is Phase.ZEROS:
            self._publish((0.0, 0.0, 0.0))
            if held >= float(self.a.settle):
                self._enter(Phase.SILENT)

        elif self._phase is Phase.SILENT:
            # Publish NOTHING. The bridge treats /cmd/evade as fresh for
            # cmd_timeout_s after the last message; going quiet is what lets
            # cmd_router emit its single handback zero and release the vehicle.
            if held >= float(self.a.gap):
                self._enter(Phase.REST)

        elif self._phase is Phase.REST:
            if held >= float(self.a.rest):
                self._trial += 1
                if self._trial >= int(self.a.trials):
                    self.get_logger().info(
                        f"{self._trial} trials complete -> {self.a.out}_events.csv")
                    self._enter(Phase.DONE)
                else:
                    self._enter(Phase.SETTLE)

        elif self._phase is Phase.DONE:
            self._close()
            raise SystemExit(0)

    def _close(self) -> None:
        for f in (self._odom_f, self._ev_f):
            try:
                f.flush()
                f.close()
            except Exception:
                pass


def main() -> None:
    argv = remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="output stem")
    ap.add_argument("--trials", type=int, default=16)
    ap.add_argument("--speed", type=float, default=DODGE_SPEED_MPS)
    ap.add_argument("--dirs", default="0,1,0;0,-1,0",
                    help="body-FLU directions, ';'-separated, cycled per trial")
    ap.add_argument("--hold", type=float, default=DODGE_DURATION_S)
    ap.add_argument("--settle", type=float, default=RECOVER_HOLD_S)
    ap.add_argument("--gap", type=float, default=PATROL_HANDOFF_S)
    ap.add_argument("--rest", type=float, default=6.0,
                    help="sim s of quiet between trials before re-settling")
    ap.add_argument("--rate", type=float, default=EVADE_CMD_RATE_HZ)
    # MEASURED 2026-08-07: publishing a zero Accel makes the bridge route via
    # send_velocity_accel_body(), whose vel+accel type_mask ArduPilot Copter
    # has no handler for -- it discards the ENTIRE setpoint, velocity included.
    # 28/28 trials moved 0.00 m under zero; the same command under none moved
    # 1.34 m. none is what evade_accel_ff_mps2=0.0 is DOCUMENTED to mean.
    ap.add_argument("--accel", choices=("none", "zero"), default="none",
                    help="none: publish only /cmd/evade. zero: also publish a"
                         " zero Accel, reproducing evasion_node._publish_evade.")
    args = ap.parse_args(argv[1:])

    rclpy.init(args=sys.argv)
    node = ManeuverProbe(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
