#!/usr/bin/env python3
"""
hz_dodge_response.py — escape displacement vs. time since the dodge command,
from Gazebo ground truth. Written to answer whether the airframe's rise time,
rather than the trigger, is what a dodge is losing to.

THAT QUESTION IS CLOSED, AND THE ANSWER IS NO (Week 6). Dataflash over a
controlled WP_ACC sweep shows achieved velocity change EXCEEDING commanded in
every arm (0.93 vs 0.81, 1.26 vs 1.17, 1.47 vs 0.97 m/s) — the vehicle
over-delivers on what it is asked for. Rise time, tilt, jerk and WP_ACC are all
retired together. The binding term is sensor reach. See docs/week6_result.md.

TWO CAVEATS BEFORE TRUSTING A NUMBER FROM THIS SCRIPT:
  * Run it in HOVER (-p hover_mode:=true on the battery). Under patrol the
    baseline extrapolation below is a straight line through a vehicle that is
    tracking waypoints, so the vehicle's own curvature lands in the escape term:
    median fit residual 0.0112 m patrol vs 0.0011 m hover, and at matched tca
    0.385 s the two disagree 19x. Patrol escape figures are inflated 3-20x.
  * Escape displacement is not fit to referee a parameter on its own — two
    IDENTICAL control arms once drifted 1.58x apart. A/B within one session, fly
    the control twice, and prefer the dataflash velocity step (PSCN/PSCE DVN/DVE
    vs VN/VE), which compares command against achievement inside a single dodge.

Escape is measured against the drone's OWN pre-dodge motion, not a fixed point:
patrol cruise (~5.25 m/s) otherwise dominates raw displacement. The baseline
velocity is fitted over the 0.2 s before the trigger and extrapolated; escape is
the residual, projected onto the commanded ENU escape direction. Ideal is
dodge_speed_mps * t, what a perfect step response would deliver.

Run alongside the live stack, then throw (or run the battery); Ctrl-C to stop:

    source /opt/ros/jazzy/setup.bash && source ~/huitzilin_ws/install/setup.bash
    python3 ~/huitzilin_ws/scripts/hz_dodge_response.py --out /tmp/dodge_resp.csv

Two measurement traps this respects:
  * /gz/dynamic_poses carries EVERY link in the world, and recording all of them
    measurably blinds the detector being observed. Only the drone model is kept.
  * That topic is stamped with Gazebo's own clock (seconds since world start)
    while the evade event carries ROS sim time, so they cannot be joined on
    stamps. Poses are keyed by ROS-sim ARRIVAL time, the same clock the event
    uses. Acceptable only because this is a displacement measurement over
    100-400 ms windows — never derive a velocity this way.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Sample points after the command, in seconds. 1.0 s is dodge_duration_s, so the
# last one is the whole manoeuvre; the early ones are the only ones that can
# matter at a tca of 0.2-0.3 s.
SAMPLES_S = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 1.00)
BASELINE_FIT_S = 0.20     # window before the trigger used to fit cruise velocity
HISTORY_S = 3.0           # pose ring-buffer span
# evasion.yaml's SHIPPED dodge_speed_mps -- assumed, not measured. It is a
# swept parameter, so a run at another value makes every figure derived from
# this constant wrong in proportion. Rescale, or re-read it from the node.
DODGE_SPEED_MPS = 1.5     # evasion.yaml dodge_speed_mps; the command is a step


class DodgeResponse(Node):
    def __init__(self, out_path: str, drone_model: str) -> None:
        super().__init__("hz_dodge_response")
        self._drone_model = drone_model
        self.set_parameters([Parameter("use_sim_time", value=True)])

        # (recv_s, xyz world). A deque, not a list: the eviction below runs once
        # per /gz/dynamic_poses message and list.pop(0) shifts the whole buffer
        # each time. This script's own docstring warns that recording too much
        # of that topic blinds the detector it is observing, so the per-message
        # cost here is not free -- it competes with the pipeline being measured.
        self._poses: deque[tuple[float, np.ndarray]] = deque()
        self._pending: list[dict] = []
        self._n_dodges = 0

        self._fh = open(out_path, "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(["dodge_idx", "t_since_cmd_s", "escape_m",
                            "ideal_m", "tca_s", "miss_m"])

        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, SENSOR_QOS)
        self.create_subscription(String, "/threat/evade_event",
                                 self._event_cb, RELIABLE_QOS)
        self.create_timer(0.25, self._drain)
        self.get_logger().info(f"recording dodge response -> {out_path}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose_cb(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.child_frame_id != self._drone_model:
                continue
            t = tr.transform.translation
            self._poses.append((self._now(),
                                np.array([t.x, t.y, t.z], dtype=np.float64)))
        cutoff = self._now() - HISTORY_S
        while self._poses and self._poses[0][0] < cutoff:
            self._poses.popleft()

    def _event_cb(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "dodge_enu" not in ev:
            return
        self._n_dodges += 1
        ev["_idx"] = self._n_dodges
        # Snapshot the pre-trigger poses now: the ring buffer will have rolled
        # past them by the time the manoeuvre finishes.
        t0 = float(ev["t_trigger_s"])
        ev["_pre"] = [(t, p) for t, p in self._poses if t <= t0]
        self._pending.append(ev)

    def _drain(self) -> None:
        """Report any dodge whose full sample window has elapsed."""
        now = self._now()
        ready = [e for e in self._pending
                 if now - float(e["t_trigger_s"]) > SAMPLES_S[-1] + 0.2]
        for ev in ready:
            self._pending.remove(ev)
            self._report(ev)

    def _report(self, ev: dict) -> None:
        t0 = float(ev["t_trigger_s"])
        d_hat = np.asarray(ev["dodge_enu"], dtype=np.float64)
        n = np.linalg.norm(d_hat)
        if n < 1e-9:
            return
        d_hat = d_hat / n

        pre = [(t, p) for t, p in ev["_pre"] if t >= t0 - BASELINE_FIT_S]
        if len(pre) < 3:
            self.get_logger().warn(
                f"dodge {ev['_idx']}: only {len(pre)} pre-trigger poses — "
                "cannot fit cruise baseline, skipping")
            return
        ts = np.array([t for t, _ in pre])
        ps = np.stack([p for _, p in pre])
        # Least-squares constant velocity through the pre-trigger window, then
        # the position it predicts at t0. Fitting rather than differencing two
        # samples keeps pose jitter out of the baseline.
        A = np.stack([np.ones_like(ts), ts - t0], axis=1)
        coef, *_ = np.linalg.lstsq(A, ps, rcond=None)
        p0, v0 = coef[0], coef[1]

        post = [(t, p) for t, p in self._poses if t >= t0]
        if not post:
            return
        rows = []
        for dt in SAMPLES_S:
            target = t0 + dt
            gap, t, p = min((abs(t - target), t, p) for t, p in post)
            if gap > 0.05:      # no ground-truth sample near this offset
                continue
            residual = p - (p0 + v0 * (t - t0))
            rows.append((dt, float(residual @ d_hat)))

        for dt, esc in rows:
            self._csv.writerow([ev["_idx"], f"{dt:.2f}", f"{esc:.4f}",
                                f"{DODGE_SPEED_MPS * dt:.4f}",
                                f"{ev.get('tca_s', float('nan')):.3f}",
                                f"{ev.get('miss_m', float('nan')):.3f}"])
        self._fh.flush()

        tca = float(ev.get("tca_s", float("nan")))
        at_tca = [esc for dt, esc in rows if abs(dt - round(tca, 2)) < 0.03]
        summary = "  ".join(f"{dt:.2f}s={esc:+.3f}" for dt, esc in rows)
        self.get_logger().warn(
            f"dodge {ev['_idx']} tca={tca:.3f} miss={ev.get('miss_m', 0):.2f} | "
            f"escape {summary}"
            + (f" | at tca: {at_tca[0]:+.3f} m" if at_tca else ""))

    def close(self) -> None:
        self._fh.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/dodge_resp.csv")
    ap.add_argument("--drone-model", default="iris_depth")
    args = ap.parse_args()

    rclpy.init()
    node = DodgeResponse(args.out, args.drone_model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
