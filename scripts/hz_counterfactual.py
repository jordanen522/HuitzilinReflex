#!/usr/bin/env python3
"""Separate a dodge that saved the aircraft from a throw that was never on target.

Reads /gz/dynamic_poses and the evade event stream from a live run and writes one
CSV row per throw: actual_min_m (closest ball-drone approach as flown),
counterfactual_min_m (closest approach against the pre-dodge cruise velocity
extrapolated forward), delta_m, fit_resid_m, and a verdict
(WOULD_HAVE_MISSED / DODGE_SAVED / DODGE_FAILED / DODGE_HURT / NO_DODGE).
The ball is ballistic and unaffected by the dodge, so its recorded track is
already the counterfactual ball path; only the drone is extrapolated, and both
minima are taken over the same full episode.

`counterfactual_min_m` is emitted BLANK on NO_DODGE rows and stays blank on
purpose, so recorded CSVs remain reproducible. Whoever scores them must
substitute `counterfactual_min_m := actual_min_m` on those rows -- the drone
never deviated, so the flown path IS the counterfactual -- and count a NO_DODGE
inside the hit radius as a loss. Dropping the blanks deletes every no-fire from
the denominator and reports a fire-conditional rate as a save rate.
Full scoring rules: docs/RESULTS.md §10.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
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

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=50)
RELIABLE_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                          history=QoSHistoryPolicy.KEEP_LAST, depth=10)

FIT_WINDOW_S = 0.20      # pre-dodge samples fitted for the cruise baseline
EPISODE_GAP_S = 1.5      # ball unseen this long -> episode over
HIT_RADIUS_M = 0.30

# Two spawners, two naming conventions, both live: matching only one leaves the
# scorer blind to every ball the other spawned. Keep in step with both.
DEFAULT_BALL_PREFIXES = ("ball_", "projectile_")

# dodge_battery.py:704  name = f"ball_{rid}_r{rep}_{int(time.time())}"
_BATTERY_NAME = re.compile(r"^ball_(?P<sid>.+)_r(?P<rep>\d+)_\d+$")
# spawn_projectile.py:452  f"projectile_{scenario_id}_{int(time.time())}"
_SPAWNER_NAME = re.compile(r"^projectile_(?P<sid>.+)_\d+$")


def parse_ball_name(name: str):
    """(scenario_id, rep) from a ball's model name, either convention.

    The join key back to the battery CSV. Returns ("", "") rather than raising
    on an unrecognised name: a hand-spawned ball is still worth scoring, it
    just cannot be attributed to a scenario -- and therefore cannot be
    reported by ball speed, which the caller must notice.
    """
    m = _BATTERY_NAME.match(name)
    if m:
        return m.group("sid"), m.group("rep")
    m = _SPAWNER_NAME.match(name)
    if m:
        return m.group("sid"), ""
    return "", ""


class Counterfactual(Node):
    def __init__(self, out_path: str, drone_model: str, ball_prefixes,
                 hit_radius: float) -> None:
        super().__init__("hz_counterfactual")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self._drone_model = drone_model
        self._ball_prefixes = tuple(p for p in ball_prefixes if p)
        if not self._ball_prefixes:
            raise ValueError("no ball prefixes — nothing would ever be scored")
        self._R = hit_radius

        self._drone: deque = deque(maxlen=4000)      # (t, xyz)
        self._balls: dict = {}                       # name -> [(t, xyz), ...]
        self._last_seen: dict = {}
        self._dodges: list = []                      # (t_arrival, payload)
        self._episode = 0

        self._fh = open(out_path, "w", newline="")
        self._csv = csv.writer(self._fh)
        self._csv.writerow(["episode", "ball", "scenario", "rep", "t_dodge_s",
                            "actual_min_m", "counterfactual_min_m", "delta_m",
                            "verdict", "fit_resid_m", "n_pre", "n_post"])
        self._fh.flush()

        self.create_subscription(TFMessage, "/gz/dynamic_poses",
                                 self._pose_cb, SENSOR_QOS)
        self.create_subscription(String, "/threat/evade_event",
                                 self._event_cb, RELIABLE_QOS)
        self.create_timer(0.5, self._reap)
        self.get_logger().info(
            "counterfactual scoring -> %s | drone '%s' balls %s R=%.2f m"
            % (out_path, drone_model, list(self._ball_prefixes), hit_radius))

    # Poses carry Gazebo's clock and evade events carry ROS sim time, so both
    # are keyed by ROS-sim ARRIVAL time to give the join a common timebase.
    # Sound only because the fit window is ~0.2 s; never derive a velocity from
    # these timestamps. Ball and drone share one TFMessage, so their relative
    # geometry is exact regardless of which clock stamps it.
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _event_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = {}
        self._dodges.append((self._now(), payload))

    def _pose_cb(self, msg: TFMessage) -> None:
        t = self._now()
        for tr in msg.transforms:
            name = tr.child_frame_id
            p = tr.transform.translation
            xyz = np.array([p.x, p.y, p.z])
            if name == self._drone_model:
                self._drone.append((t, xyz))
            elif name.startswith(self._ball_prefixes):
                self._balls.setdefault(name, []).append((t, xyz))
                self._last_seen[name] = t

    def _reap(self) -> None:
        now = self._now()
        for name in [n for n, t in list(self._last_seen.items())
                     if now - t > EPISODE_GAP_S]:
            track = self._balls.pop(name, [])
            self._last_seen.pop(name, None)
            try:
                self._score(name, track)
            except Exception as e:                    # noqa: BLE001
                self.get_logger().warn("scoring %s failed: %s" % (name, e))

    def _drone_at(self, t: float):
        """Actual drone position nearest t, or None if no sample is close."""
        if not self._drone:
            return None
        best = min(((abs(ts - t), p) for ts, p in self._drone),
                   key=lambda kv: kv[0])
        return best[1] if best[0] <= 0.05 else None

    def _score(self, name: str, track: list) -> None:
        if len(track) < 4:
            return
        self._episode += 1
        sid, rep = parse_ball_name(name)
        t_start, t_end = track[0][0], track[-1][0]

        # Actual separation at every sample, kept per-sample so the
        # counterfactual can reuse the pre-dodge half unchanged.
        actual_by_t = []
        for t, b in track:
            d = self._drone_at(t)
            if d is not None:
                actual_by_t.append((t, b, float(np.linalg.norm(b - d))))
        if not actual_by_t:
            return
        actual = min(a for _, _, a in actual_by_t)

        dodge = next(((i, td) for i, (td, _) in enumerate(self._dodges)
                      if t_start - 0.5 <= td <= t_end + 0.5), None)
        if dodge is None:
            # Blank counterfactual, deliberately: see the module docstring.
            # The scorer substitutes actual_min_m here; dropping the row
            # deletes a no-fire from the denominator.
            self._emit(name, sid, rep, "", actual, "", "", "NO_DODGE",
                       "", 0, len(track))
            return
        # Consumed, so one dodge command can never be credited to two balls.
        idx, t0 = dodge
        self._dodges.pop(idx)

        pre = [(t, p) for t, p in self._drone if t0 - FIT_WINDOW_S <= t <= t0]
        if len(pre) < 3:
            self._emit(name, sid, rep, t0, actual, "", "", "NO_FIT",
                       "", len(pre), len(track))
            return
        ts = np.array([t for t, _ in pre])
        ps = np.stack([p for _, p in pre])
        A = np.stack([np.ones_like(ts), ts - t0], axis=1)
        coef, *_ = np.linalg.lstsq(A, ps, rcond=None)
        p0, v0 = coef[0], coef[1]
        # How straight the pre-dodge cruise was. A drone mid-turn gives a large
        # residual, flagging the straight-line extrapolation below as an
        # approximation rather than ground truth.
        resid = float(np.max(np.linalg.norm(ps - (A @ coef), axis=1)))

        # Before t0 the counterfactual path IS the actual path, so it inherits
        # those distances; only the tail is recomputed against the cruise line.
        cf = np.inf
        n_post = 0
        for t, b, a in actual_by_t:
            if t < t0:
                cf = min(cf, a)
                continue
            n_post += 1
            cf = min(cf, float(np.linalg.norm(b - (p0 + v0 * (t - t0)))))
        if not np.isfinite(cf) or n_post == 0:
            self._emit(name, sid, rep, t0, actual, "", "", "NO_POST",
                       round(resid, 4), len(pre), 0)
            return

        if cf > self._R and actual > self._R:
            verdict = "WOULD_HAVE_MISSED"
        elif cf <= self._R < actual:
            verdict = "DODGE_SAVED"
        elif cf <= self._R and actual <= self._R:
            verdict = "DODGE_FAILED"
        else:
            verdict = "DODGE_HURT"
        self._emit(name, sid, rep, t0, actual, cf, actual - cf, verdict,
                   round(resid, 4), len(pre), n_post)

    def _emit(self, name, sid, rep, t0, actual, cf, delta, verdict,
              resid, n_pre, n_post):
        def fmt(v):
            return ("%.4f" % v) if isinstance(v, float) else v
        self._csv.writerow([self._episode, name, sid, rep,
                            ("%.3f" % t0) if t0 != "" else "",
                            fmt(actual), fmt(cf), fmt(delta), verdict,
                            fmt(resid), n_pre, n_post])
        self._fh.flush()
        self.get_logger().warn(
            "ep%d %s %s: actual=%s counterfactual=%s"
            % (self._episode, sid or name, verdict, fmt(actual), fmt(cf)))

    def close(self) -> None:
        self._fh.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/counterfactual.csv")
    ap.add_argument("--drone-model", default="iris_depth")
    ap.add_argument("--ball-prefixes", nargs="+",
                    default=list(DEFAULT_BALL_PREFIXES),
                    help="model-name prefixes counted as a thrown ball; "
                         "BOTH spawner conventions by default")
    ap.add_argument("--hit-radius", type=float, default=HIT_RADIUS_M)
    args = ap.parse_args()

    rclpy.init()
    node = Counterfactual(args.out, args.drone_model, args.ball_prefixes,
                          args.hit_radius)
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
