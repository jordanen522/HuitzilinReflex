#!/usr/bin/env python3
"""hz_counterfactual.py — did the DODGE produce the miss, or did the throw?

The gap this closes. dodge_battery scores success as
`dodged and min_dist_m > hit_radius_m`, and only evaluates off_target when NO
dodge fired (dodge_battery.py line 811). So on every run that DID dodge, a
throw that was never going to hit is indistinguishable from a dodge that
saved the aircraft. A 78/78 built that way may be measuring the harness's aim
error rather than the vehicle.

Method. The ball is ballistic and completely unaffected by the dodge, so its
RECORDED ground-truth track is already the counterfactual ball path -- nothing
needs propagating or modelling (no drag or gravity assumptions enter). Only the
drone has to be counterfactualised: fit its constant velocity over the window
before the dodge command and extrapolate that straight line forward. That is
precisely "what if it had kept cruising".

    actual_min_m         = min |ball(t) - drone_actual(t)|          over t
    counterfactual_min_m = min |ball(t) - drone_cruising(t)|        over t
    delta_m              = actual - counterfactual      (dodge's contribution)

Both minima are taken over the SAME full episode. Before the dodge commits the
two paths are identical by construction -- the cruise line is fitted to the
actual pre-dodge samples -- so the counterfactual inherits the actual pre-dodge
distances and only diverges after t0. Taking one minimum over the whole track
and the other over the tail would silently favour whichever window happened to
contain the close pass.

Verdicts, against hit_radius:
    WOULD_HAVE_MISSED  counterfactual  > R   -- throw was never on target
    DODGE_SAVED        counterfactual <= R < actual
    DODGE_FAILED       both <= R
    DODGE_HURT         counterfactual  > R >= actual    (made it worse)
    NO_DODGE           no dodge fired; actual is the whole story

BALL NAMES. Both spawner conventions are matched, because this repo has two and
they disagree: dodge_battery names its ball ball_<rid>_r<rep>_<epoch> and
spawn_projectile names its own projectile_<scenario>_<epoch>. Matching only the
latter is what made the oracle blind under the battery. The battery's name also
carries the scenario and repeat, which is the join key back to the battery CSV
-- without it these verdicts cannot be split by ball speed, and a blended
dodge rate is exactly the number this project refuses to report.

CLOCK. Ball and drone arrive in the SAME TFMessage, so their relative geometry
is exact whatever clock is used. Only the join to the evade event needs a
common timebase, and /gz/dynamic_poses carries Gazebo's clock while the event
carries ROS sim time. Following hz_dodge_response.py, poses are keyed by
ROS-sim ARRIVAL time for that join only. Acceptable here for the same reason:
the extrapolation window is ~0.2 s. No velocity is ever derived from arrival
times (CLAUDE.md).

LIMIT, stated because it is load-bearing: the extrapolation is a straight line,
so a drone that would have TURNED during the window is approximated. Over the
~0.3 s that matters at cruise this is small, but it is an approximation, not
ground truth. `fit_resid_m` is reported per episode so a bad fit is visible
rather than assumed away.
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

# Must stay in step with dodge_battery.py and spawn_projectile.py. See the
# module docstring for what matching only one of them costs.
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
        # How straight the pre-dodge cruise actually was. A drone mid-turn
        # gives a large residual, and the straight-line extrapolation that
        # follows is then an approximation the reader must be told about.
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
