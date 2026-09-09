"""Kinematics over a short window of body joints, and the aggression score.

Scope is aggressive motion ONLY. There are three categories -- LUNGE, STRIKE,
SHOVE -- and there is no benign class. Benign motion is not labelled; it is
simply the case where nothing crossed a threshold. The subsystem therefore
never builds a description of what a person is doing, only whether an
aggression threshold tripped, which is a privacy property as much as a scope
one: it keeps this an alarm rather than an activity log.

Two categories an earlier draft carried are deliberately absent:

  GRAPPLE_PROXIMITY -- proximity is not aggression. Two bodies close together
  and moving quickly is also an embrace, a handshake, or helping someone up.
  As a category it generates false alarms while wearing a threat label.

  RAISED_OBJECT -- not detectable from this input at all. The whitelist is
  twelve body joints and carries no object detection, so a raised hand holding
  a weapon and a raised hand hailing a taxi are the identical signal. It is
  deferred rather than dropped: it needs an object detector, which is a
  separate capability with its own compute budget.

SCORING IS CONJUNCTIVE. A category takes the MINIMUM over its required
evidence terms, never the maximum or a sum. Disjunctive scoring fires on any
one term, so a person walking briskly toward the aircraft would score as a
lunge on approach speed alone. The cost of a false positive here is a siren
pointed at a bystander, so every term must agree before anything fires.

The scores are normalised threshold ramps, NOT probabilities and NOT the
output of a trained model. Nothing here has been validated against labelled
data; see PROVENANCE, which travels with every published event.

Frame: camera optical, +Z depth away from the camera, +Y down. Metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

from huitzilin_action_escalation.pose_frame import PoseFrame

# Carried in every event so a recorded score can never be mistaken for a
# measured detection rate.
PROVENANCE = "UNVALIDATED_HEURISTIC"


class Category(Enum):
    LUNGE = "LUNGE"
    STRIKE = "STRIKE"
    SHOVE = "SHOVE"


@dataclass(frozen=True)
class FeatureConfig:
    # How much history the rates are measured over. Long enough to span a
    # strike, short enough that a completed action has left the window before
    # the next one starts.
    window_s: float = 0.6
    # Joints below this are treated as absent. A pose estimator reports a
    # confidence for an occluded limb rather than omitting it, and a
    # hallucinated wrist moves fast.
    min_joint_conf: float = 0.35


@dataclass(frozen=True)
class Features:
    """All rates are peaks over consecutive frame pairs inside the window.

    Peaks rather than window-endpoint averages: a strike is over in roughly
    150 ms, and averaging it across a 600 ms window divides its speed by four
    and hides it under the threshold.
    """

    approach_speed_mps: float = 0.0
    body_speed_mps: float = 0.0
    torso_lean_rate_radps: float = 0.0
    wrist_speed_max_mps: float = 0.0
    wrist_extension_rate_mps: float = 0.0
    bilateral_reach_rate_mps: float = 0.0
    span_s: float = 0.0
    n_frames: int = 0


@dataclass(frozen=True)
class ScoreConfig:
    """(lo, hi) ramps: at lo the term contributes 0, at hi it contributes 1.

    These are engineering starting points chosen to sit above ordinary
    movement, not values fitted to data. No measured false-positive or
    false-negative rate exists for any of them.
    """

    approach_lunge: tuple = (1.2, 2.5)
    body_speed_lunge: tuple = (1.0, 2.2)
    lean_rate_lunge: tuple = (0.5, 1.5)

    # Hand speed. A committed punch runs 3-6 m/s; the low end sits above
    # gesture and arm-swing speeds rather than at rest.
    wrist_speed_strike: tuple = (1.5, 4.0)
    # Shoulder-to-wrist opening. Bounded by arm length: the whole travel is
    # about 0.6 m, so a rate above ~3 m/s is not anatomically reachable and a
    # ramp topping out at 4.0 could never be satisfied by a real strike.
    extension_rate_strike: tuple = (1.0, 3.0)

    reach_rate_shove: tuple = (1.0, 2.5)
    approach_shove: tuple = (0.3, 1.2)


def _ramp(value: float, lo_hi) -> float:
    lo, hi = lo_hi
    if hi <= lo:
        return 0.0
    return max(0.0, min(1.0, (value - lo) / (hi - lo)))


def _mean_point(frame: PoseFrame, names, min_conf: float):
    pts = [frame.joints[n] for n in names
           if n in frame.joints and frame.joints[n].conf >= min_conf]
    if not pts:
        return None
    n = float(len(pts))
    return (sum(p.x for p in pts) / n,
            sum(p.y for p in pts) / n,
            sum(p.z for p in pts) / n)


def _dist(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
                     + (a[2] - b[2]) ** 2)


def _torso_tilt_rad(frame: PoseFrame, min_conf: float) -> Optional[float]:
    """Angle between the hip-to-shoulder vector and vertical.

    Vertical is -Y because the optical frame has +Y down. Returns None when
    either end of the torso is missing, so a partial body contributes no lean
    rate rather than a fabricated one.
    """
    shoulders = _mean_point(frame, ("shoulder_l", "shoulder_r"), min_conf)
    hips = _mean_point(frame, ("hip_l", "hip_r"), min_conf)
    if shoulders is None or hips is None:
        return None
    vx = shoulders[0] - hips[0]
    vy = shoulders[1] - hips[1]
    vz = shoulders[2] - hips[2]
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm <= 1e-6:
        return None
    # Dot with up = (0, -1, 0), clamped for acos domain safety.
    return math.acos(max(-1.0, min(1.0, -vy / norm)))


class FeatureWindow:
    """A sliding window of frames for ONE subject index.

    The node keeps one of these per subject seen in a frame. The window is
    bounded by time and nothing survives it, so this holds no history a person
    could be tracked through.
    """

    def __init__(self, config: Optional[FeatureConfig] = None):
        self.config = config or FeatureConfig()
        self._frames: List[PoseFrame] = []

    def __len__(self) -> int:
        return len(self._frames)

    def push(self, frame: PoseFrame) -> None:
        self._frames.append(frame)
        self._frames.sort(key=lambda f: f.stamp_s)
        cutoff = self._frames[-1].stamp_s - self.config.window_s
        self._frames = [f for f in self._frames if f.stamp_s >= cutoff]

    def features(self) -> Features:
        frames = self._frames
        if len(frames) < 2:
            return Features(n_frames=len(frames))

        conf = self.config.min_joint_conf
        approach = 0.0
        body = 0.0
        lean = 0.0
        wrist_speed = 0.0
        extension = 0.0
        bilateral = 0.0

        for prev, cur in zip(frames, frames[1:]):
            dt = cur.stamp_s - prev.stamp_s
            # A non-advancing stamp is a repeated or reordered frame. Every
            # rate would be an infinity or a division error; contributing
            # nothing is the only honest answer, and the staleness check in
            # escalation_policy is what turns a stream of them into a fault.
            if dt <= 0.0:
                continue

            body_names = ("hip_l", "hip_r", "shoulder_l", "shoulder_r")
            p_c = _mean_point(prev, body_names, conf)
            c_c = _mean_point(cur, body_names, conf)
            if p_c is not None and c_c is not None:
                approach = max(approach, (p_c[2] - c_c[2]) / dt)
                body = max(body, _dist(p_c, c_c) / dt)

            p_t = _torso_tilt_rad(prev, conf)
            c_t = _torso_tilt_rad(cur, conf)
            if p_t is not None and c_t is not None:
                lean = max(lean, abs(c_t - p_t) / dt)

            reach_rates = []
            for side in ("l", "r"):
                wrist = "wrist_%s" % side
                shoulder = "shoulder_%s" % side
                if not all(j in prev.joints and j in cur.joints
                           for j in (wrist, shoulder)):
                    continue
                if min(prev.joints[wrist].conf, cur.joints[wrist].conf,
                       prev.joints[shoulder].conf,
                       cur.joints[shoulder].conf) < conf:
                    continue
                pw, cw = prev.joints[wrist], cur.joints[wrist]
                ps, cs = prev.joints[shoulder], cur.joints[shoulder]
                wrist_speed = max(wrist_speed,
                                  _dist((pw.x, pw.y, pw.z),
                                        (cw.x, cw.y, cw.z)) / dt)
                p_ext = _dist((pw.x, pw.y, pw.z), (ps.x, ps.y, ps.z))
                c_ext = _dist((cw.x, cw.y, cw.z), (cs.x, cs.y, cs.z))
                extension = max(extension, (c_ext - p_ext) / dt)
                # Forward reach is the wrist moving toward the camera relative
                # to its OWN shoulder, so a whole body walking forward does
                # not register as a shove.
                reach_rates.append(((cs.z - cw.z) - (ps.z - pw.z)) / dt)

            if len(reach_rates) == 2:
                # Minimum of the two sides: a shove drives BOTH arms. One arm
                # extending is a strike, and is scored as one.
                bilateral = max(bilateral, min(reach_rates))

        return Features(
            approach_speed_mps=approach,
            body_speed_mps=body,
            torso_lean_rate_radps=lean,
            wrist_speed_max_mps=wrist_speed,
            wrist_extension_rate_mps=extension,
            bilateral_reach_rate_mps=bilateral,
            span_s=frames[-1].stamp_s - frames[0].stamp_s,
            n_frames=len(frames),
        )


def score(features: Features,
          config: Optional[ScoreConfig] = None) -> Dict[Category, float]:
    """Per-category evidence score in [0, 1]. Conjunctive; see the docstring."""
    cfg = config or ScoreConfig()
    return {
        Category.LUNGE: min(
            _ramp(features.approach_speed_mps, cfg.approach_lunge),
            _ramp(features.body_speed_mps, cfg.body_speed_lunge),
            _ramp(features.torso_lean_rate_radps, cfg.lean_rate_lunge),
        ),
        Category.STRIKE: min(
            _ramp(features.wrist_speed_max_mps, cfg.wrist_speed_strike),
            _ramp(features.wrist_extension_rate_mps,
                  cfg.extension_rate_strike),
        ),
        Category.SHOVE: min(
            _ramp(features.bilateral_reach_rate_mps, cfg.reach_rate_shove),
            _ramp(features.approach_speed_mps, cfg.approach_shove),
        ),
    }


def top_category(scores: Dict[Category, float]):
    """(category, score) for the highest score. Ties break by name, stably."""
    if not scores:
        return None, 0.0
    ordered = sorted(scores, key=lambda c: c.value)
    cat = max(ordered, key=lambda c: scores[c])
    return cat, scores[cat]
