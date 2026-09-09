"""Synthetic keypoint sequences, so the chain is exercisable with no camera.

A scenario is a base pose plus a few keyframes of offsets, interpolated up to
rate_hz at load time. Keyframes rather than literal frames because a literal
sequence is twelve joints times four numbers times every frame -- unreadable,
unreviewable, and impossible to adjust by hand. The offsets are the part a
human can actually check.

WHAT THESE ARE NOT. They are authored by the same person who set the
thresholds they are checked against, so a scenario passing is not a recall
figure and not a false-positive rate. This repo already records the same trap
for the projectile bag library, which reached 100% recall and consequently
cannot referee a threshold change. The only non-vacuous content a hand-built
suite carries is its NEGATIVE controls: the benign scenarios that must not
confirm. Those are the ones worth adding to.

`expect` is therefore part of the data, and both values are used:
  confirm     -- the policy must reach an alert on this sequence
  no_confirm  -- the policy must NOT alert on this sequence
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from huitzilin_action_escalation.pose_frame import (
    ALLOWED_JOINTS,
    SCHEMA_VERSION,
)

ALLOWED_TOP_LEVEL = frozenset({
    "name", "description", "expect", "rate_hz", "base", "keyframes",
})
ALLOWED_KEYFRAME_KEYS = frozenset({"t_s", "all", "offsets"})
EXPECT_VALUES = frozenset({"confirm", "no_confirm"})


class ScenarioRejected(ValueError):
    """A scenario file that does not meet the contract."""


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    expect: str
    rate_hz: float
    frames: List[dict]

    @property
    def duration_s(self) -> float:
        if not self.frames:
            return 0.0
        return self.frames[-1]["stamp_s"] - self.frames[0]["stamp_s"]


def _quad(value, label):
    if (not isinstance(value, (list, tuple)) or len(value) != 4
            or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                   for v in value)):
        raise ScenarioRejected("%s is %r, expected four numbers" % (label,
                                                                    value))
    return [float(v) for v in value]


def load_scenario(doc) -> Scenario:
    """Validate a decoded scenario document and expand it into frames."""
    if not isinstance(doc, dict):
        raise ScenarioRejected("scenario is %s, not a mapping"
                               % type(doc).__name__)

    unknown = sorted(set(doc) - ALLOWED_TOP_LEVEL)
    if unknown:
        raise ScenarioRejected("unknown key(s): %s" % ", ".join(unknown))

    name = doc.get("name")
    if not isinstance(name, str) or not name:
        raise ScenarioRejected("name is missing")

    expect = doc.get("expect")
    if expect not in EXPECT_VALUES:
        raise ScenarioRejected(
            "expect is %r, must be one of %s"
            % (expect, ", ".join(sorted(EXPECT_VALUES))))

    rate_hz = doc.get("rate_hz")
    if not isinstance(rate_hz, (int, float)) or rate_hz <= 0:
        raise ScenarioRejected("rate_hz is %r, expected a positive number"
                               % (rate_hz,))

    base = doc.get("base")
    if not isinstance(base, dict) or not base:
        raise ScenarioRejected("base pose is missing")
    outside = sorted(j for j in base if j not in ALLOWED_JOINTS)
    if outside:
        raise ScenarioRejected("base pose has joint(s) outside the whitelist: "
                               "%s" % ", ".join(outside))
    base_pose = {j: _quad(v, "base.%s" % j) for j, v in base.items()}

    keyframes = doc.get("keyframes")
    if not isinstance(keyframes, list) or len(keyframes) < 2:
        raise ScenarioRejected("keyframes must be a list of at least two")

    parsed = []
    for i, kf in enumerate(keyframes):
        if not isinstance(kf, dict):
            raise ScenarioRejected("keyframe %d is not a mapping" % i)
        unknown = sorted(set(kf) - ALLOWED_KEYFRAME_KEYS)
        if unknown:
            raise ScenarioRejected("keyframe %d unknown key(s): %s"
                                   % (i, ", ".join(unknown)))
        t_s = kf.get("t_s")
        if not isinstance(t_s, (int, float)) or isinstance(t_s, bool):
            raise ScenarioRejected("keyframe %d t_s is %r" % (i, t_s))
        every = _quad(kf.get("all", [0.0, 0.0, 0.0, 0.0]),
                      "keyframe %d all" % i)
        offsets = kf.get("offsets") or {}
        if not isinstance(offsets, dict):
            raise ScenarioRejected("keyframe %d offsets is not a mapping" % i)
        outside = sorted(j for j in offsets if j not in ALLOWED_JOINTS)
        if outside:
            raise ScenarioRejected(
                "keyframe %d offsets outside the whitelist: %s"
                % (i, ", ".join(outside)))
        parsed.append((float(t_s), every,
                       {j: _quad(v, "keyframe %d %s" % (i, j))
                        for j, v in offsets.items()}))

    parsed.sort(key=lambda k: k[0])
    if parsed[0][0] != 0.0:
        raise ScenarioRejected("the first keyframe must be at t_s 0.0")

    frames = _expand(base_pose, parsed, float(rate_hz), name)
    return Scenario(name=name,
                    description=str(doc.get("description", "")),
                    expect=expect, rate_hz=float(rate_hz), frames=frames)


def _offset_at(parsed, t: float, joint: str) -> List[float]:
    """Linear interpolation of a joint offset at time t."""
    prev = parsed[0]
    for kf in parsed:
        if kf[0] <= t:
            prev = kf
        else:
            nxt = kf
            span = nxt[0] - prev[0]
            frac = 0.0 if span <= 0 else (t - prev[0]) / span
            a = _total(prev, joint)
            b = _total(nxt, joint)
            return [a[i] + (b[i] - a[i]) * frac for i in range(4)]
    return _total(prev, joint)


def _total(keyframe, joint: str) -> List[float]:
    _, every, offsets = keyframe
    own = offsets.get(joint, [0.0, 0.0, 0.0, 0.0])
    return [every[i] + own[i] for i in range(4)]


def _expand(base_pose: Dict[str, List[float]], parsed, rate_hz: float,
            source: str) -> List[dict]:
    end_t = parsed[-1][0]
    step = 1.0 / rate_hz
    frames = []
    t = 0.0
    # Inclusive of the final keyframe: a strike that peaks on the last frame
    # would otherwise be cut off by floating-point drift on the loop bound.
    while t <= end_t + 1e-9:
        joints = {}
        for joint, base in base_pose.items():
            off = _offset_at(parsed, t, joint)
            conf = max(0.0, min(1.0, base[3] + off[3]))
            joints[joint] = [base[0] + off[0], base[1] + off[1],
                             base[2] + off[2], conf]
        frames.append({
            "schema": SCHEMA_VERSION,
            "stamp_s": round(t, 6),
            "frame_id": "camera_optical_frame",
            "source": source,
            "subject": 0,
            "joints": joints,
        })
        t += step
    return frames
