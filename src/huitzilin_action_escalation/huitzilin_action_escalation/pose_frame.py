"""The input contract, and the point where the privacy rules are enforced.

This subsystem consumes body-joint positions and nothing else. The privacy
guarantee is not a policy written in a document that code is trusted to
honour; it is this parser, which rejects anything it was not promised.

ALLOWED_JOINTS is COCO-17 minus keypoints 0-4 -- nose, both eyes, both ears.
Those five are the face, so they are not in the whitelist and a frame carrying
any of them is discarded whole. Twelve joints remain, all of them limb and
torso landmarks from which no face can be reconstructed.

Parsing is STRICT, not lenient. An unknown top-level key, an unknown joint
name, or a `subject` that is anything but an integer rejects the entire frame.
A lenient parser that ignored fields it did not recognise is exactly how an
`image`, a `crop` or an `embedding` field arrives and is then quietly carried
along by every downstream consumer that copies the dict.

`subject` is an index within ONE frame, used only to keep two bodies' joints
apart while scoring. It is not a track id and is not stable between frames. A
string subject is rejected because a stable identifier is the shape identity
smuggles itself in as.

In:  a JSON object, as published on /action/keypoints
Out: PoseFrame, or PoseFrameRejected
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, NamedTuple

SCHEMA_VERSION = 1

# COCO-17 keypoints 5..16. Keypoints 0-4 are nose, left eye, right eye, left
# ear, right ear, and are absent on purpose -- see the module docstring.
ALLOWED_JOINTS = frozenset({
    "shoulder_l", "shoulder_r",
    "elbow_l", "elbow_r",
    "wrist_l", "wrist_r",
    "hip_l", "hip_r",
    "knee_l", "knee_r",
    "ankle_l", "ankle_r",
})

ALLOWED_TOP_LEVEL = frozenset({
    "schema", "stamp_s", "frame_id", "source", "subject", "joints",
})

# Substrings that must never appear in a payload key. This is belt-and-braces
# next to the whitelist above -- the whitelist already rejects them -- but it
# produces a reason naming WHAT was refused, which is what makes an upstream
# misconfiguration diagnosable from /action/status instead of just absent.
FORBIDDEN_KEY_SUBSTRINGS = (
    "face", "eye", "ear", "nose", "mouth", "iris", "landmark",
    "identity", "ident", "person_id", "track_id", "reid", "re_id",
    "embed", "descriptor", "signature", "biometric",
    "image", "img", "crop", "thumbnail", "frame_data", "rgb", "video",
    "name", "gender", "age", "race", "ethnic",
)


class PoseFrameRejected(ValueError):
    """A frame that does not meet the input contract. Never partially used."""


class Joint(NamedTuple):
    x: float
    y: float
    z: float
    conf: float


@dataclass(frozen=True)
class PoseFrame:
    """One instant of one body, in the camera optical frame.

    +Z is depth away from the camera, so range to a body is its centroid z and
    something approaching the aircraft has a decreasing z. Metres throughout.
    """

    stamp_s: float
    subject: int
    joints: Dict[str, Joint]
    frame_id: str = ""
    source: str = ""


def forbidden_key_problems(keys: Iterable[str]) -> List[str]:
    """Keys whose name contains a forbidden substring, with the reason.

    Exported so the privacy test can assert against the predicate the parser
    actually uses, rather than reimplementing it and drifting.
    """
    problems = []
    for key in keys:
        low = str(key).lower()
        for bad in FORBIDDEN_KEY_SUBSTRINGS:
            if bad in low:
                problems.append("%s (contains %r)" % (key, bad))
                break
    return problems


def joint_problems(names: Iterable[str]) -> List[str]:
    """Joint names outside the whitelist. Empty means the set is acceptable."""
    return sorted(str(n) for n in names if str(n) not in ALLOWED_JOINTS)


def parse_pose_frame(payload) -> PoseFrame:
    """Validate a decoded JSON object into a PoseFrame, or raise.

    Every rejection raises rather than returning a partial frame. A frame that
    is 90% acceptable is still a frame carrying something we promised not to
    receive, and scoring the acceptable 90% of it would mean the promise held
    only for the fields nobody looked at.
    """
    if not isinstance(payload, dict):
        raise PoseFrameRejected("payload is %s, not an object"
                                % type(payload).__name__)

    bad = forbidden_key_problems(payload.keys())
    if bad:
        raise PoseFrameRejected("forbidden top-level key(s): %s"
                                % ", ".join(bad))

    unknown = sorted(set(payload) - ALLOWED_TOP_LEVEL)
    if unknown:
        raise PoseFrameRejected("unknown top-level key(s): %s"
                                % ", ".join(unknown))

    if payload.get("schema") != SCHEMA_VERSION:
        raise PoseFrameRejected("schema is %r, expected %d"
                                % (payload.get("schema"), SCHEMA_VERSION))

    stamp = payload.get("stamp_s")
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
        raise PoseFrameRejected("stamp_s is %r, expected a number" % (stamp,))

    subject = payload.get("subject", 0)
    # bool is an int subclass; a True subject is a bug, not subject 1.
    if not isinstance(subject, int) or isinstance(subject, bool):
        raise PoseFrameRejected(
            "subject is %r, expected an integer index within this frame. A "
            "string subject would be a stable identifier, which this "
            "subsystem does not accept." % (subject,))

    raw_joints = payload.get("joints")
    if not isinstance(raw_joints, dict) or not raw_joints:
        raise PoseFrameRejected("joints is missing or empty")

    bad = forbidden_key_problems(raw_joints.keys())
    if bad:
        raise PoseFrameRejected("forbidden joint name(s): %s" % ", ".join(bad))

    outside = joint_problems(raw_joints.keys())
    if outside:
        raise PoseFrameRejected("joint(s) outside the whitelist: %s"
                                % ", ".join(outside))

    joints = {}
    for name, value in raw_joints.items():
        if (not isinstance(value, (list, tuple)) or len(value) != 4
                or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                       for v in value)):
            raise PoseFrameRejected(
                "joint %s is %r, expected [x, y, z, conf]" % (name, value))
        if not 0.0 <= float(value[3]) <= 1.0:
            raise PoseFrameRejected(
                "joint %s confidence %r is outside [0, 1]" % (name, value[3]))
        joints[name] = Joint(float(value[0]), float(value[1]),
                             float(value[2]), float(value[3]))

    frame_id = payload.get("frame_id", "")
    source = payload.get("source", "")
    for label, text in (("frame_id", frame_id), ("source", source)):
        if not isinstance(text, str):
            raise PoseFrameRejected("%s is %r, expected a string"
                                    % (label, text))

    return PoseFrame(stamp_s=float(stamp), subject=int(subject),
                     joints=joints, frame_id=frame_id, source=source)
