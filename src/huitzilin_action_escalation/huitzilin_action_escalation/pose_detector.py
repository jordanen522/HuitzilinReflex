"""Decoding a single-stage pose model into body joints, with faces discarded.

This is the real detection stage: a YOLOv8-pose ONNX graph outputs 17 COCO
keypoints per person, and **five of them are the face** (nose, both eyes, both
ears, indices 0-4). Those five are dropped HERE, in this module, before any
value reaches a PoseFrame, a topic, a log or a window. The model sees a face
and this code refuses to carry it forward. That is a stronger guarantee than a
model that never computed one, because it holds whichever pose model is
swapped in later: the drop is on our side of the boundary.

Nothing here imports onnxruntime. The decode is pure numpy over a tensor the
caller supplies, so every branch of it is testable with no model file, no
camera and no ROS. pose_detector_node.py owns the session.

RANGE. A monocular camera has no depth, and this subsystem needs metric range:
LUNGE and SHOVE both require closing speed, and a pipeline reporting a constant
Z would score every approach as zero. Two modes:

  depth      - sample an aligned depth image at the joint pixel. Correct, and
               the path an OAK-D would use. Unverified: no camera exists yet.
  monocular  - estimate range from bounding-box height against an assumed
               standing subject height. Works today with any camera and gives a
               real approach signal, because a closing person grows in frame.
               It assumes a standing adult seen full length, and degrades
               exactly when that is false: a crouching, seated, partially
               occluded or unusually tall subject gets a proportionally wrong
               range and therefore a wrong closing speed. It is a scale
               estimate, not a measurement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# COCO-17 in model output order. The first five are the face and are never
# emitted. The tests assert FACE_KEYPOINT_COUNT against this tuple so the
# constant cannot drift away from the slice that relies on it.
COCO_KEYPOINTS = (
    "nose", "eye_l", "eye_r", "ear_l", "ear_r",
    "shoulder_l", "shoulder_r", "elbow_l", "elbow_r",
    "wrist_l", "wrist_r", "hip_l", "hip_r",
    "knee_l", "knee_r", "ankle_l", "ankle_r",
)
FACE_KEYPOINT_COUNT = 5
BODY_KEYPOINTS = COCO_KEYPOINTS[FACE_KEYPOINT_COUNT:]


@dataclass(frozen=True)
class DetectorConfig:
    # Person-detection confidence. Below it the frame yields nothing, which is
    # the safe direction: a missed person cannot raise an alarm.
    person_conf: float = 0.50
    # Per-joint confidence. Joints below it are omitted rather than emitted
    # weakly, because action_features treats a missing joint as absent while a
    # hallucinated one moves fast.
    joint_conf: float = 0.50
    input_size: int = 640
    # Assumed standing height of a subject. Monocular range only.
    subject_height_m: float = 1.70
    # Horizontal field of view, used to derive focal length when no CameraInfo
    # is available. 69 degrees is the OAK-D Lite colour sensor.
    fov_h_deg: float = 69.0
    range_mode: str = "monocular"
    # Outside this the height assumption has clearly failed, and a wrong range
    # is worse than no detection.
    min_range_m: float = 0.8
    max_range_m: float = 25.0


def focal_px(image_width: int, fov_h_deg: float) -> float:
    """Pinhole focal length in pixels from a horizontal field of view."""
    return (image_width / 2.0) / math.tan(math.radians(fov_h_deg) / 2.0)


def decode(raw, image_wh: Tuple[int, int],
           config: Optional[DetectorConfig] = None):
    """Decode a [1, 56, N] or [56, N] pose tensor to the best single person.

    Returns (joints_px, bbox) where joints_px maps a body joint name to
    (x_px, y_px, conf), or (None, None) when nothing clears person_conf.

    Only the highest-confidence person is decoded. This subsystem scores one
    subject at a time and has no cross-frame association, so carrying more than
    one would mean inventing an identity to keep them apart, which is the thing
    it must not do.
    """
    cfg = config or DetectorConfig()
    arr = np.asarray(raw)
    if arr.ndim == 3:
        arr = arr[0]
    if arr.shape[0] == 56:
        arr = arr.T
    if arr.ndim != 2 or arr.shape[1] != 56:
        raise ValueError("expected a [.., 56, N] pose tensor, got %r"
                         % (np.asarray(raw).shape,))

    conf = arr[:, 4]
    best = int(np.argmax(conf))
    if float(conf[best]) < cfg.person_conf:
        return None, None

    width, height = image_wh
    sx = width / float(cfg.input_size)
    sy = height / float(cfg.input_size)

    cx, cy, bw, bh = (float(v) for v in arr[best, :4])
    bbox = (cx * sx, cy * sy, bw * sx, bh * sy)

    kp = arr[best, 5:].reshape(17, 3)
    joints = {}
    # This slice is the privacy boundary. Indices below FACE_KEYPOINT_COUNT are
    # never read, so no face coordinate is ever written into the result.
    for offset, name in enumerate(BODY_KEYPOINTS):
        x, y, c = kp[FACE_KEYPOINT_COUNT + offset]
        if float(c) < cfg.joint_conf:
            continue
        joints[name] = (float(x) * sx, float(y) * sy, float(c))
    return joints, bbox


def range_from_bbox_height(bbox_h_px: float, focal: float,
                           config: Optional[DetectorConfig] = None):
    """Monocular range from apparent height. None when it is not believable.

    Similar triangles: a subject of known height H at range Z projects to
    f*H/Z pixels. Returning None outside the clamp is deliberate. A person who
    is crouching, seated or cut off by the frame edge produces a short box and
    therefore an absurdly large range, and feeding that to the tracker would
    manufacture a closing speed out of a posture change.
    """
    cfg = config or DetectorConfig()
    if bbox_h_px <= 1.0 or focal <= 0.0:
        return None
    z = focal * cfg.subject_height_m / bbox_h_px
    if not (cfg.min_range_m <= z <= cfg.max_range_m):
        return None
    return z


def sample_depth(depth_m, x_px: float, y_px: float, patch: int = 2):
    """Median depth in a small patch around a joint. None if nothing valid.

    A single pixel on a limb edge lands on the background as often as on the
    person, so a median over a patch is the minimum defensible read. Zeros and
    non-finite values are the usual stereo no-data markers and are excluded
    rather than averaged in, which would drag every edge joint toward zero.
    """
    arr = np.asarray(depth_m)
    h, w = arr.shape[:2]
    xi, yi = int(round(x_px)), int(round(y_px))
    if not (0 <= xi < w and 0 <= yi < h):
        return None
    x0, x1 = max(0, xi - patch), min(w, xi + patch + 1)
    y0, y1 = max(0, yi - patch), min(h, yi + patch + 1)
    window = arr[y0:y1, x0:x1].astype(float).ravel()
    valid = window[np.isfinite(window) & (window > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def to_metric(joints_px, bbox, image_wh: Tuple[int, int],
              config: Optional[DetectorConfig] = None,
              depth_m=None, focal: Optional[float] = None):
    """Back-project pixel joints into the camera optical frame, in metres.

    Output convention matches pose_frame.PoseFrame: +Z is depth away from the
    camera, +Y is down, origin at the principal point, which is assumed to be
    the image centre when no CameraInfo is supplied.

    Returns {} when no range can be established. An empty frame is correct
    here: without range there is no closing speed, and publishing joints with
    a fabricated Z would let the recogniser score an approach that was never
    measured.
    """
    cfg = config or DetectorConfig()
    if not joints_px:
        return {}

    width, height = image_wh
    f = focal if focal is not None else focal_px(width, cfg.fov_h_deg)
    cx, cy = width / 2.0, height / 2.0

    subject_z = None
    if cfg.range_mode != "depth" and bbox is not None:
        subject_z = range_from_bbox_height(bbox[3], f, cfg)
        if subject_z is None:
            return {}

    out = {}
    for name, (x_px, y_px, conf) in joints_px.items():
        if cfg.range_mode == "depth":
            if depth_m is None:
                return {}
            z = sample_depth(depth_m, x_px, y_px)
            if z is None or not (cfg.min_range_m <= z <= cfg.max_range_m):
                continue
        else:
            # One range for the whole body. A monocular estimate has no
            # per-joint depth to give, and pretending otherwise would invent
            # limb extension along Z that the camera cannot see.
            z = subject_z
        out[name] = (
            (x_px - cx) * z / f,
            (y_px - cy) * z / f,
            z,
            float(conf),
        )
    return out


def frame_payload(joints_metric, stamp_s: float, frame_id: str,
                  source: str = "pose_detector", subject: int = 0):
    """Build the /action/keypoints payload. Keys are the whitelist by
    construction, since joints_metric only ever holds BODY_KEYPOINTS."""
    from huitzilin_action_escalation.pose_frame import SCHEMA_VERSION
    return {
        "schema": SCHEMA_VERSION,
        "stamp_s": float(stamp_s),
        "frame_id": frame_id,
        "source": source,
        "subject": int(subject),
        "joints": {name: [round(v, 5) for v in value]
                   for name, value in joints_metric.items()},
    }
