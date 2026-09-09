"""Decoding a pose tensor into body joints, and the face drop that guards it.

Runs without ROS, without onnxruntime and without the model file: every test
here feeds a synthetic tensor, so the decode is checked on any machine.

    python -m pytest src/huitzilin_guard/test/test_pose_detector.py
"""

import math

import numpy as np
import pytest

from huitzilin_guard import pose_detector as pd
from huitzilin_guard.pose_frame import (
    ALLOWED_JOINTS,
    parse_pose_frame,
)

FACE_NAMES = ("nose", "eye_l", "eye_r", "ear_l", "ear_r")


def tensor(person_conf=0.9, joint_conf=0.9, n=20, box=(320.0, 320.0, 100.0, 400.0)):
    """A [1, 56, n] tensor whose best anchor is a full, confident body.

    Face keypoints are given DISTINCTIVE coordinates so a test can prove they
    were dropped rather than merely absent by coincidence.
    """
    arr = np.zeros((n, 56), dtype=np.float32)
    arr[:, 4] = 0.01
    best = n // 2
    arr[best, :4] = box
    arr[best, 4] = person_conf
    kp = np.zeros((17, 3), dtype=np.float32)
    for i in range(17):
        kp[i] = (7777.0, 8888.0, joint_conf) if i < 5 else (100.0 + i, 200.0 + i,
                                                            joint_conf)
    arr[best, 5:] = kp.reshape(-1)
    return arr.T[None]


def test_the_body_joint_set_is_exactly_the_privacy_whitelist():
    """The load-bearing cross-check. If the model's body keypoints and the
    parser's whitelist ever diverge, every published frame is rejected at the
    boundary and the pipeline goes silent for a reason nobody would guess."""
    assert set(pd.BODY_KEYPOINTS) == set(ALLOWED_JOINTS)
    assert len(pd.BODY_KEYPOINTS) == 12


def test_the_face_keypoint_count_matches_the_named_face_keypoints():
    """Guards the slice. FACE_KEYPOINT_COUNT is used to skip the face; if it
    drifted from the actual face names the slice would leak or over-trim."""
    assert pd.COCO_KEYPOINTS[:pd.FACE_KEYPOINT_COUNT] == FACE_NAMES
    assert pd.FACE_KEYPOINT_COUNT == 5
    assert len(pd.COCO_KEYPOINTS) == 17


def test_no_face_keypoint_survives_decoding():
    """The face coordinates are present in the tensor and must not appear in
    the output, by name or by value."""
    joints, _ = pd.decode(tensor(), (640, 640))
    for name in FACE_NAMES:
        assert name not in joints
    for value in joints.values():
        assert 7777.0 not in value and 8888.0 not in value


def test_a_low_confidence_person_yields_nothing():
    """A missed person cannot raise an alarm, which is the safe direction."""
    joints, bbox = pd.decode(tensor(person_conf=0.1), (640, 640))
    assert joints is None and bbox is None


def test_low_confidence_joints_are_omitted_not_emitted_weakly():
    joints, _ = pd.decode(tensor(joint_conf=0.1), (640, 640))
    assert joints == {}


@pytest.mark.parametrize("bad", [
    np.zeros((10, 10)),
    np.zeros((1, 40, 20)),
    np.zeros((5,)),
])
def test_a_wrongly_shaped_tensor_raises_rather_than_guessing(bad):
    with pytest.raises(ValueError):
        pd.decode(bad, (640, 640))


def test_pixel_coordinates_are_scaled_back_to_the_source_image():
    joints, bbox = pd.decode(tensor(), (1280, 640))
    assert bbox[0] == pytest.approx(640.0)      # 320 * (1280/640)
    assert joints["shoulder_l"][0] == pytest.approx((100.0 + 5) * 2.0)


def test_monocular_range_follows_similar_triangles():
    f = pd.focal_px(640, 69.0)
    cfg = pd.DetectorConfig()
    z = pd.range_from_bbox_height(400.0, f, cfg)
    assert z == pytest.approx(f * cfg.subject_height_m / 400.0)
    # A closer subject fills more of the frame and must read as nearer.
    assert pd.range_from_bbox_height(800.0, f, cfg) < z


@pytest.mark.parametrize("bbox_h", [0.0, 1.0, 5.0, 100000.0])
def test_an_unbelievable_box_height_yields_no_range(bbox_h):
    """A crouching or half-framed subject produces a short box and an absurd
    range. Returning None stops that becoming a fabricated closing speed."""
    assert pd.range_from_bbox_height(bbox_h, pd.focal_px(640, 69.0)) is None


def test_focal_length_grows_as_the_field_of_view_narrows():
    assert pd.focal_px(640, 40.0) > pd.focal_px(640, 90.0)
    assert pd.focal_px(640, 90.0) == pytest.approx(320.0 / math.tan(math.pi / 4))


def test_depth_sampling_ignores_the_no_data_markers():
    """Zeros and non-finite values are stereo no-data. Averaging them in would
    drag every joint on a limb edge toward the camera."""
    depth = np.array([[0.0, 0.0, 0.0],
                      [0.0, 4.0, np.nan],
                      [0.0, 4.2, 4.1]], dtype=np.float32)
    assert pd.sample_depth(depth, 1, 1, patch=1) == pytest.approx(4.1)
    assert pd.sample_depth(np.zeros((3, 3)), 1, 1) is None
    assert pd.sample_depth(depth, 99, 99) is None


def test_metric_output_is_empty_when_no_range_can_be_established():
    joints, _ = pd.decode(tensor(), (640, 640))
    assert pd.to_metric(joints, (320.0, 320.0, 100.0, 1.0), (640, 640)) == {}


def test_depth_mode_without_a_depth_image_publishes_nothing():
    cfg = pd.DetectorConfig(range_mode="depth")
    joints, bbox = pd.decode(tensor(), (640, 640))
    assert pd.to_metric(joints, bbox, (640, 640), cfg, depth_m=None) == {}


def test_metric_joints_round_trip_through_the_strict_parser():
    """The detector's own output must satisfy the privacy parser. If it did
    not, every frame would be rejected at the boundary at runtime."""
    joints, bbox = pd.decode(tensor(), (640, 640))
    metric = pd.to_metric(joints, bbox, (640, 640))
    assert metric
    payload = pd.frame_payload(metric, 1.5, "camera_optical_frame")
    frame = parse_pose_frame(payload)
    assert frame.stamp_s == 1.5
    assert set(frame.joints) <= ALLOWED_JOINTS
    for joint in frame.joints.values():
        assert joint.z > 0.0


def test_the_payload_carries_no_identifier():
    joints, bbox = pd.decode(tensor(), (640, 640))
    payload = pd.frame_payload(
        pd.to_metric(joints, bbox, (640, 640)), 1.0, "camera_optical_frame")
    assert payload["subject"] == 0
    assert isinstance(payload["subject"], int)
    for banned in ("image", "crop", "embedding", "person_id", "track_id"):
        assert banned not in payload
