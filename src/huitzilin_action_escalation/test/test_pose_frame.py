"""The input contract, which is where the privacy rules are enforced.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_action_escalation/test/test_pose_frame.py
"""

import pytest

from huitzilin_action_escalation.pose_frame import (
    ALLOWED_JOINTS,
    PoseFrameRejected,
    SCHEMA_VERSION,
    forbidden_key_problems,
    joint_problems,
    parse_pose_frame,
)

# The five COCO-17 keypoints that make up the face. None of them may ever be
# accepted, under any spelling this project is likely to meet.
FACE_KEYPOINTS = ("nose", "left_eye", "right_eye", "left_ear", "right_ear",
                  "eye_l", "eye_r", "ear_l", "ear_r", "mouth")


def good_payload(**overrides):
    payload = {
        "schema": SCHEMA_VERSION,
        "stamp_s": 1.0,
        "frame_id": "camera_optical_frame",
        "source": "test",
        "subject": 0,
        "joints": {
            "shoulder_l": [-0.2, -0.5, 4.0, 0.9],
            "shoulder_r": [0.2, -0.5, 4.0, 0.9],
            "hip_l": [-0.15, 0.0, 4.0, 0.9],
            "hip_r": [0.15, 0.0, 4.0, 0.9],
        },
    }
    payload.update(overrides)
    return payload


def test_a_well_formed_frame_parses():
    frame = parse_pose_frame(good_payload())
    assert frame.stamp_s == 1.0
    assert frame.subject == 0
    assert frame.joints["shoulder_l"].conf == 0.9


def test_the_whitelist_excludes_every_face_keypoint():
    """The privacy contract, asserted directly rather than inferred.

    ALLOWED_JOINTS is COCO-17 minus keypoints 0-4. If a face point were ever
    added to the whitelist every other test here would still pass, because
    they all test the parser against the whitelist rather than the whitelist
    itself.
    """
    for name in FACE_KEYPOINTS:
        assert name not in ALLOWED_JOINTS, name
    assert len(ALLOWED_JOINTS) == 12
    assert not any("eye" in j or "ear" in j or "nose" in j
                   for j in ALLOWED_JOINTS)


@pytest.mark.parametrize("name", FACE_KEYPOINTS)
def test_a_frame_carrying_a_face_keypoint_is_rejected_whole(name):
    joints = good_payload()["joints"]
    joints[name] = [0.0, -0.7, 4.0, 0.9]
    with pytest.raises(PoseFrameRejected):
        parse_pose_frame(good_payload(joints=joints))


# (payload override, fragment expected in the rejection reason)
BAD_PAYLOADS = [
    ({"image": "aGVsbG8="}, "forbidden"),
    ({"crop": [1, 2, 3, 4]}, "forbidden"),
    ({"embedding": [0.1, 0.2]}, "forbidden"),
    ({"person_id": 7}, "forbidden"),
    ({"track_id": "abc"}, "forbidden"),
    ({"identity": "j"}, "forbidden"),
    ({"rgb": "..."}, "forbidden"),
    ({"age": 30}, "forbidden"),
    ({"unexpected": 1}, "unknown top-level"),
    ({"schema": 999}, "schema"),
    ({"stamp_s": "soon"}, "stamp_s"),
    ({"subject": "person-a"}, "subject"),
    ({"subject": True}, "subject"),
    ({"joints": {}}, "joints is missing or empty"),
    ({"frame_id": 5}, "frame_id"),
]


@pytest.mark.parametrize("override,fragment", BAD_PAYLOADS,
                         ids=lambda v: str(v)[:40])
def test_the_parser_rejects(override, fragment):
    with pytest.raises(PoseFrameRejected) as exc:
        parse_pose_frame(good_payload(**override))
    assert fragment in str(exc.value)


def test_a_string_subject_is_refused_because_it_would_be_an_identifier():
    """subject is an index within one frame, never a stable id."""
    with pytest.raises(PoseFrameRejected) as exc:
        parse_pose_frame(good_payload(subject="alice"))
    assert "identifier" in str(exc.value)


@pytest.mark.parametrize("value", [
    [0.1, 0.2, 3.0],
    [0.1, 0.2, 3.0, 0.5, 0.5],
    [0.1, 0.2, 3.0, "high"],
    "not-a-list",
])
def test_a_malformed_joint_value_is_rejected(value):
    with pytest.raises(PoseFrameRejected):
        parse_pose_frame(good_payload(joints={"wrist_l": value}))


@pytest.mark.parametrize("conf", [-0.1, 1.1])
def test_a_confidence_outside_the_unit_interval_is_rejected(conf):
    with pytest.raises(PoseFrameRejected):
        parse_pose_frame(good_payload(joints={"wrist_l": [0.0, 0.0, 1.0,
                                                          conf]}))


def test_a_non_object_payload_is_rejected():
    with pytest.raises(PoseFrameRejected):
        parse_pose_frame([1, 2, 3])


def test_the_forbidden_key_predicate_is_not_vacuous():
    """Guards the predicate the privacy test also relies on.

    A predicate that returned an empty list for everything would make both
    this module and test_privacy_invariants pass while checking nothing.
    """
    assert forbidden_key_problems(["face_landmarks"]) != []
    assert forbidden_key_problems(["person_id"]) != []
    assert forbidden_key_problems(["shoulder_l", "hip_r"]) == []


def test_the_joint_predicate_is_not_vacuous():
    assert joint_problems(["nose"]) == ["nose"]
    assert joint_problems(["shoulder_l"]) == []
