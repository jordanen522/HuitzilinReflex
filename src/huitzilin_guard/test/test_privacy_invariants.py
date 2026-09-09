"""The privacy properties, asserted against the code rather than a document.

Runs without ROS (yaml only):
    python -m pytest src/huitzilin_guard/test/test_privacy_invariants.py

Like test_isolation, the source checks look at string constants and imports
rather than raw text: the modules discuss faces and identity at length in
their docstrings, explaining what is excluded and why, and a test that banned
the words would delete the reasoning while permitting the capability.
"""

import ast
import pathlib

import pytest
import yaml

from huitzilin_guard.pose_frame import (
    ALLOWED_JOINTS,
    forbidden_key_problems,
    joint_problems,
)

PKG = pathlib.Path(__file__).resolve().parents[1]
PY_FILES = sorted((PKG / "huitzilin_guard").glob("*.py"))
PARAMS_FILES = sorted((PKG / "params").glob("*.yaml"))

# Libraries whose presence would mean this subsystem had grown a face or
# identity capability. Forbidden in EVERY file, the detector included.
FORBIDDEN_LIBS = (
    "mediapipe", "dlib", "face_recognition", "insightface",
    "facenet", "deepface", "retinaface", "arcface", "torch",
    "torchvision", "tensorflow", "sklearn", "PIL",
)

# cv2 and onnxruntime are permitted in ONE file: the detector is the only place
# that legitimately holds an image and runs the model. Anywhere else they would
# mean an image path had appeared where none belongs. Narrowing the rule to a
# single file is the point; blanket-allowing them across the package would give
# up the property entirely.
IMAGE_STAGE_FILE = "pose_detector_node.py"
IMAGE_STAGE_LIBS = ("cv2", "onnxruntime")

FACE_KEYPOINTS = ("nose", "left_eye", "right_eye", "left_ear", "right_ear")


def imported_roots(source):
    roots = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def test_the_file_scan_is_not_empty():
    """Guards every check below."""
    assert len(PY_FILES) >= 6, [p.name for p in PY_FILES]
    assert len(PARAMS_FILES) >= 3, [p.name for p in PARAMS_FILES]


def test_the_joint_whitelist_excludes_every_face_keypoint():
    """COCO-17 minus keypoints 0-4. This is the privacy contract: no face
    point is ever accepted, so no face can be reconstructed from anything
    this subsystem holds."""
    for name in FACE_KEYPOINTS:
        assert name not in ALLOWED_JOINTS
    assert len(ALLOWED_JOINTS) == 12


def test_the_privacy_predicates_are_not_vacuous():
    """Positive controls. Predicates that accepted everything would make this
    whole module green while enforcing nothing."""
    assert joint_problems(["nose"]) != []
    assert forbidden_key_problems(["face_landmark"]) != []
    assert forbidden_key_problems(["identity"]) != []
    assert forbidden_key_problems(["embedding"]) != []
    assert forbidden_key_problems(["image"]) != []
    assert forbidden_key_problems(["shoulder_l"]) == []


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_no_module_imports_a_vision_or_face_library(path):
    roots = imported_roots(path.read_text(encoding="utf-8"))
    banned = FORBIDDEN_LIBS
    if path.name != IMAGE_STAGE_FILE:
        banned = banned + IMAGE_STAGE_LIBS
    for lib in banned:
        assert lib not in roots, (
            "%s imports %s. Only %s may hold an image or run the model; a "
            "vision, model or face library anywhere else means an image path "
            "has appeared where none belongs."
            % (path.name, lib, IMAGE_STAGE_FILE))


def test_the_image_stage_is_the_only_file_that_touches_an_image():
    """Guards the exemption above. If a second file started importing cv2 the
    per-file rule would quietly become a package-wide allowance."""
    holders = [p.name for p in PY_FILES
               if imported_roots(p.read_text(encoding="utf-8"))
               & set(IMAGE_STAGE_LIBS)]
    assert holders == [IMAGE_STAGE_FILE], holders


@pytest.mark.parametrize("path", PARAMS_FILES, ids=lambda p: p.name)
def test_no_params_file_enables_recording(path):
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for node, body in doc.items():
        prms = (body or {}).get("ros__parameters") or {}
        for key, value in prms.items():
            low = key.lower()
            # *_topic names a stream to read, not storage to enable. The
            # detector legitimately subscribes to /camera/image_raw; what must
            # stay off is anything that CAPTURES or KEEPS what it sees.
            if low.endswith("_topic"):
                continue
            if any(word in low for word in ("record", "video", "image",
                                            "crop", "dump", "save")):
                assert value in (False, "", None), (
                    "%s:%s sets %s=%r" % (path.name, node, key, value))


def test_recording_is_off_by_default_and_declared_explicitly():
    """record_video exists as a key so the default is visible in config, not
    only asserted in prose. It is also reported in the guard's status
    heartbeat, so the property is observable at runtime."""
    doc = yaml.safe_load(
        (PKG / "params" / "pose_detector.yaml").read_text(encoding="utf-8"))
    prms = doc["pose_detector"]["ros__parameters"]
    assert prms["record_video"] is False


def test_the_published_status_carries_no_identifying_field():
    """The status says what was DECIDED, never what was seen.

    Asserted against the node source so a field added later is caught: the
    status dict is built in one place, and any key resembling an identifier,
    an image reference or a joint position must not appear in it.

    This is the whole privacy posture of the guard in one assertion. The
    alarm's output is "a person is inside the box" and nothing else: no
    identity, no description, not even the position of the person or the
    subject index the parser accepted. An operator learns that the area is
    occupied, which is all a presence alarm is entitled to tell them.
    """
    source = (PKG / "huitzilin_guard"
              / "guard_node.py").read_text(encoding="utf-8")
    status_keys = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Dict):
            continue
        keys = [k.value for k in node.keys
                if isinstance(k, ast.Constant) and isinstance(k.value, str)]
        if "armed" in keys and "person_in_box" in keys:
            status_keys = keys
    assert status_keys, "could not locate the status payload in the node"
    assert forbidden_key_problems(status_keys) == []
    for banned in ("subject", "joints", "bbox", "keypoints", "position"):
        assert banned not in status_keys


def test_the_status_heartbeat_reports_the_recording_state():
    """Observable from topic echo, rather than only promised in a document.
    frames_rejected is the privacy telemetry: non-zero means something
    upstream is sending fields this subsystem refuses."""
    source = (PKG / "huitzilin_guard"
              / "guard_node.py").read_text(encoding="utf-8")
    assert '"recording"' in source
    assert '"frames_rejected"' in source
