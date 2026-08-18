"""
Run-loop tests for score_bags.ScorerNode — Task 5c fix round 3, MEDIUM-1.

WHAT THIS FILE EXISTS FOR
-------------------------
Fix round 2 made strict_window_bounds() raise on an inverted detection
window. That raise is correct and must stay: an inverted window admits
nothing, so scoring under it posts a silent false negative. But the raise
reached run()'s scenario loop UNGUARDED, so one broken sidecar aborted the
whole run — _report() was never reached, no artifact was written even for
the bags already scored, and every later bag went unreplayed.

test_score_bags_logic.py cannot see that: it tests the pure library, and the
library's job is to raise. The defect is in the CALLER's blast radius, which
lives in score_bags.py. So these tests drive the real ScorerNode.run(),
ScorerNode._score_one() and ScorerNode._report().

WHY THE STUBS, AND WHAT THEY DO NOT COVER
-----------------------------------------
score_bags.py imports rclpy/rosbag2_py/geometry_msgs/rosgraph_msgs at module
level, so it cannot be imported on a box without ROS. When ROS is absent
(Windows dev box, and the ROS-free CI subset) this module installs the
smallest possible placeholders for those four imports only, and removes them
again at teardown so no other test module can pick them up. When ROS is
present (the Dell) the real modules are imported and the stubs are never
installed — the tests below are identical either way.

The stubs stand in for the ROS *import surface* only. Everything under test
here — the scenario loop, the per-bag error row, the confusion-matrix
partition, the artifact write — is real code. What is NOT covered: bag
replay, the /threat/centroid subscription, ROS parameter declaration, and
graph discovery. Those need a bag and a live graph and are exercised by
run_regression.sh on the Dell, not here.
"""

import sys
import types

import yaml

from huitzilin_perception.score_bags_logic import SPAWN_LEAD_S


# ── ROS import surface, only if the real one is missing ───────────────────────

_STUBBED: list = []


def _install_ros_stubs() -> None:
    def _mod(name):
        m = types.ModuleType(name)
        sys.modules[name] = m
        _STUBBED.append(name)
        return m

    rclpy = _mod("rclpy")
    rclpy_node = _mod("rclpy.node")
    rclpy_qos = _mod("rclpy.qos")
    rclpy_ser = _mod("rclpy.serialization")
    _mod("rosbag2_py")
    geom = _mod("geometry_msgs")
    geom_msg = _mod("geometry_msgs.msg")
    rosgraph = _mod("rosgraph_msgs")
    rosgraph_msg = _mod("rosgraph_msgs.msg")

    class _Node:
        def __init__(self, *a, **k):
            pass

    class _QoSProfile:
        def __init__(self, **k):
            self.__dict__.update(k)

    class _Enum:
        RELIABLE = 1
        KEEP_LAST = 1

    rclpy.node = rclpy_node
    rclpy.qos = rclpy_qos
    rclpy.serialization = rclpy_ser
    rclpy_node.Node = _Node
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.QoSReliabilityPolicy = _Enum
    rclpy_qos.QoSHistoryPolicy = _Enum
    rclpy_ser.deserialize_message = lambda *a, **k: None
    geom.msg = geom_msg
    geom_msg.PointStamped = type("PointStamped", (), {})
    rosgraph.msg = rosgraph_msg
    rosgraph_msg.Clock = type("Clock", (), {})


try:  # real ROS (the Dell) wins; stubs are a fallback, never an override
    import rclpy  # noqa: F401
except ImportError:
    _install_ros_stubs()

from huitzilin_perception.score_bags import ScorerNode  # noqa: E402


def teardown_module(module):
    """Never leak the placeholders into another test module's import."""
    for name in _STUBBED:
        sys.modules.pop(name, None)
    if _STUBBED:
        sys.modules.pop("huitzilin_perception.score_bags", None)


# ── A ScorerNode with every I/O path replaced, and nothing else ───────────────

BAG_START = 1000.0
GOOD_TTC = 1.5
GOOD_WINDOW_S = 4.0
# Below (ttc - min(ttc, fall_time)) / 2 = 0.4307 s at ttc = 1.5 s, so the
# floor overtakes the clamped late edge and the window inverts.
INVERTING_WINDOW_S = 0.25


class _Logger:
    def info(self, *a, **k):
        pass

    warn = info
    error = info
    debug = info


def _make_node(tmp_path, scenarios, sidecars, split="all"):
    matrix_f = tmp_path / "matrix.yaml"
    matrix_f.write_text(
        yaml.safe_dump({"scenarios": scenarios}), encoding="utf-8"
    )

    node = object.__new__(ScorerNode)          # skip __init__: no ROS params
    node._bag_dir = tmp_path
    node._matrix_f = matrix_f
    node._split = split
    node._floor = 0.95
    node._out_file = tmp_path / "report.txt"
    node._detector_node_name = "detector"
    node._detection_times = []
    node._detection_positions = []
    node._listening = False
    node._exclude_topics = ["/threat/centroid"]

    node.get_logger = lambda: _Logger()
    node._discover_exclude_topics = lambda: ["/threat/centroid"]
    node._find_bag = lambda sid: tmp_path / ("week3_" + sid + ".mcap")
    node._load_sidecar = lambda sid: sidecars[sid]
    node._read_bag_start_sim_t = lambda p: BAG_START

    def _replay(bag_path):
        # Three closing detections 1.0 s after spawn: inside the strict
        # window of any well-formed positive in this file.
        t0 = BAG_START + SPAWN_LEAD_S + 1.0
        node._detection_times = [t0, t0 + 0.1, t0 + 0.2]
        node._detection_positions = [
            (t0, 6.0, 0.0, 0.0),
            (t0 + 0.1, 4.0, 0.0, 0.0),
            (t0 + 0.2, 2.0, 0.0, 0.0),
        ]
        return 20.0

    node._replay_bag = _replay
    node.start_listening = lambda: None
    node.stop_listening = lambda: list(node._detection_times)
    return node


def _pos(sid):
    return {"id": sid, "label": "positive", "speed_mps": 8.0,
            "offset_vertical_m": 0.0}


def _neg(sid):
    return {"id": sid, "label": "negative", "speed_mps": 0.0,
            "offset_vertical_m": 0.0}


def _sidecar(window_s, ttc=GOOD_TTC):
    return {"detection_window_s": window_s, "time_to_closest_s": ttc}


# ── The tests ────────────────────────────────────────────────────────────────

def test_one_inverted_window_does_not_abort_the_whole_scoring_run(tmp_path):
    """
    MEDIUM-1, the whole point. Before the caller-side guard this raised out
    of run(), so _report() never ran and NO artifact was written — not even
    for S01, which had already been scored.
    """
    scenarios = [_pos("S01"), _pos("S02"), _pos("S03"), _neg("N01")]
    sidecars = {
        "S01": _sidecar(GOOD_WINDOW_S),
        "S02": _sidecar(INVERTING_WINDOW_S),   # the broken one
        "S03": _sidecar(GOOD_WINDOW_S),
        "N01": _sidecar(GOOD_WINDOW_S, ttc=0.0),
    }
    node = _make_node(tmp_path, scenarios, sidecars)

    exit_code = node.run()          # must NOT raise

    assert exit_code == 1, "a refused window must still fail the run"
    assert node._out_file.exists(), "the artifact must be written anyway"
    report = node._out_file.read_text(encoding="utf-8")
    # every scenario is present, including the ones after the broken one
    for sid in ("S01", "S02", "S03", "N01"):
        assert sid in report
    assert "STRICT WINDOW REFUSED" in report
    # and the verdict names it rather than blaming a missing bag
    assert "1 scenario(s) excluded from metrics" in report
    assert "S02:" in report


def test_a_refused_window_is_an_error_row_not_a_false_negative(tmp_path):
    """
    The row must stay OUT of the confusion matrix. Counting a harness error
    as a detection result is what once put "FP rate 25%" on a run with zero
    actual false positives.
    """
    scenarios = [_pos("S01"), _pos("S02"), _pos("S03")]
    sidecars = {
        "S01": _sidecar(GOOD_WINDOW_S),
        "S02": _sidecar(INVERTING_WINDOW_S),
        "S03": _sidecar(GOOD_WINDOW_S),
    }
    node = _make_node(tmp_path, scenarios, sidecars)
    node.run()
    report = node._out_file.read_text(encoding="utf-8")

    # Two scored positives, both TP. The refused one is NOT an FN.
    assert "TP=2  FN=0  TN=0  FP=0  EXCLUDED=1" in report
    # ...and it leaves the recall DENOMINATOR too, with the exclusion named.
    assert "(2/2 positives scored, 1 excluded)" in report


def test_the_refused_row_carries_the_error_flag_and_the_two_numbers(tmp_path):
    """Row shape at the _score_one seam: error=True, pass=False, actionable."""
    node = _make_node(
        tmp_path, [_pos("S02")], {"S02": _sidecar(INVERTING_WINDOW_S)}
    )
    row = node._score_one(_pos("S02"))

    assert row["error"] is True
    assert row["pass"] is False
    assert "STRICT WINDOW REFUSED" in row["note"]
    assert "detection_window_s=" + str(INVERTING_WINDOW_S) in row["note"]
    assert "time_to_closest_s=" + str(GOOD_TTC) in row["note"]


def test_the_negative_branch_is_guarded_too(tmp_path):
    """
    Negatives call strict_window_bounds() with ttc = 0.0, where the window
    inverts only for a NEGATIVE detection_window_s. Nothing validates that
    field, so the route is real and it must not abort the run either.
    """
    node = _make_node(
        tmp_path, [_neg("N01")], {"N01": _sidecar(-1.0, ttc=0.0)}
    )
    row = node._score_one(_neg("N01"))

    assert row["error"] is True
    assert row["pass"] is False
    assert "STRICT WINDOW REFUSED" in row["note"]


def test_a_well_formed_library_run_is_untouched_by_the_guard(tmp_path):
    """
    The guard must be invisible at every shipped configuration. No error
    row, no exclusion, and the artifact reads exactly as before.
    """
    scenarios = [_pos("S01"), _pos("S02")]
    sidecars = {"S01": _sidecar(GOOD_WINDOW_S), "S02": _sidecar(GOOD_WINDOW_S)}
    node = _make_node(tmp_path, scenarios, sidecars)

    assert node.run() == 0
    report = node._out_file.read_text(encoding="utf-8")
    assert "TP=2  FN=0" in report
    assert "EXCLUDED" not in report
    assert "STRICT WINDOW REFUSED" not in report
    assert "PASS" in report
