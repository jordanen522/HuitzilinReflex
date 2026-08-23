"""
Run-loop tests for score_bags.ScorerNode — the earlier revision a later revision, MEDIUM-1.

WHAT THIS FILE EXISTS FOR
-------------------------
strict_window_bounds() raises on an inverted detection
window. That raise is correct and must stay: an inverted window admits
nothing, so scoring under it posts a silent false negative. But the raise
reached run()'s scenario loop UNGUARDED, so one broken sidecar aborted the
whole run — _report() was never reached, no artifact was written even for
the bags already scored, and every later bag went unreplayed.

test_score_bags_logic.py cannot see that: it tests the pure library, and the
library's job is to raise. The defect is in the CALLER's blast radius, which
lives in score_bags.py. So these tests drive the real ScorerNode.run(),
ScorerNode._score_one() and ScorerNode._report().

Two further invariants cover the same thing -- run() must not lose the
artifact. The first:
print(report) ran ahead of the artifact write and outside any try, so a
non-UTF-8 console killed _report() before the file existed. The second
is the §11 enforcement: a source-level check that no window-refusing
library call in score_bags.py sits outside a guard, which is the one
thing the behavioural tests below structurally cannot see.

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

import ast
import pathlib
import sys
import textwrap
import types

import yaml

from huitzilin_perception.score_bags_logic import SPAWN_LEAD_S


# --- ROS import surface, only if the real one is missing ---------------------

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
from huitzilin_perception import score_bags as _score_bags  # noqa: E402


def teardown_module(module):
    """Never leak the placeholders into another test module's import."""
    for name in _STUBBED:
        sys.modules.pop(name, None)
    if _STUBBED:
        sys.modules.pop("huitzilin_perception.score_bags", None)


# --- A ScorerNode with every I/O path replaced, and nothing else -------------

BAG_START = 1000.0
GOOD_TTC = 1.5
GOOD_WINDOW_S = 4.0
# Below (ttc - min(ttc, fall_time)) / 2 = 0.4307 s at ttc = 1.5 s, so the
# floor overtakes the clamped late edge and the window inverts.
INVERTING_WINDOW_S = 0.25


class _Logger:
    """Records, so a test can assert a failure was reported rather than
    swallowed. Every level lands in the same list."""

    def __init__(self):
        self.messages = []

    def info(self, msg, *a, **k):
        self.messages.append(str(msg))

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

    node._test_log = _Logger()          # test-only handle on the log
    node.get_logger = lambda: node._test_log
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


# --- The tests ---------------------------------------------------------------

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


# --- The console is the secondary sink (a later revision, MEDIUM-A) ---------------

class _NonUtf8Stdout:
    """
    A console that behaves like a real cp1252 one: every write is encoded,
    and a character the codec cannot map raises. The failure comes from the
    same encode the real sink performs, not from a mock of print().
    """

    encoding = "cp1252"

    def __init__(self):
        self.written = []

    def write(self, s):
        s.encode(self.encoding)     # raises on ═ ─ ✓ ✗ — as cp1252 does
        self.written.append(s)
        return len(s)

    def flush(self):
        pass

    @property
    def text(self):
        return "".join(self.written)


def test_a_console_that_cannot_encode_the_report_does_not_cost_the_artifact(
    tmp_path, monkeypatch
):
    """
    MEDIUM-A. print(report) used to run BEFORE the guarded write and outside
    any try, so on a non-UTF-8 stdout a completely well-formed run raised
    UnicodeEncodeError out of _report() and the artifact was never created
    at all — strictly worse than the zero-byte file round 3 fixed.

    This pins both halves of the fix: the write must come first, and the
    print must not propagate. Reordering alone leaves run() raising; guarding
    alone, with the old order, leaves the artifact missing.
    """
    scenarios = [_pos("S01"), _pos("S02")]
    sidecars = {"S01": _sidecar(GOOD_WINDOW_S), "S02": _sidecar(GOOD_WINDOW_S)}
    node = _make_node(tmp_path, scenarios, sidecars)
    monkeypatch.setattr(sys, "stdout", _NonUtf8Stdout())

    exit_code = node.run()          # must NOT raise

    assert exit_code == 0, (
        "a console encoding failure must not change the scoring verdict — "
        "run_regression.sh and CI gate on this exit code"
    )
    assert node._out_file.exists(), "the artifact must be written first"
    report = node._out_file.read_text(encoding="utf-8")
    assert "TP=2  FN=0" in report
    assert "PASS" in report
    assert "═" in report, (
        "the artifact keeps the exact text, not the degraded one"
    )


def test_the_degraded_console_still_carries_every_number(
    tmp_path, monkeypatch
):
    """
    The guard must not be a silent swallow. Every number and the verdict are
    ASCII, so the operator still gets a usable report with only the rules and
    ticks substituted — and the substitution is logged.
    """
    scenarios = [_pos("S01"), _pos("S02")]
    sidecars = {"S01": _sidecar(GOOD_WINDOW_S), "S02": _sidecar(GOOD_WINDOW_S)}
    node = _make_node(tmp_path, scenarios, sidecars)
    fake = _NonUtf8Stdout()
    monkeypatch.setattr(sys, "stdout", fake)

    node.run()

    printed = fake.text
    assert printed, "the console must still receive the report"
    assert "TP=2  FN=0" in printed
    assert "Recall (TP rate): 100.0%" in printed
    assert "PASS" in printed
    assert [m for m in node._test_log.messages if "cp1252" in m], (
        "an unrenderable console must be reported, not silently swallowed: "
        f"log was {node._test_log.messages}"
    )


# --- §11 enforcement: no unguarded window-refusing call site ----------------
#
# Round 3 guarded the two calls in _score_one that can refuse a window, and
# nothing stopped a THIRD from being added outside a try — which restores
# MEDIUM-1 in full and which the five behavioural tests above cannot see,
# since they exercise the two sites that exist. That is a SOURCE-LEVEL
# invariant, so it is checked at the source rather than by behaviour.
#
# Run against 4f04fc7 this names lines 454, 460 and 563 — the three real
# pre-fix sites, including the negative-control site at :563 that the round-2
# review missed and that was found only by going site by site.

WINDOW_REFUSING_CALLS = {"strict_window_bounds", "is_in_window_strict"}
ABSORBS_A_WINDOW_REFUSAL = {"ValueError", "Exception"}


def _handler_catches(handler) -> set:
    """The exception names a single except clause catches."""
    caught = handler.type
    if caught is None:                                  # bare except:
        return {"Exception"}
    if isinstance(caught, ast.Name):                    # except ValueError:
        return {caught.id}
    if isinstance(caught, ast.Tuple):                   # except (A, B):
        return {e.id for e in caught.elts if isinstance(e, ast.Name)}
    return set()


def _is_a_real_guard(handler) -> bool:
    """
    A handler counts only if it absorbs a window refusal AND does something
    other than re-raise it. A handler whose whole body is a bare raise leaves
    the blast radius exactly as it was, so it must not satisfy this check —
    otherwise the check is satisfiable cosmetically.
    """
    if not (_handler_catches(handler) & ABSORBS_A_WINDOW_REFUSAL):
        return False
    body = handler.body
    return not (len(body) == 1 and isinstance(body[0], ast.Raise))


def unguarded_window_calls(source: str) -> list:
    """
    Line numbers of calls to a window-refusing library function that do NOT
    sit in the body of a try whose handler absorbs the refusal.

    Handler bodies and finally clauses deliberately do not count as guarded:
    a raise from either escapes exactly as an unguarded one does.
    """
    tree = ast.parse(source)
    guarded_node_ids = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Try):
            continue
        if not any(_is_a_real_guard(h) for h in node.handlers):
            continue
        for stmt in node.body:
            for sub in ast.walk(stmt):
                # tree stays referenced for the comprehension below, so these
                # object ids remain valid for the whole function.
                guarded_node_ids.add(id(sub))
    return sorted(
        node.lineno
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id in WINDOW_REFUSING_CALLS
        and id(node) not in guarded_node_ids
    )


def test_no_window_refusing_call_in_score_bags_sits_outside_a_guard():
    """
    The invariant round 3 established but could not enforce. If this fails,
    the named line is one broken sidecar away from killing a whole scoring
    run before any artifact is written.
    """
    source = pathlib.Path(_score_bags.__file__).read_text(encoding="utf-8")

    # Vacuity guard FIRST: if the library API is renamed, the set above goes
    # stale and the check silently passes on everything. Fail loudly instead.
    called_names = {
        n.func.id
        for n in ast.walk(ast.parse(source))
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
    }
    missing = WINDOW_REFUSING_CALLS - called_names
    assert not missing, (
        f"{sorted(missing)} is no longer called in score_bags.py, so this "
        f"check has gone vacuous. Update WINDOW_REFUSING_CALLS to the "
        f"current window-refusing library API rather than deleting this."
    )

    offenders = unguarded_window_calls(source)
    assert offenders == [], (
        f"score_bags.py calls a window-refusing library function OUTSIDE a "
        f"try that absorbs the refusal, at line(s) {offenders} of "
        f"{_score_bags.__file__}. An unguarded refusal escapes the scenario "
        f"loop in run() and kills the whole scoring run before _report() "
        f"writes the artifact. Wrap the call and return "
        f"self._unusable_window_row(...), as the two sites in _score_one do."
    )


def test_the_enforcement_check_detects_the_shapes_it_claims_to():
    """
    The check above passes at HEAD, so on its own it cannot demonstrate that
    it works. These synthetic sources are the discriminating cases, including
    the two ways a cosmetic edit could try to satisfy it.
    """
    unguarded = textwrap.dedent(
        """
        def f(a, b, c):
            lo, hi = strict_window_bounds(a, b, c)
            return lo, hi
        """
    )
    assert unguarded_window_calls(unguarded) == [3], (
        "a bare call must be named"
    )

    guarded = textwrap.dedent(
        """
        def f(a, b, c):
            try:
                lo, hi = strict_window_bounds(a, b, c)
            except ValueError:
                return None
            return lo, hi
        """
    )
    assert unguarded_window_calls(guarded) == []

    wrong_exception = textwrap.dedent(
        """
        def f(a, b, c):
            try:
                lo, hi = strict_window_bounds(a, b, c)
            except KeyError:
                return None
        """
    )
    assert unguarded_window_calls(wrong_exception) == [4], (
        "a try that catches something else is not a guard"
    )

    reraise_only = textwrap.dedent(
        """
        def f(a, b, c):
            try:
                lo, hi = strict_window_bounds(a, b, c)
            except ValueError:
                raise
        """
    )
    assert unguarded_window_calls(reraise_only) == [4], (
        "a handler that only re-raises changes nothing and must not count"
    )

    in_the_handler = textwrap.dedent(
        """
        def f(a, b, c):
            try:
                pass
            except ValueError:
                lo, hi = strict_window_bounds(a, b, c)
        """
    )
    assert unguarded_window_calls(in_the_handler) == [6], (
        "a raise from inside a handler escapes exactly as an unguarded one"
    )

    mixed = textwrap.dedent(
        """
        def f(a, b, c):
            try:
                lo, hi = strict_window_bounds(a, b, c)
                ok = is_in_window_strict(a, b, c)
            except ValueError:
                return None
            hi2 = strict_window_bounds(a, b, c + 1.0)
            return lo, hi, ok, hi2
        """
    )
    assert unguarded_window_calls(mixed) == [8], (
        "only the unguarded site may be named, and by its own line number"
    )
