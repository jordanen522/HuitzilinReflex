"""Unit tests for StageProfiler (detector per-stage latency instrumentation).

Runs without ROS:
    python -m pytest src/huitzilin_perception/test/test_stage_profiler.py

What is being pinned: the instrument itself, because this session already
shipped two pieces of *misleading* instrumentation — a sim-time compute figure
that structurally reads 0 ms, and a derived rtf of 0.00 from it. A profiler
that silently mis-attributes time is worse than none, since the whole point is
to decide which detector stage to optimise.
"""

import time

from huitzilin_perception.stage_profiler import StageProfiler


def _burn(seconds: float) -> None:
    """Busy-wait, not sleep: the profiler measures wall time either way, but a
    busy loop keeps the test honest about CPU-bound stages like the numpy
    passes it exists to measure."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        pass


def test_report_ranks_hottest_stage_first():
    p = StageProfiler(window_frames=1)
    with p.stage("cheap"):
        _burn(0.002)
    with p.stage("hot"):
        _burn(0.020)
    p.frame_done()
    line = p.report()
    assert line.index("hot") < line.index("cheap"), line


def test_ranking_is_by_total_not_per_call():
    """A stage that runs on every frame outranks a rare expensive one.

    This is the decision the report has to get right: 2 ms x 20 frames is
    40 ms of the budget, while 20 ms x 1 frame is 20 ms. Ordering by per-call
    mean would point the optimisation at the wrong stage.
    """
    p = StageProfiler(window_frames=50)
    for _ in range(20):
        with p.stage("every_frame"):
            _burn(0.002)
    with p.stage("rare"):
        _burn(0.020)
    line = p.report()
    assert line.index("every_frame") < line.index("rare"), line
    assert "x20" in line and "x1" in line, line


def test_stage_call_counts_expose_the_funnel():
    """A stage called fewer times than there were frames sits past an early out.

    _cloud_cb returns early at eight points, and it counts EVERY arriving cloud
    as a frame — a frame that bails at the range gate delays the next cloud just
    as much as one that publishes. So the diagnostic value is in the gap between
    the frame count and each stage's call count: here five clouds arrived, all
    five were deserialised, only one reached clustering.
    """
    p = StageProfiler(window_frames=100)
    for _ in range(5):
        with p.stage("deserialize"):
            _burn(0.001)
        p.frame_done()
    with p.stage("cluster"):                # only the last frame got this far
        _burn(0.001)
    line = p.report()
    assert "frames=5" in line, line
    assert "deserialize" in line and "x5" in line, line
    assert "cluster" in line and "x1" in line, line


def test_due_fires_only_at_the_window():
    p = StageProfiler(window_frames=3)
    assert not p.due()
    p.frame_done()
    p.frame_done()
    assert not p.due()
    p.frame_done()
    assert p.due()


def test_report_resets_so_windows_do_not_accumulate():
    p = StageProfiler(window_frames=1)
    with p.stage("alpha"):
        _burn(0.001)
    p.frame_done()
    p.report()
    assert not p.due()
    line = p.report()
    assert "frames=0" in line, line
    assert "alpha" not in line, "stale stage survived the reset: " + line


def test_exception_inside_a_stage_still_records_it():
    """The deque-mutation race that cost a battery run raised mid-callback.

    If an exception discarded the timing, the profiler would go blind exactly
    on the frames worth investigating.
    """
    p = StageProfiler(window_frames=1)
    try:
        with p.stage("boom"):
            _burn(0.001)
            raise RuntimeError("deque mutated during iteration")
    except RuntimeError:
        pass
    p.frame_done()
    line = p.report()
    assert "boom" in line, line


def test_empty_window_reports_without_dividing_by_zero():
    p = StageProfiler(window_frames=5)
    line = p.report()
    assert "frames=0" in line
    assert "0.0 ms/frame" in line


def test_shares_sum_to_about_one_hundred_percent():
    p = StageProfiler(window_frames=1)
    for name in ("alpha", "beta", "gamma"):
        with p.stage(name):
            _burn(0.003)
    p.frame_done()
    line = p.report()
    shares = [int(tok.strip("(%)")) for tok in line.split()
              if tok.startswith("(") and tok.endswith("%)")]
    assert len(shares) == 3, line
    # Rounding to whole percent can drift by 1 per stage.
    assert 97 <= sum(shares) <= 103, line
