"""The perception-side hardware overlay, checked as data.

Both flips asserted here are staged in Week 5 from the DepthAI contract and
verified against a live stream in Week 6 (S6-3). Getting either wrong produces
a topic that looks alive and a detector that receives nothing, which is an
expensive thing to debug on an aircraft.
"""

import pathlib

import yaml

PERCEPTION = pathlib.Path(__file__).resolve().parents[1]


def hw():
    doc = yaml.safe_load((PERCEPTION / "params" / "hw_detector.yaml").read_text(encoding="utf-8"))
    return doc["detector"]["ros__parameters"]


def hw_evasion():
    doc = yaml.safe_load((PERCEPTION / "params" / "hw_evasion.yaml").read_text(encoding="utf-8"))
    return doc["evasion"]["ros__parameters"]


def test_cloud_convention_is_optical():
    """DepthAI emits optical convention (x right, y down, z forward). Leaving
    gz_flu rotates every point 90 degrees and nothing clears the ROI gate."""
    assert hw()["cloud_convention"] == "optical"


def test_cloud_subscription_is_best_effort():
    """DepthAI publishes best-effort. A RELIABLE subscription does not match a
    best-effort publisher in ROS 2 -- the topic appears in `ros2 topic list`
    and the detector receives nothing at all."""
    assert hw()["cloud_reliable"] is False


def test_no_diagnostic_flag_ships_in_the_hardware_config():
    """debug_dump_dir costs ~40 ms/frame on its own, a quarter of the 150 ms
    budget, and invalidates every latency number from the run."""
    p = hw()
    assert p["debug_dump_dir"] == ""
    assert p["debug_funnel"] is False
    assert p["profile_stages"] is False


def test_the_detector_overlay_does_not_re_open_a_measured_null():
    """roi_max_range_m 5->8 and cluster_min_points 5->3 were each measured over
    a full battery with the baseline best on every column. They may only be
    re-opened once S6-4 has measured a materially different range or frame
    rate, and then deliberately."""
    for null in ("roi_max_range_m", "cluster_min_points"):
        assert null not in hw()


def test_the_evasion_overlay_does_not_re_open_a_measured_null():
    """min_track_updates and dodge_speed_mps are EVASION parameters. Asserting
    them absent from hw_detector.yaml was vacuous -- they could never appear
    there -- and left hw_evasion.yaml, the file that can actually re-open them,
    unguarded. Its own header lists both as nulls.

    dodge_speed_mps's null was re-established in Week 6 on better evidence: a
    within-session hover A/B at 1.5 / 3.0 / 6.0 (4x the command buys 1.095x the
    escape). The older 1.5->4.0 patrol sweep is superseded and should not be
    cited, but the guard this test enforces is unchanged."""
    for null in ("min_track_updates", "dodge_speed_mps"):
        assert null not in hw_evasion()
