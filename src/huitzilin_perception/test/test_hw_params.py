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
    doc = yaml.safe_load((PERCEPTION / "params" / "hw_detector.yaml").read_text())
    return doc["detector"]["ros__parameters"]


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


def test_the_overlay_does_not_re_open_a_measured_null():
    """roi_max_range_m 5->8, min_track_updates 3->2 and cluster_min_points
    5->3 were each measured over a full battery with the baseline best on
    every column. They may only be re-opened once S6-4 has measured a
    materially different range or frame rate, and then deliberately."""
    for null in ("roi_max_range_m", "min_track_updates", "cluster_min_points"):
        assert null not in hw()
