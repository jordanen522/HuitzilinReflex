"""The two camera models must differ ONLY in their optics. ROS-free, so CI runs it.

Why this test exists. `iris_ar0234` is a COPY of `iris_depth` with one sensor
block swapped. That copy is the cheapest way to get a second optics
configuration without taxing every baseline run with a second rendered camera,
but it puts ~120 lines of flight plugins in two places, and CLAUDE.md records
what losing them costs: `iris_depth` merge-includes the BARE
`iris_with_standoffs`, which ships no flight plugins, so a model missing them
gives SITL no FDM ("No JSON sensor message received", "link 1 down", nothing on
:9002, no lift) while Gazebo steps fine and the cloud streams. The failure
looks like a networking problem, not a model problem.

The other half matters just as much: the 8 m/s baseline and the 20 m/s result
are only comparable if the AIRFRAME is identical and only
the sensor changed -- otherwise the improvement could be coming from lighter
mass, different drag, or a moved camera mount. These assertions are what let
the two lanes be compared at all.
"""
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
BASELINE_SDF = PKG / "models" / "iris_depth" / "model.sdf"
LONGRANGE_SDF = PKG / "models" / "iris_ar0234" / "model.sdf"

# NEITHER SDF IS STRICTLY WELL-FORMED XML. Both carry `-----` underlines inside
# their header comments, and XML forbids `--` in a comment; Gazebo's parser
# accepts them, ElementTree does not. Comments hold nothing this file asserts
# on, so they are stripped before parsing rather than reformatted -- rewriting
# the baseline model's header to satisfy a test would be changing the artefact
# under measurement.
_COMMENT = re.compile(r"<!--.*?-->", re.DOTALL)


def _model(path):
    root = ET.fromstring(_COMMENT.sub("", path.read_text(encoding="utf-8")))
    model = root.find("./model")
    assert model is not None, f"{path.name} has no <model>"
    return model


@pytest.fixture(scope="module")
def baseline():
    return _model(BASELINE_SDF)


@pytest.fixture(scope="module")
def longrange():
    return _model(LONGRANGE_SDF)


def _plugin_signature(model):
    """Every plugin, by filename+name, with its full serialised body.

    Compares the plugins' CONTENT, not just their presence: a LiftDrag with a
    different area or a rotor pointed at the wrong joint would pass a count.
    """
    out = []
    for plugin in model.findall("./plugin"):
        body = ET.tostring(plugin, encoding="unicode")
        out.append((plugin.get("filename"), plugin.get("name"), body))
    return sorted(out)


def test_both_models_exist():
    assert BASELINE_SDF.is_file()
    assert LONGRANGE_SDF.is_file()


def test_flight_plugins_are_identical(baseline, longrange):
    """The one that stops a silent no-lift."""
    assert _plugin_signature(longrange) == _plugin_signature(baseline)


def test_the_long_range_model_kept_the_ardupilot_plugin(longrange):
    """Named explicitly, because its absence is the failure CLAUDE.md
    describes and a plugin-set diff against a broken baseline would pass."""
    names = {p.get("name") for p in longrange.findall("./plugin")}
    assert "ArduPilotPlugin" in names
    assert sum(1 for p in longrange.findall("./plugin")
               if p.get("name") == "gz::sim::systems::LiftDrag") == 8
    assert sum(1 for p in longrange.findall("./plugin")
               if p.get("name") == "gz::sim::systems::ApplyJointForce") == 4


def test_airframe_geometry_is_unchanged(baseline, longrange):
    """The 2.5x comparison needs the same aircraft on both sides.

    Mass, inertia and link poses must match, or an improvement could be coming
    from a lighter or better-balanced airframe rather than from the sensor.
    """
    def links(model):
        return {lnk.get("name"): ET.tostring(lnk.find("./inertial"),
                                            encoding="unicode")
                for lnk in model.findall("./link")
                if lnk.find("./inertial") is not None}

    assert links(longrange) == links(baseline)


def test_camera_mount_pose_is_unchanged(baseline, longrange):
    """Same mount, so the defended sector is measured from the same origin."""
    def camera_link_pose(model):
        for lnk in model.findall("./link"):
            if lnk.get("name") == "camera_link":
                pose = lnk.find("./pose")
                return None if pose is None else pose.text.split()
        return "missing"

    assert camera_link_pose(longrange) == camera_link_pose(baseline)


# ── the optics, which are the whole point of the second model ────────────────

def _sensor(model):
    sensors = model.findall(".//sensor[@type='depth_camera']")
    assert len(sensors) == 1, "expected exactly one depth camera"
    return sensors[0]


def test_the_optics_actually_differ(baseline, longrange):
    """If these ever matched, the long-range lane would be measuring the
    OAK-D under a different name -- the exact substitution CLAUDE.md records
    as having cost a full run."""
    def optics(model):
        cam = _sensor(model).find("./camera")
        return (cam.find("./horizontal_fov").text,
                cam.find("./image/width").text,
                cam.find("./image/height").text)

    assert optics(longrange) != optics(baseline)


def test_long_range_clip_clears_the_modelled_reach(longrange):
    """A clip at the reach manufactures a zero with no error anywhere.

    sigma grows as z^2, so the ROI ceiling must clear 26 m by 4 sigma --
    27.2 m per synthetic_depth.required_roi_max_range_m -- and the render clip
    must clear the ROI ceiling in turn. iris_depth's 19.0 m would have made
    26 m unmeasurable while reporting a clean run.
    """
    from huitzilin_perception.synthetic_depth import required_roi_max_range_m

    far = float(_sensor(longrange).find("./camera/clip/far").text)
    assert far > required_roi_max_range_m(26.0)


def test_long_range_camera_declares_no_sdf_noise(longrange):
    """depth_noise_node owns the error for this lane, and must own all of it.

    iris_depth carries a 1 cm per-pixel gaussian. Left in place it would be
    both 30x too small at 26 m and structurally wrong -- independent noise
    smears the cluster while averaging the centroid error DOWN by ~sqrt(N) --
    and, worse, it would mean `sigma_ref_m: 0.0` is not a noiseless control.
    """
    assert _sensor(longrange).find("./camera/noise") is None


def test_baseline_camera_is_untouched(baseline):
    """The 8 m/s baseline is the comparison's anchor; pin its optics so the
    long-range work cannot quietly drift them."""
    cam = _sensor(baseline).find("./camera")
    assert cam.find("./image/width").text == "640"
    assert cam.find("./image/height").text == "480"
    assert float(cam.find("./clip/far").text) == 19.0
    # And it keeps the 1 cm noise it has always had.
    assert cam.find("./noise") is not None


def test_the_two_lanes_publish_to_different_topics(baseline, longrange):
    """Same trap as `oracle_detector` and `detector` both publishing
    /threat/centroid: two uncorrelated views of one ball, silently merged."""
    assert (_sensor(longrange).find("./topic").text
            != _sensor(baseline).find("./topic").text)
