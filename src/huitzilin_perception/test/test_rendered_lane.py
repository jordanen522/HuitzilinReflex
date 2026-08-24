"""The rendered long-range lane, checked as configuration. ROS-free, so CI runs it.

WHAT THIS FILE IS FOR. The rendered lane is the only lane in the repo that can
be evidence for "a depth sensor sees the ball at 20 m/s": the oracle lane
asserts the centroid from Gazebo truth and the synthetic-depth lane fabricates
the cloud, so neither can. That makes its configuration load-bearing in a way
the others' is not -- and almost every way it can be wrong is SILENT. A gate
sized for a 5 m ball, a QoS mismatch on a 6 MB cloud, a model that never gets
copied into the share directory: each produces a run that starts cleanly, logs
nothing alarming, and reports a depressed save rate that reads as a sensor
limit. CLAUDE.md names that failure class as one that has already invalidated
whole result sets in this project.

None of it is reachable from the launch file's own asserts, because a launch
file imports `launch` and `launch_ros` and CI's ROS-free subset cannot import
it. That is why huitzilin_perception/rendered_lane.py exists as a pure module,
and why this file tests the module rather than the launch.

WHAT IT DELIBERATELY DOES NOT COVER. The two camera models are held to each
other by test_camera_models.py, and the noise model itself by
test_depth_noise.py. This file covers the seams between them: params against
node defaults, params against the world, battery geometry against the ROI
ceiling.
"""

import ast
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

from huitzilin_perception.depth_noise import required_cluster_max_extent_m
from huitzilin_perception.rendered_lane import (
    AR0234_FAR_CLIP_M,
    DESIGN_REACH_M,
    assert_gates_clear_the_rendered_range,
)
from huitzilin_perception.synthetic_depth import required_roi_max_range_m

PKG = pathlib.Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]

RENDERED_PARAMS = PKG / "params" / "rendered_detector.yaml"
BASELINE_PARAMS = PKG / "params" / "detector.yaml"
SYNTHETIC_PARAMS = PKG / "params" / "synthetic_depth_detector.yaml"
BATTERY = PKG / "config" / "week7_rendered_battery.yaml"
BASELINE_WORLD = PKG / "worlds" / "huitzilin_runway.sdf"
LONGRANGE_WORLD = PKG / "worlds" / "huitzilin_runway_ar0234.sdf"
LONGRANGE_MODEL = PKG / "models" / "iris_ar0234" / "model.sdf"
NOISE_NODE = PKG / "huitzilin_perception" / "depth_noise_node.py"
SETUP_PY = PKG / "setup.py"
BATTERY_SCRIPT = REPO / "scripts" / "run_dodge_battery.sh"

# Same reason as test_camera_models.py: neither SDF is strictly well-formed XML
# (both carry `-----` underlines in their header comments, and XML forbids `--`
# inside a comment). Gazebo's parser accepts them, ElementTree does not.
_COMMENT = re.compile(r"<!--.*?-->", re.DOTALL)


def _params(path, node="detector"):
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    return doc[node]["ros__parameters"]


@pytest.fixture(scope="module")
def rendered():
    return _params(RENDERED_PARAMS)


@pytest.fixture(scope="module")
def baseline():
    return _params(BASELINE_PARAMS)


@pytest.fixture(scope="module")
def battery():
    return yaml.safe_load(BATTERY.read_text(encoding="utf-8"))


def _rows(battery_doc):
    return {row["id"]: row for row in battery_doc["runs"]}


# the guard itself

def test_the_shipped_long_range_config_clears_its_own_gates(rendered):
    assert_gates_clear_the_rendered_range(rendered)


def test_the_short_range_baseline_config_is_not_nagged(baseline):
    """detector.yaml at 5.0 m / 0.35 m is CORRECT for its own range, and the
    fidelity-gate arm runs the same launch file on it. A guard that refused it
    would force the gate to fly a config the Week 4 reference never flew, which
    would destroy the only comparison the gate exists to make."""
    assert_gates_clear_the_rendered_range(baseline)


def test_a_long_roi_with_a_short_extent_gate_is_refused(baseline):
    """The failure the guard exists for: someone opens the ROI to reach 26 m
    and leaves the extent gate where a 5 m sensor left it. Nothing errors at
    runtime -- cluster_and_split just re-clusters the stretched ball and
    fragments it below cluster_min_points on a large share of frames."""
    bad = dict(baseline, roi_max_range_m=28.0)
    with pytest.raises(RuntimeError, match="cluster_max_extent_m"):
        assert_gates_clear_the_rendered_range(bad)


def test_an_roi_that_does_not_clear_the_design_reach_is_refused(rendered):
    """sigma grows as z^2, so the ceiling must clear 26 m by 4 sigma. The draws
    a tight ceiling clips are the LONG ones -- a throw's first detections, the
    ones that buy tca -- so this shortens the sensor exactly where it counts."""
    needed = required_roi_max_range_m(DESIGN_REACH_M)
    # The extent gate is given generous headroom so THIS branch is what fires.
    # A tight ceiling also tightens the extent requirement, and a test that let
    # the first check fire would pass while asserting nothing about the second.
    tight = dict(rendered, roi_max_range_m=DESIGN_REACH_M + 0.1,
                 cluster_max_extent_m=5.0)
    assert tight["roi_max_range_m"] < needed
    with pytest.raises(RuntimeError, match="design reach"):
        assert_gates_clear_the_rendered_range(tight)


def test_an_roi_past_the_far_clip_is_refused(rendered):
    """A ceiling the optics cannot reach can never bind. It does no harm to the
    cloud and every harm to the write-up: it names a range as the sensor's."""
    with pytest.raises(RuntimeError, match="far clip"):
        assert_gates_clear_the_rendered_range(
            dict(rendered, roi_max_range_m=AR0234_FAR_CLIP_M + 1.0,
                 cluster_max_extent_m=5.0))


def test_the_guard_is_specific_to_this_lanes_noise_structure():
    """DO NOT "fix" synthetic_depth_detector.yaml to satisfy this guard.

    It carries roi_max_range_m 28.0 with cluster_max_extent_m 0.35, which this
    guard would refuse -- and that pairing is nonetheless CORRECT there. The
    two lanes stretch the ball differently:

      synthetic  the whole ball is displaced along the ray by ONE draw per
                 cloud, so its extent is unchanged: 0.069-0.073 m measured at
                 every range from 1 m to 26 m.
      rendered   the error field is correlated over ~7 px while the 80 mm ball
                 spans only ~5 px at 26 m, so the patch is STRETCHED rather
                 than displaced: 1.6 sigma peak-to-peak, hence 0.080+1.6*0.30.

    week6_synthetic_depth.launch.py therefore does not call this guard, and
    wiring it in there would make a measured lane's config fail a check written
    for a different error model -- where the "fix" would be editing a
    configuration after its results were already recorded.
    """
    synthetic = _params(SYNTHETIC_PARAMS)
    with pytest.raises(RuntimeError):
        assert_gates_clear_the_rendered_range(synthetic)


# params against the node at the other end of /oak/points

def _node_constant(name):
    """A module-level constant out of depth_noise_node.py, read as SOURCE.

    depth_noise_node imports rclpy, so CI's ROS-free subset cannot import it;
    parsing is the only way this pin can run where it is needed.
    """
    tree = ast.parse(NOISE_NODE.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {NOISE_NODE.name}")


def test_cloud_reliability_matches_the_publisher(rendered):
    """A reliability mismatch leaves the subscription unconnected while the
    topic still appears in `ros2 topic list` -- and the run reports a detector
    that never saw the ball.

    This pin is not hypothetical. rendered_detector.yaml was first written
    `false`, against depth_noise_node's older BEST_EFFORT default, and it would
    have flown that way: BEST_EFFORT on this lane's ~6.2 MB cloud fragments
    every message past the 64 kB UDP limit and drops the whole cloud whenever
    one fragment is lost -- 3.99 Hz against 14.43 Hz reliable, measured on a
    7.37 MB cloud (see detector.yaml). A TRANSPORT failure that scores as a
    perception one.
    """
    assert rendered["cloud_reliable"] is _node_constant("DEFAULT_CLOUD_RELIABLE")
    assert rendered["cloud_reliable"] is True


def test_cloud_queue_depth_matches_the_publisher(rendered):
    assert rendered["cloud_queue_depth"] == _node_constant(
        "DEFAULT_CLOUD_QUEUE_DEPTH")


def test_cloud_convention_is_the_gazebo_body_frame(rendered):
    """gz-sim PointCloudPacked stays in the sensor BODY frame (X fwd, Y left,
    Z up); <optical_frame_id> renames the header only. depth_noise_node
    preserves that convention exactly, so what reaches the detector is still
    gz_flu. Reading it as `optical` takes depth off the wrong axis -- the bug
    that had the noise stage publishing a noiseless sensor while reporting a
    clean run (docs/optics_probe.md)."""
    assert rendered["cloud_convention"] == "gz_flu"


def test_the_params_file_names_the_node_the_launch_file_starts():
    """In ROS 2 a mistyped node name in a params file silently loads NOTHING:
    every parameter falls back to detector_node's in-code defaults, which
    differ from this file where it matters most (min_publish_score 0.3 vs
    0.006). The run would look configured and be flying a different sensor."""
    doc = yaml.safe_load(RENDERED_PARAMS.read_text(encoding="utf-8"))
    assert list(doc) == ["detector"]
    assert "ros__parameters" in doc["detector"]


# params against the physics they were sized against

def test_the_extent_gate_is_sized_at_the_roi_ceiling(rendered):
    """AT THE CEILING, NOT AT THE 26 m DESIGN REACH -- the distinction this
    file's first version got wrong.

    Nothing clamps the cloud to the design reach. roi_max_range_m is the gate
    that decides which returns reach the clusterer, so a ball anywhere inside
    it must survive the extent gate. Sizing at 26 m gave 0.56 m and would have
    fragmented every ball between ~26.7 m and the 28 m ceiling: the LONGEST
    first detections, which are exactly the ones that buy tca.

    Bounded above as well as below, so the value stays the arithmetic rounded
    up rather than a round number that happens to clear it.
    """
    needed = required_cluster_max_extent_m(rendered["roi_max_range_m"])
    assert needed <= rendered["cluster_max_extent_m"] <= needed + 0.02
    # And it is genuinely larger than the reach-sized value that was wrong.
    assert (rendered["cluster_max_extent_m"]
            > required_cluster_max_extent_m(DESIGN_REACH_M))


def test_the_difference_threshold_is_unchanged_from_the_baseline(
        rendered, baseline):
    """The prediction registered in this file's header -- that static terrain
    flickers foreground past z = 15 m on its own noise -- is a prediction ONLY
    while diff_threshold_m is the baseline's 0.10 m. Changing it before the
    first run would turn a falsifiable prediction into a tuned result, and
    would confound the comparison against Week 4 on two axes at once."""
    assert rendered["diff_threshold_m"] == baseline["diff_threshold_m"]


def test_the_background_model_stays_on(rendered):
    """The synthetic lane turns use_persistent_bg off because its cloud holds
    the ball and nothing else. Here there is a real scene, and the persistent
    map is what closed the patrol blindness in Week 4. Turning it off would
    make this a different experiment, not a tuned one."""
    assert rendered["use_persistent_bg"] is True


def test_no_diagnostic_flag_ships_in_the_scored_config(rendered):
    """debug_dump_dir costs ~40 ms/frame and invalidates every latency number
    from the run; debug_funnel is the tool that answers the header's
    prediction and belongs to diagnostic runs only."""
    assert rendered["debug_dump_dir"] == ""
    assert rendered["debug_funnel"] is False
    assert rendered["profile_stages"] is False


def test_the_far_clip_constant_tracks_the_camera_it_describes():
    """AR0234_FAR_CLIP_M is a copy of a number that lives in the SDF. Copies
    drift; this is the only thing stopping it."""
    root = ET.fromstring(_COMMENT.sub(
        "", LONGRANGE_MODEL.read_text(encoding="utf-8")))
    sensors = root.findall(".//sensor[@type='depth_camera']")
    assert len(sensors) == 1
    assert float(sensors[0].find("./camera/clip/far").text) == AR0234_FAR_CLIP_M


# the world file

def _world(path):
    return ET.fromstring(_COMMENT.sub("", path.read_text(encoding="utf-8")))


def test_both_worlds_carry_the_same_world_name():
    """`<world name>` is the root of the gz topic namespace that
    spawn_projectile's ApplyLinkWrench pair and gz_pose_bridge both address
    (/world/huitzilin_runway/wrench). Renaming it in the long-range world would
    re-point every one of them at a topic nothing serves: the ball would spawn
    at rest and drop. That is a physics failure that looks exactly like a
    perception failure."""
    assert (_world(LONGRANGE_WORLD).find("./world").get("name")
            == _world(BASELINE_WORLD).find("./world").get("name"))


def test_the_two_worlds_differ_only_in_the_drone_include():
    """Everything else -- physics, plugins, sun, ground plane -- must be
    identical, or the 8 m/s baseline and the 20 m/s result are not the same
    experiment with one axis changed."""
    def not_the_drone(path):
        world = _world(path).find("./world")
        return [ET.tostring(child, encoding="unicode").strip()
                for child in world if child.tag != "include"]

    assert not_the_drone(LONGRANGE_WORLD) == not_the_drone(BASELINE_WORLD)


def test_the_long_range_world_flies_the_long_range_drone():
    """The entity NAME differs as well as the uri, and that is deliberate: a
    world that kept the baseline name would lie about which camera it flies, in
    a study whose entire claim is which sensor was used. The price is
    DRONE_MODEL=iris_ar0234 on the harness, which a later test pins."""
    def drone(path):
        includes = _world(path).find("./world").findall("./include")
        assert len(includes) == 1
        inc = includes[0]
        return (inc.find("./uri").text, inc.find("./name").text,
                inc.find("./pose").text)

    uri, name, pose = drone(LONGRANGE_WORLD)
    base_uri, base_name, base_pose = drone(BASELINE_WORLD)
    assert uri == "model://iris_ar0234"
    assert name == "iris_ar0234"
    assert (uri, name) != (base_uri, base_name)
    # The mount is unchanged, so the defended sector is measured from the same
    # origin at the same altitude.
    assert pose == base_pose


def test_the_long_range_model_is_installed():
    """setup.py's data_files is the whole reason a model reaches the share
    directory. Omitting iris_ar0234 fails in the worst way available: Gazebo
    logs "Unable to find uri" ONCE and then starts the world with no drone in
    it, which downstream reads as total detection failure."""
    assert "models/iris_ar0234/*" in SETUP_PY.read_text(encoding="utf-8")


# the battery

MATCHED_ENVELOPE = ("R01", "R02", "R03")

# The offset rule as this battery's header derives it -- re-derived for this
# lane rather than copied from the oracle one. The ball must spawn OUTSIDE
# anything the pipeline can see, or the pipeline is handed a detection it never
# had to acquire and the cell delivers a shorter sensor than its label.
SPAWN_DEAD_S = 0.27
DRONE_CLOSURE_MPS = 4.2
DELIVERED_RATE_HZ = 23.0


def _required_offset_m(speed_mps, roi_ceiling_m):
    return (roi_ceiling_m
            + SPAWN_DEAD_S * speed_mps
            + DRONE_CLOSURE_MPS * (SPAWN_DEAD_S + 1.0 / DELIVERED_RATE_HZ)
            + speed_mps / DELIVERED_RATE_HZ)


def test_the_matched_envelope_varies_only_speed(battery):
    """A 2.5x claim between R01 and R03 is a claim about SPEED only if speed is
    the only thing that differs. Anything else varying here -- geometry, aim,
    n -- would leave the difference attributable to two causes at once."""
    rows = _rows(battery)
    defaults = battery["defaults"]
    keys = ("approach_angle_deg", "miss_distance_m", "expect_dodge", "repeats")

    profiles = {rid: tuple(rows[rid].get(k, defaults.get(k)) for k in keys)
                + (rows[rid].get("offset_forward_m",
                                 defaults["offset_forward_m"]),)
                for rid in MATCHED_ENVELOPE}
    assert len(set(profiles.values())) == 1, profiles

    speeds = [rows[rid]["speed_mps"] for rid in MATCHED_ENVELOPE]
    assert speeds == sorted(speeds)
    assert len(set(speeds)) == len(speeds)


def test_the_envelope_spans_the_claim(battery):
    """The claim is 20 m/s against an 8 m/s baseline, so both endpoints must be
    IN this table. Re-using Week 4's 78/78 as the baseline arm would compare a
    patrol dodge count against a hover save rate -- different quantities under
    different rules."""
    rows = _rows(battery)
    assert rows["R01"]["speed_mps"] == 8.0
    assert rows["R03"]["speed_mps"] == 20.0
    assert rows["R03"]["speed_mps"] / rows["R01"]["speed_mps"] == 2.5


def test_every_envelope_row_has_equal_n(battery):
    """Unequal n invites reading a difference in interval width as a difference
    in effect."""
    rows = _rows(battery)
    assert len({rows[rid]["repeats"] for rid in MATCHED_ENVELOPE}) == 1


def test_the_spawn_offset_clears_the_roi_ceiling_at_every_speed(
        battery, rendered):
    """The outermost gate in this lane is the detector's roi_max_range_m, not
    the camera's far clip. A ball that spawns inside it enters the gate already
    visible, and first_det_range_m -- which in this lane IS the reach
    measurement -- becomes an artefact of the spawn point."""
    rows = _rows(battery)
    ceiling = float(rendered["roi_max_range_m"])
    for rid in MATCHED_ENVELOPE + ("R04",):
        offset = rows[rid].get("offset_forward_m",
                               battery["defaults"]["offset_forward_m"])
        assert offset >= _required_offset_m(rows[rid]["speed_mps"], ceiling), rid


def test_the_negative_control_expects_no_dodge(battery):
    """Recall and false-positive rate are separate numbers, and this row is the
    only source of the second one. Unlike the synthetic lane's D10 -- whose
    cloud contains the ball and nothing else, leaving background differencing
    no clutter to reject -- this cloud carries ground plane, runway and
    standoffs, so a fire here means the stack built a threat out of scene
    clutter."""
    rows = _rows(battery)
    assert rows["R04"]["expect_dodge"] is False
    assert rows["R04"]["miss_distance_m"] > 0.30
    assert all(rows[rid]["expect_dodge"] is True for rid in MATCHED_ENVELOPE)


def test_the_fidelity_gate_reproduces_the_week4_geometry_exactly(battery):
    """G01/G02 are B03/B02 re-flown. Their whole value is being the SAME
    scenario as a result that already exists, so their geometry is pinned to
    Week 4's and is deliberately exempt from the offset rule above -- they are
    not measuring reach, they are checking that the plumbing did not change
    what the baseline sensor sees."""
    rows = _rows(battery)
    assert rows["G01"]["speed_mps"] == 14.0     # B03
    assert rows["G02"]["speed_mps"] == 8.0      # B02
    for rid in ("G01", "G02"):
        assert rows[rid]["offset_forward_m"] == 6.0
        assert rows[rid]["miss_distance_m"] == 0.0
        assert rows[rid]["approach_angle_deg"] == 0.0


def test_every_row_id_is_unique(battery):
    ids = [row["id"] for row in battery["runs"]]
    assert len(ids) == len(set(ids))


# the harness that flies it

def test_the_battery_script_can_run_this_lane():
    """A battery config nothing selects is a document, not an experiment."""
    script = BATTERY_SCRIPT.read_text(encoding="utf-8")
    assert '"${MODE}" == "week7"' in script
    assert "week7_rendered_battery.yaml" in script


def test_the_battery_script_can_name_the_long_range_drone():
    """dodge_battery defaults drone_model to iris_depth, which does not exist
    in the ar0234 world -- it would find no drone and score nothing. DRONE_MODEL
    is a named variable rather than something buried in EXTRA_ARGS so that the
    one axis differing between the two week7 arms is visible in the command."""
    script = BATTERY_SCRIPT.read_text(encoding="utf-8")
    assert "drone_model:=${DRONE_MODEL:-iris_depth}" in script
