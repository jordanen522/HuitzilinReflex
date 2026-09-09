"""The guard node end to end, without a ROS graph, a camera or a drone.

Needs a built workspace (it imports rclpy through the node):
    ./scripts/run_tests.sh -k guard_node

Named in the CI deny-list for that reason; every other module in this package
reads node source as text and needs no ROS.

The node is built with object.__new__ and hand-rolled stubs rather than a
mocking library, matching the rest of this suite: a duck-typed publisher that
records what it was handed says more about what the node did than an
assertion that some mock was called.
"""

import json

import pytest

from huitzilin_guard.guard_node import GuardNode
from huitzilin_guard.presence import PresenceLatch, PresencePolicy
from huitzilin_sim.box import Box

BOX = Box(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, alt_m=2.0)
POLICY = PresencePolicy(confirm_frames=3, clear_after_s=3.0,
                        min_alert_s=3.0, max_alert_s=300.0,
                        stale_input_s=5.0)


class Recorder:
    """A publisher that keeps what it was handed."""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class Logger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class Msg:
    def __init__(self, data):
        self.data = data


class Request:
    def __init__(self, data):
        self.data = data


class Response:
    success = False
    message = ""


def make_node(armed=True, position=(2.5, 0.0, 2.0)):
    """A guard node with everything ROS replaced by a stub.

    The drone sits at (2.5, 0) -- on the southern edge of the box -- looking
    north, which is the geometry a perimeter patrol actually flies.
    """
    node = object.__new__(GuardNode)
    node.box = BOX
    node._latch = PresenceLatch(POLICY)
    node.armed = armed
    node._position_enu = position
    # Yaw +90 degrees about up: nose turns from east to north, so camera
    # depth (+z optical) maps onto world +y.
    node._quat_xyzw = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
    node._person_in_box = False
    node._frames_seen = 0
    node._frames_rejected = 0
    node._frames_unplaceable = 0
    node._alarm_pub = Recorder()
    node._status_pub = Recorder()

    node.clock = {"t": 0.0}
    node._now_s = lambda: node.clock["t"]
    node.get_logger = lambda: Logger()
    return node


def detection(depth_m, stamp_s=0.0):
    """A body standing depth_m straight ahead of the lens."""
    return Msg(json.dumps({
        "schema": 1,
        "stamp_s": stamp_s,
        "frame_id": "camera_optical_frame",
        "source": "pose_detector",
        "subject": 0,
        "joints": {
            "hip_l": [-0.2, 0.5, depth_m, 0.9],
            "hip_r": [0.2, 0.5, depth_m, 0.9],
            "shoulder_l": [-0.2, -0.1, depth_m, 0.9],
            "shoulder_r": [0.2, -0.1, depth_m, 0.9],
        },
    }))


def status_of(node):
    node._publish_status()
    return json.loads(node._status_pub.sent[-1])


def edges(node):
    """The alarm stream with runs collapsed, i.e. just the transitions.

    The node publishes a LEVEL every time it evaluates the latch, because
    alert_signal holds a dead-man and clears the siren when the stream
    stops. So the raw list is mostly repeats, and what a test cares about is
    where it changed.
    """
    out = []
    for value in node._alarm_pub.sent:
        if not out or out[-1] != value:
            out.append(value)
    return out


def test_the_alarm_state_is_republished_not_only_sent_on_edges():
    """An edge-only publisher is indistinguishable from a dead node, and
    alert_signal would clear the siren stale_off_s after every alarm."""
    node = make_node()
    for i in range(4):
        node.clock["t"] = i * 0.1
        node._tick()
    assert len(node._alarm_pub.sent) == 4
    assert edges(node) == [False]


def test_a_person_inside_the_box_raises_the_alarm_after_the_confirm_window():
    """2 m ahead of a drone on the southern edge is (2.5, 2.0): inside."""
    node = make_node()
    for i in range(3):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))
    assert node._person_in_box is True
    assert edges(node) == [False, True]
    assert node._alarm_pub.sent[-1] is True


def test_a_person_outside_the_box_never_raises_it():
    """8 m ahead is (2.5, 8.0): past the northern edge at y = 5."""
    node = make_node()
    for i in range(10):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(8.0))
    assert node._person_in_box is False
    assert True not in node._alarm_pub.sent


def test_the_alarm_clears_once_the_person_has_left_for_long_enough():
    node = make_node()
    for i in range(3):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))
    assert node._alarm_pub.sent[-1] is True

    t = 0.2
    while t < 4.0:
        t += 0.5
        node.clock["t"] = t
        node._detection_cb(detection(8.0))     # outside, still detected
        node._tick()
    assert edges(node) == [False, True, False]
    assert node._person_in_box is False


def test_a_disarmed_guard_fires_no_alarm_at_all():
    """The ADT property: the panel is off, so nothing sounds."""
    node = make_node(armed=False)
    for i in range(10):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))
        node._tick()
    assert node._alarm_pub.sent == []


def test_a_disarmed_guard_still_reports_the_person_it_can_see():
    """Reporting an empty box while somebody stands in it would be a lie told
    by the only instrument an operator has."""
    node = make_node(armed=False)
    node._detection_cb(detection(2.0))
    report = status_of(node)
    assert report["armed"] is False
    assert report["person_in_box"] is True
    assert report["alarm"] is False
    assert report["frames_seen"] == 1


def test_arming_applies_the_confirm_window_rather_than_firing_instantly():
    """Somebody already standing in the box when the panel is armed must
    still be confirmed, not sirened on the first frame."""
    node = make_node(armed=False)
    for i in range(5):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))

    node._srv_arm(Request(True), Response())
    assert node._alarm_pub.sent == []

    node.clock["t"] = 1.0
    node._detection_cb(detection(2.0))
    assert True not in node._alarm_pub.sent    # one frame is not three
    node.clock["t"] = 1.1
    node._detection_cb(detection(2.0))
    node.clock["t"] = 1.2
    node._detection_cb(detection(2.0))
    assert node._alarm_pub.sent[-1] is True


def test_disarming_silences_a_sounding_alarm_immediately():
    """min_alert_s protects against a flickering detector, not against an
    operator deliberately switching the panel off."""
    node = make_node()
    for i in range(3):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))
    assert node._alarm_pub.sent[-1] is True

    node.clock["t"] = 0.5
    resp = node._srv_arm(Request(False), Response())
    assert resp.success is True
    assert edges(node) == [False, True, False]
    assert node._alarm_pub.sent[-1] is False
    assert node.armed is False


def test_the_arm_service_reports_which_way_it_was_switched():
    node = make_node(armed=False)
    assert "armed" in node._srv_arm(Request(True), Response()).message
    assert "disarmed" in node._srv_arm(Request(False), Response()).message


def test_a_frame_carrying_a_face_joint_is_refused_and_counted():
    """The privacy boundary, exercised through the node rather than only
    through the parser. frames_rejected is the telemetry that says an
    upstream is sending fields this subsystem refuses."""
    node = make_node()
    node._detection_cb(Msg(json.dumps({
        "schema": 1, "stamp_s": 0.0, "subject": 0,
        "joints": {"nose": [0.0, 0.0, 2.0, 0.9]},
    })))
    assert node._frames_rejected == 1
    assert True not in node._alarm_pub.sent
    assert status_of(node)["frames_rejected"] == 1


def test_malformed_json_is_refused_rather_than_crashing_the_node():
    node = make_node()
    node._detection_cb(Msg("{not json"))
    assert node._frames_rejected == 1


def test_a_detection_with_no_odometry_is_dropped_not_guessed_at():
    """Without odometry there is no way to say WHERE the person is. Treating
    that as "not in the box" would be a guess in the unsafe direction."""
    node = make_node(position=None)
    for i in range(5):
        node.clock["t"] = i * 0.1
        node._detection_cb(detection(2.0))
    assert node._frames_unplaceable == 5
    assert True not in node._alarm_pub.sent
    assert status_of(node)["have_odom"] is False


def test_an_unconverged_attitude_is_dropped_rather_than_placing_the_person():
    """A zero quaternion would put every detection at the drone's own
    position, which on a perimeter patrol is a point on the box edge."""
    node = make_node()
    node._quat_xyzw = (0.0, 0.0, 0.0, 0.0)
    node._detection_cb(detection(2.0))
    assert node._frames_unplaceable == 1
    assert True not in node._alarm_pub.sent


def test_the_status_heartbeat_states_that_nothing_is_recorded():
    node = make_node()
    assert status_of(node)["recording"] is False


def test_the_status_carries_the_box_so_the_guarded_area_is_observable():
    """An operator has to be able to confirm which rectangle is live without
    reading the config off the aircraft."""
    assert status_of(make_node())["box"] == [0.0, 5.0, 0.0, 5.0]


def test_an_empty_box_reports_no_detection_age_rather_than_a_fault():
    """The detector publishes only when it sees somebody, so silence is the
    normal resting state and must not read as a failure."""
    assert status_of(make_node())["secs_since_detection"] is None


def test_the_node_holds_no_mavlink_connection():
    """Belt and braces next to test_isolation, asserted on the live class.

    patrol_node carries `mav` and reaches send_position_ned through it. This
    node must have neither, so there is no object here through which a
    setpoint could be sent.

    create_client is deliberately NOT checked this way: rclpy.Node defines
    it, so every node in the workspace has the attribute and no subclass can
    remove it. The guarantee that matters is that this package never CALLS
    it, which test_isolation asserts against the source with an AST walk --
    the same reason the ban lives on the call rather than on std_srvs.
    """
    for banned in ("mav", "send_position_ned", "cmd_pub"):
        assert not hasattr(GuardNode, banned)


def test_a_bad_box_stops_the_node_instead_of_running_inside_it():
    """box_from_params raises at construction. An inverted box contains
    nobody, so the alarm would never fire while the patrol looked healthy."""
    from huitzilin_sim.box import BoxError, box_from_params
    values = {"box_min_x": 5.0, "box_max_x": 0.0, "box_min_y": 0.0,
              "box_max_y": 5.0, "box_alt_m": 2.0}
    with pytest.raises(BoxError):
        box_from_params(values.__getitem__)
