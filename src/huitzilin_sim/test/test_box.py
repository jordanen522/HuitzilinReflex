"""The guarded rectangle: containment, corners, waypoints, and the fence.

Runs without ROS:
    python -m pytest src/huitzilin_sim/test/test_box.py

Every failure this module catches is one that is silent in flight. An
inverted box contains nobody and the alarm simply never fires; a box outside
the geofence flies correctly until it reaches a corner, then returns to
launch in the middle of a patrol.
"""

import math

import pytest

from huitzilin_sim.box import (
    Box,
    BoxError,
    box_from_params,
    check_fence_radius,
)
from huitzilin_sim.mav_bridge import MavBridge

# The shipped default: 5 m square anchored at the arming point.
SQUARE = Box(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, alt_m=2.0)
# A rectangle, because a city block is not a square. 80 m by 25 m.
BLOCK = Box(min_x=-40.0, max_x=40.0, min_y=-12.5, max_y=12.5, alt_m=3.0)


def test_a_point_in_the_middle_of_the_box_is_inside():
    assert SQUARE.contains_xy(2.5, 2.5) is True


def test_points_beyond_each_of_the_four_edges_are_outside():
    assert SQUARE.contains_xy(-0.01, 2.5) is False
    assert SQUARE.contains_xy(5.01, 2.5) is False
    assert SQUARE.contains_xy(2.5, -0.01) is False
    assert SQUARE.contains_xy(2.5, 5.01) is False


def test_a_person_standing_exactly_on_the_boundary_is_inside():
    """Edges are inclusive. An exclusive bound would leave a hairline seam
    around the whole perimeter that reads as a detector fault."""
    for x, y in ((0.0, 2.5), (5.0, 2.5), (2.5, 0.0), (2.5, 5.0)):
        assert SQUARE.contains_xy(x, y) is True
    for corner in SQUARE.corners_enu():
        assert SQUARE.contains_xy(*corner) is True


def test_altitude_is_never_consulted_when_testing_containment():
    """contains_xy takes no z at all, which is the guarantee. Range to a
    person is a monocular scale estimate, so a height test would drop a
    crouching or badly ranged person out of a box they are standing in."""
    with pytest.raises(TypeError):
        SQUARE.contains_xy(2.5, 2.5, 1.8)


def test_a_rectangle_is_supported_not_only_a_square():
    assert BLOCK.contains_xy(39.0, 0.0) is True
    assert BLOCK.contains_xy(0.0, 13.0) is False
    assert BLOCK.width_m == 80.0
    assert BLOCK.depth_m == 25.0


def test_the_corners_run_counter_clockwise_from_the_minimum_corner():
    assert SQUARE.corners_enu() == [(0.0, 0.0), (5.0, 0.0),
                                    (5.0, 5.0), (0.0, 5.0)]


def test_the_waypoints_are_the_corners_converted_by_the_one_frame_helper():
    """MavBridge.enu_to_ned is the single conversion site in the repository.
    Recomputing it here rather than hard-coding the answer is what makes this
    a check on box.py instead of a copy of its output."""
    flat = SQUARE.to_waypoints_ned()
    assert len(flat) == 12

    expected = []
    for x, y in SQUARE.corners_enu():
        expected.extend(MavBridge.enu_to_ned(x, y, SQUARE.alt_m))
    assert flat == expected


def test_the_waypoints_put_the_drone_above_the_ground_not_below_it():
    """NED d is negative for up. A sign error here flies into the ground,
    and it is the single easiest mistake to make in this frame."""
    flat = SQUARE.to_waypoints_ned()
    downs = flat[2::3]
    assert downs == [-2.0, -2.0, -2.0, -2.0]


def test_the_furthest_corner_is_the_one_that_has_to_fit_in_the_fence():
    assert SQUARE.max_corner_radius_m() == pytest.approx(math.hypot(5.0, 5.0))
    assert SQUARE.max_corner_radius_m() == pytest.approx(7.0710678, abs=1e-6)


def test_an_inverted_box_is_rejected_and_the_message_names_the_axis():
    """An inverted box contains nobody, so the alarm never fires while the
    patrol still flies. It has to fail at construction or not at all."""
    with pytest.raises(BoxError) as exc:
        Box(min_x=5.0, max_x=0.0, min_y=0.0, max_y=5.0)
    assert "x (east)" in str(exc.value)

    with pytest.raises(BoxError) as exc:
        Box(min_x=0.0, max_x=5.0, min_y=5.0, max_y=0.0)
    assert "y (north)" in str(exc.value)


def test_a_zero_area_box_is_rejected_on_whichever_axis_collapsed():
    with pytest.raises(BoxError) as exc:
        Box(min_x=2.0, max_x=2.0, min_y=0.0, max_y=5.0)
    assert "x (east)" in str(exc.value)


def test_a_non_finite_bound_is_rejected_by_name():
    """NaN fails every comparison without raising, so an unchecked NaN gives
    a box that rejects every point and an alarm that never fires."""
    with pytest.raises(BoxError) as exc:
        Box(min_x=0.0, max_x=float("nan"), min_y=0.0, max_y=5.0)
    assert "max_x" in str(exc.value)


def test_a_zero_or_negative_altitude_is_rejected():
    """Altitude is metres above the arming point, so zero flies into it."""
    with pytest.raises(BoxError):
        Box(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, alt_m=0.0)


def test_a_box_inside_the_geofence_is_accepted():
    """The shipped 5 m square reaches 7.07 m, inside the 10 m FENCE_RADIUS."""
    check_fence_radius(SQUARE, 10.0)


def test_a_box_whose_corner_leaves_the_geofence_is_refused_before_takeoff():
    """The failure this prevents is not a refusal to arm: the drone flies,
    reaches a corner, breaches and RTLs mid-patrol, which looks like a
    flight-controller fault rather than a configuration one."""
    with pytest.raises(BoxError) as exc:
        check_fence_radius(BLOCK, 10.0)
    message = str(exc.value)
    assert "FENCE_RADIUS" in message
    assert "%.2f" % BLOCK.max_corner_radius_m() in message


def test_an_edge_can_sit_inside_the_fence_while_a_corner_does_not():
    """The geofence is a circle and the box is a rectangle, so checking an
    edge midpoint instead of a corner would pass a box that breaches."""
    box = Box(min_x=0.0, max_x=9.0, min_y=0.0, max_y=9.0)
    assert box.max_x < 10.0                       # every edge is inside
    assert box.max_corner_radius_m() > 10.0       # the corner is not
    with pytest.raises(BoxError):
        check_fence_radius(box, 10.0)


def test_a_missing_or_nonsense_fence_radius_is_refused():
    with pytest.raises(BoxError):
        check_fence_radius(SQUARE, 0.0)
    with pytest.raises(BoxError):
        check_fence_radius(SQUARE, float("nan"))


def test_a_box_is_built_from_the_declared_parameter_names():
    """box_from_params is shared by patrol_node and guard_node so the two
    cannot disagree about what a parameter name means."""
    values = {"box_min_x": 1.0, "box_max_x": 4.0,
              "box_min_y": 2.0, "box_max_y": 6.0, "box_alt_m": 2.5}
    box = box_from_params(values.__getitem__)
    assert box == Box(min_x=1.0, max_x=4.0, min_y=2.0, max_y=6.0, alt_m=2.5)


def test_a_box_is_immutable_once_built():
    """Validation happens at construction, so a box that could be edited
    afterwards would be a box that had never been checked."""
    with pytest.raises(Exception):
        SQUARE.max_x = 50.0
