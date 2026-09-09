"""Placing a detection in the world, checked against hand-computed values.

Runs without ROS:
    python -m pytest src/huitzilin_guard/test/test_world.py

The expected values below are worked out by hand rather than recorded from a
run. A test that captured the implementation's own output would pass just as
happily with the axes swapped, which is the failure this module exists to
catch: a sign error here does not crash, it moves people across box edges.
"""

import math

import pytest

from huitzilin_guard.world import (
    TORSO_JOINTS,
    WorldTransformError,
    camera_point_to_world,
    optical_to_flu,
    person_point_optical,
    rotate_by_quat,
)

LEVEL = (0.0, 0.0, 0.0, 1.0)          # identity: nose along +x east, level


def quat_yaw(deg):
    """Yaw about +Z (up), the only rotation with a one-line closed form."""
    half = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_roll(deg):
    """Roll about +X (forward). This is what banking into a turn looks like."""
    half = math.radians(deg) / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


def test_a_point_straight_ahead_of_the_lens_is_forward_in_the_body_frame():
    """Optical +Z is depth away from the lens; FLU +X is forward."""
    assert optical_to_flu(0.0, 0.0, 5.0) == (5.0, 0.0, 0.0)


def test_optical_right_becomes_body_right_which_is_negative_left():
    assert optical_to_flu(2.0, 0.0, 0.0) == (0.0, -2.0, 0.0)


def test_optical_down_becomes_body_down_which_is_negative_up():
    """The sign that is easiest to get wrong: image Y increases DOWNWARD."""
    assert optical_to_flu(0.0, 3.0, 0.0) == (0.0, 0.0, -3.0)


def test_a_level_drone_facing_east_sees_a_person_to_its_east():
    """Identity attitude means body +X is world +X, which is east in ENU."""
    x, y, z = camera_point_to_world((0.0, 0.0, 6.0), (0.0, 0.0, 2.0), LEVEL)
    assert (x, y, z) == pytest.approx((6.0, 0.0, 2.0))


def test_yawing_ninety_degrees_puts_the_same_person_to_the_north():
    """A yaw of +90 degrees about up turns the nose from east to north, so a
    person 6 m ahead moves from (6, 0) to (0, 6)."""
    x, y, _z = camera_point_to_world((0.0, 0.0, 6.0), (0.0, 0.0, 2.0),
                                     quat_yaw(90.0))
    assert (x, y) == pytest.approx((0.0, 6.0), abs=1e-9)


def test_the_drones_own_position_is_added_so_the_box_is_the_same_frame():
    x, y, _z = camera_point_to_world((0.0, 0.0, 6.0), (10.0, -4.0, 2.0),
                                     LEVEL)
    assert (x, y) == pytest.approx((16.0, -4.0))


def test_roll_displaces_a_detection_sideways_by_roughly_tilt_times_range():
    """The reason full attitude is used instead of yaw alone.

    A roll about the forward axis does not move a point lying ON that axis,
    which is exactly why a straight-ahead test cannot detect the bug. On an
    off-axis point the displacement is range times sin(tilt), and it is
    largest at the corners, where a perimeter patrol banks hardest.
    """
    ahead = (0.0, 0.0, 6.0)
    level = camera_point_to_world(ahead, (0.0, 0.0, 2.0), LEVEL)
    rolled = camera_point_to_world(ahead, (0.0, 0.0, 2.0), quat_roll(10.0))
    assert rolled == pytest.approx(level)

    off_axis = (0.0, 1.0, 6.0)          # 1 m below the lens axis, 6 m out
    _lx, ly, lz = camera_point_to_world(off_axis, (0.0, 0.0, 2.0), LEVEL)
    _rx, ry, rz = camera_point_to_world(off_axis, (0.0, 0.0, 2.0),
                                        quat_roll(10.0))
    assert ly == pytest.approx(0.0)
    assert ry == pytest.approx(math.sin(math.radians(10.0)), abs=1e-9)
    assert abs(rz - lz) > 0.0


def test_an_unnormalised_quaternion_rotates_without_also_scaling():
    """An unnormalised quaternion would scale the vector as well as rotate
    it, which reads downstream as a person at the wrong range rather than as
    a bad orientation."""
    doubled = tuple(2.0 * c for c in quat_yaw(90.0))
    x, y, _z = camera_point_to_world((0.0, 0.0, 6.0), (0.0, 0.0, 2.0),
                                     doubled)
    assert (x, y) == pytest.approx((0.0, 6.0), abs=1e-9)


def test_a_zero_quaternion_is_refused_rather_than_collapsing_the_detection():
    """The EKF publishes (0,0,0,0) before it converges. Rotating by it would
    put every detection at the drone's own position, which on a perimeter
    patrol is a point ON the box edge, so the alarm would fire on the
    aircraft's own location."""
    with pytest.raises(WorldTransformError):
        rotate_by_quat((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))


def test_the_person_point_is_the_torso_centroid():
    """Hips and shoulders: limbs swing, and a single wrist can be a metre
    from the person it belongs to."""
    joints = {
        "hip_l": (-0.2, 0.5, 6.0), "hip_r": (0.2, 0.5, 6.0),
        "shoulder_l": (-0.2, -0.1, 6.0), "shoulder_r": (0.2, -0.1, 6.0),
        "wrist_l": (-1.5, 0.0, 6.0),          # flung out; must not drag it
    }
    x, y, z = person_point_optical(joints)
    assert (x, y, z) == pytest.approx((0.0, 0.2, 6.0))


def test_a_partly_visible_person_falls_back_to_whatever_joints_exist():
    """Somebody half behind a parked car has no hips in frame. Refusing to
    place them would make partial visibility a blind spot exactly where
    someone trying not to be seen would stand."""
    joints = {"wrist_l": (1.0, 0.0, 4.0), "elbow_l": (3.0, 0.0, 4.0)}
    assert not set(joints) & set(TORSO_JOINTS)
    x, _y, z = person_point_optical(joints)
    assert (x, z) == pytest.approx((2.0, 4.0))


def test_a_body_with_no_joints_at_all_yields_nothing():
    assert person_point_optical({}) is None
