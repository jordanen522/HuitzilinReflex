"""The guarded rectangle: one box that is both the flight path and the alarm.

The box is the whole configuration surface a user is expected to touch. Four
numbers and an altitude describe where the drone flies and where a person
counts as an intruder, and they are deliberately the SAME four numbers: a
deployment that patrolled one rectangle while alarming on another would look
healthy from every angle -- the drone flies, the detector detects, the siren
stays quiet -- while guarding nothing.

The drone flies the PERIMETER only. It never crosses the interior, so a large
box is watched from its edges inward as far as the camera reaches, rather
than surveyed. docs/guard.md states what that does and does not cover.

This module lives in huitzilin_sim rather than huitzilin_guard for two
reasons that are both load-bearing. patrol_node needs it, and
test_isolation.py asserts that huitzilin_sim never references the guard
package -- an import the other way would break the isolation guarantee the
guard package exists to keep. And to_waypoints_ned must convert through
MavBridge.enu_to_ned, which lives here; the guard package is not permitted to
import the MAVLink bridge at all.

ENU metres throughout (x east, y north, REP-103), as every ROS topic in this
workspace is. The NED conversion happens in exactly one place, at the edge.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from huitzilin_sim.mav_bridge import MavBridge

# The five parameters a node declares to describe a box. Exported so the
# config test can assert the patrol and guard blocks agree without listing
# the names a second time and drifting from them.
BOX_PARAM_NAMES = ("box_min_x", "box_max_x", "box_min_y", "box_max_y",
                   "box_alt_m")


class BoxError(ValueError):
    """A box that cannot be flown or guarded. Always raised at construction.

    Every failure here is silent at runtime if it is allowed through: an
    inverted box contains nobody, so the alarm never fires while the patrol
    looks perfectly healthy.
    """


@dataclass(frozen=True)
class Box:
    """An axis-aligned rectangle in ENU metres, plus the patrol altitude.

    Rectangles, not only squares: min and max are independent per axis, so a
    city block that is 80 m by 25 m is expressed directly.

    Coordinates are relative to the EKF origin -- where the aircraft was
    armed -- because that is the frame /huitzilin/odom reports in. A box
    written against any other origin is not wrong at construction and cannot
    be detected here; it simply guards the wrong patch of ground.
    """

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    alt_m: float = 2.0

    def __post_init__(self) -> None:
        for name in ("min_x", "max_x", "min_y", "max_y", "alt_m"):
            value = getattr(self, name)
            # NaN fails every comparison below without raising, so an
            # unchecked NaN produces a box that rejects every point and an
            # alarm that never fires. Catch it by name instead.
            if not math.isfinite(value):
                raise BoxError(
                    "%s is %r, which is not a finite number. A non-finite "
                    "bound makes contains_xy false for every point, so the "
                    "alarm would never fire and nothing would look wrong."
                    % (name, value))

        for axis, low, high in (("x (east)", self.min_x, self.max_x),
                                ("y (north)", self.min_y, self.max_y)):
            if low >= high:
                raise BoxError(
                    "box is inverted or zero-width on %s: min %g is not less "
                    "than max %g. Such a box contains nobody, so the alarm "
                    "never fires while the patrol still flies."
                    % (axis, low, high))

        if self.alt_m <= 0.0:
            raise BoxError(
                "box_alt_m is %g. Patrol altitude is metres ABOVE the arming "
                "point, so zero or negative flies the aircraft into the "
                "ground." % self.alt_m)

    @property
    def width_m(self) -> float:
        """East-west extent."""
        return self.max_x - self.min_x

    @property
    def depth_m(self) -> float:
        """North-south extent."""
        return self.max_y - self.min_y

    def contains_xy(self, x: float, y: float) -> bool:
        """Is this ground position inside the guarded area?

        Two-dimensional ON PURPOSE, and the altitude is not consulted. Range
        to a person is a monocular scale estimate (see pose_detector), so the
        height it implies is the least trustworthy number available; testing
        it would drop a tall person, a crouching one, or one at a badly
        estimated range out of a box they are standing in.

        Edges are INSIDE. A person on the boundary line is in the area, and
        an exclusive bound would leave a hairline seam around the perimeter
        that reads as a detector fault rather than as a geometry choice.
        """
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y)

    def corners_enu(self) -> List[Tuple[float, float]]:
        """The four ground corners, counter-clockwise from (min_x, min_y).

        Counter-clockwise is arbitrary but fixed: the patrol must fly a
        consistent circuit, and a winding that changed with the numbers would
        reverse the flight direction when somebody widened the box.
        """
        return [(self.min_x, self.min_y),
                (self.max_x, self.min_y),
                (self.max_x, self.max_y),
                (self.min_x, self.max_y)]

    def to_waypoints_ned(self) -> List[float]:
        """The perimeter as the flat [n, e, d, ...] list patrol_node consumes.

        Converted ONLY through MavBridge.enu_to_ned. That static helper is the
        single frame-conversion site in this repository; a second conversion
        written inline here is how a sign error enters and then shows up as a
        mirrored flight path nobody can source.
        """
        flat: List[float] = []
        for x, y in self.corners_enu():
            n, e, d = MavBridge.enu_to_ned(x, y, self.alt_m)
            flat.extend([n, e, d])
        return flat

    def max_corner_radius_m(self) -> float:
        """Distance from the origin to the furthest corner.

        The geofence is a CIRCLE centred on the arming point; the box is a
        rectangle. This is the number that has to fit inside it, and it is
        always a corner rather than an edge midpoint.
        """
        return max(math.hypot(x, y) for x, y in self.corners_enu())


def box_from_params(lookup) -> Box:
    """Build a Box from a callable returning one declared parameter value.

    Shared by patrol_node and guard_node so the two cannot disagree about
    what a parameter name means, only about what value it holds -- and
    test_guard_params.py catches that.
    """
    return Box(min_x=float(lookup("box_min_x")),
               max_x=float(lookup("box_max_x")),
               min_y=float(lookup("box_min_y")),
               max_y=float(lookup("box_max_y")),
               alt_m=float(lookup("box_alt_m")))


def check_fence_radius(box: Box, fence_radius_m: float) -> None:
    """Raise unless every corner sits inside the flight controller's fence.

    hw_frame.parm sets FENCE_RADIUS 10 with FENCE_ACTION 1 (RTL). A box whose
    corners fall outside that circle does not fail on the bench and does not
    fail at takeoff: the drone flies, reaches a corner, breaches, and returns
    to launch mid-patrol. Checking here turns that into a refusal to start
    that names the number to change.

    SITL loads no fence parameters at all, so this check is the only thing
    standing between a too-large box and a surprise on the first real flight.
    """
    if not math.isfinite(fence_radius_m) or fence_radius_m <= 0.0:
        raise BoxError(
            "fence_radius_m is %r. It must be the positive FENCE_RADIUS the "
            "flight controller is actually running." % (fence_radius_m,))

    reach = box.max_corner_radius_m()
    if reach > fence_radius_m:
        raise BoxError(
            "the box reaches %.2f m from the arming point but the geofence "
            "is %.2f m (FENCE_RADIUS in params/hw_frame.parm, mirrored by "
            "the supervisor's fence_radius_m). The aircraft would breach at "
            "a corner and RTL mid-patrol. Shrink the box, move it toward the "
            "origin, or raise BOTH FENCE_RADIUS and the supervisor value "
            "together -- test_hw_config.py pins them to each other."
            % (reach, fence_radius_m))
