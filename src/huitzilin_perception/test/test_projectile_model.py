"""The projectile must not collide with anything. ROS-free, so CI runs it.

Why this test exists. The ball is meant to fly its parabola straight through the
airframe: hits are scored geometrically from /gz/dynamic_poses against
hit_radius, and a physical strike instead tumbles the vehicle, ArduPilot
crash-checks it, and every remaining scenario in the battery aborts on
MIN_SPAWN_Z with the wreck on the runway.

That was "fixed" once already, with

    <surface><collide_bitmask>0x00</collide_bitmask></surface>

which is the WRONG PARENT -- collide_bitmask is a child of <surface><contact>.
SDFormat dropped it, warned on every spawn, and the ball went on colliding until
2026-08-07, when a dataflash log showed six of eight throws producing an
uncommanded roll excursion of 33-48 deg (past ATC_ANGLE_MAX=30) beginning
0.27-0.47 s after each dodge -- the ball-arrival window -- and the last one
rolling to 180 deg and crashing. All five sweep cells died that way, taking
every S20N false-dodge control with them.

So the assertion here is deliberately structural, not a value check: there must
be NO collision geometry at all. A link without collision geometry cannot
collide under any physics engine, which is the only version of this that cannot
regress into a silently-ignored attribute a third time.
"""
import pathlib
import xml.etree.ElementTree as ET

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]      # src/huitzilin_perception
MODEL_SDF = PKG / "models" / "projectile" / "model.sdf"


@pytest.fixture(scope="module")
def link():
    root = ET.parse(MODEL_SDF).getroot()
    lnk = root.find("./model/link")
    assert lnk is not None, "projectile model has no <link>"
    return lnk


def test_projectile_has_no_collision_geometry(link):
    # The whole point: nothing to collide WITH, rather than a flag asking the
    # physics engine not to. See the module docstring for what the flag cost.
    assert link.find("collision") is None, (
        "projectile/model.sdf grew a <collision> element again. The ball must "
        "pass through the airframe -- a real strike flips the vehicle and "
        "aborts the rest of the battery on MIN_SPAWN_Z."
    )


def test_no_collide_bitmask_anywhere(link):
    # If someone restores contact for an impact-damage study, they must not
    # reintroduce the misparented bitmask that started this.
    stray = [el.tag for el in link.iter() if el.tag == "collide_bitmask"]
    assert not stray, (
        "collide_bitmask is back. It is a child of <surface><contact>, not of "
        "<surface>; misparented, SDFormat discards it silently and the ball "
        "collides anyway. Delete the collision geometry instead."
    )


def test_gravity_stays_off(link):
    # spawn_projectile restores gravity as a persistent -mass*g wrench at throw
    # time. If the link had gravity on, the ball would free-fall during the
    # ~0.5 s the `gz` create call costs and never reach the frustum.
    grav = link.find("gravity")
    assert grav is not None and grav.text.strip() == "false"


def test_inertial_survives_collision_removal(link):
    # The launch impulse and the gravity wrench act on <inertial>, not on
    # <collision>, so removing collision geometry must leave the ballistics
    # untouched. mass is also coupled to spawn_projectile's mass_kg default.
    mass = link.find("./inertial/mass")
    assert mass is not None and float(mass.text) == pytest.approx(0.150)


def test_visual_survives_collision_removal(link):
    # The depth detector sees <visual>; the oracle reads /gz/dynamic_poses.
    # Neither needs <collision>, but the visual must still be there.
    radius = link.find("./visual/geometry/sphere/radius")
    assert radius is not None and float(radius.text) == pytest.approx(0.040)
