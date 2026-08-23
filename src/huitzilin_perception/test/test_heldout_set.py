"""The held-out set: geometry and split-hygiene invariants. ROS-free.

This split exists to score detector recall on scenarios the detector was
never tuned against. These tests keep the yaml honest about that: they check
that the held-out IDs are disjoint (by speed and by the full parameter
triple) from every tuning split, that the negative budget is kept, and that
the set's own geometry is internally consistent.

Two amendments are load-bearing here and are pinned as such, because both of
them RELAX something and a relaxation is exactly what should be hard to make
silently:

  A3(a)  the standoff co-varies with speed, because gravity compensation adds
         vz0 = 0.5*g*t on top of the horizontal speed and the resulting loft
         carries the ball out of the camera's +/-11 deg vertical sector. That
         bounds the standoff at d <= 0.0793 * v^2.
  A3(b)  the +/-28 deg approach arm was deleted because it lies outside the
         +/-13.5 deg horizontal sector for the whole flight and would have
         designed four guaranteed misses into the set -- a built-in 14/18
         ceiling. Recall on this set is therefore recall WITHIN THE DEFENDED
         SECTOR, not recall over an arbitrary approach.

What this file cannot check, and what no test can: that nobody ever replays an
H-bag while choosing a threshold. That is a procedural discipline, not a
structural one -- see docs/bag_capture_runbook.md.
"""

import math
import pathlib

import pytest
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]
MATRIX = PKG / "config" / "scenario_matrix.yaml"

G_MPS2 = 9.81

#: The camera in models/iris_ar0234/model.sdf: 27.0 deg horizontal, 22.0 deg
#: vertical. test_camera_models.py holds the SDF to those numbers; this file
#: holds the scenario geometry to the same ones.
SECTOR_HALF_H_DEG = 13.5
SECTOR_HALF_V_DEG = 11.0

#: 18 positives (one spare, so a single void bag doesn't shrink the usable
#: set below 17) and 6 negatives -- kept >=25% of the total per the
#: library's negative-budget rule below.
N_POSITIVES = 18
N_NEGATIVES = 6


@pytest.fixture(scope="module")
def matrix():
    return yaml.safe_load(MATRIX.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def by_id(matrix):
    return {s["id"]: s for s in matrix["scenarios"]}


def _held(matrix, label=None):
    out = [s for s in matrix["scenarios"] if s["id"].startswith("H")]
    return [s for s in out if label is None or s["label"] == label]


# -- size and shape ----------------------------------------------------------

def test_the_set_is_the_size_enumerated(matrix):
    """17 is the usable minimum; 18 is what was enumerated. A set that shrinks
    to 17 after a void bag is a set whose denominator moved."""
    assert len(_held(matrix, "positive")) == N_POSITIVES
    assert len(_held(matrix, "negative")) == N_NEGATIVES
    assert N_POSITIVES >= 17


def test_the_negative_budget_is_kept(matrix):
    """The library's own >= 25 % rule, which exists so a false-positive rate
    has enough denominator to mean anything."""
    pos, neg = len(_held(matrix, "positive")), len(_held(matrix, "negative"))
    assert neg / (pos + neg) >= 0.25


def test_every_id_is_unique(matrix):
    ids = [s["id"] for s in matrix["scenarios"]]
    assert len(ids) == len(set(ids))


def test_the_splits_do_not_overlap(matrix):
    """A scenario in two splits is a scenario that is both tuned on and held
    out. This is the structural check that keeps `heldout` meaningful."""
    seen = {}
    for name, members in matrix["split"].items():
        for member in members:
            assert member not in seen, (member, seen.get(member), name)
            seen[member] = name


def test_the_heldout_split_is_exactly_the_h_entries(matrix, by_id):
    held = matrix["split"]["heldout"]
    assert sorted(held) == sorted(s["id"] for s in _held(matrix))
    assert all(h in by_id for h in held)


def test_no_h_scenario_appears_in_a_tuning_split(matrix):
    """The whole point. `tune` is fitted against and `train` was; either one
    containing an H id would silently convert the headline number into a fit
    statistic."""
    for name in ("train", "tune", "test"):
        assert not [m for m in matrix["split"][name] if m.startswith("H")]


# -- disjointness from the data the detector was ever fitted on --------------

def test_no_held_out_positive_repeats_a_tuning_parameter_triple(matrix):
    """Disjoint on speed alone -- H uses 7/9/13/16/18/20, S/N use 4/8/12/14 and
    T uses 6/11/17 -- so this cannot be a re-run of tuning data even by a
    coincidence of angle and miss."""
    others = {(s.get("speed_mps"), s.get("approach_angle_deg"),
               s.get("miss_distance_m"))
              for s in matrix["scenarios"] if not s["id"].startswith("H")}
    for s in _held(matrix, "positive"):
        triple = (s["speed_mps"], s["approach_angle_deg"],
                  s["miss_distance_m"])
        assert triple not in others, (s["id"], triple)


def test_the_speed_axis_is_disjoint_from_every_other_split(matrix):
    h_speeds = {s["speed_mps"] for s in _held(matrix, "positive")}
    other = {s.get("speed_mps") for s in matrix["scenarios"]
             if not s["id"].startswith("H") and s["label"] == "positive"}
    assert not (h_speeds & other)


def test_the_speed_axis_reaches_20_mps(matrix):
    """20 m/s appears in no earlier split. A held-out set that stopped at
    17 m/s would leave the fastest scenario of interest unmeasured."""
    assert 20.0 in {s["speed_mps"] for s in _held(matrix, "positive")}


# -- the geometry amendments (A3) --------------------------------------------

def test_every_positive_stays_inside_the_vertical_sector(matrix):
    """Amendment A3(a). compensate_gravity keeps the HORIZONTAL speed at
    speed_mps and adds vz0 = 0.5*g*t, so the apex is g*t^2/8 at half the
    standoff and the worst elevation seen from the drone is
    atan(g*d / (4*v^2)). A row past 11 deg puts the ball out of frame in
    mid-flight, and that would score as a perception failure."""
    for s in _held(matrix, "positive"):
        v, d = s["speed_mps"], s["offset_forward_m"]
        elev = math.degrees(math.atan(G_MPS2 * d / (4.0 * v * v)))
        assert elev <= SECTOR_HALF_V_DEG, (s["id"], elev)


def test_the_loft_does_not_make_the_speed_label_a_lie(matrix):
    """The same bound read the other way: the ball's TRUE launch speed is
    hypot(v, vz0), and `speed_mps` has to stay an honest description of the
    projectile or every per-speed row is mislabelled."""
    for s in _held(matrix, "positive"):
        v, d = s["speed_mps"], s["offset_forward_m"]
        true_v = math.hypot(v, 0.5 * G_MPS2 * d / v)
        assert true_v / v <= 1.10, (s["id"], true_v)


def test_every_approach_angle_is_inside_the_horizontal_sector(matrix):
    """Amendment A3(b). +/-28 deg is outside +/-13.5 deg for the whole flight;
    keeping it would have designed four guaranteed misses into the set."""
    for s in _held(matrix, "positive"):
        assert abs(s["approach_angle_deg"]) <= SECTOR_HALF_H_DEG, s["id"]


def test_the_deleted_out_of_sector_case_survives_as_a_negative(matrix, by_id):
    """The sector limit is not hidden by A3(b) -- it moves to where a
    non-detection is the CORRECT answer. Without this row the amendment would
    have simply deleted an inconvenient case."""
    hn05 = by_id["HN05"]
    assert hn05["label"] == "negative"
    bearing = math.degrees(math.atan(hn05["miss_distance_m"]
                                     / hn05["offset_forward_m"]))
    assert bearing > SECTOR_HALF_H_DEG, bearing


def test_the_standoff_is_a_function_of_speed_and_nothing_else(matrix):
    """Offset co-varies with speed BY PHYSICS, per A3(a). One offset per speed
    keeps it a clean covariate; two offsets at one speed would make it a second
    uncontrolled variable that no result could separate."""
    by_speed = {}
    for s in _held(matrix, "positive"):
        by_speed.setdefault(s["speed_mps"], set()).add(s["offset_forward_m"])
    assert all(len(v) == 1 for v in by_speed.values()), by_speed


def test_every_positive_compensates_gravity(matrix):
    """A flat throw from 2 m altitude is on the ground after 0.639 s and about
    3 m of travel. On every one of these standoffs it would never reach the
    drone -- a geometry failure that would score as a perception failure on all
    18 positives at once."""
    for s in _held(matrix, "positive"):
        assert s.get("compensate_gravity") is True, s["id"]


# -- the axes are actually crossed, not just declared ------------------------

def test_the_axes_are_covered_rather_than_nominal(matrix):
    """A set that declares three axes and varies one is a set with 18 copies of
    the same scenario."""
    pos = _held(matrix, "positive")
    assert len({s["speed_mps"] for s in pos}) == 6
    assert len({s["approach_angle_deg"] for s in pos}) == 7
    assert len({s["miss_distance_m"] for s in pos}) == 4
    # Every speed carries the same number of scenarios, so no single speed
    # dominates the recall fraction.
    counts = {}
    for s in pos:
        counts[s["speed_mps"]] = counts.get(s["speed_mps"], 0) + 1
    assert len(set(counts.values())) == 1, counts


def test_the_angle_axis_is_symmetric(matrix):
    """An asymmetric angle axis would confound approach side with speed."""
    angles = {s["approach_angle_deg"] for s in _held(matrix, "positive")}
    assert {-a for a in angles} == angles


# -- derived fields agree with the parameters they are derived from ----------

def test_the_derived_fields_match_their_own_arithmetic(matrix):
    """time_to_closest_s anchors score_bags' strict detection window, so an
    entry whose derived fields drift from its parameters moves the window the
    result is measured in."""
    for s in _held(matrix):
        v, d = s["speed_mps"], s["offset_forward_m"]
        expect_ttc = (d / v) if v > 0 else 0.0
        assert s["time_to_closest_s"] == pytest.approx(expect_ttc, abs=1e-3)

        a = math.radians(s["approach_angle_deg"])
        expect_ca = (abs(d * math.sin(a) + s["miss_distance_m"] * math.cos(a))
                     if v > 0 else 0.0)
        assert s["closest_approach_m"] == pytest.approx(expect_ca, abs=1e-3)


def test_negatives_with_no_spawn_carry_zero_speed(matrix, by_id):
    """capture_scenario.sh decides whether to spawn at all from speed > 0. A
    clean-patrol negative with a nonzero speed would silently throw a ball."""
    for sid in ("HN01", "HN02", "HN03"):
        assert by_id[sid]["speed_mps"] == 0.0
