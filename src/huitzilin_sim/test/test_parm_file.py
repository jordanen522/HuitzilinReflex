"""Parsing and diffing .parm files.

The syntax rules here are CLAUDE.md sharp edges turned into assertions, so a
bad .parm fails in CI rather than on a flight controller.
"""

import pytest

from huitzilin_sim.parm_file import (
    Mismatch,
    ParmSyntaxError,
    diff_params,
    parse_parm,
)


def test_space_and_comma_forms_both_parse():
    """sitl_frame.parm is space-separated, huitzilin_3p5_6s.parm is comma."""
    assert parse_parm("FRAME_CLASS 1\nFRAME_TYPE 1\n") == {
        "FRAME_CLASS": 1.0, "FRAME_TYPE": 1.0}
    assert parse_parm("MOT_THST_EXPO,0.55\n") == {"MOT_THST_EXPO": 0.55}


def test_comment_only_and_blank_lines_are_skipped():
    text = "# header\n\n   \nFRAME_CLASS 1\n   # indented comment\n"
    assert parse_parm(text) == {"FRAME_CLASS": 1.0}


def test_inline_comment_is_rejected():
    """An inline comment breaks MAVProxy. Catch it here, not on the board."""
    with pytest.raises(ParmSyntaxError) as e:
        parse_parm("FRAME_CLASS 1  # quad-X\n")
    assert "MAVProxy" in str(e.value)


def test_duplicate_key_is_rejected():
    """The later value silently wins, which is how a safety parameter goes
    missing without anyone editing the line they were looking at."""
    with pytest.raises(ParmSyntaxError) as e:
        parse_parm("FENCE_ENABLE 1\nFENCE_ENABLE 0\n")
    assert "twice" in str(e.value)


def test_non_numeric_value_is_rejected():
    with pytest.raises(ParmSyntaxError):
        parse_parm("FENCE_ENABLE yes\n")


def test_malformed_line_is_rejected():
    with pytest.raises(ParmSyntaxError):
        parse_parm("FENCE_ENABLE\n")
    with pytest.raises(ParmSyntaxError):
        parse_parm("FENCE_ENABLE 1 2\n")


def test_names_are_normalised_to_upper_case():
    assert parse_parm("frame_class 1\n") == {"FRAME_CLASS": 1.0}


def test_negative_and_exponent_values_parse():
    assert parse_parm("A -1\nB 1e3\n") == {"A": -1.0, "B": 1000.0}


# --- diff_params -------------------------------------------------------------

def test_matching_readback_reports_nothing():
    assert diff_params({"A": 1.0}, {"A": 1.0}) == []


def test_missing_and_differing_are_reported_separately():
    out = diff_params({"A": 1.0, "B": 2.0}, {"A": 5.0})
    kinds = {m.name: m.kind for m in out}
    assert kinds == {"A": "differs", "B": "missing"}


def test_extra_live_parameters_are_ignored():
    """A real flight controller has hundreds of parameters this file does not
    speak for; reporting them would bury the ones that matter."""
    assert diff_params({"A": 1.0}, {"A": 1.0, "UNRELATED": 7.0}) == []


def test_tolerance_absorbs_float_readback_noise():
    assert diff_params({"A": 1.0}, {"A": 1.0 + 1e-9}) == []
    assert diff_params({"A": 1.0}, {"A": 1.1}) != []


def test_mismatch_renders_readably():
    assert "expected" in str(Mismatch("A", 1.0, 2.0))
