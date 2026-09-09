"""This subsystem cannot reach the projectile pipeline or the flight stack.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_action_escalation/test/test_isolation.py

The check is on STRING CONSTANTS IN CODE, not on raw file text. Several files
here name /payload/alarm in a docstring, on purpose, to record why the alert
is not published there -- and a test that banned the words would delete the
explanation while permitting the mistake. Docstrings are excluded; every
other string literal is checked, which is where a topic name has to appear to
do any harm.
"""

import ast
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
SRC = PKG.parent
PY_FILES = sorted(list((PKG / "huitzilin_action_escalation").glob("*.py"))
                  + list((PKG / "launch").glob("*.py")))
YAML_FILES = sorted(list((PKG / "params").glob("*.yaml"))
                    + list((PKG / "scenarios").glob("*.yaml")))

# Publishing on any of these, or calling them, is how this subsystem could
# move the aircraft. /payload/alarm is on the list because supervisor.py
# transitions PATROL -> EVADE on it, and that is the only edge into EVADE in
# the whole state machine.
FORBIDDEN_TOPICS = (
    "/cmd/evade", "/cmd/evade_accel", "cmd_vel", "/payload/alarm",
    "/huitzilin/arm", "/huitzilin/takeoff", "/huitzilin/set_mode",
    "/huitzilin/start_patrol", "/threat/centroid", "/threat/cue",
    "/oak/points", "/gz/dynamic_poses",
)

FORBIDDEN_IMPORTS = ("geometry_msgs", "std_srvs", "sensor_msgs",
                     "rcl_interfaces", "huitzilin_perception", "pymavlink")


def code_strings(source: str):
    """Every string constant that is not a docstring."""
    tree = ast.parse(source)
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            body = getattr(node, "body", None)
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and id(n) not in docstrings]


def imported_modules(source: str):
    names = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            names.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_the_file_scan_is_not_empty():
    """Guards every check below. An empty glob would pass all of them."""
    assert len(PY_FILES) >= 8, [p.name for p in PY_FILES]
    assert len(YAML_FILES) >= 8, [p.name for p in YAML_FILES]


def test_the_string_scan_ignores_docstrings_but_catches_real_constants():
    """Positive control. Without this the AST walk could return nothing and
    every isolation check below would pass vacuously."""
    source = ('"""A docstring naming /cmd/evade harmlessly."""\n'
              'topic = "/cmd/evade"\n')
    found = code_strings(source)
    assert found == ["/cmd/evade"]
    assert code_strings('"""Only /payload/alarm in a docstring."""\n') == []


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_no_source_file_names_a_flight_or_projectile_topic_in_code(path):
    strings = code_strings(path.read_text(encoding="utf-8"))
    for text in strings:
        for topic in FORBIDDEN_TOPICS:
            assert topic not in text, (
                "%s has the string %r in code, which would let this "
                "subsystem reach the projectile or flight stack"
                % (path.name, text))


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_no_source_file_imports_a_flight_or_projectile_package(path):
    for module in imported_modules(path.read_text(encoding="utf-8")):
        root = module.split(".")[0]
        assert root not in FORBIDDEN_IMPORTS, (
            "%s imports %s. Without geometry_msgs there is no Twist and no "
            "evade command; without std_srvs there is no arm or takeoff "
            "client. Keeping them out is the isolation guarantee."
            % (path.name, module))


@pytest.mark.parametrize("path", YAML_FILES, ids=lambda p: p.name)
def test_no_config_file_points_at_a_flight_or_projectile_topic(path):
    text = path.read_text(encoding="utf-8")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            continue          # the yaml headers explain the boundary
        for topic in FORBIDDEN_TOPICS:
            assert topic not in line, "%s: %s" % (path.name, stripped)


def test_the_manifest_declares_no_flight_dependency():
    """The strongest guarantee available: a service client to /huitzilin/arm
    cannot be written without editing package.xml, and a manifest edit shows
    up in every diff."""
    manifest = (PKG / "package.xml").read_text(encoding="utf-8")
    for dep in ("geometry_msgs", "std_srvs", "sensor_msgs",
                "huitzilin_perception"):
        assert "<depend>%s</depend>" % dep not in manifest
        assert "<exec_depend>%s</exec_depend>" % dep not in manifest


def test_neither_existing_package_depends_on_this_one():
    """Isolation runs both ways. If the projectile packages imported this
    one, a change here could alter their behaviour."""
    for pkg in ("huitzilin_sim", "huitzilin_perception"):
        root = SRC / pkg
        if not root.is_dir():
            pytest.skip("%s not present in this checkout" % pkg)
        for path in root.rglob("*.py"):
            assert "huitzilin_action_escalation" not in path.read_text(
                encoding="utf-8"), path
        assert "huitzilin_action_escalation" not in (
            root / "package.xml").read_text(encoding="utf-8")


def test_the_manifest_is_well_formed_and_declares_ament_python():
    """A malformed package.xml does not fail the build; it downgrades it.

    An XML comment containing a double hyphen makes the manifest unparseable.
    colcon then falls back to build type 'python' instead of 'ament_python',
    silently skips the ament_prefix_path environment hook, and the package
    installs cleanly while remaining invisible to `ros2 launch` and
    `ros2 run`. Everything reports success; nothing can find the package.
    """
    import xml.dom.minidom

    path = PKG / "package.xml"
    xml.dom.minidom.parse(str(path))       # raises if not well-formed
    text = path.read_text(encoding="utf-8")
    assert "<build_type>ament_python</build_type>" in text

    body = "\n".join(line for line in text.splitlines()
                     if "<!--" not in line and "-->" not in line)
    assert "--" not in body, (
        "a double hyphen inside an XML comment makes package.xml malformed, "
        "which downgrades the build type without failing the build")
