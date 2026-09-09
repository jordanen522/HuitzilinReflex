"""This subsystem cannot reach the projectile pipeline or the flight stack.

Runs without ROS (stdlib only):
    python -m pytest src/huitzilin_guard/test/test_isolation.py

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
PY_FILES = sorted(list((PKG / "huitzilin_guard").glob("*.py"))
                  + list((PKG / "launch").glob("*.py")))
YAML_FILES = sorted((PKG / "params").glob("*.yaml"))

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

# sensor_msgs is NOT here: pose_detector_node needs Image to read a camera.
# nav_msgs is not here either: guard_node needs Odometry to know where the
# drone is, and odometry is telemetry in, not command out. Allowing a message
# package does not loosen the real guarantee, which is the topic ban above --
# /oak/points remains forbidden by name, so the projectile depth cloud still
# cannot be subscribed from this package.
FORBIDDEN_IMPORTS = ("geometry_msgs", "rcl_interfaces",
                     "huitzilin_perception", "pymavlink")

# std_srvs WAS on the list above and is not any more, because the guard panel
# has to offer an arm switch and SetBool is how. The permission is narrowed
# to the one file that serves that switch, exactly as cv2 is narrowed to the
# one file that holds an image; a package-wide allowance would give the
# property up rather than relocate it.
SERVICE_FILE = "guard_node.py"

# The replacement for the old blanket ban, and a sharper rule than it was. A
# service SERVER cannot move the aircraft; a service CLIENT can, by calling
# the flight stack's arm, takeoff, set_mode or start_patrol. std_srvs is
# needed for both, so the check moved from the import to the call.
FORBIDDEN_CALLS = ("create_client",)

# huitzilin_sim as a whole is permitted: clock_guard and box live there. This
# one module inside it is the flight stack itself -- it owns the MAVLink
# connection and the setpoint senders -- so importing it would hand this
# package direct command of the aircraft without touching package.xml at all.
FORBIDDEN_SUBMODULES = ("huitzilin_sim.mav_bridge",)


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
    assert len(YAML_FILES) >= 3, [p.name for p in YAML_FILES]


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
            "evade command. Keeping these out is the isolation guarantee."
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
    """A manifest edit shows up in every diff, which is what makes this the
    strongest check available. geometry_msgs is the one that matters most:
    without Twist there is no velocity command to publish anywhere."""
    manifest = (PKG / "package.xml").read_text(encoding="utf-8")
    for dep in ("geometry_msgs", "huitzilin_perception"):
        assert "<depend>%s</depend>" % dep not in manifest
        assert "<exec_depend>%s</exec_depend>" % dep not in manifest


def test_std_srvs_is_declared_and_its_replacement_rules_are_live():
    """std_srvs is permitted now; the guarantee it carried is not gone.

    Asserted together so that dropping the dependency cannot make the three
    narrower tests below pass vacuously instead of failing loudly.
    """
    manifest = (PKG / "package.xml").read_text(encoding="utf-8")
    assert "<depend>std_srvs</depend>" in manifest
    assert any(p.name == SERVICE_FILE for p in PY_FILES)


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_only_the_arm_service_file_imports_std_srvs(path):
    """Guards the exemption. A second file importing std_srvs would quietly
    turn a one-file permission into a package-wide one."""
    if path.name == SERVICE_FILE:
        return
    roots = {m.split(".")[0]
             for m in imported_modules(path.read_text(encoding="utf-8"))}
    assert "std_srvs" not in roots, (
        "%s imports std_srvs. Only %s may, and only to SERVE the arm switch."
        % (path.name, SERVICE_FILE))


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_no_source_file_creates_a_service_client(path):
    """This subsystem answers requests; it does not make them.

    create_client is the one call that would let the guard reach the flight
    stack's arm, takeoff, set_mode or start_patrol services. Banning the call
    rather than the message package is what let std_srvs in without giving up
    the guarantee.
    """
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Attribute):
            assert node.attr not in FORBIDDEN_CALLS, (
                "%s calls %s. This subsystem may drive lights and a siren; "
                "it may not call the flight stack." % (path.name, node.attr))


def test_the_service_client_ban_is_not_vacuous():
    """Positive control for the AST walk above."""
    found = [n.attr for n in ast.walk(ast.parse("self.create_client(X, 'y')"))
             if isinstance(n, ast.Attribute)]
    assert "create_client" in found


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: p.name)
def test_no_source_file_imports_the_mavlink_bridge(path):
    """huitzilin_sim is allowed; the MAVLink bridge inside it is not.

    clock_guard and box are shared utilities with no flight capability.
    mav_bridge owns the connection and the setpoint senders, so importing it
    would let this package command the aircraft directly.
    """
    source = path.read_text(encoding="utf-8")
    for module in imported_modules(source):
        for banned in FORBIDDEN_SUBMODULES:
            assert not module.startswith(banned), (
                "%s imports %s" % (path.name, module))
    for text in code_strings(source):
        assert "MavBridge" not in text, path.name


def test_neither_existing_package_depends_on_this_one():
    """Isolation runs both ways. If the projectile packages imported this
    one, a change here could alter their behaviour."""
    for pkg in ("huitzilin_sim", "huitzilin_perception"):
        root = SRC / pkg
        if not root.is_dir():
            pytest.skip("%s not present in this checkout" % pkg)
        for path in root.rglob("*.py"):
            # IMPORTS, not raw text. huitzilin_sim/box.py explains in its
            # docstring why it lives there rather than in the guard package,
            # and a raw-text ban would delete that explanation while still
            # permitting the import it warns about.
            for module in imported_modules(path.read_text(encoding="utf-8")):
                assert not module.startswith("huitzilin_guard"), path
        assert "huitzilin_guard" not in (
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
