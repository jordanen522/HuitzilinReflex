"""Consistency checks over the parameter files, run as unit tests.

The hardware overlays cannot be validated by flying yet, so the things that
would otherwise be caught only by a crash are asserted here instead: that an
overlay's node name matches the file it overlays, that its keys exist in the
file it overrides, and that the geofence parameters do not contradict each
other.
"""

from __future__ import annotations

import re

# ROS 2 wildcard keys address whatever node loads the file, so there is no name
# to bind them to.
WILDCARD_NODE_KEYS = frozenset({"/**", "**", "*", "/*"})

_NODE_NAME_RE = re.compile(r'super\(\)\.__init__\(\s*["\']([A-Za-z_][A-Za-z0-9_]*)["\']')

# RTL_ALT is centimetres. FENCE_ALT_MAX is metres. Comparing them without this
# conversion is the whole trap: a plausible-looking RTL_ALT of 4 means 4 cm.
RTL_ALT_CM_PER_M = 100.0


def flatten_ros_params(doc) -> dict[str, dict]:
    """{node_name: {param: value}} from a parsed ROS 2 params yaml."""
    out = {}
    for node, body in (doc or {}).items():
        if isinstance(body, dict) and isinstance(body.get("ros__parameters"), dict):
            out[node] = body["ros__parameters"]
    return out


def check_overlay(base_doc, overlay_doc) -> list[str]:
    """Problems with `overlay_doc` used as a hardware overlay on `base_doc`.

    Two failure modes, both silent in ROS 2 at runtime. A node name that does
    not match means the whole overlay loads nothing at all. A parameter name
    that does not exist in the base means a value someone believes is being
    applied is not declared anywhere.
    """
    base = flatten_ros_params(base_doc)
    overlay = flatten_ros_params(overlay_doc)
    problems = []

    for node, params in overlay.items():
        if node not in base:
            problems.append(
                "node %r is not in the base file (ROS 2 would load none of its "
                "%d parameters); base has %s"
                % (node, len(params), sorted(base)))
            continue
        unknown = sorted(set(params) - set(base[node]) - {"use_sim_time"})
        if unknown:
            problems.append(
                "node %r overlays parameters absent from the base: %s"
                % (node, unknown))
    return problems


def declared_node_names(source: str) -> set:
    """Node names a Python source file passes to `super().__init__("...")`."""
    return set(_NODE_NAME_RE.findall(source))


def check_node_names(doc, declared) -> list[str]:
    """Problems with a params yaml's node keys against the names nodes declare.

    check_overlay only compares an overlay against its base file, so a typo
    present in BOTH passes -- and in ROS 2 a params file addressed to a node
    name that no node uses loads absolutely nothing, silently. This is the
    check that binds the yaml to the Python.
    """
    declared = set(declared)
    problems = []
    for node in flatten_ros_params(doc):
        if node in WILDCARD_NODE_KEYS:
            continue
        if node not in declared:
            problems.append(
                "node %r is not declared by any node (ROS 2 would load none of "
                "its parameters); declared names are %s"
                % (node, sorted(declared)))
    return problems


def check_fence_consistency(parms: dict) -> list[str]:
    """Problems with a .parm file's geofence and failsafe set.

    The headline check is RTL_ALT against FENCE_ALT_MAX. SAFETY_CASE.md sets a
    5 m fence ceiling and separately describes RTL climbing to RTL_ALT, whose
    ArduPilot default is 15 m -- so an RTL triggered by a fence breach would
    climb straight through the ceiling it is responding to.
    """
    problems = []

    def want(name, value):
        if name not in parms:
            problems.append("%s is not set" % name)
        elif abs(parms[name] - value) > 1e-6:
            problems.append("%s is %g, expected %g" % (name, parms[name], value))

    want("FRAME_CLASS", 1)     # 0 arms and accepts takeoff with zero lift
    want("FRAME_TYPE", 1)
    want("FENCE_ENABLE", 1)
    want("FENCE_TYPE", 3)      # circle + altitude
    want("FENCE_RADIUS", 10)
    want("FENCE_ALT_MAX", 5)
    want("FENCE_ACTION", 1)    # RTL
    want("FS_THR_ENABLE", 1)
    want("BATT_FS_LOW_ACT", 2)  # RTL

    if "RTL_ALT" not in parms:
        problems.append(
            "RTL_ALT is not set; ArduPilot defaults to 15 m, which is above "
            "the 5 m fence ceiling")
    elif "FENCE_ALT_MAX" in parms:
        rtl_m = parms["RTL_ALT"] / RTL_ALT_CM_PER_M
        if rtl_m >= parms["FENCE_ALT_MAX"]:
            problems.append(
                "RTL_ALT %g cm = %g m is not below FENCE_ALT_MAX %g m; a "
                "fence-breach RTL would climb through the ceiling it is "
                "responding to" % (parms["RTL_ALT"], rtl_m, parms["FENCE_ALT_MAX"]))

    if parms.get("ARMING_CHECK") == 0:
        problems.append(
            "ARMING_CHECK 0 hides the pre-arm message that names the actual "
            "fault")

    if not any(name.startswith("RC") and name.endswith("_OPTION")
               and value == 31
               for name, value in parms.items()):
        problems.append(
            "no RC*_OPTION is set to 31 (motor emergency stop); the "
            "kill-switch would not be mapped")

    return problems
