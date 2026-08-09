#!/usr/bin/env python3
"""Set and dump live ArduPilot parameters on the ISOLATED lab SITL.

Complements scripts/hw_param_readback.py, which reads and diffs but has no
write path (its main() is fetch() then diff_params()). Reading/diffing is NOT
reimplemented here -- use that script for verification.

A set is not believed until it is read back. ArduPilot silently ignores a
PARAM_SET for a name that does not exist in the build, and CLAUDE.md records a
whole invalid experiment that came from exactly that (PSC_ACC_XY / WPNAV_ACCEL
/ ANGLE_MAX do not exist here). So an unknown name is an ERROR, never a no-op.

  lab_param_set.py --dump before.parm
  lab_param_set.py --set PSC_JERK_NE=20 --set ATC_ANGLE_MAX=45
  lab_param_set.py --get PSC_JERK_NE ATC_ANGLE_MAX

Exit codes: 0 ok, 1 a set or verify failed, 2 could not reach the vehicle.
"""

from __future__ import annotations

import argparse
import sys
import time

DEFAULT_CONN = "tcp:127.0.0.1:5762"   # 5760 is MAVProxy's; 5762/5763 are spare


def connect(conn: str, timeout_s: float):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(conn)
    if m.wait_heartbeat(timeout=timeout_s) is None:
        raise TimeoutError("no heartbeat on %s" % conn)
    return m


def fetch_all(m, timeout_s: float) -> dict:
    m.mav.param_request_list_send(m.target_system, m.target_component)
    live: dict = {}
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            break
        live[msg.param_id.strip("\x00").upper()] = float(msg.param_value)
        if msg.param_count and len(live) >= msg.param_count:
            break
    return live


def get_one(m, name: str, timeout_s: float = 5.0):
    """Read one parameter, or None if the build does not have it."""
    name = name.upper()
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.param_id.strip("\x00").upper() == name:
            return float(msg.param_value)
    return None


def set_one(m, name: str, value: float, tol: float = 1e-4,
            attempts: int = 4) -> float:
    """Set a parameter and return the READ-BACK value. Raises if it did not take."""
    name = name.upper()
    before = get_one(m, name)
    if before is None:
        raise KeyError(
            "%s does not exist in this build -- a PARAM_SET for it would be "
            "silently ignored (see CLAUDE.md on the invalid PSC_ACC_XY run)"
            % name)
    for _ in range(attempts):
        m.param_set_send(name, float(value))
        time.sleep(0.6)
        after = get_one(m, name)
        if after is not None and abs(after - float(value)) <= tol:
            return after
    raise ValueError("%s did not take: wanted %r, still reads %r"
                     % (name, float(value), get_one(m, name)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connection", default=DEFAULT_CONN)
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--dump", metavar="FILE",
                    help="write the whole table as NAME VALUE lines")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--get", nargs="*", default=[], metavar="NAME")
    args = ap.parse_args()

    try:
        m = connect(args.connection, args.timeout)
    except Exception as e:                      # noqa: BLE001
        print("ERROR: %s" % e)
        return 2

    rc = 0
    try:
        for spec in args.set:
            if "=" not in spec:
                print("ERROR: --set wants NAME=VALUE, got %r" % spec)
                rc = 1
                continue
            name, _, raw = spec.partition("=")
            try:
                before = get_one(m, name.strip())
                after = set_one(m, name.strip(), float(raw))
                print("SET %s: %r -> %r" % (name.strip().upper(), before, after))
            except Exception as e:              # noqa: BLE001
                print("FAILED %s: %s" % (name.strip().upper(), e))
                rc = 1

        for name in args.get:
            v = get_one(m, name)
            print("GET %s = %r" % (name.upper(), v))
            if v is None:
                rc = 1

        if args.dump:
            live = fetch_all(m, args.timeout)
            with open(args.dump, "w") as fh:
                for k in sorted(live):
                    fh.write("%s %.6f\n" % (k, live[k]))
            print("DUMPED %d parameters -> %s" % (len(live), args.dump))
            if len(live) < 1000:
                print("WARNING: only %d parameters -- table looks truncated"
                      % len(live))
                rc = 1
    finally:
        try:
            m.close()
        except Exception:                       # noqa: BLE001
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
