#!/usr/bin/env python3
"""Compare a flight controller's live parameters against hw_frame.parm.

Thin wrapper: the parsing and comparison live in huitzilin_sim.parm_file and
are unit-tested there, so this script does not reimplement them in bash or in
an untested copy.

  ./scripts/hw_param_readback.py
  ./scripts/hw_param_readback.py --connection /dev/ttyACM0 --expected other.parm

Exit codes: 0 all parameters match, 1 mismatches, 2 could not reach the FC.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "huitzilin_sim"))

from huitzilin_sim.parm_file import diff_params, parse_parm  # noqa: E402

DEFAULT_BAUD = 115200          # matches preflight_hw.sh's heartbeat check

DEFAULT_EXPECTED = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "huitzilin_sim", "params", "hw_frame.parm")


def fetch(connection, timeout_s, baud=DEFAULT_BAUD):
    from pymavlink import mavutil

    # baud is explicit: mavutil defaults to 57600, while preflight_hw.sh's own
    # heartbeat check uses 115200. Mismatched, this reported a dead FC on a
    # perfectly good link. Ignored for udp:/tcp: connection strings.
    master = mavutil.mavlink_connection(connection, baud=baud)
    # finally, because all three exits mattered: the timeout raise, the early
    # break, and the normal return each left a serial port claimed. On
    # /dev/ttyACM0 that makes the *next* tool report a dead flight controller.
    try:
        if master.wait_heartbeat(timeout=timeout_s) is None:
            raise TimeoutError("no heartbeat on %s" % connection)
        master.mav.param_request_list_send(master.target_system,
                                           master.target_component)
        live = {}
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
            if msg is None:
                break
            live[msg.param_id.strip("\x00").upper()] = float(msg.param_value)
            if msg.param_count and len(live) >= msg.param_count:
                break
        return live
    finally:
        try:
            master.close()
        except Exception:                # noqa: BLE001 — teardown must not raise
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--connection", default="/dev/ttyACM0")
    ap.add_argument("--expected", default=DEFAULT_EXPECTED)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args()

    with open(args.expected) as fh:
        expected = parse_parm(fh.read())

    try:
        live = fetch(args.connection, args.timeout, args.baud)
    except Exception as e:
        print("ERROR: could not read parameters: %s" % e)
        return 2

    problems = diff_params(expected, live)
    if not problems:
        print("OK: all %d expected parameters match" % len(expected))
        return 0

    print("MISMATCH: %d of %d expected parameters" % (len(problems), len(expected)))
    for m in problems:
        print("  %s" % m)
    print("")
    print("Load the file rather than fixing these by hand:")
    print("  MAVProxy: param load %s" % os.path.relpath(args.expected))
    return 1


if __name__ == "__main__":
    sys.exit(main())
