#!/usr/bin/env python3
"""
HuitzilinReflex — minimal, hardened pymavlink control bridge for ArduPilot SITL.

Pure pymavlink, no ROS dependency. The ROS 2 node (mav_bridge_node.py) wraps this.

Design rules:
  * NED inside, conversions exposed as static helpers (the ROS node converts to/from ENU).
  * Velocity setpoints use MAV_FRAME_BODY_OFFSET_NED  (velocity relative to heading).
  * Position setpoints use   MAV_FRAME_LOCAL_NED       (offset from EKF origin).
  * Caller (or the ROS watchdog) must re-send setpoints faster than ~3 s or AP stops.
"""
import argparse
import math
import time
from pymavlink import mavutil

# --- type_mask bitfields for SET_POSITION_TARGET_LOCAL_NED ---------------------
# Bit (0-indexed): 0..2 Pos x/y/z, 3..5 Vel x/y/z, 6..8 Acc x/y/z,
#                  9 force, 10 yaw, 11 yaw_rate.  A SET bit = "ignore this field".
MASK_VEL_ONLY = 0b0000111111000111   # use velocity x/y/z only            (4039)
MASK_POS_ONLY = 0b0000111111111000   # use position x/y/z only            (4088)
MASK_POS_YAW  = 0b0000101111111000   # use position x/y/z + yaw           (clears yaw bit)


class MavBridge:
    def __init__(self, connect="udp:127.0.0.1:14550", source_system=255):
        self.conn_str = connect
        self.master = mavutil.mavlink_connection(connect, source_system=source_system)
        self.target_system = 0
        self.target_component = 0

    # -- lifecycle -------------------------------------------------------------
    def connect(self, timeout=30):
        """Wait for the first heartbeat and latch the target ids."""
        print(f"[bridge] connecting on {self.conn_str} ...")
        hb = self.master.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise TimeoutError("no heartbeat — is SITL running and the endpoint right?")
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        print(f"[bridge] heartbeat: sys={self.target_system} comp={self.target_component}")

    def request_streams(self, rate_hz=10):
        """Ask ArduPilot to emit the telemetry we need at a fixed rate."""
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS):
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)

    def wait_ekf_ready(self, timeout=60):
        """Block until the EKF/GPS is healthy enough to arm in GUIDED (SITL: quick)."""
        print("[bridge] waiting for EKF/GPS ...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT",
                                         blocking=True, timeout=2)
            if msg and msg.lat != 0:
                print("[bridge] EKF/GPS ready")
                return True
        raise TimeoutError("EKF/GPS never became ready")

    # -- mode / arm / takeoff --------------------------------------------------
    def set_mode(self, mode_name="GUIDED", timeout=10):
        mode_name = mode_name.upper()
        mapping = self.master.mode_mapping()
        if mode_name not in mapping:
            raise ValueError(f"unknown mode {mode_name}; have {list(mapping)}")
        self.master.set_mode(mapping[mode_name])
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and hb.custom_mode == mapping[mode_name]:
                print(f"[bridge] mode = {mode_name}")
                return True
        raise TimeoutError(f"mode {mode_name} not confirmed")

    def arm(self, arm=True, timeout=10):
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if arm else 0, 0, 0, 0, 0, 0, 0)
        if arm:
            self.master.motors_armed_wait()
            print("[bridge] ARMED")
        else:
            self.master.motors_disarmed_wait()
            print("[bridge] disarmed")
        return True

    def takeoff(self, alt_m, timeout=90):
        """GUIDED takeoff. Must be armed and in GUIDED first.

        NOTE: timeout is WALL-clock. Gazebo headless on Iris Xe runs at ~24%
        real-time, so 90 s wall ≈ 22 s sim — enough for a slow climb to 2 m.
        Returns as soon as altitude is reached, so the generous bound is free.
        """
        self.master.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, float(alt_m))
        print(f"[bridge] takeoff -> {alt_m} m")
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type="LOCAL_POSITION_NED",
                                         blocking=True, timeout=2)
            if msg and -msg.z >= 0.95 * alt_m:   # NED: down is +z, so altitude = -z
                print(f"[bridge] reached {alt_m} m")
                return True
        raise TimeoutError("takeoff altitude not reached")

    # -- setpoints -------------------------------------------------------------
    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        """Body-frame velocity (m/s) + yaw rate (rad/s). x fwd, y right, z down."""
        mask = MASK_VEL_ONLY
        if yaw_rate != 0.0:
            mask &= ~(1 << 11)        # un-ignore yaw_rate
        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            mask,
            0, 0, 0,                  # x, y, z position (ignored)
            vx, vy, vz,               # velocity
            0, 0, 0,                  # acceleration (ignored)
            0, yaw_rate)              # yaw, yaw_rate

    def send_position_ned(self, north, east, down, yaw=None):
        """Absolute position setpoint (m) in LOCAL_NED, offset from EKF origin."""
        mask = MASK_POS_ONLY if yaw is None else MASK_POS_YAW
        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            north, east, down,
            0, 0, 0,
            0, 0, 0,
            0.0 if yaw is None else yaw, 0)

    # -- telemetry -------------------------------------------------------------
    def get_state(self):
        """Non-blocking snapshot of pose + velocity in NED."""
        lp = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        att = self.master.recv_match(type="ATTITUDE", blocking=False)
        state = {}
        if lp:
            state.update(dict(n=lp.x, e=lp.y, d=lp.z, vn=lp.vx, ve=lp.vy, vd=lp.vz))
        if att:
            state.update(dict(roll=att.roll, pitch=att.pitch, yaw=att.yaw))
        return state

    # -- frame helpers (NED <-> ENU) used by the ROS node ---------------------
    @staticmethod
    def ned_to_enu(n, e, d):
        return (e, n, -d)             # ENU x=East, y=North, z=Up

    @staticmethod
    def enu_to_ned(x, y, z):
        return (y, x, -z)


# --- standalone self-test: arm, takeoff, nudge, land --------------------------
def _selftest(connect):
    b = MavBridge(connect)
    b.connect()
    b.request_streams(10)
    b.wait_ekf_ready()
    b.set_mode("GUIDED")
    b.arm(True)
    b.takeoff(2.0)

    print("[selftest] commanding 1.0 m/s body-forward for 3 s; measuring vx...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        b.send_velocity_body(1.0, 0.0, 0.0)   # MUST re-send < 3 s; we do it at 10 Hz
        s = b.get_state()
        if "vn" in s:
            speed = math.hypot(s["vn"], s["ve"])
            print(f"  t={time.time()-t0:4.1f}s measured horiz speed={speed:4.2f} m/s")
        time.sleep(0.1)

    print("[selftest] holding, then LAND")
    b.send_velocity_body(0.0, 0.0, 0.0)
    b.set_mode("LAND")
    print("[selftest] done.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udp:127.0.0.1:14550")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        _selftest(args.connect)
    else:
        b = MavBridge(args.connect)
        b.connect()
        print("connected; import this module to use MavBridge.")
