#!/usr/bin/env python3
"""
HuitzilinReflex — minimal, hardened pymavlink control bridge for ArduPilot.

Pure pymavlink, no ROS dependency. The ROS 2 node (mav_bridge_node.py) wraps this.
The same code talks to SITL (behind MAVProxy --out) and to the real flight
controller (behind mavlink-router); both hand it a UDP endpoint.

Design rules:
  * NED inside, conversions exposed as static helpers (the ROS node converts to/from ENU).
  * Velocity setpoints use MAV_FRAME_BODY_OFFSET_NED  (velocity relative to heading).
  * Position setpoints use   MAV_FRAME_LOCAL_NED       (offset from EKF origin).
  * Caller (or the ROS watchdog) must re-send setpoints faster than ~3 s or AP stops.
  * Every read of the link goes through get_state(), under one lock. The
    blocking helpers (set_mode, arm, takeoff) poll that shared state instead of
    reading the socket themselves, so the node keeps publishing telemetry from
    another thread while a service call waits.
"""
import math
import threading
import time
from pymavlink import mavutil

# type_mask bitfields for SET_POSITION_TARGET_LOCAL_NED
# Bit (0-indexed): 0..2 Pos x/y/z, 3..5 Vel x/y/z, 6..8 Acc x/y/z,
#                  9 force, 10 yaw, 11 yaw_rate.  A SET bit = "ignore this field".
MASK_VEL_ONLY  = 0b0000111111000111  # use velocity x/y/z only            (4039)
MASK_VEL_ACCEL = 0b0000111000000111  # use velocity AND acceleration      (3591)
MASK_POS_ONLY  = 0b0000111111111000  # use position x/y/z only            (4088)
MASK_POS_YAW   = 0b0000101111111000  # use position x/y/z + yaw           (clears yaw bit)

# Max MAVLink messages get_state() drains per call. Comfortably above one
# stream period's worth at 15 Hz telemetry, but bounded so a backlog cannot
# stall the caller's timer tick.
_DRAIN_MAX_MSGS = 200

# How often the blocking helpers re-check the shared state.
_POLL_S = 0.05

# ArduPilot signals "no reading" with sentinels rather than omitting the field.
# 65535 mV would read as a 65.5 V pack and -1% as a real charge level, so both
# have to be mapped to None before anything compares them to a threshold.
_BATT_MV_UNKNOWN = 65535
_BATT_PCT_UNKNOWN = -1

# MAV_STATE at or beyond CRITICAL means the flight controller has itself
# decided something is wrong. That is a failsafe, not a mode change.
_MAV_STATE_CRITICAL = 5


class MavBridge:
    """Deliberately ROS-free pymavlink wrapper, so it can be unit-tested.

    Owns the ONLY NED<->ENU conversion in the codebase (ned_to_enu /
    enu_to_ned). Velocity setpoints go out as MAV_FRAME_BODY_OFFSET_NED,
    positions as MAV_FRAME_LOCAL_NED.
    """

    def __init__(self, connect="udpin:0.0.0.0:14552", source_system=255, log=print):
        self.conn_str = connect
        self.log = log
        self.master = mavutil.mavlink_connection(connect, source_system=source_system)
        self.target_system = 0
        self.target_component = 0
        self._state = {}   # last-known telemetry (recv_match is lossy per tick)
        # Re-entrant: the blocking helpers send under it and then poll
        # get_state(), which takes it again.
        self._io = threading.RLock()

    @classmethod
    def offline(cls, master, target_system=1, log=print):
        """A bridge over an already-built link object, for tests: __init__
        would open a real socket."""
        b = cls.__new__(cls)
        b.conn_str = "offline"
        b.log = log
        b.master = master
        b.target_system = target_system
        b.target_component = 1
        b._state = {}
        b._io = threading.RLock()
        return b

    # lifecycle
    def connect(self, timeout=30):
        """Wait for the first heartbeat and latch the target ids."""
        self.log(f"[bridge] connecting on {self.conn_str} ...")
        hb = self.master.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise TimeoutError(
                "no heartbeat on %s -- is SITL (with --out to this port) or "
                "mavlink-router (hardware) running?" % self.conn_str)
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        self.log(f"[bridge] heartbeat: sys={self.target_system} comp={self.target_component}")

    def request_streams(self, rate_hz=10):
        """Ask ArduPilot to emit the telemetry we need at a fixed rate."""
        with self._io:
            for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                           mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                           mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                           mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS):
                self.master.mav.command_long_send(
                    self.target_system, self.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)

    def send_heartbeat(self):
        """Announce the companion computer. MAVProxy did this in SITL; behind
        mavlink-router nothing does. It is what lets the flight controller's
        GCS failsafe (FS_GCS_ENABLE) notice a dead companion."""
        with self._io:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)

    def _wait_for(self, predicate, timeout):
        """Poll the shared telemetry until predicate(state) holds.

        WALL-clock: these waits run before or outside any flight to time, and
        a sim-time bound would itself stall if /clock never advanced. Bounded,
        because pymavlink's own motors_armed_wait() loops forever on a refused
        arm, which on a service callback stalls whatever shares its thread.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate(self.get_state()):
                return True
            time.sleep(_POLL_S)
        return False

    # mode / arm / takeoff
    def set_mode(self, mode_name="GUIDED", timeout=10):
        mode_name = mode_name.upper()
        mapping = self.master.mode_mapping()
        if mode_name not in mapping:
            raise ValueError(f"unknown mode {mode_name}; have {list(mapping)}")
        with self._io:
            self.master.set_mode(mapping[mode_name])
        if not self._wait_for(lambda s: s.get("mode") == mode_name, timeout):
            raise TimeoutError(f"mode {mode_name} not confirmed")
        self.log(f"[bridge] mode = {mode_name}")
        return True

    def arm(self, arm=True, timeout=10):
        # param2=0. NOT a force-arm: ArduPilot's force-arm magic value is 21196,
        # and forcing is forbidden here anyway (it hides the frame/EKF fault you
        # actually need to see).
        with self._io:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1 if arm else 0, 0, 0, 0, 0, 0, 0)
        if not self._wait_for(lambda s: s.get("armed") is bool(arm), timeout):
            raise TimeoutError(
                "%s not confirmed within %.0f s -- check PreArm messages "
                "(FRAME_CLASS/FRAME_TYPE, EKF)"
                % ("arm" if arm else "disarm", timeout))
        self.log("[bridge] ARMED" if arm else "[bridge] disarmed")
        return True

    def takeoff(self, alt_m, timeout=90):
        """GUIDED takeoff. Must be armed and in GUIDED first.

        NOTE: timeout is WALL-clock. Gazebo headless on Iris Xe runs at ~24%
        real-time, so 90 s wall ≈ 22 s sim — enough for a slow climb to 2 m.
        Returns as soon as altitude is reached, so the generous bound is free.
        """
        with self._io:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, float(alt_m))
        self.log(f"[bridge] takeoff -> {alt_m} m")
        # NED: down is +z, so altitude = -d.
        if not self._wait_for(lambda s: "d" in s and -s["d"] >= 0.95 * alt_m, timeout):
            raise TimeoutError("takeoff altitude not reached")
        self.log(f"[bridge] reached {alt_m} m")
        return True

    # setpoints
    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        """Body-frame velocity (m/s) + yaw rate (rad/s). x fwd, y right, z down."""
        mask = MASK_VEL_ONLY
        if yaw_rate != 0.0:
            mask &= ~(1 << 11)        # un-ignore yaw_rate
        with self._io:
            self.master.mav.set_position_target_local_ned_send(
                0, self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                mask,
                0, 0, 0,                  # x, y, z position (ignored)
                vx, vy, vz,               # velocity
                0, 0, 0,                  # acceleration (ignored)
                0, yaw_rate)              # yaw, yaw_rate

    def send_velocity_accel_body(self, vx, vy, vz, ax, ay, az, yaw_rate=0.0):
        """Body-frame velocity + acceleration feedforward. x fwd, y right, z down.

        Why this exists. A velocity-only setpoint asks the position controller
        to discover the acceleration for itself, and it does so gently: escape
        displacement was measured rising at ~2 m/s^2 while the tilt ceiling
        permits g*tan(ATC_ANGLE_MAX) = 5.66 m/s^2. The controller was never
        demanding what the airframe could already give. An acceleration term
        states the demand directly instead of waiting for the velocity error
        to build it.

        Raising dodge_speed does NOT do this -- 1.5 -> 4.0 m/s was measured as
        a null, escape identical to within a centimetre at every sample --
        because a larger velocity error meets the same internal shaping.

        Whether ArduPilot honours the acceleration fields in GUIDED must be
        MEASURED, not assumed: an ignored field looks exactly like a lever
        with no effect. If an A/B shows nothing, report "unsupported", not
        "no effect".
        """
        mask = MASK_VEL_ACCEL
        if yaw_rate != 0.0:
            mask &= ~(1 << 11)        # un-ignore yaw_rate
        with self._io:
            self.master.mav.set_position_target_local_ned_send(
                0, self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                mask,
                0, 0, 0,                  # x, y, z position (ignored)
                vx, vy, vz,               # velocity
                ax, ay, az,               # acceleration feedforward
                0, yaw_rate)              # yaw, yaw_rate

    def send_position_ned(self, north, east, down, yaw=None):
        """Absolute position setpoint (m) in LOCAL_NED, offset from EKF origin."""
        mask = MASK_POS_ONLY if yaw is None else MASK_POS_YAW
        with self._io:
            self.master.mav.set_position_target_local_ned_send(
                0, self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mask,
                north, east, down,
                0, 0, 0,
                0, 0, 0,
                0.0 if yaw is None else yaw, 0)

    # telemetry
    def get_state(self):
        """Non-blocking snapshot of pose + attitude + velocity in NED.

        Drains the whole receive buffer and dispatches by type. It must NOT be
        written as two type-filtered calls:

            lp  = recv_match(type="LOCAL_POSITION_NED", blocking=False)
            att = recv_match(type="ATTITUDE",           blocking=False)

        because recv_match(type=X) *discards* every message it scans past. The
        first call throws away the ATTITUDE messages queued ahead of the
        position it wants, so attitude only survives when one happens to land
        after a LOCAL_POSITION_NED. Position stays perfectly fresh (it is
        fetched first) while yaw silently freezes at a value minutes old —
        measured live: odom position matched Gazebo truth to the
        millimetre while odom yaw read 180.0 deg against a true 85.8 deg.

        That asymmetry is expensive. Every consumer of the odom quaternion
        breaks with it: spawn_projectile threw the ball ~94 deg away from where
        the camera was actually looking (so the detector never saw it), and the
        detector's egomotion compensation rotated clouds by a dead attitude, so
        static ground stopped cancelling and 76% of frames tripped the
        fg_max_points flood guard. It hid for a whole session because a stale
        yaw is still correct for a while right after takeoff.

        The drain is capped so a backlog cannot starve the caller's tick.

        `fc_msgs` counts every message from the autopilot and only ever grows.
        A caller that sees it stop growing knows the link is down, even though
        the rest of the snapshot still holds the last thing the autopilot said.
        """
        with self._io:
            return self._drain()

    def _drain(self):
        for _ in range(_DRAIN_MAX_MSGS):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            if self.target_system and msg.get_srcSystem() == self.target_system:
                self._state["fc_msgs"] = self._state.get("fc_msgs", 0) + 1
            kind = msg.get_type()
            if kind == "LOCAL_POSITION_NED":
                self._state.update(dict(n=msg.x, e=msg.y, d=msg.z,
                                        vn=msg.vx, ve=msg.vy, vd=msg.vz))
            elif kind == "ATTITUDE":
                self._state.update(dict(roll=msg.roll, pitch=msg.pitch,
                                        yaw=msg.yaw))
            elif kind == "SYS_STATUS":
                mv = getattr(msg, "voltage_battery", _BATT_MV_UNKNOWN)
                pct = getattr(msg, "battery_remaining", _BATT_PCT_UNKNOWN)
                self._state.update(dict(
                    batt_v=None if mv in (0, _BATT_MV_UNKNOWN) else mv / 1000.0,
                    batt_pct=None if pct == _BATT_PCT_UNKNOWN else float(pct)))
            elif kind == "HEARTBEAT":
                # MAVProxy, the GCS and this bridge all heartbeat on the same
                # link. Only the autopilot's own says anything about armed
                # state, mode or failsafe; a GCS heartbeat would report
                # disarmed forever.
                if self.target_system and msg.get_srcSystem() != self.target_system:
                    continue
                base = getattr(msg, "base_mode", 0)
                self._state.update(dict(
                    armed=bool(base & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                    mode=self.mode_name(msg),
                    fc_failsafe=getattr(msg, "system_status", 0) >= _MAV_STATE_CRITICAL))
        return dict(self._state)

    @staticmethod
    def mode_name(heartbeat):
        """Flight-mode string, or None if it cannot be resolved.

        pymavlink's mode table is keyed by vehicle type and raises on
        combinations it does not know. A supervisor deciding whether to hand
        control back to patrol must not die because a mode name was
        unrecognised, so an unknown mode is reported as absent, not fatal.
        """
        try:
            return mavutil.mode_string_v10(heartbeat)
        except Exception:
            return None

    # frame helpers (NED <-> ENU) used by the ROS node
    @staticmethod
    def ned_to_enu(n, e, d):
        return (e, n, -d)             # ENU x=East, y=North, z=Up

    @staticmethod
    def enu_to_ned(x, y, z):
        return (y, x, -z)

    @staticmethod
    def ned_rpy_to_enu_quat(roll, pitch, yaw):
        """ArduPilot ATTITUDE (NED/FRD roll-pitch-yaw, rad) -> ENU/FLU body
        quaternion (x, y, z, w) per REP-103.

        Equivalent Euler mapping: yaw' = pi/2 - yaw, pitch' = -pitch,
        roll' = roll, composed ZYX. (NED yaw 0 = North = ENU yaw 90°;
        NED nose-up pitch is positive, ENU/FLU nose-up pitch is negative.)
        Unit-tested in test/test_frames.py.
        """
        hy = (math.pi / 2.0 - yaw) / 2.0
        hp = -pitch / 2.0
        hr = roll / 2.0
        cy, sy = math.cos(hy), math.sin(hy)
        cp, sp = math.cos(hp), math.sin(hp)
        cr, sr = math.cos(hr), math.sin(hr)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )
