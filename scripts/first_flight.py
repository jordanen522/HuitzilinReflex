from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:0.0.0.0:14551')
master.wait_heartbeat()
print("Heartbeat received")

# Set GUIDED mode
master.set_mode('GUIDED')
time.sleep(1)

# Arm
master.arducopter_arm()
master.motors_armed_wait()
print("Armed")

# Takeoff to 5 metres
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 5)
print("Taking off")
time.sleep(10)
print("Holding position")
