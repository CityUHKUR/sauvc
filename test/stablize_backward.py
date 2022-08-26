#!/usr/bin/env python

"""
move backward in stablize mode
"""

import sys
# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'STABILIZE'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

try:
    # Arm
    # master.arducopter_arm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    # stop thruster
    master.mav.manual_control_send(
        master.target_system,
        0,	  # -1000 to 1000, static=0, backward<0, forward>0
        0,    # -1000 to 1000, static=0, left<0, right>0
        500,	# 0 to 1000, static=500, downward<500, upward>500
        0,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0)    # useless

    # backward
    for i in range(5):
        master.mav.manual_control_send(
            master.target_system,
            -800,	  # -1000 to 1000, static=0, backward<0, forward>0
            0,    # -1000 to 1000, static=0, left<0, right>0
            500,	# 0 to 1000, static=500, downward<500, upward>500
            0,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
            0)    # useless
        time.sleep(1)

    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()

except KeyboardInterrupt:
    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()