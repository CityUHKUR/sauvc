#!/usr/bin/env python

"""
stablize mode only
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
    # stablize
    time.sleep(30)

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