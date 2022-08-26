#!/usr/bin/env python

"""
manual mode
"""

import sys
# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)    
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

try:
    # Arm
    master.arducopter_arm()
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

    # stablize
    master.mav.manual_control_send(
        master.target_system,
        500,	  # -1000 to 1000, static=0, backward<0, forward>0
        0,    # -1000 to 1000, static=0, left<0, right>0
        600,	# 0 to 1000, static=500, downward<500, upward>500
        0,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0)	  # useless
    time.sleep(1)

    # Disarm
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    
except KeyboardInterrupt:
    # Disarm
    master.arducopter_disarm()
    master.motors_disarmed_wait()