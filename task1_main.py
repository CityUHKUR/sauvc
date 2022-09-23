#!/usr/bin/env python

import time
import sys
import math
import threading

import rospy
from std_msgs.msg import String, Bool

from pymavlink import mavutil # Import mavutil
from pymavlink.quaternion import QuaternionBase # Imports for attitude

def send_manual_control(x,y,z,r):
    master.mav.manual_control_send(
        master.target_system,
        x,	  # -1000 to 1000, static=0, backward<0, forward>0
        y,    # -1000 to 1000, static=0, left<0, right>0
        z,    # 0 to 1000, static=500, downward<500, upward>500
        r,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0    # useless (for other purpose)
    )

def callback(msg):
    global glob_flare_in_middle
    glob_flare_in_middle = msg.data
    
def callback1(msg):
    global glob_flare_in_middle
    glob_gate_found = msg.data
def callback2(msg):
    global glob_flare_in_middle
    glob_ready_to_rush = msg.data
def callback3(msg):
    global_direction = msg.data


def listener(args):
    rospy.init_node('task1_cv_msg_listener', anonymous=True)
    rospy.Subscriber("/flare_in_middle",Bool,callback)
    rospy.Subscriber("/gate_found",Bool,callback1)
    rospy.Subscriber("/ready_to_rush",Bool,callback2)
    rospy.Subscriber("/direction",String,callback3)

    rospy.spin()

t1 = threading.Thread(target=listener, name='thread_get_ros')
t1.start()


def task1_v2():
    ##########################################
    ### Go stright first, Then search gate ###
    ##########################################
    # Flag
    flare_in_middle = False
    flare_avoided = False
    flare_pass = False
    gate_found = False
    ready_to_rush = False
    gate_pass = False

    # Constant
    time_to_pass_flare = 10
    time_to_avoid = 5
    time_to_go_right = 40
    time_from_right_to_left = 80
    time_to_rush = 5

    # variable
    time_forwarded = 0
    direction = 0

    # global variable
    global time_y_moved

    #initial global variable
    time_y_moved = 0

    #####################################################################
    ### Go straigh to pass flare area and avoid if flare is in middle ###
    ### if find gate, break, go to follow gate                        ###
    #####################################################################
    last_time = 0
    while (time_forwarded < time_to_pass_flare and not gate_found):
        ## Todo read flare is in middle

        ## Todo read gate found
        '''
        if gate_found:
            break
        '''
        # go straigh or avoid to right
        if flare_in_middle and not gate_found:
            t = time.time()
            while (time.time() - t < time_to_avoid):
                send_manual_control(0,400,500,0)
            flare_in_middle = False
            flare_avoided = True
            last_time += time_to_avoid
            time_y_moved += time_to_avoid
        else:
            send_manual_control(400,0,500,0)
            time_forwarded += time.time() - last_time
            last_time = time.time()

    #########################################################
    ### passed the flare, move horizontal to found gate " ###
    #########################################################
    if flare_avoided:
        time_to_go_right -= time_to_avoid

    if not gate_found:
        # if not gate found go right
        t = time.time()
        time_passed = 0
        while (time.time() - t < time_to_go_right and not gate_found):
            ## Todo: check if gate found
            '''
            if gate_found:
                time_passed = time.time() - t
                break
            '''
            send_manual_control(0,400,500,0)
            time_passed = time.time() - t   # cal time pass
        time_y_moved += time_passed # add time pass to time_y_moved
 
    if not gate_found:
        # if not gate found, go left
        t = time.time()
        time_passed = 0
        while (time.time() - t < time_from_right_to_left and not gate_found):
            ## Todo: check if gate found
            '''
            if gate_found:
                time_passed = time.time() - t
                break
            '''
            send_manual_control(0,-400,500,0)
            time_passed = time.time() - t   # cal time pass
        time_y_moved -= time_passed # add time pass to time_y_moved
    
    ######################################
    ### found gate and prepare to pass ###
    ######################################
    last_time = 0
    if gate_found:
        t = time.time()
        y_ = 0
        while not gate_pass:
            if flare_in_middle:
                while (time_forwarded < time_to_pass_flare and not gate_found):
                    ## Todo read flare is in middle

                    ## Todo read gate found
                    '''
                    if gate_found:
                        break
                    '''
                    # go straigh or avoid to right
                    if flare_in_middle and not gate_found:
                        t = time.time()
                        while (time.time() - t < time_to_avoid):
                            send_manual_control(0,400,500,0)
                        flare_in_middle = False
                        flare_avoided = True
                        last_time += time_to_avoid
                        time_y_moved += time_to_avoid
                    else:
                        send_manual_control(400,0,500,0)
                        time_forwarded += time.time() - last_time
                        last_time = time.time()
                flare_in_middle = False

            ## Todo: read direction
            direction = 0
            ## Todo: read ready
            '''
            if ready_to_rush:
                break
            '''
            if (direction == -1):
                send_manual_control(400,-400,500,0)
                y_ -= time.time() - t
                t = time.time()
            if (direction == 1):
                send_manual_control(400,400,500,0)
                y_ += time.time() - t
                t = time.time()
            if (direction == 0):
                send_manual_control(400,0,500,0)
                t = time.time()
        time_y_moved += y_

    ########################
    ### rush to the gate ###
    ########################
    if ready_to_rush:
        t = time.time()
        while(time.time - t < time_to_rush):
            send_manual_control(1000,0,500,0)
    
    send_manual_control(0,0,500,0)

def task1_v1():
    ##########################################
    ### Go stright first, Then search gate ###
    ##########################################
    # Flag
    flare_in_middle = False
    flare_avoided = False
    flare_pass = False
    gate_found = False
    ready_to_rush = False
    gate_pass = False

    # Constant
    time_to_pass_flare = 10
    time_to_avoid = 5
    time_to_go_right = 40
    time_from_right_to_left = 80
    time_to_rush = 5

    # variable
    time_forwarded = 0
    direction = 0

    # global variable
    global time_y_moved

    #initial global variable
    time_y_moved = 0

    #####################################################################
    ### Go straigh to pass flare area and avoid if flare is in middle ###
    #####################################################################
    last_time = 0
    while (time_forwarded < time_to_pass_flare):
        ## Todo read flare is in middle
            
        # go straigh or avoid to right
        if flare_in_middle:
            # hard code to pass flare
            t = time.time()
            while (time.time() - t < time_to_avoid):
                send_manual_control(0,400,500,0)
            flare_in_middle = False
            flare_avoided = True
            last_time += time_to_avoid
            time_y_moved += time_to_avoid
        else:
            send_manual_control(400,0,500,0)
            time_forwarded += time.time() - last_time
            last_time = time.time()

    #########################################################
    ### passed the flare, move horizontal to found gate " ###
    #########################################################
    if flare_avoided:
        time_to_go_right -= time_to_avoid

    if not gate_found:
        # if not gate found go right
        t = time.time()
        time_passed = 0
        while (time.time() - t < time_to_go_right and not gate_found):
            ## Todo: check if gate found
            '''
            if gate_found:
                time_passed = time.time() - t
                break
            '''
            send_manual_control(0,400,500,0)
            time_passed = time.time() - t   # cal time pass
        time_y_moved += time_passed # add time pass to time_y_moved
 
    if not gate_found:
        # if not gate found, go left
        t = time.time()
        time_passed = 0
        while (time.time() - t < time_from_right_to_left and not gate_found):
            ## Todo: check if gate found
            '''
            if gate_found:
                time_passed = time.time() - t
                break
            '''
            send_manual_control(0,-400,500,0)
            time_passed = time.time() - t   # cal time pass
        time_y_moved -= time_passed # add time pass to time_y_moved
    
    ######################################
    ### found gate and prepare to pass ###
    ######################################
    last_time = 0
    if gate_found:
        t = time.time()
        y_ = 0
        while not gate_pass:
            ## Todo: read direction
            
            ## Todo: read ready
            '''
            if ready_to_rush:
                break
            '''
            if (direction == -1):
                send_manual_control(400,-400,500,0)
                y_ -= time.time() - t
                t = time.time()
            if (direction == 1):
                send_manual_control(400,400,500,0)
                y_ += time.time() - t
                t = time.time()
            if (direction == 0):
                send_manual_control(400,0,500,0)
                t = time.time()
        time_y_moved += y_

    ########################
    ### rush to the gate ###
    ########################
    if ready_to_rush:
        t = time.time()
        while(time.time - t < time_to_rush):
            send_manual_control(1000,0,500,0)
    
    send_manual_control(0,0,500,0)


### Start program ###

# Create the connectionc
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'ALT_HOLD'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

# set depth
time.sleep(2) # wait it go to zero depth
send_manual_control(0,0,400,0) # 20% downward force 
time.sleep(0.5) # change param, maybe:0.5s, 20cm
send_manual_control(0,0,500,0)
time.sleep(1)

# Do Task 1,2,3,4
task1_v1()

# Disarm
send_manual_control(0,0,500,0)
time.sleep(3)
master.arducopter_disarm()
print("Waiting for the vehicle to disarm")
master.motors_disarmed_wait()
print('Disarmed!')