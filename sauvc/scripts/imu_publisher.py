#!/usr/bin/env python
"""
To publish IMU data
"""

import rospy
from pymavlink import mavutil
from geometry_msgs.msg import Vector3

def talker():
    pub_acc = rospy.Publisher('/imu_data/acceleration', Vector3, queue_size=10)
    pub_gyro = rospy.Publisher('/imu_data/angular_speed', Vector3, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = master.recv_match()
        xacc = master.messages["RAW_IMU"].xacc
        yacc = master.messages["RAW_IMU"].yacc
        zacc = master.messages["RAW_IMU"].zacc
        xgyro = master.messages["RAW_IMU"].xgyro
        ygyro = master.messages["RAW_IMU"].ygyro
        zgyro = master.messages["RAW_IMU"].zgyro       
    #    print(xacc," ",yacc," ", zacc)
    #    print(xgyro," ",ygyro," ", zgyro)

        imu_acc = Vector3()
        imu_acc.x = xacc
        imu_acc.y = yacc
        imu_acc.z = zacc
        imu_gyro = Vector3()
        imu_gyro.x = xgyro
        imu_gyro.y = ygyro
        imu_gyro.z = zgyro
        pub_acc.publish(imu_acc)
        pub_gyro.publish(imu_gyro)

if __name__ == '__main__':
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat()

    try:
        talker()
    except rospy.ROSInterruptException:
        pass