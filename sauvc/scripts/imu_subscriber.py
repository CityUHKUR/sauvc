#!/usr/bin/env python
"""
To subscribe IMU data
"""

import rospy
from geometry_msgs.msg import Vector3

def callback(data):
    print(data.x, " ", data.y," ", data.z)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu_data/acceleration", Vector3, callback)
    rospy.Subscriber("/imu_data/angular_speed", Vector3, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass