#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def callback(msg):
    print("image received")
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow('sub_image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass


def listener():
    rospy.init_node('cv2_listener', anonymous=True)
    rospy.Subscriber("/image", Image, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
