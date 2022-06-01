#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

cap = cv2.VideoCapture(0)
bridge = CvBridge()

def talker(): 
    pub = rospy.Publisher('/image', Image, queue_size=10)
    rospy.init_node('cv2_talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(image_message)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass