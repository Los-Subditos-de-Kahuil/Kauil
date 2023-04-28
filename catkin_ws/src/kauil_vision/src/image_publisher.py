#! /usr/bin/env python
"""
Read information from the cameras and publish information to a ROS topic

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/04/27
"""
# ------------------------- Imports --------------------------
import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------- Class ----------------------------
class ImagePublisher():

    def __init__(self):
        rospy.init_node("kauil_image_publisher")

        self.imageRight = None
        self.imageLeft = None
        self.bridge = CvBridge()

        #! Check index
        self.rightCam = cv2.VideoCapture(0)
        self.leftCam = cv2.VideoCapture(1)
        
        self.rightPublisher = rospy.Publisher("/right_camera")
        self.leftPublisher = rospy.Publisher("/left_camera")


        self.rate = rospy.Rate(FS)

    def run(self):
        try:
            while not rospy.is_shutdown():
                retR, self.imageRight = self.rightCam.read()
                retL, self.imageLeft = self.leftCam.read()
                if not retR:
                    print("Failed to read right camera")
                if not retL:
                    print("Fialed to read left camera")

                self.rightPublisher.publish(self.bridge.cv2_to_imgmsg(self.imageRight))
                self.leftPublisher.publish(self.bridge.cv2_to_imgmsg(self.imageLeft))

                self.rate.sleep()
        
        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    imgPub = ImagePublisher()
    imgPub.run()