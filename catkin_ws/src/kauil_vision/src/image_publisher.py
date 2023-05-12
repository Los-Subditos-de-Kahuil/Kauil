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
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------- Class ----------------------------
class ImagePublisher:
    """
    # Image Publisher
    This node reads information from the cameras and publishes images to a ROS topic.

    ## Attributes
    - imageRight (`numpy.ndarray`): image from right camera
    - imageLeft (`numpy.ndarray`): image from left camera
    - bridge (`CvBridge`): bridge between ROS and OpenCV
    - rate (`rospy.Rate`): rate at which the node will run
    - rightCam (`cv2.VideoCapture`): right camera
    - leftCam (`cv2.VideoCapture`): left camera
    - rightPublisher (`rospy.Publisher`): publisher to right_camera topic
    - leftPublisher (`rospy.Publisher`): publisher to left_camera topic

    ## Methods
    - __init__(): initialize node, publishers and initial values, as well as the cameras
    - run(): main loop

    ## Publishers
    - `right_camera`: right camera image
    - `left_camera`: left camera image
    """

    def __init__(self):
        """Initialize node, publishers and initial values, as well as the cameras"""
        # * Initialize node
        rospy.init_node("kauil_image_publisher")
        self.rate = rospy.Rate(FS)

        # * Initial values
        self.imageRight = None
        self.imageLeft = None
        self.bridge = CvBridge()

        # * Initialize cameras
        self.rightCam = cv2.VideoCapture(
            "http://admin:@192.168.1.101/videostream.cgi?[?rate=6]"
        )
        self.leftCam = cv2.VideoCapture(
            "http://admin:@192.168.1.102/videostream.cgi?[?rate=6]"
        )

        # * Publishers
        self.rightPublisher = rospy.Publisher("/right_camera", Image, queue_size=1)
        self.leftPublisher = rospy.Publisher("/left_camera", Image, queue_size=1)

    def run(self, verbose=False):
        """Main loop

        Args:
            verbose (bool, optional): Whether to print information or not. Defaults to False.
        """
        try:
            while not rospy.is_shutdown():
                # * Read from cameras
                retR, self.imageRight = self.rightCam.read()
                retL, self.imageLeft = self.leftCam.read()
                if not retR and verbose:
                    print("Failed to read right camera")
                if not retL and verbose:
                    print("Fialed to read left camera")

                # * Publish images
                self.rightPublisher.publish(
                    self.bridge.cv2_to_imgmsg(self.imageRight, "bgr8")
                )
                self.leftPublisher.publish(
                    self.bridge.cv2_to_imgmsg(self.imageLeft, "bgr8")
                )

                # * Sleep
                self.rate.sleep()

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    imgPub = ImagePublisher()
    verbose = True
    imgPub.run(verbose=verbose)
