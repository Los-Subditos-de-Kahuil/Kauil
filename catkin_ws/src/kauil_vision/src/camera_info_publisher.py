#! /usr/bin/env python
"""
Load the calibration files and publish the camera info to the topics for each camera.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo


class Publisher:
    """
    # Camera Info Publisher
    This node loads the calibration files and publishes the camera info to the topics for each camera.

    ## Attributes
    - rate (`rospy.Rate`): rate at which the node will run
    - rpublisher (`rospy.Publisher`): publisher to /right_camera/camera_info topic
    - lpublisher (`rospy.Publisher`): publisher to /left_camera/camera_info topic
    - rpath (`str`): path to right camera calibration file
    - lpath (`str`): path to left camera calibration file
    - rcalibration (`dict`): right camera calibration parameters
    - lcalibration (`dict`): left camera calibration parameters

    ## Methods
    - __init__(): initialize node, publishers and load calibration files
    - run(): Runs the node. Publishes the camera info to the topics for each camera

    ## Publishers
    - `/right_camera/camera_info`: right camera info
    - `/left_camera/camera_info`: left camera info
    """

    def __init__(self):
        """Initialize node, publishers and load calibration files"""
        # * Initialize node
        rospy.init_node("camera_info_publisher")
        self.rate = rospy.Rate(50)

        # * Publishers
        self.rpublisher = rospy.Publisher(
            "/right_camera/camera_info", CameraInfo, queue_size=10
        )
        self.lpublisher = rospy.Publisher(
            "/left_camera/camera_info", CameraInfo, queue_size=10
        )

        # * Load calibration files
        rospy.logwarn(os.getcwd())
        self.rpath = "../catkin_ws/src/kauil_vision/resources/rost.yaml"
        self.lpath = "../catkin_ws/src/kauil_vision/resources/lost.yaml"
        with open(self.rpath) as f:
            self.rcalibration = yaml.load(f)
        with open(self.lpath) as f:
            self.lcalibration = yaml.load(f)

    def run(self):
        """Runs the node. Publishes the camera info to the topics for each camera"""
        while not rospy.is_shutdown():
            # * Publish right camera info
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = rospy.Time.now()
            camera_info_msg.width = self.rcalibration["image_width"]
            camera_info_msg.height = self.rcalibration["image_height"]
            camera_info_msg.K = self.rcalibration["camera_matrix"]["data"]
            camera_info_msg.D = self.rcalibration["distortion_coefficients"]["data"]
            camera_info_msg.R = self.rcalibration["rectification_matrix"]["data"]
            camera_info_msg.P = self.rcalibration["projection_matrix"]["data"]
            camera_info_msg.distortion_model = self.rcalibration["distortion_model"]
            self.rpublisher.publish(camera_info_msg)

            # * Publish left camera info
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = rospy.Time.now()
            camera_info_msg.width = self.lcalibration["image_width"]
            camera_info_msg.height = self.lcalibration["image_height"]
            camera_info_msg.K = self.lcalibration["camera_matrix"]["data"]
            camera_info_msg.D = self.lcalibration["distortion_coefficients"]["data"]
            camera_info_msg.R = self.lcalibration["rectification_matrix"]["data"]
            camera_info_msg.P = self.lcalibration["projection_matrix"]["data"]
            camera_info_msg.distortion_model = self.lcalibration["distortion_model"]
            self.lpublisher.publish(camera_info_msg)

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        Publisher().run()
    except rospy.ROSInterruptException:
        pass
