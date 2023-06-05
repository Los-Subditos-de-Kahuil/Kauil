#! /usr/bin/env python

import rospy
import yaml
import os
from sensor_msgs.msg import CameraInfo



class Publisher():
    def __init__(self):
        rospy.init_node("camera_info_publisher")
        self.rate = rospy.Rate(50)
        self.rpublisher = rospy.Publisher("/right_camera/camera_info", CameraInfo, queue_size=10)
	self.lpublisher = rospy.Publisher("/left_camera/camera_info", CameraInfo, queue_size=10)
        self.rpath = "../catkin_ws/src/kauil_vision/resources/rost.yaml"
	self.lpath = "../catkin_ws/src/kauil_vision/resources/lost.yaml"
	rospy.logwarn(os.getcwd())
        with open(self.rpath) as f:
                self.rcalibration = yaml.load(f)
	with open(self.lpath) as f:
                self.lcalibration = yaml.load(f)

    def run(self):
        while not rospy.is_shutdown():
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
            self.rate.sleep()

if __name__ == "__main__":
    publisher = Publisher()
    publisher.run()
