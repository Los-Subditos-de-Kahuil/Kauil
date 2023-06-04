#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class Calibrator:
    def __init__(self):
        rospy.init_node("calibrator")
        rospy.Subscriber("/scan", LaserScan, self.callback)
        self.scan = None
        self.angles = None
        self.rate = rospy.Rate(50)

    def callback(self, msg):
        self.scan = msg
        self.angles = np.arange(  # Get all the angles in the scan
            self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment
        )

    def get_scan_between_angles(self, start_angle, end_angle):
        """Gets the minimum scan data between two angles

        Args:
            start_angle (float): start angle in radians
            end_angle (float): end angle in radians

        Returns:
            float: minimum scan data between two angles
        """
        # * Normalize angles
        if abs(start_angle) > np.pi:
            start_angle = np.sign(start_angle) * (abs(start_angle) - 2 * np.pi)
        if abs(end_angle) > np.pi:
            end_angle = np.sign(end_angle) * (abs(end_angle) - 2 * np.pi)

        start_index = np.argmin(  # Find angle closest to start_angle
            np.abs(self.angles - start_angle)
        )
        end_index = np.argmin(  # Find angle closest to end_angle
            np.abs(self.angles - end_angle)
        )

        # * If front is requested, we need to split the ranges
        if start_index > end_index:
            return min(
                np.concatenate(  # Concatenate the two ranges
                    (self.scan.ranges[start_index:], self.scan.ranges[:end_index])
                )
            )
        
        scan = min(self.scan.ranges[start_index:end_index])
        if np.isnan(scan):
            scan = self.scan.range_max
        return scan

    def run(self):
        while not rospy.is_shutdown():
            if not self.scan is None:
                print(self.get_scan_between_angles(np.deg2rad(0 - 2), np.deg2rad(0 + 2)))
                print(self.scan.angle_min)
                print(self.scan.angle_max)
                self.rate.sleep()

if __name__ == "__main__":
    calibrator = Calibrator()
    calibrator.run()