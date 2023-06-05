#! /usr/bin/env python
"""
Read the LiDAR scan at given angle to find out to which direction it corresponds.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


class Calibrator:
    """
    # Calibrator
    This node reads the LiDAR scan at given angle to find out to which direction it corresponds.

    ## Attributes
    - rate (`rospy.Rate`): rate at which the node runs
    - scan (`LaserScan`): LiDAR scan message
    - angles (`np.array`): all available angles in the scan

    ## Methods
    - __init__(): initializes the node and subscribers, and sets the initial conditions
    - callback(`msg`): callback function for the LiDAR scan. Updates the scan and angles attributes
    - get_scan_between_angles(`start_angle`, `end_angle`): gets the minimum scan data between two angles
    - run(`angle_in_degrees`): runs the node. Prints the distance at given angle and the angle range

    ## Subscribers
    - `/scan`: LiDAR scan message
    """

    def __init__(self):
        """Initializes the node and subscribers, and sets the initial conditions."""
        # * Initialize node
        rospy.init_node("calibrator")
        self.rate = rospy.Rate(50)

        # * Initial values
        self.scan = None
        self.angles = None

        # * Subscribers
        rospy.Subscriber("/scan", LaserScan, self.callback)

    def callback(self, msg):
        """Callback function for the LiDAR scan. Updates the scan and angles attributes.

        Args:
            msg (LaserScan): LiDAR scan message
        """
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

        # * If scan is nan, return max range
        if np.isnan(scan):
            scan = self.scan.range_max

        return scan

    def run(self, angle_in_degrees=0):
        """Runs the node. Prints the distance at given angle and the angle range.

        Args:
            angle_in_degrees (int, optional): Desired angle in degrees. Defaults to 0.
        """
        while not rospy.is_shutdown():
            if self.scan is not None:
                # * Print distance at given angle
                print(
                    self.get_scan_between_angles(
                        np.deg2rad(angle_in_degrees - 2),
                        np.deg2rad(angle_in_degrees + 2),
                    )
                )

                # * Print angle range
                print(self.scan.angle_min)
                print(self.scan.angle_max)

                # * Sleep
                self.rate.sleep()


if __name__ == "__main__":
    try:
        Calibrator().run()
    except rospy.ROSInterruptException:
        pass
