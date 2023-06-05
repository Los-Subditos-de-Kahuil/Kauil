#!/usr/bin/env python
"""
Move Kauil in a straight line for X seconds.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
# ------------------------- Imports --------------------------
import rospy
from geometry_msgs.msg import Twist
import sys

# ------------------------ Variables -------------------------
rospy.init_node("foward_t")

# Velocity publisher
vel_cmd = Twist()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Frequency
rate = rospy.Rate(60)

# ------------------------ Functions -------------------------
def end_callback():
    """Callback on shutdown"""
    # Stop Kauil
    vel_cmd.linear.x = 0.0
    vel_cmd.angular.z = 0.0
    pub.publish(vel_cmd)


rospy.on_shutdown(end_callback)


def run(verbose=False, seconds=3.6):
    """Move Kauil in a straight line for X seconds

    Args:
        verbose (bool, optional): To be talkative. Defaults to False.
        seconds (float, optional): Time to move. Defaults to 3.6.
    """
    # * Initiate node
    if verbose:
        print("foward_t node initialized.")

    # * Initiate time
    t0 = rospy.get_rostime().to_sec()
    try:
        while not rospy.is_shutdown():
            tf = rospy.get_rostime().to_sec()

            # * If time is over stop
            if tf - t0 >= seconds:
                if verbose:
                    print("Task finished.")

                v = 0.0
                w = 0.0
                vel_cmd.linear.x = v
                vel_cmd.angular.z = w
                pub.publish(vel_cmd)
                break

            if verbose:
                print(str(tf - t0) + " seconds have passed.")

            # * Move Kauil
            v = 0.28  # Max speed
            w = 0.0  # No angular velocity
            vel_cmd.linear.x = v
            vel_cmd.angular.z = w
            pub.publish(vel_cmd)

            # * Sleep
            rate.sleep()

    except rospy.exceptions.ROSInterruptException:
        pass


# --------------------------- Main ---------------------------
if __name__ == "__main__":
    verbose = False
    if len(sys.argv) > 1:
        if sys.argv[1] == "--verbose":
            verbose = True
        else:
            print("Wrong usage")
            print("Expecting: foward_t --verbose")
            exit(1)
    run(verbose=verbose)
