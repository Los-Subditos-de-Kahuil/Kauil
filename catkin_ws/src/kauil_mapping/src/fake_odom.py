#!/usr/bin/env python
"""
Publishes a fake odometry message based on the cmd_vel topic.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/10
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# * Constants
F = 50  # Frequency
T = 1.0 / F  # Period


class FalseOdom:
    """
    # False Odom
    This node publishes a fake odometry message based on the cmd_vel topic.

    ## Attributes
    - x (`float`): x-coordinate of robot
    - v (`float`): linear velocity of robot
    - rate (`rospy.Rate`): rate at which the node will run
    - publisher (`rospy.Publisher`): publisher to fake_odom topic

    ## Methods
    - __init__(): initialize node, subscribers, publishers and initial values
    - callback(msg): callback for cmd_vel topic
    - run(): main loop

    ## Subscribers
    - `cmd_vel`: cmd_vel topic

    ## Publishers
    - `fake_odom`: fake_odom topic
    """

    def __init__(self):
        """Initialize node, subscribers, publishers and initial values"""
        # * Initialize node
        rospy.init_node("fake_odometry")
        self.rate = rospy.Rate(F)

        # * Subscribe to cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.callback)

        # * Publish to fake_odom topic
        self.publisher = rospy.Publisher("fake_odom", Odometry, queue_size=10)

        # * Initial values
        self.x = 0.0
        self.v = 0.0

    def callback(self, msg):
        """Callback for cmd_vel topic

        Args:
            msg (Twist): message from cmd_vel topic
        """
        self.v = msg.linear.x

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # * Update position
            self.x += self.v * T

            # * Publish to fake_odom topic
            odometry = Odometry()
            odometry.child_frame_id = "laser"
            odometry.header.frame_id = "fake_odom"
            odometry.header.stamp = rospy.Time.now()
            odometry.pose.pose.position.x = self.x
            self.publisher.publish(odometry)

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        FalseOdom().run()
    except rospy.ROSInterruptException:
        pass
