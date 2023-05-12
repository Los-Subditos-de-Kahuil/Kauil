#! /usr/bin/env python
"""
Listen to joy topic and publish to cmd_vel topic

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/09
"""
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# * Constants
V_MAX = 0.5
W_MAX = 0.5


class JoyToCmdVel:
    """
    # Joy To Cmd Vel
    This node listens to the joy topic (published by using a controller with joysticks) and publishes to cmd_vel topic.

    ## Attributes
    - x (`float`): x axis value
    - y (`float`): y axis value
    - rate (`rospy.Rate`): rate at which the node will run
    - cmd_vel (`rospy.Publisher`): publisher to cmd_vel topic

    ## Methods
    - __init__(): initialize node, subscribers, publishers and initial values
    - end_callback(): if node dies, for instance, by keyboard interrupt, we stop the robot
    - joy_callback(msg): callback for joy topic
    - run(): main loop

    ## Subscribers
    - `joy`: joystick topic

    ## Publishers
    - `cmd_vel`: cmd_vel topic
    """

    def __init__(self):
        """Initialize node, subscribers, publishers and initial values"""
        # * Initialize node
        rospy.init_node("joy_to_cmd_vel")
        self.rate = rospy.Rate(60)
        rospy.on_shutdown(self.end_callback)

        # * Initial values
        self.x = 0.0
        self.y = 0.0

        # * Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # * Publish to cmd_vel topic
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def end_callback(self):
        """If node dies, for instance, by keyboard interrupt, we stop
        the robot
        """
        self.cmd_vel.publish(Twist())

    def joy_callback(self, msg):
        """Callback for joy topic.

        Args:
            msg (Joy): Joy message
        """
        # Axes' indexes obtained from running `rostopic echo joy`
        # They correspond to the right joystick
        self.x = msg.axes[3]
        self.y = msg.axes[4]

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # * Create Twist message
            twist = Twist()

            # * Set linear velocity
            twist.linear.x = V_MAX * self.y

            # * Set angular velocity
            twist.angular.z = W_MAX * self.x

            # * Publish
            self.cmd_vel.publish(twist)

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        JoyToCmdVel().run()
    except rospy.ROSInterruptException:
        pass
