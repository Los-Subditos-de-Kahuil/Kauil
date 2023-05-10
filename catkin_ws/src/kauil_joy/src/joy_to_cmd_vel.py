#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# * Constants
V_MAX = 0.5
W_MAX = 0.5


class JoyToCmdVel:
    def __init__(self):
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
