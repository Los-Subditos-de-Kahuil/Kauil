#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

F = 50
T = 1.0 / F

class FalseOdom:
    def __init__(self):
        rospy.init_node("fake_odometry")
        self.rate = rospy.Rate(F)

        rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.publisher = rospy.Publisher("fake_odom", Odometry, queue_size=10)

        self.x = 0.0
        self.v = 0.0

    def callback(self, msg):
        self.v = msg.linear.x

    def run(self):
        while not rospy.is_shutdown():
            self.x += self.v * T
            odometry = Odometry()
            odometry.child_frame_id = "laser"
            odometry.header.frame_id = "fake_odom"
            odometry.header.stamp = rospy.Time.now()
            odometry.pose.pose.position.x = self.x
            self.publisher.publish(odometry)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        FalseOdom().run()
    except rospy.ROSInterruptException:
        pass