#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

FS = 50
T = 1.0 / FS

class JoyToCmdCamL:
    def __init__(self):
        # * Initialize node
        rospy.init_node("joy_to_cmd_l_cam")
        self.rate = rospy.Rate(FS)

        self.calls = {
                        'w' : 0,
                        'a' : 0,
                        'd' : 0,
                        's' : 0
                      }

        # * Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)


        # * Publish to cmd_vel topic
        self.publisher = rospy.Publisher("/left_camera/cmd", String, queue_size=10)

    def joy_callback(self, msg):
        """Callback for joy topic.

        Args:
            msg (Joy): Joy message
        """
        u_d = msg.axes[-1]
        l_r = msg.axes[-2]
        if u_d == 1:
            self.calls['w'] = 1
            self.calls['s'] = 0
        elif u_d == -1:
            self.calls['w'] = 0
            self.calls['s'] = 1
        else:
            self.calls['w'] = 0
            self.calls['s'] = 0

        if l_r == 1:
            self.calls['a'] = 1
            self.calls['d'] = 0
        elif l_r == -1:
            self.calls['a'] = 0
            self.calls['d'] = 1
        else:
            self.calls['a'] = 0
            self.calls['d'] = 0


    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            message = String()
            for key in self.calls.keys():
                message.data = ''
                if self.calls[key] == 1:
                    message.data = str(key)
                    self.publisher.publish(message)

            # * Sleep
            self.rate.sleep()


if __name__ == "__main__":
    try:
        camJoy = JoyToCmdCamL()
        camJoy.run()
    except rospy.ROSInterruptException:
        pass
