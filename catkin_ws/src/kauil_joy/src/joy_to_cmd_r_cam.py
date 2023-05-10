#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

FS = 50
T = 1.0 / FS

class JoyToCmdCamR:
    def __init__(self):
        # * Initialize node
        rospy.init_node("joy_to_cmd_r_cam")
        self.rate = rospy.Rate(FS)

        self.calls = {
                        'j' : 0,
                        'l' : 0,
                        'k' : 0,
                        'i' : 0
                      }

        # * Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)


        # * Publish to cmd_vel topic
        self.publisher = rospy.Publisher("/right_camera/cmd", String, queue_size=10)

    def joy_callback(self, msg):
        """Callback for joy topic.

        Args:
            msg (Joy): Joy message
        """
        self.calls['k'] = msg.buttons[0]
        self.calls['l'] = msg.buttons[1]
        self.calls['j'] = msg.buttons[2]
        self.calls['i'] = msg.buttons[3]
        

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
        camJoy = JoyToCmdCamR()
        camJoy.run()
    except rospy.ROSInterruptException:
        pass
