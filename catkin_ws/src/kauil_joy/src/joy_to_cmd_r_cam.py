#! /usr/bin/env python
"""
Listen to joy topic and publish to /right_camera/cmd topic

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/10
"""
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# * Constants
FS = 50  # Node frequency
T = 1.0 / FS  # Node period


class JoyToCmdCamR:
    """
    # Joy To Cmd Cam R
    This node listens to the joy topic (published by using a controller with joysticks) and publishes to /right_camera/cmd topic.

    ## Attributes
    - calls (`dict`): dictionary with the keys associated to the r_camera_position_listener node
    - rate (`rospy.Rate`): rate at which the node will run
    - publisher (`rospy.Publisher`): publisher to /right_camera/cmd topic

    ## Methods
    - __init__(): initialize node, subscribers, publishers and initial values
    - joy_callback(msg): callback for joy topic
    - run(): main loop

    ## Subscribers
    - `joy`: joystick topic

    ## Publishers
    - `/right_camera/cmd`: topic to move the right camera
    """

    def __init__(self):
        """Initialize node, subscribers, publishers and initial values"""
        # * Initialize node
        rospy.init_node("joy_to_cmd_r_cam")
        self.rate = rospy.Rate(FS)

        # * Initial values
        self.calls = {"j": 0, "l": 0, "k": 0, "i": 0}

        # * Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # * Publish to cmd_vel topic
        self.publisher = rospy.Publisher("/right_camera/cmd", String, queue_size=10)

    def joy_callback(self, msg):
        """Callback for joy topic.

        Args:
            msg (Joy): Joy message
        """
        # Indexes obtained from running `rostopic echo joy`
        # They correspond to the right button-pad in the controller
        self.calls["k"] = msg.buttons[0]  # Lower button
        self.calls["l"] = msg.buttons[1]  # Right button
        self.calls["j"] = msg.buttons[2]  # Left button
        self.calls["i"] = msg.buttons[3]  # Upper button

    def run(self):
        """Main loop"""
        while not rospy.is_shutdown():
            # * Publish pressed buttons
            message = String()
            for key in self.calls.keys():
                message.data = ""
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
