#! /usr/bin/env python
"""
Listen to joy topic and publish to /left_camera/cmd topic

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


class JoyToCmdCamL:
    """
    # Joy To Cmd Cam L
    This node listens to the joy topic (published by using a controller with joysticks) and publishes to /left_camera/cmd topic.

    ## Attributes
    - calls (`dict`): dictionary with the keys associated to the l_camera_position_listener node
    - rate (`rospy.Rate`): rate at which the node will run
    - publisher (`rospy.Publisher`): publisher to /left_camera/cmd topic

    ## Methods
    - __init__(): initialize node, subscribers, publishers and initial values
    - joy_callback(msg): callback for joy topic
    - run(): main loop

    ## Subscribers
    - `joy`: joystick topic

    ## Publishers
    - `/left_camera/cmd`: topic to move the left camera
    """

    def __init__(self):
        """Initialize node, subscribers, publishers and initial values"""
        # * Initialize node
        rospy.init_node("joy_to_cmd_l_cam")
        self.rate = rospy.Rate(FS)

        # * Initial values
        self.calls = {"w": 0, "a": 0, "d": 0, "s": 0}

        # * Subscribe to joy topic
        rospy.Subscriber("joy", Joy, self.joy_callback)

        # * Publish to cmd_vel topic
        self.publisher = rospy.Publisher("/left_camera/cmd", String, queue_size=10)

    def joy_callback(self, msg):
        """Callback for joy topic.

        Args:
            msg (Joy): Joy message
        """
        # Indexes obtained from running `rostopic echo joy`
        # They correspond to the left button-pad in the controller
        # * Get values
        u_d = msg.axes[-1]  # Up and down
        l_r = msg.axes[-2]  # Left and right

        # * Control up and down movement
        if u_d == 1:
            self.calls["w"] = 1
            self.calls["s"] = 0
        elif u_d == -1:
            self.calls["w"] = 0
            self.calls["s"] = 1
        else:
            self.calls["w"] = 0
            self.calls["s"] = 0

        # * Control left and right movement
        if l_r == 1:
            self.calls["a"] = 1
            self.calls["d"] = 0
        elif l_r == -1:
            self.calls["a"] = 0
            self.calls["d"] = 1
        else:
            self.calls["a"] = 0
            self.calls["d"] = 0

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
        camJoy = JoyToCmdCamL()
        camJoy.run()
    except rospy.ROSInterruptException:
        pass
