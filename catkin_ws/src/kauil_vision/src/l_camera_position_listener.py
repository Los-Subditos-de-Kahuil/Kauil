#! /usr/bin/env python
"""
Read information from ROS topic and move cameras

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/09
"""
# ------------------------- Imports --------------------------
import rospy
from std_msgs.msg import String
import requests

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------- Class ----------------------------
class CameraMover:
    """
    # Camera Mover
    This node reads information from a ROS topic and moves the left camera.

    ## Attributes
    - rate (`rospy.Rate`): rate at which the node will run
    - command (`str`): command to be sent to the cameras
    - commands (`dict`): dictionary of commands

    ## Methods
    - __init__(): initialize node, subscriber and commands
    - show_keys(): show the keys (instructions) to move the camera
    - callback(): callback function for the subscriber
    - run(): main loop

    ## Subscribers
    - `/left_camera/cmd`: topic to receive commands
    """

    def __init__(self):
        """Initialize node and subscriber, as well as the commands."""
        # * Initialize node
        rospy.init_node("kauil_l_camera_position_listener")
        self.rate = rospy.Rate(FS)

        # * Subscribe to command topic
        rospy.Subscriber("/left_camera/cmd", String, self.callback)

        # * Initialize commands
        self.command = None
        self.commands = {
            "w": "http://192.168.1.101/decoder_control.cgi?command=0[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "a": "http://192.168.1.101/decoder_control.cgi?command=6[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "d": "http://192.168.1.101/decoder_control.cgi?command=4[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "s": "http://192.168.1.101/decoder_control.cgi?command=2[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
        }

    def show_keys(self):
        """Show the keys (instructions) to move the camera."""
        print("w: UP Left")
        print("a: LFT Left")
        print("s: DWN Left")
        print("d: RHT Left")

    def callback(self, msg):
        """Callback function for the command subscriber

        Args:
            msg (String): message received from the topic
        """
        self.command = msg.data

    def run(self, verbose=False):
        """Main loop

        Args:
            verbose (bool, optional): Whether to show the keys (instructions) to move the camera. Defaults to False.
        """
        if verbose:
            self.show_keys()
        try:
            while not rospy.is_shutdown():
                if self.command is not None and self.command != "":
                    # * Send request
                    req = self.commands[self.command]
                    requests.get(req, auth=("admin", "pass"))
                    self.command = ""

                # * Sleep
                self.rate.sleep()

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    mover = CameraMover()
    verbose = True
    mover.run(verbose=verbose)
