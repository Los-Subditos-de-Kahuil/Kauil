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
    This node reads information from a ROS topic and moves the right camera.

    ## Attributes
    - `rate`: rate at which the node runs
    - `command`: command received from the topic
    - `commands`: dictionary with the commands to move the camera

    ## Methods
    - __init__(): initialize node, subscriber and commands
    - show_keys(): show the keys (instructions) to move the camera
    - callback(): callback function for the commands subscriber
    - run(): main loop

    ## Subscribers
    - `/right_camera/cmd`: topic to receive commands
    """

    def __init__(self):
        """Initialize node and subscriber, as well as the commands."""
        # * Initialize node
        rospy.init_node("kauil_r_camera_position_listener")
        self.rate = rospy.Rate(FS)

        # * Subscribe to command topic
        rospy.Subscriber("/right_camera/cmd", String, self.callback)

        # * Initialize commands
        self.command = None
        self.commands = {
            "j": "http://192.168.1.102/decoder_control.cgi?command=6[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "l": "http://192.168.1.102/decoder_control.cgi?command=4[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "k": "http://192.168.1.102/decoder_control.cgi?command=2[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
            "i": "http://192.168.1.102/decoder_control.cgi?command=0[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
        }

    def show_keys(self):
        """Show the keys (instructions) to move the camera."""
        print("i: UP Right")
        print("j: LFT Right")
        print("k: DWN Right")
        print("l: RHT Right")

    def callback(self, msg):
        """Callback function for the command subscriber

        Args:
            msg (String): message received from the topic
        """
        self.command = msg.data

    def run(self, verbose=False):
        """Main loop.

        Args:
            verbose (bool, optional): whether to show the keys (instructions) or not. Defaults to False.
        """
        if verbose:
            self.show_keys()
        try:
            while not rospy.is_shutdown():
                # * Send request
                if self.command is not None and self.command != "":
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
