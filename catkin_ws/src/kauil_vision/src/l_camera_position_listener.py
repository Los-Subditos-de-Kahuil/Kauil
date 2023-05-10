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
class CameraMover():

    def __init__(self):
        rospy.init_node("kauil_l_camera_position_listener")

        rospy.Subscriber("/left_camera/cmd", String,self.callback)
        
        self.command = None

        self.commands = {'w' : "http://192.168.1.101/decoder_control.cgi?command=0[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         'a' : "http://192.168.1.101/decoder_control.cgi?command=6[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         'd' : "http://192.168.1.101/decoder_control.cgi?command=4[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         's' : "http://192.168.1.101/decoder_control.cgi?command=2[&onestep=1&degree=5&user=admin&pwd=&next_url=]"}

        self.rate = rospy.Rate(FS)

    def show_keys(self):
        print("w: UP Left")
        print("a: LFT Left")
        print("s: DWN Left")
        print("d: RHT Left")

    def run(self, verbose=False):
        if verbose:
            self.show_keys()
        try:
            while not rospy.is_shutdown():
                if self.command is not None and self.command != '':
                    req = self.commands[self.command]
                    requests.get(req, auth=('admin', 'pass'))
                    self.command = ''

                self.rate.sleep()
        
        except rospy.exceptions.ROSInterruptException:
            pass

    def callback(self, msg):
        self.command = msg.data

if __name__ == "__main__":
    mover = CameraMover()
    verbose = True
    mover.run(verbose=verbose)