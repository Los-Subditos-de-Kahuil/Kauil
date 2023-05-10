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
        rospy.init_node("kauil_r_camera_position_listener")

        rospy.Subscriber("/right_camera/cmd", String,self.callback)
        
        self.command = None

        self.commands = {
                         'j' : "http://192.168.1.102/decoder_control.cgi?command=6[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         'l' : "http://192.168.1.102/decoder_control.cgi?command=4[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         'k' : "http://192.168.1.102/decoder_control.cgi?command=2[&onestep=1&degree=5&user=admin&pwd=&next_url=]",
                         'i' : "http://192.168.1.102/decoder_control.cgi?command=0[&onestep=1&degree=5&user=admin&pwd=&next_url=]"}

        self.rate = rospy.Rate(FS)

    def show_keys(self):
        print("i: UP Right")
        print("j: LFT Right")
        print("k: DWN Right")
        print("l: RHT Right")

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