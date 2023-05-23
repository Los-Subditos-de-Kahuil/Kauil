#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32

class ArduinoMotorNode:
    def __init__(self, port, baud_rate):
        rospy.init_node('rpm_reader')
        self.serial_port = serial.Serial(port, baud_rate, timeout=1)
        self.pub_wr = rospy.Publisher('/wr', Float32, queue_size=10)
        self.pub_wl = rospy.Publisher('/wl', Float32, queue_size=10)
        rospy.Subscriber('arduino_data', String, self.arduino_callback)

    def arduino_callback(self, data):
        velocities = data.data.split(',')
        if len(velocities) == 2:
            try:
                wr = float(velocities[0])
                wl = float(velocities[1])
                self.pub_wr.publish(wr)
                self.pub_wl.publish(wl)
            except ValueError:
                rospy.logwarn("Invalid velocity values received")
        else:
            rospy.logwarn("Invalid data received from Arduino")

    def run(self):
        rospy.spin()
        self.serial_port.close()

if __name__ == '__main__':
    port = "/dev/ttyUSB0"
    baud_rate = 9600
    node = ArduinoMotorNode(port, baud_rate)
    node.run()
