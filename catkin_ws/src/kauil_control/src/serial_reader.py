#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Float32

class ArduinoMotorNode:
    def __init__(self, port, rate):
        rospy.init_node('rpm_reader')
        self.serial_port = serial.Serial(port, rate, timeout=1)
        self.pub_wr = rospy.Publisher('/wr', Float32, queue_size=10)
        self.pub_wl = rospy.Publisher('/wl', Float32, queue_size=10)

    def read_serial_data(self):
        while not rospy.is_shutdown():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode().rstrip('\r\n')
                velocities = data.split(',')
                if len(velocities) == 2:
                    try:
                        wr_rpm = float(velocities[0])
                        wl_rpm = float(velocities[1])
                        wr_rad_per_sec = self.rpm_to_rad_per_sec(wr_rpm)
                        wl_rad_per_sec = self.rpm_to_rad_per_sec(wl_rpm)
			wr_final = self.mechanical_transformation(wr_rad_per_sec)
			wl_final = self.mechanical_transformation(wl_rad_per_sec)
                        self.pub_wr.publish(wr_final)
                        self.pub_wl.publish(wl_final)
                    except ValueError:
                        rospy.logwarn("Invalid velocity values received")
                else:
                    rospy.logwarn("Invalid data received from Arduino")

    def rpm_to_rad_per_sec(self, rpm):
        return rpm * 0.01047  # Factor de conversion de RPM a rad/s

    def mechanical_transformation(self, rad_per_sec):
	return rad_per_sec * 0.78

    def run(self):
        self.read_serial_data()
        self.serial_port.close()

if __name__ == '__main__':
    port = "/dev/ttyACM0" 
    rate = 9600 
    node = ArduinoMotorNode(port, rate)
    node.run()
