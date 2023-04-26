#! usr/bin/env python
"""
Listen the twist required and communicate movement values to the Arduino through the serial port.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/04/25
"""
# ------------------------- Imports --------------------------
import rospy
import serial

from geometry_msgs.msg import Twist

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------ Functions -------------------------
def v(x):
    x = min(max(x, -1.0), 1.0)
    return int(471(x) + 1457)

def w(z):
    z = min(max(z, -1.0), 1.0)
    return int(447(z) + 1433)


# ------------------------- Class ----------------------------
class Converter():
    def __init__(self):
        rospy.init_node("kauil_control_manager")

        #* Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.callback_twist)

        self.v, self.w = 0.0, 0.0

        self.rate = rospy.Rate(FS)

        #! Check port!!!
        self.arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

    def callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def arduino_write(self, message):
        self.arduino.write(bytes(message, 'utf-8'))

    def end_callback(self):
        """Callback on shutdown"""
        # Stop Gothmog
        x = v(0.0)
        z = w(0.0)
        x_msg = "CH1:" + str(x)
        z_msg = "CH2:" + str(z)
        self.arduino_write(x_msg)
        self.arduino_write(z_msg)
        self.rate.sleep()

    def run(self):
        try:
            while not rospy.is_shutdown():
                x = v(self.v)
                z = w(self.w)
                x_msg = "CH1:" + str(x)
                z_msg = "CH2:" + str(z)
                self.arduino_write(x_msg)
                self.arduino_write(z_msg)
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

# --------------------------- Main ---------------------------
if __name__ == "__main__":
    converter = Converter()
    rospy.on_shutdown(converter.end_callback)
    converter.run()
