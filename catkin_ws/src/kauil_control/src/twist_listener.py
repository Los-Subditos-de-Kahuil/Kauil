#! /usr/bin/env python
"""
Listen to the twist required and communicate movement values to the Arduino through the serial port.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/04/25
"""
# ------------------------- Imports --------------------------
import rospy
import serial
import numpy as np

from geometry_msgs.msg import Twist

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------ Functions -------------------------
def v(x):
    """Convert linear velocity to PWM value

    Args:
        x (float): Linear velocity

    Returns:
        int: PWM value
    """
    x *= 1.0 / 0.28
    x = -min(max(x, -1.0), 1.0)
    return int(471 * (x) + 1432)


def w(z):
    """Convert angular velocity to PWM value

    Args:
        z (float): Angular velocity

    Returns:
        int: PWM value
    """
    z *= 1.0 / (np.pi / 6)
    z = min(max(z, -1.0), 1.0)
    return int(447 * (z) + 1393)


# ------------------------- Class ----------------------------
class Converter:
    """
    # Converter
    Listen to the twist required and communicate corresponding PWM values to the Arduino through the serial port.

    ## Attributes
    - rate (`rospy.Rate`): Rate at which the node runs
    - v (`float`): Linear velocity
    - w (`float`): Angular velocity
    - arduino (`serial.Serial`): Serial connection to Arduino Nano

    ## Methods
    - __init__(): Initializes the node, serial connection and subscribers, and sets initial values
    - callback_twist(`msg`): Callback function for the /cmd_vel topic
    - arduino_write(`message`): Write message to Arduino
    - end_callback(): Callback on shutdown
    - run(): Runs the node. Send PWM values to the Arduino to achieve the desired movement

    ## Subscribers
    - /cmd_vel: Twist message with the desired movement
    """

    def __init__(self):
        """Initializes the node, serial connection and subscribers, and sets initial values"""
        # * Initialize node
        rospy.init_node("kauil_control_manager")
        self.rate = rospy.Rate(FS)

        # * Initial values
        self.v, self.w = 0.0, 0.0

        # * Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.callback_twist)

        # * Arduino
        self.arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=None)

    def callback_twist(self, msg):
        """Callback function for the /cmd_vel topic

        Args:
            msg (Twist): Twist message
        """
        self.v = msg.linear.x
        self.w = msg.angular.z

    def arduino_write(self, message):
        """Write message to Arduino

        Args:
            message (str): Message to write
        """
        print(message)
        self.arduino.write(message.encode("utf-8"))

    def end_callback(self):
        """Callback on shutdown"""
        # Stop Kauil
        x = v(0.0)
        z = w(0.0)
        x_msg = "CH2:" + str(x)
        z_msg = "CH1:" + str(z)
        self.arduino_write(x_msg)
        self.arduino_write(z_msg)
        self.rate.sleep()

    def run(self):
        """Run the node. Send PWM values to the Arduino to achieve the desired movement"""
        try:
            while not rospy.is_shutdown():
                # * Convert to PWM
                x = v(self.v)
                z = w(self.w)

                # * Prepare message
                x_msg = "CH2:" + str(x) + "\n"
                z_msg = "CH1:" + str(z) + "\n"

                # * Send message
                self.arduino.flush()
                self.arduino_write(x_msg)
                self.arduino_write(z_msg)

                # * Sleep
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass


# --------------------------- Main ---------------------------
if __name__ == "__main__":
    converter = Converter()
    rospy.on_shutdown(converter.end_callback)
    converter.run()
