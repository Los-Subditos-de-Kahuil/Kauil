#! /usr/bin/env python
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
import numpy as np

from geometry_msgs.msg import Twist

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS

# ------------------------ Functions -------------------------
def v(x):
    """
    Transform a linear velocity value in m/s to the duration 
    of the duty cycle of the Arduino counter.

    Args:
        x (float): linear velocity

    Returns:
        int: duty cycle length
    """
    #* Knowing max vel is 0.28 m/s transform to 1.0
    x *= (1.0/0.28)
    x = -min(max(x, -1.0), 1.0)
    return int(471*(x) + 1432)

def w(z):
    """
    Transform an angular velocity value in rads/s to the
    duration of the duty cycle of the Arduino counter

    Args:
        z (float): angular velocity

    Returns:
        int: duty cycle length
    """
    #* Knowing max ang vel is pi/6 rads/s
    #* transform to 1.0
    z *= (1.0 / (np.pi / 6))
    z = min(max(z, -1.0), 1.0)
    return int(447*(z) + 1393)


# ------------------------- Class ----------------------------
class Converter():
    """
    Converter from standard movement measurements (m/s, rads/s)
    to duty cycles required by arduino to control the motors.

    ...

    Attributes
    ----------
    v : float
        linear velocity
    
    w : float
        angular velocity

    rate : object (rospy.Rate)
        ros work rate

    arduino : object (serial.Serial)
        serial communicatior with arduino

    Methods
    -------
    callback_twist(msg : object (ros.msg))
        extracts linear and angular velocity from the ROS topic 
        /cmd_vel msg

    arduino_write(message : str)
        message to write through serial to arduino
    
    end_callback()
        actions to do on ROS close
    
    run()
        prepares and sends the message of duty cycles to arduino

    """
    def __init__(self):
        """
        Initialization of the class
        """
        rospy.init_node("kauil_control_manager")

        #* Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.callback_twist)

        self.v, self.w = 0.0, 0.0

        self.rate = rospy.Rate(FS)

        #! Check port and authorization!!!
        self.arduino = serial.Serial(port='/dev/ttyUSB0', 
                                     baudrate=9600, 
                                     timeout=1)

    def callback_twist(self, msg):
        """
        extracts linear and angular velocity from the ROS topic 
        /cmd_vel msg

        Args:
            msg (object (ros.msg)): message of twist
        """
        self.v = msg.linear.x
        self.w = msg.angular.z

    def arduino_write(self, message):
        """
        message to write through serial to arduino

        Args:
            message (str): message to be sent through serial
        """
        self.arduino.write(message.encode('utf-8'))

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
        """prepares and sends the message of duty cycles to arduino"""
        try:
            while not rospy.is_shutdown():
                x = v(self.v)
                z = w(self.w)
                x_msg = "CH1:" + str(x) + "\n"
                z_msg = "CH2:" + str(z) + "\n"
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
