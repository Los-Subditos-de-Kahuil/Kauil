#!/usr/bin/env python
"""
Reads RPM values from Arduino, converts them to rad/s, the transforms them from the motor to the wheel velocity, and finally publishes them to the /wr and /wl topics.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/05/25
"""
import rospy
import serial
from std_msgs.msg import Float32


class ArduinoMotorNode:
    """
    # Arduino Motor Node
    Reads RPM values from Arduino, converts them to rad/s, the transforms them from the motor to the wheel velocity, and finally publishes them to the /wr and /wl topics.

    ## Attributes
    - serial_port (`serial.Serial`): Serial connection to Arduino Leonardo
    - pub_wr (`rospy.Publisher`): Publisher to the /wr topic
    - pub_wl (`rospy.Publisher`): Publisher to the /wl topic

    ## Methods
    - __init__(`port`, `baud_rate`): Initializes the node, serial connection and publishers
    - end_callback(): Callback function that is called when the node is shutdown. Closes the serial port.
    - read_serial_data(): Reads data from serial port, transforms it and publishes it to the /wr and /wl topics.
    - rpm_to_rad_per_sec(`rpm`): Converts RPM to rad/s
    - mechanical_transformation(`rad_per_sec`): Transforms from motor to wheel velocity
    - run(): Runs the node. Reads data from serial port, transforms it and publishes it to the /wr and /wl topics. Closes the serial port when the node is shutdown.

    ## Publishers
    - /wr: Right wheel velocity
    - /wl: Left wheel velocity
    """

    def __init__(self, port, baud_rate):
        """Initializes the node, serial connection and publishers

        Args:
            port (str): Serial port to connect to Arduino Leonardo
            baud_rate (int): Baud rate of the serial connection
        """
        # * Initialize node
        rospy.init_node("rpm_reader")

        # * Initialize serial port
        self.serial_connection = serial.Serial(port, baud_rate, timeout=1)

        # * Publishers
        self.pub_wr = rospy.Publisher("/wr", Float32, queue_size=10)
        self.pub_wl = rospy.Publisher("/wl", Float32, queue_size=10)

        # * End callback
        rospy.on_shutdown(self.end_callback)

    def end_callback(self):
        """Callback function that is called when the node is shutdown. Closes the serial port."""
        self.serial_connection.close()

    def read_serial_data(self):
        """Reads data from serial port, transforms it and publishes it to the /wr and /wl topics."""
        while not rospy.is_shutdown():
            if self.serial_connection.in_waiting > 0:
                # * Read data from serial port
                data = self.serial_connection.readline().decode().rstrip("\r\n")
                velocities = data.split(",")
                if len(velocities) == 2:
                    try:
                        # * Convert RPM to rad/s
                        wr_rpm = float(velocities[0])
                        wl_rpm = float(velocities[1])
                        wr_rad_per_sec = self.rpm_to_rad_per_sec(wr_rpm)
                        wl_rad_per_sec = self.rpm_to_rad_per_sec(wl_rpm)

                        # * Transform from motor to wheel velocity
                        wr_final = self.mechanical_transformation(wr_rad_per_sec)
                        wl_final = self.mechanical_transformation(wl_rad_per_sec)

                        # * Publish data
                        self.pub_wr.publish(wr_final)
                        self.pub_wl.publish(wl_final)
                    except ValueError:
                        rospy.logwarn("Invalid velocity values received")
                else:
                    rospy.logwarn("Invalid data received from Arduino")

    def rpm_to_rad_per_sec(self, rpm):
        """Converts RPM to rad/s

        Args:
            rpm (float): RPM value

        Returns:
            float: rad/s value
        """
        return rpm * 0.01047  # Conversion factor from RPM to rad/s

    def mechanical_transformation(self, rad_per_sec):
        """Transforms from motor to wheel velocity

        Args:
            rad_per_sec (float): motor velocity

        Returns:
            float: wheel velocity
        """
        return rad_per_sec * 0.78  # Conversion factor from motor to wheel velocity

    def run(self):
        """Runs the node. Reads data from serial port, transforms it and publishes it to the /wr and /wl topics. Closes the serial port when the node is shutdown."""
        self.read_serial_data()
        self.serial_connection.close()


if __name__ == "__main__":
    try:
        ArduinoMotorNode(port="/dev/ttyACM1", baud_rate=9600).run()
    except rospy.ROSInterruptException:
        pass
