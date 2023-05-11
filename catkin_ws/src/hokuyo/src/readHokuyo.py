#! /usr/bin/env python
# ------------------------- Imports --------------------------
import rospy
import serial

# ------------------------ Variables -------------------------
# * Frequency and period
FS = 50
T = 1.0 / FS


ser = serial.Serial(port='/dev/ttyACM1', 
                                     baudrate=9600, 
                                     timeout=1) # open serial port
rospy.init_node("hokuyo_reader")

rate = rospy.Rate(FS)
print("Hi, master!")
while True:

    try:
        while not rospy.is_shutdown():
            mesage = ser.read()
            print(mesage)

            rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        pass

    