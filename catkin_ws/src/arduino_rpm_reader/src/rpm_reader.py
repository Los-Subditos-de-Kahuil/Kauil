#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def rpm_callback(data):
    rpm = data.data
    rospy.loginfo("Velocidad angular en RPM: %s", rpm)

def rpm_reader():
    rospy.init_node('rpm_reader', anonymous=True)
    rospy.Subscriber('rpm_topic', String, rpm_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rpm_reader()
    except rospy.ROSInterruptException:
        pass

