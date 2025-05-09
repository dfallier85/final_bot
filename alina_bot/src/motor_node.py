#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import serial

# Setup serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
except serial.SerialException:
    rospy.logerr("Motor controller not connected on /dev/ttyUSB0")
    exit()

def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z

    # Simple conversion (customize as needed)
    motor_command = f"{linear:.2f},{angular:.2f}\n"
    ser.write(motor_command.encode('utf-8'))

def motor_node():
    rospy.init_node('motor_node')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Motor node started. Listening to /cmd_vel...")
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_node()
    except rospy.ROSInterruptException:
        pass

