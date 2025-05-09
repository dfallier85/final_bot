#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

class JoyPoseController:
    def __init__(self):
        rospy.init_node('joy_pose_controller')
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linear_scale = rospy.get_param('~linear_scale', 0.5)  # max speed
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)

    def joy_callback(self, data):
        # --- Handle velocity control from joystick axes ---
        twist = Twist()
        twist.linear.x = self.linear_scale * data.axes[1]   # Left stick up/down
        twist.angular.z = self.angular_scale * data.axes[0] # Left stick left/right
        self.cmd_vel_pub.publish(twist)

        # --- Handle button presses for pose publishing ---
        if data.buttons[0] == 1:
            self.publish_initial_pose()
        if data.buttons[1] == 1:
            self.publish_goal_pose()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 1.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.init_pub.publish(msg)
        rospy.loginfo("Initial pose sent from joystick.")

    def publish_goal_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = 4.0
        msg.pose.position.y = 2.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("Goal pose sent from joystick.")

if __name__ == '__main__':
    JoyPoseController()
    rospy.spin()

