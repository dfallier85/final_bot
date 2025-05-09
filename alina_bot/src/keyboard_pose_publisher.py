#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import sys, tty, termios

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def publish_initial_pose(pub):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 1.0
    msg.pose.pose.orientation.w = 1.0
    pub.publish(msg)
    rospy.loginfo("Published initial pose.")

def publish_goal_pose(pub):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = 4.0
    msg.pose.position.y = 2.0
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    rospy.loginfo("Published goal pose.")

if __name__ == '__main__':
    rospy.init_node('keyboard_pose_publisher')
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    print("Press 'i' to publish initial pose, 'g' to publish goal, or 'q' to quit.")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'i':
            publish_initial_pose(init_pub)
        elif key == 'g':
            publish_goal_pose(goal_pub)
        elif key == 'q':
            break

