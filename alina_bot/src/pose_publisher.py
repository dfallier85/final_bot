#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# Publishes the robot's initial estimated position on the map
def publish_initial_pose():
    # Create a publisher for the /initialpose topic
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Give time for subscribers to connect

    # Define the initial pose message
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"  # Pose is relative to the map frame

    # Set position (x, y) and orientation (quaternion, facing forward)
    initial_pose.pose.pose.position.x = 1.0
    initial_pose.pose.pose.position.y = 1.0
    initial_pose.pose.pose.orientation.w = 1.0

    # Set basic covariance values for x, y, and yaw (theta)
    initial_pose.pose.covariance[0] = 0.25  # x variance
    initial_pose.pose.covariance[7] = 0.25  # y variance
    initial_pose.pose.covariance[35] = 0.06853891945200942  # yaw variance

    pub.publish(initial_pose)
    rospy.loginfo("Initial pose published.")

# Publishes a navigation goal position
def publish_goal_pose():
    # Create a publisher for the /move_base_simple/goal topic
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)

    # Define the goal pose message
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"  # Goal is relative to the map frame

    # Set target position (x, y) and orientation (facing forward)
    goal.pose.position.x = 4.0
    goal.pose.position.y = 2.0
    goal.pose.orientation.w = 1.0

    pub.publish(goal)
    rospy.loginfo("Goal pose published.")

# Main function
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pose_publisher', anonymous=True)

    rospy.sleep(2)  # Give time for everything to initialize properly
    publish_initial_pose()
    rospy.sleep(1)  # Small delay between publishes
    publish_goal_pose()

