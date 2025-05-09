#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

ros::Publisher goal_pub;
ros::Publisher init_pub;

// Convert goal pose from RViz to 2D yaw pose
void handle_goal(const geometry_msgs::PoseStamped &goal) {
  geometry_msgs::PoseStamped simple_goal;
  simple_goal.header = goal.header;
  simple_goal.header.frame_id = "map";  // Change if alina_bot uses something else
  simple_goal.pose.position.x = goal.pose.position.x;
  simple_goal.pose.position.y = goal.pose.position.y;

  tf::Quaternion q(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w
  );

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Store yaw as a single axis quaternion
  tf::Quaternion yaw_quat = tf::createQuaternionFromYaw(yaw);
  simple_goal.pose.orientation.x = yaw_quat.x();
  simple_goal.pose.orientation.y = yaw_quat.y();
  simple_goal.pose.orientation.z = yaw_quat.z();
  simple_goal.pose.orientation.w = yaw_quat.w();

  goal_pub.publish(simple_goal);
}

// Convert initial pose to 2D yaw pose
void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &init_pose) {
  geometry_msgs::PoseStamped simple_init;
  simple_init.header = init_pose.header;
  simple_init.header.frame_id = "map";  // Change if needed
  simple_init.pose.position.x = init_pose.pose.pose.position.x;
  simple_init.pose.position.y = init_pose.pose.pose.position.y;

  tf::Quaternion q(
    init_pose.pose.pose.orientation.x,
    init_pose.pose.pose.orientation.y,
    init_pose.pose.pose.orientation.z,
    init_pose.pose.pose.orientation.w
  );

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  tf::Quaternion yaw_quat = tf::createQuaternionFromYaw(yaw);
  simple_init.pose.orientation.x = yaw_quat.x();
  simple_init.pose.orientation.y = yaw_quat.y();
  simple_init.pose.orientation.z = yaw_quat.z();
  simple_init.pose.orientation.w = yaw_quat.w();

  init_pub.publish(simple_init);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alina_click_to_2d");
  ros::NodeHandle nh;

  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_2d", 10);
  init_pub = nh.advertise<geometry_msgs::PoseStamped>("initial_2d", 10);

  ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal", 10, handle_goal);
  ros::Subscriber init_sub = nh.subscribe("initialpose", 10, handle_initial_pose);

  ros::spin();
  return 0;
}

