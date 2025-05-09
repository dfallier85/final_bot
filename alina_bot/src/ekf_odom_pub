#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// Robot parameters
const double TICKS_PER_REV = 360.0;
const double WHEEL_RADIUS = 0.05;           // meters
const double WHEEL_BASE = 0.4;              // distance between wheels in meters
const double TICK_DISTANCE = 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;

// Encoder state
int last_left_ticks = 0;
int last_right_ticks = 0;
bool left_init = false;
bool right_init = false;

// Pose state
double x = 0.0, y = 0.0, theta = 0.0;

ros::Publisher odom_pub;
tf::TransformBroadcaster* tf_broadcaster_ptr;

ros::Time last_time;

void publish_odometry(double d_left, double d_right, ros::Time current_time) {
  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  double d_center = (d_left + d_right) / 2.0;
  double d_theta = (d_right - d_left) / WHEEL_BASE;

  x += d_center * cos(theta + d_theta / 2.0);
  y += d_center * sin(theta + d_theta / 2.0);
  theta += d_theta;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  // Publish TF
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp = current_time;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_link";

  odom_tf.transform.translation.x = x;
  odom_tf.transform.translation.y = y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;

  tf_broadcaster_ptr->sendTransform(odom_tf);

  // Publish Odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = d_center / dt;
  odom.twist.twist.angular.z = d_theta / dt;

  odom_pub.publish(odom);
}

void leftCallback(const std_msgs::Int16::ConstPtr& msg) {
  static int prev = 0;
  if (!left_init) {
    last_left_ticks = msg->data;
    left_init = true;
    return;
  }

  int delta = msg->data - last_left_ticks;
  last_left_ticks = msg->data;

  double d_left = delta * TICK_DISTANCE;
  publish_odometry(d_left, 0.0, ros::Time::now());
}

void rightCallback(const std_msgs::Int16::ConstPtr& msg) {
  static int prev = 0;
  if (!right_init) {
    last_right_ticks = msg->data;
    right_init = true;
    return;
  }

  int delta = msg->data - last_right_ticks;
  last_right_ticks = msg->data;

  double d_right = delta * TICK_DISTANCE;
  publish_odometry(0.0, d_right, ros::Time::now());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle nh;

  tf_broadcaster_ptr = new tf::TransformBroadcaster();

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

  ros::Subscriber left_sub = nh.subscribe("/left_ticks", 10, leftCallback);
  ros::Subscriber right_sub = nh.subscribe("/right_ticks", 10, rightCallback);

  last_time = ros::Time::now();
  ros::spin();

  delete tf_broadcaster_ptr;
  return 0;
}
