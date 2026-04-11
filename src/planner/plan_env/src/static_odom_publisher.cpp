#include <algorithm>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_odom_publisher");
  ros::NodeHandle nh("~");

  std::string odom_topic;
  std::string frame_id;
  std::string child_frame_id;
  double publish_rate;
  double pos_x, pos_y, pos_z;
  double quat_x, quat_y, quat_z, quat_w;

  // TEMP: This static odom is only for host-side OAK-D validation.
  nh.param("odom_topic", odom_topic, std::string("odom"));
  nh.param("frame_id", frame_id, std::string("world"));
  nh.param("child_frame_id", child_frame_id, std::string("base_link"));
  nh.param("publish_rate", publish_rate, 20.0);
  nh.param("pos_x", pos_x, 0.0);
  nh.param("pos_y", pos_y, 0.0);
  nh.param("pos_z", pos_z, 1.0);
  nh.param("quat_x", quat_x, 0.0);
  nh.param("quat_y", quat_y, 0.0);
  nh.param("quat_z", quat_z, 0.0);
  nh.param("quat_w", quat_w, 1.0);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose.position.x = pos_x;
  odom_msg.pose.pose.position.y = pos_y;
  odom_msg.pose.pose.position.z = pos_z;
  odom_msg.pose.pose.orientation.x = quat_x;
  odom_msg.pose.pose.orientation.y = quat_y;
  odom_msg.pose.pose.orientation.z = quat_z;
  odom_msg.pose.pose.orientation.w = quat_w;
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  ROS_INFO("Publishing static odom on %s at %.1f Hz, pose=(%.3f, %.3f, %.3f)",
           odom_topic.c_str(), publish_rate, pos_x, pos_y, pos_z);

  ros::Rate rate(std::max(publish_rate, 1.0));
  while (ros::ok())
  {
    odom_msg.header.stamp = ros::Time::now();
    odom_pub.publish(odom_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
