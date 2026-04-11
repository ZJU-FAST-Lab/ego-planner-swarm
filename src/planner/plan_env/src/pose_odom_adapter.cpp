#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace
{

class PoseOdomAdapter
{
public:
  explicit PoseOdomAdapter(const ros::NodeHandle& nh)
      : nh_(nh)
  {
    nh_.param("input_type", input_type_, std::string("odom"));
    nh_.param("input_topic", input_topic_, std::string("/mavros/local_position/odom"));
    nh_.param("output_topic", output_topic_, std::string("/fcu/odom"));
    nh_.param("frame_id", frame_id_, std::string("world"));
    nh_.param("child_frame_id", child_frame_id_, std::string("base_link"));
    nh_.param("override_frame_id", override_frame_id_, true);
    nh_.param("override_child_frame_id", override_child_frame_id_, true);
    nh_.param("zero_twist_on_pose_input", zero_twist_on_pose_input_, true);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 20);

    if (input_type_ == "pose")
    {
      pose_sub_ = nh_.subscribe(input_topic_, 50, &PoseOdomAdapter::poseCallback, this);
    }
    else if (input_type_ == "odom")
    {
      odom_sub_ = nh_.subscribe(input_topic_, 50, &PoseOdomAdapter::odomCallback, this);
    }
    else
    {
      ROS_FATAL("Unsupported input_type '%s'. Expected 'odom' or 'pose'.", input_type_.c_str());
      ros::shutdown();
      return;
    }

    ROS_INFO("pose_odom_adapter: input_type=%s, input=%s, output=%s, frame_id=%s, child_frame_id=%s",
             input_type_.c_str(), input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
             child_frame_id_.c_str());
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    nav_msgs::Odometry out = *msg;
    if (override_frame_id_)
    {
      out.header.frame_id = frame_id_;
    }
    if (override_child_frame_id_)
    {
      out.child_frame_id = child_frame_id_;
    }
    odom_pub_.publish(out);

    if (!logged_first_msg_)
    {
      ROS_INFO("pose_odom_adapter: received first odom message on %s", input_topic_.c_str());
      logged_first_msg_ = true;
    }
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    nav_msgs::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = override_frame_id_ ? frame_id_ : msg->header.frame_id;
    out.child_frame_id = override_child_frame_id_ ? child_frame_id_ : child_frame_id_;
    out.pose.pose = msg->pose;

    if (zero_twist_on_pose_input_)
    {
      out.twist.twist.linear.x = 0.0;
      out.twist.twist.linear.y = 0.0;
      out.twist.twist.linear.z = 0.0;
      out.twist.twist.angular.x = 0.0;
      out.twist.twist.angular.y = 0.0;
      out.twist.twist.angular.z = 0.0;
    }

    odom_pub_.publish(out);

    if (!logged_first_msg_)
    {
      ROS_INFO("pose_odom_adapter: received first pose message on %s", input_topic_.c_str());
      logged_first_msg_ = true;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;

  std::string input_type_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  bool override_frame_id_{true};
  bool override_child_frame_id_{true};
  bool zero_twist_on_pose_input_{true};
  bool logged_first_msg_{false};
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_odom_adapter");
  ros::NodeHandle nh("~");

  PoseOdomAdapter adapter(nh);
  ros::spin();
  return 0;
}
