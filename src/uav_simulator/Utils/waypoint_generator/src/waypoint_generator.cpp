#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "waypoint_generator/sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

class WaypointGenerator : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    WaypointGenerator(std::string name) : Node(name)
    {
        trigged_time = rclcpp::Time(0);

        this->declare_parameter("waypoint_type", string("manual"));
        this->get_parameter("waypoint_type", waypoint_type);

        auto sub1 = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
        auto sub2 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10, std::bind(&WaypointGenerator::goal_callback, this, std::placeholders::_1));
        auto sub3 = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "traj_start_trigger", 10, std::bind(&WaypointGenerator::traj_start_trigger_callback, this, std::placeholders::_1));

        pub1 = this->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
        pub2 = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);
    }

private:
    // 变量
    string waypoint_type = string("manual");
    bool is_odom_ready;
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::Path waypoints;

    // series waypoint needed
    std::deque<nav_msgs::msg::Path> waypointSegments;
    rclcpp::Time trigged_time;

    // 声明发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;

    void load_seg(int segid, const rclcpp::Time &time_base)
    {
        std::string seg_str = boost::str(bfmt("seg%d/") % segid);
        double yaw;
        double time_of_start = 0.0;

        RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);

        // 参数获取并断言
        rcpputils::assert_true(this->get_parameter(seg_str + "yaw", yaw));
        rcpputils::assert_true(
            (yaw > -3.1499999) && (yaw < 3.14999999),
            (boost::format("Yaw out of range: %.3f") % yaw).str());
        rcpputils::assert_true(this->get_parameter(seg_str + "time_of_start", time_of_start));
        rcpputils::assert_true(time_of_start >= 0.0);

        std::vector<double> ptx, pty, ptz;
        rcpputils::assert_true(this->get_parameter(seg_str + "x", ptx));
        rcpputils::assert_true(this->get_parameter(seg_str + "y", pty));
        rcpputils::assert_true(this->get_parameter(seg_str + "z", ptz));

        rcpputils::assert_true(!ptx.empty());
        rcpputils::assert_true((ptx.size() == pty.size()) && (ptx.size() == ptz.size()));

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_of_start);

        // 计算初始偏航角
        // 使用tf2提取yaw
        // 从消息转换为tf2四元数
        tf2::Quaternion quat;
        tf2::fromMsg(odom.pose.pose.orientation, quat);

        // mmz @todo: 不确定这样转换是否正确
        // 计算yaw角度，使用tf2::Quaternion的getAngle方法获取四元数的角度
        double roll_, pitch_, yaw_;
        tf2::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_); // 转换为roll, pitch, yaw角度
        double baseyaw = yaw_;

        // 生成路径点
        for (size_t k = 0; k < ptx.size(); ++k)
        {
            geometry_msgs::msg::PoseStamped pt;

            // 设置四元数
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, baseyaw + yaw);
            pt.pose.orientation = tf2::toMsg(quat);

            // 坐标转换
            Eigen::Vector2d dp(ptx.at(k), pty.at(k));
            Eigen::Vector2d rdp;
            rdp.x() = std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
            rdp.y() = -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();

            pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
            pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
            pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;

            path_msg.poses.push_back(pt);
        }

        waypointSegments.push_back(path_msg);
    }

    void load_waypoints(const rclcpp::Time &time_base)
    {
        int seg_cnt = 0;
        waypointSegments.clear();
        rcpputils::assert_true(this->get_parameter("segment_cnt", seg_cnt));
        for (int i = 0; i < seg_cnt; ++i)
        {
            load_seg(i, time_base);
            if (i > 0)
            {
                rcpputils::assert_true(rclcpp::Time(waypointSegments[i - 1].header.stamp) < rclcpp::Time(waypointSegments[i].header.stamp));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Overall load %zu segments", waypointSegments.size());
    }

    void publish_waypoints()
    {
        waypoints.header.frame_id = std::string("world");
        waypoints.header.stamp = rclcpp::Clock().now();
        pub1->publish(waypoints);
        geometry_msgs::msg::PoseStamped init_pose;
        init_pose.header = odom.header;
        init_pose.pose = odom.pose.pose;
        waypoints.poses.insert(waypoints.poses.begin(), init_pose);
        // pub2.publish(waypoints);
        waypoints.poses.clear();
    }

    void publish_waypoints_vis()
    {
        nav_msgs::msg::Path wp_vis = waypoints;
        geometry_msgs::msg::PoseArray poseArray;
        poseArray.header.frame_id = std::string("world");
        poseArray.header.stamp = rclcpp::Clock().now();

        {
            geometry_msgs::msg::Pose init_pose;
            init_pose = odom.pose.pose;
            poseArray.poses.push_back(init_pose);
        }

        for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it)
        {
            geometry_msgs::msg::Pose p;
            p = it->pose;
            poseArray.poses.push_back(p);
        }
        pub2->publish(poseArray);
    }

    // 回调函数
    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr &msg)
    {
        is_odom_ready = true;
        odom = *msg;

        if (waypointSegments.size())
        {
            rclcpp::Time expected_time = waypointSegments.front().header.stamp;
            if (rclcpp::Time(odom.header.stamp) >= expected_time)
            {
                waypoints = waypointSegments.front();

                std::stringstream ss;
                ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
                for (auto &pose_stamped : waypoints.poses)
                {
                    ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                              pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                              pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                              pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                              pose_stamped.pose.orientation.z
                       << std::endl;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), ss.str());

                publish_waypoints_vis();
                publish_waypoints();

                waypointSegments.pop_front();
            }
        }
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
    {
        /*    if (!is_odom_ready) {
                ROS_ERROR("[waypoint_generator] No odom!");
                return;
            }*/

        trigged_time = rclcpp::Clock().now(); // odom.header.stamp;
        // ROS_ASSERT(trigged_time > ros::Time(0));

        // ros::NodeHandle n("~");
        // n.param("waypoint_type", waypoint_type, string("manual"));
        this->get_parameter("waypoint_type", waypoint_type);

        if (waypoint_type == string("circle"))
        {
            waypoints = circle();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == string("eight"))
        {
            waypoints = eight();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == string("points"))
        {
            waypoints = point();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == string("series"))
        {
            load_waypoints(trigged_time);
        }
        else if (waypoint_type == string("manual-lonely-waypoint"))
        {
            if (msg->pose.position.z > -0.1)
            {
                // if height > 0, it's a valid goal;
                geometry_msgs::msg::PoseStamped pt = *msg;
                waypoints.poses.clear();
                waypoints.poses.push_back(pt);
                publish_waypoints_vis();
                publish_waypoints();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
            }
        }
        else
        {
            if (msg->pose.position.z > 0)
            {
                // if height > 0, it's a normal goal;
                geometry_msgs::msg::PoseStamped pt = *msg;
                if (waypoint_type == string("noyaw"))
                {
                    // double yaw = tf::getYaw(odom.pose.pose.orientation);
                    // 从消息转换为tf2四元数
                    tf2::Quaternion quat;
                    tf2::fromMsg(odom.pose.pose.orientation, quat);

                    // mmz @todo: 不确定这样转换是否正确
                    // 计算yaw角度，使用tf2::Quaternion的getAngle方法获取四元数的角度
                    double roll_, pitch_, yaw_;
                    tf2::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_); // 转换为roll, pitch, yaw角度
                    double yaw = yaw;
                    // pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    tf2::Quaternion quaternion;
                    quaternion.setRPY(0.0, 0.0, yaw); // Roll, Pitch, Yaw
                    pt.pose.orientation = tf2::toMsg(quaternion);
                }
                waypoints.poses.push_back(pt);
                publish_waypoints_vis();
            }
            else if (msg->pose.position.z > -1.0)
            {
                // if 0 > height > -1.0, remove last goal;
                if (waypoints.poses.size() >= 1)
                {
                    waypoints.poses.erase(std::prev(waypoints.poses.end()));
                }
                publish_waypoints_vis();
            }
            else
            {
                // if -1.0 > height, end of input
                if (waypoints.poses.size() >= 1)
                {
                    publish_waypoints_vis();
                    publish_waypoints();
                }
            }
        }
    }

    void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped &msg)
    {
        if (!is_odom_ready)
        {
            RCLCPP_ERROR(this->get_logger(), "[waypoint_generator] No odom!");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "[waypoint_generator] Trigger!");
        static rclcpp::Time trigged_time = odom.header.stamp;
        // rclcpp::Time current_time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME);
        rcpputils::assert_true(trigged_time > rclcpp::Time(0));

        // Declare and get the waypoint type parameter
        // std::string waypoint_type = this->declare_parameter<std::string>("waypoint_type", "manual");
        this->get_parameter("waypoint_type", waypoint_type);

        RCLCPP_ERROR_STREAM(this->get_logger(), "Pattern " << waypoint_type << " generated!");
        if (waypoint_type == "free")
        {
            waypoints = point();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == "circle")
        {
            waypoints = circle();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == "eight")
        {
            waypoints = eight();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == "point")
        {
            waypoints = point();
            publish_waypoints_vis();
            publish_waypoints();
        }
        else if (waypoint_type == "series")
        {
            load_waypoints(trigged_time);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointGenerator>("waypoint_generator");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
