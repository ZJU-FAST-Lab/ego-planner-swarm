#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"


rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr _cmd_sub;

quadrotor_msgs::msg::PositionCommand _cmd;
double init_x, init_y, init_z;

bool rcv_cmd = false;

// 接收位置指令信息的回调函数
void rcvPosCmdCallBack(const quadrotor_msgs::msg::PositionCommand cmd)
{
    rcv_cmd = true;
    _cmd = cmd;
}

// 发布里程计信息的函数
void pubOdom()
{
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = rclcpp::Clock().now();
    odom.header.frame_id = "world";

    if (rcv_cmd)
    {
        // 更新位置信息
        odom.pose.pose.position.x = _cmd.position.x;
        odom.pose.pose.position.y = _cmd.position.y;
        odom.pose.pose.position.z = _cmd.position.z;

        // 计算无人机的姿态方向
        Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8 * Eigen::Vector3d(0, 0, 1);
        Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
        Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
        Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
        Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
        Eigen::Vector3d zB = xB.cross(yB);
        Eigen::Matrix3d R;
        R.col(0) = xB;
        R.col(1) = yB;
        R.col(2) = zB;
        Eigen::Quaterniond q(R);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();

        // 更新速度和加速度
        odom.twist.twist.linear.x = _cmd.velocity.x;
        odom.twist.twist.linear.y = _cmd.velocity.y;
        odom.twist.twist.linear.z = _cmd.velocity.z;

        odom.twist.twist.angular.x = _cmd.acceleration.x;
        odom.twist.twist.angular.y = _cmd.acceleration.y;
        odom.twist.twist.angular.z = _cmd.acceleration.z;
    }
    else
    {
        // 如果没有接收到指令，则使用初始状态
        odom.pose.pose.position.x = init_x;
        odom.pose.pose.position.y = init_y;
        odom.pose.pose.position.z = init_z;

        odom.pose.pose.orientation.w = 1.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
    }

    // 发布里程计信息
    _odom_pub->publish(odom);
}


int main(int argc, char *argv[])
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odom_generator");

    // 读取参数
    node->declare_parameter("init_x", 0.0);
    node->declare_parameter("init_y", 0.0);
    node->declare_parameter("init_z", 0.0);
    node->get_parameter("init_x", init_x);
    node->get_parameter("init_y", init_y);
    node->get_parameter("init_z", init_z);

    // 创建订阅者和发布者
    _cmd_sub = node->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "command", 1, rcvPosCmdCallBack);
    _odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

    // 主循环，发布里程计信息
    rclcpp::Rate rate(100);  // 100Hz
    bool status = rclcpp::ok();
    while (status)
    {
        pubOdom();
        rclcpp::spin_some(node);
        status = rclcpp::ok();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}