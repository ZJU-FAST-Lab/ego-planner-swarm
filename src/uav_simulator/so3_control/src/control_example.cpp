#include <Eigen/Eigen>
#include "quadrotor_msgs/msg/position_command.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <limits>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = rclcpp::Node::make_shared("quad_sim_example");

  // 创建发布者
  auto cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);

  // 延迟2秒钟
  rclcpp::sleep_for(2s);

  while (rclcpp::ok())
  {
    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      auto cmd = quadrotor_msgs::msg::PositionCommand();
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub->publish(cmd);
      rclcpp::sleep_for(10ms);

      // 处理回调
      rclcpp::spin_some(node);  
    }

    /*** example 2: velocity control ***/
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      auto cmd = quadrotor_msgs::msg::PositionCommand();
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // Disable lower-order commands by setting to NaN
      cmd.position.y = std::numeric_limits<float>::quiet_NaN();
      cmd.position.z = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub->publish(cmd);
      rclcpp::sleep_for(10ms);

      // 处理回调
      rclcpp::spin_some(node);  
    }

    /*** example 3: acceleration control ***/
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      auto cmd = quadrotor_msgs::msg::PositionCommand();
      cmd.position.x = std::numeric_limits<float>::quiet_NaN();
      cmd.position.y = std::numeric_limits<float>::quiet_NaN();
      cmd.position.z = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub->publish(cmd);
      rclcpp::sleep_for(10ms);

      // 处理回调
      rclcpp::spin_some(node);  
    }
  }

  rclcpp::shutdown();
  return 0;
}