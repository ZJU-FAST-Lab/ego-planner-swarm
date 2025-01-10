#include <rclcpp/rclcpp.hpp>
#include "drone_detect/drone_detector.h"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<detect::DroneDetector>("drone_detect");
    node->test();

    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    