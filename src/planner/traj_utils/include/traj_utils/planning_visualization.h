#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
// #include <bspline_opt/uniform_bspline.h>
#include <iostream>
// #include <bspline_opt/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;
    // ros::Publisher guide_vector_pub;
    // ros::Publisher intermediate_state_pub;

  public:
    ~PlanningVisualization() {}
    PlanningVisualization(const rclcpp::Node::SharedPtr &node)
      : node_(node)
    {
      // 初始化发布者，调整消息类型和队列大小
      goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>("goal_point", 2);
      global_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("global_list", 2);
      init_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("init_list", 2);
      optimal_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("optimal_list", 2);
      a_star_list_pub = node_->create_publisher<visualization_msgs::msg::Marker>("a_star_list", 20);
    }

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id, bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(std::vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(std::vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(std::vector<std::vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const std::vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif