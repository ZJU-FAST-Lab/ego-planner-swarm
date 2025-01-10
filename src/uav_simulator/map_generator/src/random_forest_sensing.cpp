#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <random>

#include "rclcpp/rclcpp.hpp"

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
// default_random_engine eng(4);
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

// 定义订阅者和发布者
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _local_map_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::msg::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::msg::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

// 随机地图生成
void RandomMapGenerate() {
    pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  RCLCPP_WARN(rclcpp::get_logger("RandomMapGenerate"), "Finished generate random map"); 

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;

}

// 生成随机障碍，柱形和圆形
// 相较于上面那个函数增加了距离限制和缩放因子
void RandomMapGenerateCylinder() {
    pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  // 生成柱形
  for (int i = 0; i < _obs_num && rclcpp::ok(); i++) {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);
    
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--;
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    obs_position.push_back( Eigen::Vector2d(x,y) );
    

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w*inf) / _resolution);
    double radius = (w*inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  // 生成圆形
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  RCLCPP_WARN(rclcpp::get_logger("RandomMapGenerateCylinder"), "Finished generate random map "); 

  // 将cloudmap转换为一个基于kd tree的点云地图
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

// 里程计信息订阅回调
void rcvOdometryCallback(const nav_msgs::msg::Odometry &odom) {
    if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
    _has_odom = true;

    _state = {
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z,
        odom.twist.twist.linear.x,
        odom.twist.twist.linear.y,
        odom.twist.twist.linear.z,
        0.0,
        0.0,
        0.0
    };
}

int i = 0;
// 发布点云信息
void pubSensedPoints() {
    // 将点云转换为 ROS2 消息格式并发布
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    _all_map_pub->publish(globalMap_pcd);

    return; // 有这个return后续的代码都不会执行

    /* ---------- only publish points around current position ---------- */
    // 只有地图生成完毕且有位置信息（里程计数据）时，才会发布局部地图
    if (!_map_ok || !_has_odom) return;

    pcl::PointCloud<pcl::PointXYZ> localMap;

    // 设置搜索点
    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();

    if (std::isnan(searchPoint.x) || std::isnan(searchPoint.y) || std::isnan(searchPoint.z))
        return;

    // 搜索感知范围内的点并构建局部地图
    if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                    pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pcl::PointXYZ pt = cloudMap.points[pointIdxRadiusSearch[i]];
            localMap.points.push_back(pt);
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("map_server"), "No obstacles.");
        return;
    }

    // 发布局部地图
    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    _local_map_pub->publish(localMap_pcd);
}

// 根据点击的位置，在地图中添加一个随机大小的柱状障碍物，并将其发布为一个局部地图
void clickCallback(const geometry_msgs::msg::PoseStamped &msg) {
    // 提取点的位置并生成障碍物
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;
    double w = rand_w(eng);
    double h;
    pcl::PointXYZ pt_random;

    // 将点击的位置对齐到网格
    x = std::floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = std::floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 计算障碍物的宽度
    int widNum = std::ceil(w / _resolution);

    // 生成障碍物
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            h = rand_h(eng);
            int heiNum = std::ceil(h / _resolution);
            for (int t = -1; t < heiNum; t++) {
                pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = (t + 0.5) * _resolution + 1e-2;
                clicked_cloud_.points.push_back(pt_random);
                cloudMap.points.push_back(pt_random);
            }
        }
    }

    // 更新点云属性并发布局部地图
    clicked_cloud_.width = clicked_cloud_.points.size();
    clicked_cloud_.height = 1;
    clicked_cloud_.is_dense = true;

    pcl::toROSMsg(clicked_cloud_, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    click_map_pub_->publish(localMap_pcd);

    cloudMap.width = cloudMap.points.size();

    return;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("random_map_sensing");

    // 创建发布者
    _local_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/local_cloud", 1);
    _all_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 1);
    click_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_render_node/local_map", 1);

    // 创建订阅者
    _odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("odometry", 50, rcvOdometryCallback);
    // auto click_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal", 10, clickCallback);

    // 声明和获取参数
    node->declare_parameter("init_state_x", 0.0);
    node->declare_parameter("init_state_y", 0.0);
    node->declare_parameter("map/x_size", 50.0);
    node->declare_parameter("map/y_size", 50.0);
    node->declare_parameter("map/z_size", 5.0);
    node->declare_parameter("map/obs_num", 30);
    node->declare_parameter("map/resolution", 0.1);
    node->declare_parameter("map/circle_num", 30);

    node->declare_parameter("ObstacleShape/lower_rad", 0.3);
    node->declare_parameter("ObstacleShape/upper_rad", 0.8);
    node->declare_parameter("ObstacleShape/lower_hei", 3.0);
    node->declare_parameter("ObstacleShape/upper_hei", 7.0);

    node->declare_parameter("ObstacleShape/radius_l", 7.0);
    node->declare_parameter("ObstacleShape/radius_h", 7.0);
    node->declare_parameter("ObstacleShape/z_l", 7.0);
    node->declare_parameter("ObstacleShape/z_h", 7.0);
    node->declare_parameter("ObstacleShape/theta", 7.0);

    node->declare_parameter("sensing/radius", 10.0);
    node->declare_parameter("sensing/rate", 10.0);
    node->declare_parameter("min_distance", 1.0);

    node->get_parameter("init_state_x", _init_x);
    node->get_parameter("init_state_y", _init_y);
    node->get_parameter("map/x_size", _x_size);
    node->get_parameter("map/y_size", _y_size);
    node->get_parameter("map/z_size", _z_size);
    node->get_parameter("map/obs_num", _obs_num);
    node->get_parameter("map/resolution", _resolution);
    node->get_parameter("map/circle_num", circle_num_);

    node->get_parameter("ObstacleShape/lower_rad", _w_l);
    node->get_parameter("ObstacleShape/upper_rad", _w_h);
    node->get_parameter("ObstacleShape/lower_hei", _h_l);
    node->get_parameter("ObstacleShape/upper_hei", _h_h);

    node->get_parameter("ObstacleShape/radius_l", radius_l_);
    node->get_parameter("ObstacleShape/radius_h", radius_h_);
    node->get_parameter("ObstacleShape/z_l", z_l_);
    node->get_parameter("ObstacleShape/z_h", z_h_);
    node->get_parameter("ObstacleShape/theta", theta_);

    node->get_parameter("sensing/radius", _sensing_range);
    node->get_parameter("sensing/rate", _sense_rate);
    node->get_parameter("min_distance", _min_dist);

    // 地图边界和障碍物的设置
    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;
    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;
    _obs_num = std::min(_obs_num, static_cast<int>(_x_size * 10));
    _z_limit = _z_size;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // 初始化随机数生成器
    unsigned int seed = rd();
    // unsigned int seed = 2433201515;
    std::cout << "seed=" << seed << std::endl;
    eng.seed(seed);

    // 生成随机地图
    // RandomMapGenerate();
    RandomMapGenerateCylinder();

    // 设置循环频率并开始主循环
    rclcpp::Rate loop_rate(_sense_rate);
    while (rclcpp::ok()) {
        // 发布感知到的点云数据
        pubSensedPoints();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}