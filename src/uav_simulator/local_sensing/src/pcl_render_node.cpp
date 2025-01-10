#include <iostream>
#include <fstream>
#include <vector>

// ROS2 依赖项
#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"

// TF 相关
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"

// PCL 相关
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV 和 Eigen 相关
#include <Eigen/Eigen>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

// 自定义头文件
#include "depth_render.cuh"
#include "quadrotor_msgs/msg/position_command.hpp"
using namespace cv;
using namespace std;
using namespace Eigen;

int *depth_hostptr;
cv::Mat depth_mat;

int width, height;
double fx, fy, cx, cy;

DepthRender depthrender; 

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_world;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub;

rclcpp::TimerBase::SharedPtr local_sensing_timer;
rclcpp::TimerBase::SharedPtr estimation_timer;

bool has_global_map = false;
bool has_local_map = false;
bool has_odom = false;

Eigen::Matrix4d cam02body;
Eigen::Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::msg::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate; 
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

rclcpp::Time last_odom_stamp = rclcpp::Time::max();
Eigen::Vector3d last_pose_world;

void render_currentpose();
void render_pcl_world();

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index) 
{
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

    return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
{
    Eigen::Vector3i idx;
    idx(0) = std::min( std::max( int( (pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
    idx(1) = std::min( std::max( int( (pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
    idx(2) = std::min( std::max( int( (pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);              
  
    return idx;
};

void rcvOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  has_odom = true;
  _odom = *odom;
  Eigen::Matrix4d Pose_receive = Eigen::Matrix4d::Identity();

  // 存储得到的姿态信息
  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom->pose.pose.position.x;
  request_position.y() = odom->pose.pose.position.y;
  request_position.z() = odom->pose.pose.position.z;
  request_pose.x() = odom->pose.pose.orientation.x;
  request_pose.y() = odom->pose.pose.orientation.y;
  request_pose.z() = odom->pose.pose.orientation.z;
  request_pose.w() = odom->pose.pose.orientation.w;
  
  Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
  Pose_receive(0,3) = request_position(0);
  Pose_receive(1,3) = request_position(1);
  Pose_receive(2,3) = request_position(2);

  Eigen::Matrix4d body_pose = Pose_receive;

  // 转换到相机姿态
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3,3>(0,0);

  last_odom_stamp = odom->header.stamp;

  last_pose_world(0) = odom->pose.pose.position.x;
  last_pose_world(1) = odom->pose.pose.position.y;
  last_pose_world(2) = odom->pose.pose.position.z;

  // 发布 tf 变换
//   static tf2_ros::TransformBroadcaster br;
//   geometry_msgs::msg::TransformStamped transformStamped;

//   transformStamped.header.stamp = rclcpp::Clock().now();
//   transformStamped.header.frame_id = "world";
//   transformStamped.child_frame_id = "camera";

//   transformStamped.transform.translation.x = cam2world(0, 3);
//   transformStamped.transform.translation.y = cam2world(1, 3);
//   transformStamped.transform.translation.z = cam2world(2, 3);

//   transformStamped.transform.rotation.x = cam2world_quat.x();
//   transformStamped.transform.rotation.y = cam2world_quat.y();
//   transformStamped.transform.rotation.z = cam2world_quat.z();
//   transformStamped.transform.rotation.w = cam2world_quat.w();

//   br.sendTransform(transformStamped);
}

void pubCameraPose()
{
  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "map";  
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();

  pub_pose->publish(camera_pose);  
}

// 基于当前的传感器信息和无人机位置，渲染感知到的点云信息
void renderSensedPoints()
{ 
  //if(! has_global_map || ! has_odom) return;
  // 检查地图的可用性
  if( !has_global_map && !has_local_map) return;
  
  // 检查姿态可用性
  if( !has_odom ) return;
  render_currentpose();
  render_pcl_world();
}

vector<float> cloud_data;
// 接收全局地图信息的回调
void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
{
  if (has_global_map)
    return;

  RCLCPP_WARN(rclcpp::get_logger("rcvGlobalPointCloudCallBack"), "Global Pointcloud received..");

  // Load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  pcl::fromROSMsg(*pointcloud_map, cloudIn); 

  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }

  printf("global map has points: %d.\n", (int)cloud_data.size() / 3 );
  std::cout<< "global map has points: " << (int)cloud_data.size() / 3 << std::endl;

  // Pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  
  // Allocate depth_hostptr
  depth_hostptr = (int*)malloc(width * height * sizeof(int));

  has_global_map = true;
}

// 局部地图信息的回调
void rcvLocalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
{
  // Load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  pcl::fromROSMsg(*pointcloud_map, cloudIn);  

  if(cloudIn.points.size() == 0) return;
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int*)malloc(width * height * sizeof(int));

  has_local_map = true;
}

// 渲染世界坐标系下的点云
void render_pcl_world()
{
    pcl::PointCloud<pcl::PointXYZ> localMap; // 存储当前帧的点云数据
    pcl::PointXYZ pt_in;

    Eigen::Vector4d pose_in_camera;
    Eigen::Vector4d pose_in_world;
    Eigen::Vector3d pose_pt;

    // 从深度图中提取点
    for (int u = 0; u < width; u++) {
        for (int v = 0; v < height; v++) {
            // 获取对应位置的深度
            float depth = depth_mat.at<float>(v, u);
            
            if (depth == 0.0)
                continue;

            // 转换到相机坐标系
            pose_in_camera(0) = (u - cx) * depth / fx;
            pose_in_camera(1) = (v - cy) * depth / fy;
            pose_in_camera(2) = depth;
            pose_in_camera(3) = 1.0;
            
            // 转换到世界坐标系
            pose_in_world = cam2world * pose_in_camera;

            // 超出视野则筛掉
            if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon)
                continue;

            // 提取三维坐标并添加到点云
            pose_pt = pose_in_world.head(3);
            pt_in.x = pose_pt(0);
            pt_in.y = pose_pt(1);
            pt_in.z = pose_pt(2);

            localMap.points.push_back(pt_in);
        }
    }

    // 设置map的属性
    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    // 信息格式转换
    sensor_msgs::msg::PointCloud2 local_map_pcl;
    pcl::toROSMsg(localMap, local_map_pcl);
    local_map_pcl.header.frame_id = "/map";
    local_map_pcl.header.stamp = last_odom_stamp; // 使用当前时间戳

    // 发布点云
    pub_pcl_world->publish(local_map_pcl);
}

// 无需传递时间，直接使用 ROS2 时钟来获取当前时间
void render_currentpose()
{
  rclcpp::Clock clock;
  double this_time = clock.now().seconds();  // 获取当前时间（秒）

  // 通过取逆获取从世界坐标系到相机坐标系的变换矩阵
  Eigen::Matrix4d cam_pose = cam2world.inverse();

  // 变换矩阵转换为数组
  double pose[4 * 4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      pose[j + 4 * i] = cam_pose(i, j);

  // 渲染深度图像
  depthrender.render_pose(pose, depth_hostptr);

  // 初始化深度图
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;

  // 填充深度图像矩阵
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      // 毫米转为米
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 0;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i, j) = depth;
    }
  }

  // 发布深度图像信息
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth->publish(*out_msg.toImageMsg());

  // 生成伪彩色图像
  cv::Mat adjMap;
  depth_mat.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // 发布伪彩色图像信息
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color->publish(*cv_image_colored.toImageMsg());
  // cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv) {
  // Initialize ROS 2 node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pcl_render");

  node->declare_parameter("cam_width", 640);
  node->declare_parameter("cam_height", 480);
  node->declare_parameter("cam_fx", 387.0);
  node->declare_parameter("cam_fy", 387.0);
  node->declare_parameter("cam_cx", 320.0);
  node->declare_parameter("cam_cy", 240.0);
  node->declare_parameter("sensing_horizon", 100.0);
  node->declare_parameter("sensing_rate", 10.0);
  node->declare_parameter("estimation_rate", 5.0);
  node->declare_parameter("map/x_size", 10.0);
  node->declare_parameter("map/y_size", 10.0);
  node->declare_parameter("map/z_size", 10.0);

  // Get parameters
  node->get_parameter("cam_width", width);
  node->get_parameter("cam_height", height);
  node->get_parameter("cam_fx", fx);
  node->get_parameter("cam_fy", fy);
  node->get_parameter("cam_cx", cx);
  node->get_parameter("cam_cy", cy);
  node->get_parameter("sensing_horizon", sensing_horizon);
  node->get_parameter("sensing_rate", sensing_rate);
  node->get_parameter("estimation_rate", estimation_rate);
  node->get_parameter("map/x_size", _x_size);
  node->get_parameter("map/y_size", _y_size);
  node->get_parameter("map/z_size", _z_size);

  std::cout<< "camera parameter" << fx << fy << cx << cy << width << height << std::endl;
  depthrender.set_para(fx, fy, cx, cy, width, height);

  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  cam02body << 0.0, 0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0, 0.0,
               0.0, -1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0;

  // init cam2world transformation
  cam2world = Matrix4d::Identity();

  // Create ROS 2 subscribers and publishers
  auto global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "global_map", 1, rcvGlobalPointCloudCallBack);
  
  auto local_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "local_map", 1, rcvLocalPointCloudCallBack);
  
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odometry", 50, rcvOdometryCallback);

  pub_depth = node->create_publisher<sensor_msgs::msg::Image>("depth", 1000);
  pub_color = node->create_publisher<sensor_msgs::msg::Image>("colordepth", 1000);
  pub_pose = node->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 1000);
  // pub_pcl_world = node->create_publisher<sensor_msgs::msg::PointCloud2>("rendered_pcl", 1);
  pub_pcl_world = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/cloud", 1);

  // Set up timers for sensing and estimation
  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  auto local_sensing_timer = node->create_wall_timer(
    std::chrono::duration<double>(sensing_duration), std::bind(&renderSensedPoints));
  
  auto estimation_timer = node->create_wall_timer(
    std::chrono::duration<double>(estimate_duration), std::bind(&pubCameraPose));

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size/2.0;
  _gl_yl = -_y_size/2.0;
  _gl_zl =   0.0;
  
  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(node);  
    status = rclcpp::ok();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}