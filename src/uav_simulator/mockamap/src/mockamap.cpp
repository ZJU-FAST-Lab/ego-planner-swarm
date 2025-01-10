#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (uint32_t i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  RCLCPP_INFO(rclcpp::get_logger("optimizeMap"), "finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("mockamap");

  // 创建一个 ROS2 发布者
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2       output;
  // Fill in the cloud data

  // 获取参数
  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

  node->declare_parameter("seed", 4546);
  node->declare_parameter("update_freq", 1.0);
  node->declare_parameter("resolution", 0.38);
  node->declare_parameter("x_length", 100);
  node->declare_parameter("y_length", 100);
  node->declare_parameter("z_length", 10);
  node->declare_parameter("type", 3);

  node->get_parameter("seed", seed);
  node->get_parameter("update_freq", update_freq);
  node->get_parameter("resolution", scale);
  node->get_parameter("x_length", sizeX);
  node->get_parameter("y_length", sizeY);
  node->get_parameter("z_length", sizeZ);
  node->get_parameter("type", type);

  // 调整尺寸和分辨率
  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  // 配置地图生成信息
  mocka::Maps::BasicInfo info;
  info.node       = node;  // ROS2中没有 NodeHandle 私有部分，已移除
  info.sizeX      = sizeX;
  info.sizeY      = sizeY;
  info.sizeZ      = sizeZ;
  info.seed       = seed;
  info.scale      = scale;
  info.output     = &output;
  info.cloud      = &cloud;

  // 生成地图
  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  // 订阅循环
  rclcpp::Rate loop_rate(update_freq);
  while (rclcpp::ok())
  {
    pcl_pub->publish(output);
    rclcpp::spin_some(node);  // 这里使用 spin_some 代替 ros::spinOnce()
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
