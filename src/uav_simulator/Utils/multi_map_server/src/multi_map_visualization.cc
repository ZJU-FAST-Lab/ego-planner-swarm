#include <iostream>
#include <pose_utils/pose_utils.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud.hpp>
#include <multi_map_server/msg/multi_occupancy_grid.hpp>
#include <multi_map_server/msg/multi_sparse_map3_d.hpp>
#include <multi_map_server/Map2D.h>
#include <multi_map_server/Map3D.h>

rclcpp::Publisher<multi_map_server::msg::MultiOccupancyGrid>::SharedPtr pub1;
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub2;

vector<Map2D> maps2d;
vector<geometry_msgs::msg::Pose> origins2d;
vector<Map3D> maps3d;
vector<geometry_msgs::msg::Pose> origins3d;

void maps2d_callback(const multi_map_server::msg::MultiOccupancyGrid::ConstPtr &msg)
{
    // Merge map
    maps2d.resize(msg->maps.size(), Map2D(4));
    for (unsigned int k = 0; k < msg->maps.size(); k++)
        maps2d[k].Replace(msg->maps[k]);
    origins2d = msg->origins;
    // Assemble and publish map
    multi_map_server::msg::MultiOccupancyGrid m;
    m.maps.resize(maps2d.size());
    m.origins.resize(maps2d.size());
    for (unsigned int k = 0; k < maps2d.size(); k++)
    {
        m.maps[k] = maps2d[k].GetMap();
        m.origins[k] = origins2d[k];
    }
    pub1->publish(m);
}

void maps3d_callback(const multi_map_server::msg::MultiSparseMap3D::ConstPtr &msg)
{
    // Update incremental map
    maps3d.resize(msg->maps.size());
    for (unsigned int k = 0; k < msg->maps.size(); k++)
        maps3d[k].UnpackMsg(msg->maps[k]);
    origins3d = msg->origins;
    // Publish
    sensor_msgs::msg::PointCloud m;
    for (unsigned int k = 0; k < msg->maps.size(); k++)
    {
        colvec po(6);
        po(0) = origins3d[k].position.x;
        po(1) = origins3d[k].position.y;
        po(2) = origins3d[k].position.z;
        colvec poq(4);
        poq(0) = origins3d[k].orientation.w;
        poq(1) = origins3d[k].orientation.x;
        poq(2) = origins3d[k].orientation.y;
        poq(3) = origins3d[k].orientation.z;
        po.rows(3, 5) = R_to_ypr(quaternion_to_R(poq));
        colvec tpo = po.rows(0, 2);
        mat Rpo = ypr_to_R(po.rows(3, 5));
        vector<colvec> pts = maps3d[k].GetOccupancyWorldFrame(OCCUPIED);
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            colvec pt = Rpo * pts[i] + tpo;
            geometry_msgs::msg::Point32 _pt;
            _pt.x = pt(0);
            _pt.y = pt(1);
            _pt.z = pt(2);
            m.points.push_back(_pt);
        }
    }
    // Publish
    m.header.stamp = rclcpp::Clock().now();
    m.header.frame_id = string("/map");
    pub2->publish(m);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("multi_map_visualization");

    auto sub1 = node->create_subscription<multi_map_server::msg::MultiOccupancyGrid>(
        "dmaps2d", 1, maps2d_callback);
    auto sub2 = node->create_subscription<multi_map_server::msg::MultiSparseMap3D>(
        "dmaps3d", 1, maps3d_callback);

    pub1 = node->create_publisher<multi_map_server::msg::MultiOccupancyGrid>("maps2d", 1);
    pub2 = node->create_publisher<sensor_msgs::msg::PointCloud>("map3d", 1);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}