/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#ifndef VISUALIZATION_BASE_HPP
#define VISUALIZATION_BASE_HPP
#include <iostream>
#include <fstream>
#include "math.h"
#include <iomanip>

#include "armadillo"
#include "pose_utils.h"
#include "ros_geometry_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS message header
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"

#include "visualization_msgs/Marker.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

// pcl header
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "param_vis.hpp"

using namespace std;
using namespace arma;
using namespace Eigen;

// /drone_0_ego_planner_node/grid_map/occupancy
// sensor_msgs/PointCloud2
// /vins_estimator/imu_propagate
// nav_msgs/Odometry
namespace MultiPathVisualize {
	class VisualizationBase{
	public:
		VisualizationBase(const ros::NodeHandle &nh, const Param &param);
		virtual ~VisualizationBase(){}
		void rcvLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int agent_id);

		void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id);

		void rcvOdom0Callback(const nav_msgs::Odometry::ConstPtr &msg);
		void rcvOdom1Callback(const nav_msgs::Odometry::ConstPtr &msg);
		void rcvOdom2Callback(const nav_msgs::Odometry::ConstPtr &msg);

		void pointCloudPublish();


	protected:
		ros::NodeHandle nh_;
		int agent_num_;
		Param param_;
		// point cloud map in installed in kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ> global_map_kdtree_;
		pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler_;
		bool has_global_map_;

		// ros publisher
		vector<ros::Publisher> path_pub_vec_;
		vector<ros::Publisher> model_pub_vec_;

		sensor_msgs::PointCloud2 globalMap_pcd_;
		ros::Publisher point_cloud_pub_;
		// ros subscriber
		vector<ros::Subscriber> local_map_sub_;
		vector<ros::Subscriber> odom_sub_;
		// 
		vector<nav_msgs::Path> path_msg_vec_;
		vector<visualization_msgs::Marker> mesh_model_vec_;

		pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud_;

		double min_height_, max_height_;

		bool first_flag;
		ros::Time start_time_;
	};

VisualizationBase::VisualizationBase(const ros::NodeHandle &nh, const Param &param) : nh_(nh), param_(param) {
	ROS_WARN("In constructor");

	local_map_sub_.resize(param_.agent_num);
	odom_sub_.resize(param_.agent_num);

	path_msg_vec_.resize(param_.agent_num);
	path_pub_vec_.resize(param_.agent_num);
	mesh_model_vec_.resize(param_.agent_num);
	model_pub_vec_.resize(param_.agent_num);

	odom_sub_[0] = nh_.subscribe("/drone0/odom", 100, \
								&VisualizationBase::rcvOdom0Callback, this, ros::TransportHints().tcpNoDelay());
	odom_sub_[1] = nh_.subscribe("/drone1/odom", 100, \
								&VisualizationBase::rcvOdom1Callback, this, ros::TransportHints().tcpNoDelay());
	odom_sub_[2] = nh_.subscribe("/drone2/odom", 100, \
								&VisualizationBase::rcvOdom2Callback, this, ros::TransportHints().tcpNoDelay());
	// initialize point cloud subscriber  ros::TransportHints().tcpNoDelay()
	for (int i = 0; i < param_.agent_num; i++) {
		local_map_sub_[i] = nh_.subscribe<sensor_msgs::PointCloud2>("/drone" + to_string(i) + "/local_map", 100, \
							boost::bind(&VisualizationBase::rcvLocalMapCallback, this, _1, i));
		// odom_sub_[i] = nh_.subscribe<nav_msgs::Odometry>("/drone" + to_string(i) + "/odom", 100, \
							boost::bind(&VisualizationBase::rcvOdomCallback, this, _1, i));
		// odom_sub_[i] = nh_.subscribe("/drone" + to_string(i) + "/odom", 100, \
							&VisualizationBase::rcvOdomCallbackBase, this, ros::TransportHints().tcpNoDelay());
		path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
		model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);

	}

	// initialize point_cloud pub
	point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulate_local_map", 100);
	total_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

	min_height_ = 10;
	max_height_ = 0;

	first_flag = false;
}

void VisualizationBase::rcvLocalMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int agent_id) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_total_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(*msg, *temp_cloud);

	// 设置滤波器对象
	// pcl::copyPointCloud(*temp_cloud, *cloud_filtered); //复制
	// pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud (cloud_filtered);            //设置输入点云
	// pass.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
	// pass.setFilterLimits (0.2, 2.4);        //设置在过滤字段的范围
	// pass.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
	// pass.filter (*temp_cloud);            //执行滤波，保存过滤结果在cloud_filtered
	
	
	*total_cloud_ += *temp_cloud;

	// down sampling
	pcl::copyPointCloud(*total_cloud_, *temp_total_cloud); //复制
	voxel_sampler_.setLeafSize(0.1f, 0.1f, 0.1f);
	voxel_sampler_.setInputCloud(temp_total_cloud);
	voxel_sampler_.filter(*total_cloud_);

	// publish accumulated point clouds
	pointCloudPublish();
}

void VisualizationBase::rcvOdom0Callback(const nav_msgs::Odometry::ConstPtr &msg) {
	rcvOdomCallback(msg, 0);
}

void VisualizationBase::rcvOdom1Callback(const nav_msgs::Odometry::ConstPtr &msg) {
	rcvOdomCallback(msg, 1);
}

void VisualizationBase::rcvOdom2Callback(const nav_msgs::Odometry::ConstPtr &msg) {
	rcvOdomCallback(msg, 2);
}

void VisualizationBase::rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id) {
	if (!first_flag) {
		start_time_ = msg->header.stamp;
		// cout << "msg->header.time = " << msg->header.stamp << endl;
		first_flag = true;
	}
	colvec pose(6);	
	pose(0) = msg->pose.pose.position.x;
	pose(1) = msg->pose.pose.position.y;
	pose(2) = msg->pose.pose.position.z;
	colvec q(4);
	q(0)	= msg->pose.pose.orientation.w;
	q(1)	= msg->pose.pose.orientation.x;
	q(2)	= msg->pose.pose.orientation.y;
	q(3)	= msg->pose.pose.orientation.z;
	pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));

	geometry_msgs::PoseStamped p;
	p.header = msg->header;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = string("world");
	p.pose.position.x = pose(0);
	p.pose.position.y = pose(1);
	p.pose.position.z = pose(2);
	q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
	p.pose.orientation.w = q(0);
	p.pose.orientation.x = q(1);
	p.pose.orientation.y = q(2);
	p.pose.orientation.z = q(3);	 

	// publish the path of agents
	if (agent_id == 1 && (msg->header.stamp - start_time_).toSec() > param_.stop_time) {
		p = path_msg_vec_[agent_id].poses.back();
	} 
	path_msg_vec_[agent_id].header = msg->header;
	path_msg_vec_[agent_id].header.stamp = ros::Time::now();
	path_msg_vec_[agent_id].header.frame_id = string("world");	
	path_msg_vec_[agent_id].poses.push_back(p);
	path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);

	// publish drone mesh model
	mesh_model_vec_[agent_id].header.frame_id = string("world");
	mesh_model_vec_[agent_id].header.stamp = ros::Time::now();
	mesh_model_vec_[agent_id].ns = "mesh";
	mesh_model_vec_[agent_id].id = 0;
	mesh_model_vec_[agent_id].type = visualization_msgs::Marker::MESH_RESOURCE;
	mesh_model_vec_[agent_id].action = visualization_msgs::Marker::ADD;
	mesh_model_vec_[agent_id].pose.position.x = p.pose.position.x;
	mesh_model_vec_[agent_id].pose.position.y = p.pose.position.y;
	mesh_model_vec_[agent_id].pose.position.z = p.pose.position.z;		
    mesh_model_vec_[agent_id].pose.orientation.w = p.pose.orientation.w;
    mesh_model_vec_[agent_id].pose.orientation.x = p.pose.orientation.x;
    mesh_model_vec_[agent_id].pose.orientation.y = p.pose.orientation.y;
    mesh_model_vec_[agent_id].pose.orientation.z = p.pose.orientation.z;
    mesh_model_vec_[agent_id].scale.x = param_.scale;
    mesh_model_vec_[agent_id].scale.y = param_.scale;
    mesh_model_vec_[agent_id].scale.z = param_.scale;
    mesh_model_vec_[agent_id].color.a = param_.color_a[agent_id];
    mesh_model_vec_[agent_id].color.r = param_.color_r[agent_id];
    mesh_model_vec_[agent_id].color.g = param_.color_g[agent_id];
    mesh_model_vec_[agent_id].color.b = param_.color_b[agent_id];
    mesh_model_vec_[agent_id].mesh_resource = param_.mesh_resource;
    model_pub_vec_[agent_id].publish(mesh_model_vec_[agent_id]);
}

void VisualizationBase::pointCloudPublish() {
	pcl::toROSMsg(*total_cloud_, globalMap_pcd_);
	// ROS_WARN("total point cloud size = %d", total_cloud_->points.size());
	globalMap_pcd_.header.frame_id = "world";
	point_cloud_pub_.publish(globalMap_pcd_);
}
}
#endif