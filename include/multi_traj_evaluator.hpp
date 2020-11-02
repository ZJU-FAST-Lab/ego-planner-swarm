/*
* @Author: JC_Zhu
* @Email:	jiangchaozhu@zju.edu.cn
* @Create Date:	 2020-11-01 16:52:14
* @Last Modified by:   JC Zhu
* @Last Modified time: 2020-11-02 14:51:33
*/
// c++ header

#ifndef MULTI_TRAJ_EVALUATOT_HPP
#define MULTI_TRAJ_EVALUATOT_HPP
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
#include "visualization_msgs/Marker.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

// user-defined header
#include "param.hpp"
#include "timer.hpp"

// pcl header
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace arma;
using namespace Eigen;

namespace MultiTrajEvaluation {
	class MultiTrajEvaluatorBase{
	public:
		MultiTrajEvaluatorBase(const ros::NodeHandle &nh, const Param &param);
		virtual ~MultiTrajEvaluatorBase(){}
		virtual void resultToFile() = 0;
		void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &msg);
		void getClosestDistToStaticObs(const nav_msgs::Path& path, const pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree, int agent_id);
		void getClosestDistToOtherDrones(const nav_msgs::Path& path1, const nav_msgs::Path& path2, int id1, int id2, double dist_tol);
		bool CollisionDetect(const nav_msgs::Path& path1, const nav_msgs::Path& path2, double dist_tol);
		void pathPublish();
	protected:
		ros::NodeHandle nh_;
		Param param_;
		Timer timer_;

		// point cloud map in installed in kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ> global_map_kdtree_;
		bool has_global_map_;

		// ros publisher
		vector<ros::Publisher> path_pub_vec_;
		vector<ros::Publisher> model_pub_vec_;

		// ros subscriber
		ros::Subscriber global_map_sub_;
		// 
		vector<nav_msgs::Path> path_msg_vec_;
		vector<visualization_msgs::Marker> mesh_model_vec_;

		// traj_length and traj_time 
		vector<double> traj_length_vec_;
		vector<double> traj_time_vec_;

		// collision 
		vector<vector<int>> collision_times_2dvec_;
		vector<vector<ros::Time>> last_collision_time_2dvec_;

		vector<double> min_dist_to_static_obs_vec_;
		vector<vector<double>> min_dist_to_other_agent_2dvec_;
		double global_min_dist_;

		// result file and file name
		string result_fn_;
		fstream result_file_;

		rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

	};

MultiTrajEvaluatorBase::MultiTrajEvaluatorBase(const ros::NodeHandle &nh, const Param &param) 
												: nh_(nh), param_(std::move(param)) {
	ROS_WARN("In constructor");
	// 1-d vector initialize
	path_pub_vec_.resize(param_.agent_num);
	model_pub_vec_.resize(param_.agent_num);

	path_msg_vec_.resize(param_.agent_num);
	mesh_model_vec_.resize(param_.agent_num);

	traj_length_vec_.resize(param_.agent_num);
	traj_time_vec_.resize(param_.agent_num);
	min_dist_to_static_obs_vec_.resize(param_.agent_num);

	// 2-d vector initialize
	collision_times_2dvec_.resize(param_.agent_num);
	last_collision_time_2dvec_.resize(param_.agent_num);
	min_dist_to_other_agent_2dvec_.resize(param_.agent_num);
	for (int i = 0; i < param_.agent_num; i++) {
		collision_times_2dvec_[i].resize(param_.agent_num);
		last_collision_time_2dvec_[i].resize(param_.agent_num);
		min_dist_to_other_agent_2dvec_[i].resize(param_.agent_num);
	}

	global_map_sub_ = nh_.subscribe(param_.map_topic_name, 1, &MultiTrajEvaluatorBase::rcvGlobalPointCloudCallBack, this); 

	global_min_dist_ = 10000;

	// visual_toos is used to mark the collide part of trajectories if any collision is detected
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers", nh_));
	visual_tools_->loadMarkerPub();
	visual_tools_->deleteAllMarkers();
	visual_tools_->enableBatchPublishing();

	result_file_.open(param_.result_fn, ios::out);
}

void MultiTrajEvaluatorBase::rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
	if(has_global_map_)
		return;

	ROS_WARN("Global Pointcloud received..");
	vector<float> cloud_data;
	//load global map
	pcl::PointCloud<pcl::PointXYZ> cloudIn;
	pcl::PointXYZ pt_in;
	//transform map to point cloud format
	pcl::fromROSMsg(pointcloud_map, cloudIn);
	for(int i = 0; i < int(cloudIn.points.size()); i++){
		pt_in = cloudIn.points[i];
		cloud_data.push_back(pt_in.x);
		cloud_data.push_back(pt_in.y);
		cloud_data.push_back(pt_in.z);
	}
	ROS_WARN("global map has points: %d.", (int)cloud_data.size() / 3 );

	global_map_kdtree_.setInputCloud(cloudIn.makeShared());
	has_global_map_ = true;
}

void MultiTrajEvaluatorBase::getClosestDistToStaticObs(const nav_msgs::Path& path, const pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree, int agent_id)
{
  if (has_global_map_) {
    for(int i = 0; i < path.poses.size(); i++) {
      pcl::PointXYZ search_point(path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
      vector<int> point_idx_NNSearch(1);
      vector<float> point_NN_squared_distance(1);
      kd_tree.nearestKSearch(search_point, 1, point_idx_NNSearch, point_NN_squared_distance);
      min_dist_to_static_obs_vec_[agent_id] = sqrt(point_NN_squared_distance.back()) < min_dist_to_static_obs_vec_[agent_id] ? \
                                         	  sqrt(point_NN_squared_distance.back()) : min_dist_to_static_obs_vec_[agent_id];   
      global_min_dist_ = sqrt(point_NN_squared_distance.back()) < global_min_dist_? \
                 		 sqrt(point_NN_squared_distance.back()) : global_min_dist_;
    }    
  }
}

void MultiTrajEvaluatorBase::getClosestDistToOtherDrones(const nav_msgs::Path& path1, const nav_msgs::Path& path2, int id1, int id2, double dist_tol)
{
  for (int i = 0; i < path1.poses.size(); i++) {
    double dist = getDist(path1.poses[i].pose, path2.poses[i].pose);
    min_dist_to_other_agent_2dvec_[id1][id2] = dist < min_dist_to_other_agent_2dvec_[id1][id2] ? \
                                         	   dist : min_dist_to_other_agent_2dvec_[id1][id2];   
    global_min_dist_ = dist < global_min_dist_ ? dist : global_min_dist_;
    if (dist < dist_tol) {
      collision_times_2dvec_[id1][id2]++;
    } 
  }
}

bool MultiTrajEvaluatorBase::CollisionDetect(const nav_msgs::Path& path1, const nav_msgs::Path& path2, double dist_tol)
{
  double min_dist = 10000;
  for (int i = 0; i < path1.poses.size(); i++) {
    const geometry_msgs::Pose pose1 = path1.poses[i].pose;
    for (int j = 0; j < path2.poses.size(); j++) {
      const geometry_msgs::Pose pose2 = path2.poses[i].pose;
      double dist = getDist(pose1, pose2);
      min_dist = dist < min_dist ? dist : min_dist;
    }
  }
  global_min_dist_ = min_dist < global_min_dist_ ? min_dist : global_min_dist_;
  if (min_dist < dist_tol)
    return true;
  else 
    return false;
}

void MultiTrajEvaluatorBase::pathPublish()
{
  for(int agent_id = 0; agent_id < param_.agent_num; agent_id++) {
    path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);
  }
}
}
#endif