/*
* @Author: JC_Zhu
* @Email:	jiangchaozhu@zju.edu.cn
* @Create Date:	 2020-11-01 16:52:14
* @Last Modified by:   zuzu
* @Last Modified time: 2020-11-02 00:12:00
*/
// c++ header
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
#include <rviz_visual_tools/rviz_visual_tools.h>

// user-defined header
#include "param.hpp"

// pcl header
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace arma;
using namespace Eigen;

namespace MultiTrajEvaluation {
	class MultiTrajEvaluatorBase{
	public:
		MultiTrajEvaluatorBase(const ros::NodeHandle &nh, const Param &param) 
								: nh_(nh), param_(std::move(param)) {
		ROS_WARN("In constructor");
		initialize();
		}

		~MultiTrajEvaluatorBase() {
			result_file_.close();
		}

		void initialize();

		// void publishPath();
	private:
		ros::NodeHandle nh_;
		Param param_;

		// ros publisher
		vector<ros::Publisher> path_pub_vec_;
		vector<ros::Publisher> model_pub_vec_;
		// ros subscriber
		vector<ros::Subscriber> odom_sub_vec_;
		vector<ros::Subscriber> start_signal_sub_vec_, finish_signal_sub_vec_;
		ros::Subscriber global_map_sub_;

		vector<bool> is_start_vec_, is_finish_vec_;
		int finish_num_;
		// 
		vector<nav_msgs::Path> path_msg_vec_;

		// traj_length and traj_time 
		vector<double> traj_length_vec_;
		vector<double> traj_time_vec_;

		// collision 
		vector<vector<int>> collision_times_2dvec_;
		vector<vector<ros::Time>> last_collision_time_2dvec_;

		vector<double> min_dist_to_static_obs_vec_;
		vector<vector<double>> min_dist_to_other_agent_2dvec_;
		double global_min_dist_;


		// 
		bool has_global_map_;

		// result file and file name
		string result_fn_;
		fstream result_file_;

		rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

		// point cloud map in installed in kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ> global_map_kdtree_;

		void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &msg);
		void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id);
		void rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id);

		void resultToFile();
	};

	void MultiTrajEvaluatorBase::initialize()
	{
		// 1-d vector initialize
		path_pub_vec_.resize(param_.agent_num);
		model_pub_vec_.resize(param_.agent_num);

		odom_sub_vec_.resize(param_.agent_num);
		start_signal_sub_vec_.resize(param_.agent_num);
		finish_signal_sub_vec_.resize(param_.agent_num);
		is_start_vec_.resize(param_.agent_num);
		is_finish_vec_.resize(param_.agent_num);
		finish_num_ = 0;

		path_msg_vec_.resize(param_.agent_num);

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

		// subscriber and publisher initialize
		for (int i = 0; i < param_.agent_num; i++) {
			
			odom_sub_vec_[i] = nh_.subscribe<nav_msgs::Odometry>(\
				"/drone_" + to_string(i) + "_visual_slam/odom", 100, \
				boost::bind(&MultiTrajEvaluatorBase::rcvOdomCallback, this, _1, i));
			finish_signal_sub_vec_[i] = nh_.subscribe<std_msgs::Bool>(\
				"/drone_" + to_string(i) + "_planning/finish", 10, \
				boost::bind(&MultiTrajEvaluatorBase::rcvFinishSignalCallback, this, _1, i));
			
			path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
			model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);
			for (int j = 0; j < param_.agent_num; j++) {
				min_dist_to_other_agent_2dvec_[i][j] = 10000;
			}
			min_dist_to_static_obs[i] = 10000;
		}
		global_map_sub_ = nh_.subscribe( "/map_generator/global_cloud", 1,	&MultiTrajEvaluatorBase::rcvGlobalPointCloudCallBack, this); 
		global_min_dist_ = 10000;

		// visual_toos is used to mark the collide part of trajectories if any collision is detected
		visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers", nh_));
		visual_tools_->loadMarkerPub();
		visual_tools_->deleteAllMarkers();
		visual_tools_->enableBatchPublishing();

		result_file_.open(param_.result_fn, ios::out);
	}

	void MultiTrajEvaluatorBase::rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &msg)
	{
		if (has_global_map_) 
			return;
		ROS_WARN("Global Pointcloud received..");
		//transform map from ros message format to pcl format
		pcl::PointCloud<pcl::PointXYZ> cloudIn;
		pcl::fromROSMsg(msg, cloudIn);
		pcl::PointXYZ pt_in;
		vector<float> cloud_data;
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

	void MultiTrajEvaluatorBase::rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id)
	{
		if (msg->data) {
			is_finish_vec_[agent_id] = true;
			finish_num_++;
			if (finish_num_ == param_.agent_num) {
				resultToFile();
			}
		}
	}

	void MultiTrajEvaluatorBase::rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id) 
	{
		if(!is_finish_vec_[agent_id]) {
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

		path_msg_vec_[agent_id].header = msg->header;
		path_msg_vec_[agent_id].header.stamp = ros::Time::now();
		path_msg_vec_[agent_id].header.frame_id = string("world");	
		path_msg_vec_[agent_id].poses.push_back(p);
		path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);

		geometry_msgs::PoseStamped p2;
		Isometry3d pose1, pose2;

		for(int i = 0; i < param_.agent_num; i++) {
			if (i != agent_id && path_msg_vec_[i].poses.size() > 0) {
				double dist = getDist(p.pose, path_msg_vec_[i].poses.back().pose);
				if (dist < 0.5) {
					// if collision detected, draw the line from current drone to the other drone
					pose1.translation().x() = p.pose.position.x;
					pose1.translation().y() = p.pose.position.y;
					pose1.translation().z() = p.pose.position.z;
					pose2.translation().x() = path_msg_vec_[i].poses.back().pose.position.x;
					pose2.translation().y() = path_msg_vec_[i].poses.back().pose.position.y;
					pose2.translation().z() = path_msg_vec_[i].poses.back().pose.position.z;
					visual_tools_->publishLine(pose1, pose2, rviz_visual_tools::RED, rviz_visual_tools::XXLARGE);
					visual_tools_->trigger();

					// if the first collision happens, record time for further use
					if (collision_times_2dvec_[agent_id][i] == 0) {
						last_collision_time_2dvec_[agent_id][i] = ros::Time::now();
						collision_times_2dvec_[agent_id][i] += 1;
					} else if ((ros::Time::now() - last_collision_time_2dvec_[agent_id][i]).toSec() > 1){
						// if the collison happens 1s after the last one, make it count 
						last_collision_time_2dvec_[agent_id][i] = ros::Time::now();
						collision_times_2dvec_[agent_id][i] += 1;
					}
				}
				min_dist_to_other_agent_2dvec_[agent_id][i] = dist < min_dist_to_other_agent_2dvec_[agent_id][i] ? \
													 dist : min_dist_to_other_agent_2dvec_[agent_id][i];
				global_min_dist_ = dist < global_min_dist_ ? dist : global_min_dist_;
			}
		}
		// get the closet dist to static environments
		if (has_global_map_) {
			pcl::PointXYZ search_point(pose(0), pose(1), pose(2));
			vector<int> point_idx_NNSearch(1);
			vector<float> point_NN_squared_distance(1);
			global_map_kdtree_.nearestKSearch(search_point, 1, point_idx_NNSearch, point_NN_squared_distance);
			min_dist_to_static_obs_vec_[agent_id] = point_NN_squared_distance.back() < min_dist_to_static_obs_vec_[agent_id] ? \
											 point_NN_squared_distance.back() : min_dist_to_static_obs_vec_[agent_id];	
			global_min_dist_ = sqrt(point_NN_squared_distance.back()) < global_min_dist_ ? sqrt(point_NN_squared_distance.back()) : global_min_dist_;	
		}
	}
	}

	void MultiTrajEvaluatorBase::resultToFile()
	{
		result_file_ << "id" << "\t" << "traj_len" << "\t" << "min_dist_to_static_obs" << endl;
		for (int i = 0; i < param_.agent_num; i++) {
			traj_length_vec_[i] = getPathLength(path_msg_vec_[i]);
			result_file_ << i << "\t" << traj_length_vec_[i] << "\t" << setprecision(3) << sqrt(min_dist_to_static_obs_vec_[i]) << endl;
		}
		result_file_ << "min_dist_to_each_agent" << endl;
		for (int i = 0; i < param_.agent_num; i++) {
			for (int j = 0; j < param_.agent_num; j++) {
				result_file_ << setprecision(3) << min_dist_to_other_agent_2dvec_[i][j] << "\t";
			}
			result_file_ << endl;
		}
		result_file_ << "collision_times_to_each_agent" << endl;
		for (int i = 0; i < param_.agent_num; i++) {
			for (int j = 0; j < param_.agent_num; j++) {
				result_file_ << collision_times_2dvec_[i][j] << "\t";
			}
			result_file_ << endl;
		}
		result_file_ << "global_min_dist = " << global_min_dist_ << endl;
	}
}

