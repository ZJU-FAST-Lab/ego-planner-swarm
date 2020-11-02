/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
* @Create Date:   2020-11-02 13:50:30
* @Last Modified by:   JC Zhu
* @Last Modified time: 2020-11-02 14:54:21
*/
#ifndef EGO_SWARM_EVALUATOT_HPP
#define EGO_SWARM_EVALUATOT_HPP

#include "multi_traj_evaluator.hpp"

namespace MultiTrajEvaluation {
	class EgoSwarmEvaluator : public MultiTrajEvaluatorBase
	{
	public:
		EgoSwarmEvaluator(const ros::NodeHandle &nh, const Param &param)
							: MultiTrajEvaluatorBase(nh, std::move(param)) {
		ROS_WARN("in EgoSwarmEvaluator constructor");

		odom_sub_vec_.resize(param_.agent_num);
		start_signal_sub_vec_.resize(param_.agent_num);
		finish_signal_sub_vec_.resize(param_.agent_num);
		is_start_vec_.resize(param_.agent_num);
		is_finish_vec_.resize(param_.agent_num);
		finish_num_ = 0;

		// subscriber and publisher initialize
		for (int i = 0; i < param_.agent_num; i++) {
			
			odom_sub_vec_[i] = nh_.subscribe<nav_msgs::Odometry>(\
				"/drone_" + to_string(i) + "_visual_slam/odom", 100, \
				boost::bind(&EgoSwarmEvaluator::rcvOdomCallback, this, _1, i));
			finish_signal_sub_vec_[i] = nh_.subscribe<std_msgs::Bool>(\
				"/drone_" + to_string(i) + "_planning/finish", 10, \
				boost::bind(&EgoSwarmEvaluator::rcvFinishSignalCallback, this, _1, i));
			
			path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
			model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);
			
			for (int j = 0; j < param_.agent_num; j++) {
				min_dist_to_other_agent_2dvec_[i][j] = 10000;
			}
			min_dist_to_static_obs_vec_[i] = 10000;
		}
		}

		~EgoSwarmEvaluator() {
			result_file_.close();
		}

	private:
		vector<ros::Subscriber> odom_sub_vec_;
		vector<ros::Subscriber> start_signal_sub_vec_, finish_signal_sub_vec_;
		

		vector<bool> is_start_vec_, is_finish_vec_;
		int finish_num_;

		void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id);

		void rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id);

		void resultToFile() override;
	};



void EgoSwarmEvaluator::rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id)
{
	if (msg->data) {
		is_finish_vec_[agent_id] = true;
		finish_num_++;
		if (finish_num_ == param_.agent_num) {
			resultToFile();
		}
	}
}

void EgoSwarmEvaluator::rcvOdomCallback(const nav_msgs::Odometry::ConstPtr &msg, int agent_id) 
{
	// ROS_WARN("in rcvOdomCallback!!!");

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

		// publish the path of agents
		path_msg_vec_[agent_id].header = msg->header;
		path_msg_vec_[agent_id].header.stamp = ros::Time::now();
		path_msg_vec_[agent_id].header.frame_id = string("world");	
		path_msg_vec_[agent_id].poses.push_back(p);
		path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);
		// publish drone mesh model
		mesh_model_vec_[agent_id].header.frame_id = param_.frame_id;
		mesh_model_vec_[agent_id].header.stamp = ros::Time::now();
		mesh_model_vec_[agent_id].ns = "mesh";
		mesh_model_vec_[agent_id].id = 0;
		mesh_model_vec_[agent_id].type = visualization_msgs::Marker::MESH_RESOURCE;
		mesh_model_vec_[agent_id].action = visualization_msgs::Marker::ADD;
		mesh_model_vec_[agent_id].pose.position.x = pose(0);
		mesh_model_vec_[agent_id].pose.position.y = pose(1);
		mesh_model_vec_[agent_id].pose.position.z = pose(2);		
	    mesh_model_vec_[agent_id].pose.orientation.w = q(0);
	    mesh_model_vec_[agent_id].pose.orientation.x = q(1);
	    mesh_model_vec_[agent_id].pose.orientation.y = q(2);
	    mesh_model_vec_[agent_id].pose.orientation.z = q(3);
	    mesh_model_vec_[agent_id].scale.x = param_.scale;
	    mesh_model_vec_[agent_id].scale.y = param_.scale;
	    mesh_model_vec_[agent_id].scale.z = param_.scale;
	    mesh_model_vec_[agent_id].color.a = param_.color_a;
	    mesh_model_vec_[agent_id].color.r = param_.color_r;
	    mesh_model_vec_[agent_id].color.g = param_.color_g;
	    mesh_model_vec_[agent_id].color.b = param_.color_b;
	    mesh_model_vec_[agent_id].mesh_resource = param_.mesh_resource;
	    model_pub_vec_[agent_id].publish(mesh_model_vec_[agent_id]);

		Isometry3d pose1, pose2;

		// detect collision between other drones and this one
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
			min_dist_to_static_obs_vec_[agent_id] = sqrt(point_NN_squared_distance.back()) < min_dist_to_static_obs_vec_[agent_id] ? \
											 		sqrt(point_NN_squared_distance.back()) : min_dist_to_static_obs_vec_[agent_id];	
			global_min_dist_ = sqrt(point_NN_squared_distance.back()) < global_min_dist_ ? sqrt(point_NN_squared_distance.back()) : global_min_dist_;	
		}
	}
}

void EgoSwarmEvaluator::resultToFile()
{
	result_file_ << "id" << "\t" << "traj_len" << "\t" << "min_dist_to_static_obs" << endl;
	for (int i = 0; i < param_.agent_num; i++) {
		traj_length_vec_[i] = getPathLength(path_msg_vec_[i]);
		result_file_ << i << "\t" << traj_length_vec_[i] << "\t" << setprecision(3) << min_dist_to_static_obs_vec_[i] << endl;
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
#endif