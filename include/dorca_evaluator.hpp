/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#ifndef DORCA_EVALUATOT_HPP
#define DORCA_EVALUATOT_HPP

#include "evaluator_base.hpp"
#include "gazebo_msgs/ModelStates.h"

namespace MultiPathEvaluation {
	class DorcaEvaluator : public EvaluatorBase
	{
	public:
		DorcaEvaluator(const ros::NodeHandle &nh, const Param &param)
							: EvaluatorBase(nh, std::move(param)) {
			ROS_WARN("in DorcaEvaluator constructor");

			start_signal_sub_vec_.resize(param_.agent_num);
			finish_signal_sub_vec_.resize(param_.agent_num);
			is_start_vec_.resize(param_.agent_num);
			is_finish_vec_.resize(param_.agent_num);
			finish_num_ = 0;

			stata_sub_ = nh_.subscribe(param_.odom_topic_name, 1, &DorcaEvaluator::rcvOdomCallback, this, ros::TransportHints().tcpNoDelay());

			// subscriber and publisher initialize
			for (int i = 0; i < param_.agent_num; i++) {
				start_signal_sub_vec_[i] = nh_.subscribe<std_msgs::Bool>(\
					"/robot"+to_string(i)+"/start", 1, \
					boost::bind(&DorcaEvaluator::rcvStartSignalCallback, this, _1, i));		

				finish_signal_sub_vec_[i] = nh_.subscribe<std_msgs::Bool>(\
					"/robot"+to_string(i)+"/finish", 1, \
					boost::bind(&DorcaEvaluator::rcvFinishSignalCallback, this, _1, i));
				
				path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
				model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);
				
				for (int j = 0; j < param_.agent_num; j++) {
					min_dist_to_other_agent_2dvec_[i][j] = 10000;
				}
				min_dist_to_static_obs_vec_[i] = 10000;
			}
		}

		~DorcaEvaluator() {
			result_file_.close();
		}

	private:
		ros::Subscriber stata_sub_;
		vector<ros::Subscriber> start_signal_sub_vec_, finish_signal_sub_vec_;
		

		vector<bool> is_start_vec_, is_finish_vec_;
		int finish_num_;

		void rcvOdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

		void rcvStartSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id);
		void rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id);

		void resultToFile() override;
	};


void DorcaEvaluator::rcvStartSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id)
{
  if(msg->data) {
	is_start_vec_[agent_id] = true;
	traj_length_vec_[agent_id] = 0;

	cout << "\033[47;31mdrone[" << agent_id << "] start\033[0m" << endl;
	traj_time_vec_[agent_id] = ros::Time::now().toSec();
  }
} 

void DorcaEvaluator::rcvFinishSignalCallback(const std_msgs::Bool::ConstPtr& msg, int agent_id)
{
	if(msg->data) {
		is_finish_vec_[agent_id] = true;
		traj_time_vec_[agent_id] = ros::Time::now().toSec() - traj_time_vec_[agent_id];
		cout << "\033[47;31mdrone[" << agent_id << "] finish\033[0m" << endl;
		// cout << "\033[47;31mtraj_length_vec_[" << agent_id << "] = " << traj_length_vec_[agent_id] << "\033[0m" << endl \
			 << "\033[47;31mtraj_time_vec_[" << agent_id << "] = " << traj_time_vec_[agent_id] << "\033[0m" << endl;
		finish_num_++;
		if (finish_num_ == param_.agent_num) {
			resultToFile();
		}
	}
}

void DorcaEvaluator::rcvOdomCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) 
{
	colvec pose(6);
	colvec q(4);
	for(int i = 0; i < msg->pose.size()-1; i++) {
		string name = msg->name[i+1];
		string modelName = name.substr(0,4);
		string substring = name.substr(5,1);	
		int index = stoi(substring, nullptr, 10);
		int agent_id = index-1;

		pose(0) = msg->pose[i+1].position.x;
		pose(1) = msg->pose[i+1].position.y;
		pose(2) = msg->pose[i+1].position.z;
		colvec q(4);
		q(0)	= msg->pose[i+1].orientation.w;
		q(1)	= msg->pose[i+1].orientation.x;
		q(2)	= msg->pose[i+1].orientation.y;
		q(3)	= msg->pose[i+1].orientation.z;
		pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));

		geometry_msgs::PoseStamped p;
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

		if (!is_start_vec_[agent_id] || is_finish_vec_[agent_id])
			continue;
		// publish the path of agents
		path_msg_vec_[agent_id].header = p.header;
		path_msg_vec_[agent_id].header.stamp = ros::Time::now();
		path_msg_vec_[agent_id].header.frame_id = string("world");	
		path_msg_vec_[agent_id].poses.push_back(p);
		path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);

		// Mesh model												  
		mesh_model_vec_[agent_id].header.frame_id = param_.frame_id;
		mesh_model_vec_[agent_id].header.stamp = ros::Time::now();
		mesh_model_vec_[agent_id].ns = "mesh";
		mesh_model_vec_[agent_id].id = 0;
		mesh_model_vec_[agent_id].type = visualization_msgs::Marker::MESH_RESOURCE;
		mesh_model_vec_[agent_id].action = visualization_msgs::Marker::ADD;
		mesh_model_vec_[agent_id].pose.position.x = msg->pose[i+1].position.x;
		mesh_model_vec_[agent_id].pose.position.y = msg->pose[i+1].position.y;
		mesh_model_vec_[agent_id].pose.position.z = msg->pose[i+1].position.z;

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
		for (int i = 0; i < param_.agent_num; i++) {
			if (path_msg_vec_[i].poses.size() > 0) {
				for (int j = i+1; j < param_.agent_num; j++) {
					if (path_msg_vec_[j].poses.size() > 0) {
						double dist = getDist(path_msg_vec_[i].poses.back().pose, path_msg_vec_[j].poses.back().pose);
						if (dist < 0.8) {
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
							if (collision_times_2dvec_[i][j] == 0) {
								last_collision_time_2dvec_[i][j] = ros::Time::now();
								collision_times_2dvec_[i][j] += 1;
							} else if ((ros::Time::now() - last_collision_time_2dvec_[i][j]).toSec() > 1){
								// if the collison happens 1s after the last one, make it count 
								last_collision_time_2dvec_[i][j] = ros::Time::now();
								collision_times_2dvec_[i][j] += 1;
							}				
						}
						min_dist_to_other_agent_2dvec_[i][j] = dist < min_dist_to_other_agent_2dvec_[i][j] ? \
															   dist : min_dist_to_other_agent_2dvec_[i][j];
						global_min_dist_ = dist < global_min_dist_ ? dist : global_min_dist_;						
					}

				}				
			}

		}
  }
}

void DorcaEvaluator::resultToFile()
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