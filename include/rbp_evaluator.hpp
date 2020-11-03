/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#ifndef RBP_EVALUATOT_HPP
#define RBP_EVALUATOT_HPP

#include "evaluator_base.hpp"

namespace  MultiPathEvaluation {
	class RbpSwarmEvaluator : public EvaluatorBase
	{
	public:
		RbpSwarmEvaluator(const ros::NodeHandle &nh, const Param &param)
							: EvaluatorBase(nh, std::move(param)) {
		ROS_WARN("in RbpSwarmEvaluator constructor");

		path_sub_vec_.resize(param_.agent_num);

		finish_num_ = 0;

		// subscriber and publisher initialize
		for (int i = 0; i < param_.agent_num; i++) {
			path_sub_vec_[i] = nh_.subscribe<nav_msgs::Path>(\
				"/desired_trajectory/mav" + to_string(i), 100, \
				boost::bind(&RbpSwarmEvaluator::rcvPathCallback, this, _1, i));
			
			path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
			model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);
			
			for (int j = 0; j < param_.agent_num; j++) {
				min_dist_to_other_agent_2dvec_[i][j] = 10000;
			}
			min_dist_to_static_obs_vec_[i] = 10000;
		}
		rbp_run_once_flag_ = false;
		}

		~RbpSwarmEvaluator() {
			result_file_.close();
		}

	private:
		vector<ros::Subscriber> path_sub_vec_;
		// vector<ros::Subscriber> start_signal_sub_vec_, finish_signal_sub_vec_;
		
		// vector<bool> is_start_vec_, is_finish_vec_;
		int finish_num_;

		bool rbp_run_once_flag_;

		void rcvPathCallback(const nav_msgs::Path::ConstPtr &msg, int agent_id);

		void resultToFile() override;
	};

void RbpSwarmEvaluator::rcvPathCallback(const nav_msgs::Path::ConstPtr &msg, int agent_id) 
{
	
	if (path_msg_vec_[agent_id].poses.size() == 0) {
		path_msg_vec_[agent_id].header = msg->header;
		path_msg_vec_[agent_id].poses = msg->poses;
		finish_num_++;    
	} else if (!rbp_run_once_flag_ && has_global_map_ && finish_num_ == param_.agent_num) {
		rbp_run_once_flag_ = true;
		resultToFile();
	}
	path_pub_vec_[agent_id].publish(path_msg_vec_[agent_id]);
}

void RbpSwarmEvaluator::resultToFile()
{
	ROS_WARN("result to file");
	result_file_ << "id" << "\t" << "traj_len" << "\t" << "min_dist_to_static_obs" << endl;
	for (int i = 0; i < param_.agent_num; i++) {
		traj_length_vec_[i] = getPathLength(path_msg_vec_[i]);
		getClosestDistToStaticObs(path_msg_vec_[i], global_map_kdtree_, i);
		result_file_ << i << "\t" << traj_length_vec_[i] << "\t" << setprecision(3) << min_dist_to_static_obs_vec_[i] << endl;
		for (int j = i+1; j < param_.agent_num; j++) {
			getClosestDistToOtherDrones(path_msg_vec_[i], path_msg_vec_[j], i, j, 1);
		}
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
	result_file_ << "global_min_dist" << "\t" << global_min_dist_ <<  endl;
}
}
#endif