/*
* @Author: JC_Zhu
* @Email:	jiangchaozhu@zju.edu.cn
*/
#ifndef DMPC_SWAMR_EVALUATOT_HPP
#define DMPC_SWAMR_EVALUATOT_HPP

#include "evaluator_base.hpp"

namespace MultiPathEvaluation {
	class DmpcEvaluator : public EvaluatorBase 
	{
	public:
		DmpcEvaluator(const ros::NodeHandle &nh, const Param &param)
					: EvaluatorBase(nh, std::move(param)) {
			ROS_WARN("in DmpcEvaluator constructor");
			// subscriber and publisher initialize
			for (int i = 0; i < param_.agent_num; i++) {
				path_pub_vec_[i] = nh_.advertise<nav_msgs::Path>("/drone" + to_string(i) + "/vis_path", 100);
				model_pub_vec_[i] = nh_.advertise<visualization_msgs::Marker>("/drone" + to_string(i) + "/model", 100);
				
				for (int j = 0; j < param_.agent_num; j++) {
					min_dist_to_other_agent_2dvec_[i][j] = 10000;
				}
				min_dist_to_static_obs_vec_[i] = 10000;
			}

			final_pose_vec_.resize(param_.agent_num);

			readDmpcTrajFromFile(param_.traj_fn);
		}
		~DmpcEvaluator() {
			result_file_.close();
		}

		void resultToFile() override;
	private:
		void readDmpcTrajFromFile(const string &fn);
		vector<geometry_msgs::PoseStamped> final_pose_vec_;
	};

void DmpcEvaluator::readDmpcTrajFromFile(const string &fn)
{
	fstream traj_file;
	traj_file.open(fn);

	string str;
	int len = 0, start_len = 7;
	while(!traj_file.eof()) {
		getline(traj_file, str);
		vector<string> res;
		string result;
		stringstream input(str);
		while(input>>result)
			res.push_back(result);
		// len 0: N, N_cmd and other params
		// len 1~3: start.x, start.y, start.z
		// len 4~6: target.x, target.y, target.z
		// len 7~7+3*N-1: path[0].x, path[0].y path[0].z , ... , path[N-1].x, path[N-1].y path[N-1].z 
		if (len < 4) {
			// do nothing
		} else if (len == 4) { // target.x
			for (int agent_id = 0; agent_id < res.size(); agent_id++) {
			final_pose_vec_[agent_id].pose.position.x = stod(res[agent_id]);
			}
		} else if (len == 5) { // target.y
			for (int agent_id = 0; agent_id < res.size(); agent_id++) {
			final_pose_vec_[agent_id].pose.position.y = stod(res[agent_id]);
			}
		} else if (len == 6) { // target.z
			for (int agent_id = 0; agent_id < res.size(); agent_id++) {
			final_pose_vec_[agent_id].pose.position.z = stod(res[agent_id]);
			}
		} else if (len >= start_len && len < start_len+3*param_.agent_num) {
			int agent_id = int((len-start_len)/3);
			int xyz_id = int((len-start_len)%3);
			for (int point_id = 0; point_id < res.size(); point_id++) {
			if (xyz_id == 0) {	
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = stod(res[point_id]);
				path_msg_vec_[agent_id].poses.push_back(ps);
			} else if (xyz_id == 1) {
				path_msg_vec_[agent_id].poses[point_id].pose.position.y = stod(res[point_id]);
			} else {
				path_msg_vec_[agent_id].poses[point_id].pose.position.z = stod(res[point_id]);
			}
			}
		} else {
			break;
		}
		len++;
	}
	// add final points into path
	for(int agent_id = 0; agent_id < param_.agent_num; agent_id++) {
		path_msg_vec_[agent_id].poses.push_back(final_pose_vec_[agent_id]);
		path_msg_vec_[agent_id].header.frame_id = string("world");
		path_msg_vec_[agent_id].header.stamp = ros::Time::now();
	}
	resultToFile();
	ros::Duration(2).sleep();
	pathPublish();
}

void DmpcEvaluator::resultToFile() 
{
	ROS_WARN("result to file");
	result_file_ << "id" << "\t" << "traj_len" << "\t" << "min_dist_to_static_obs" << endl;
	for (int i = 0; i < param_.agent_num; i++) {
		traj_length_vec_[i] = getPathLength(path_msg_vec_[i]);
		result_file_ << i << "\t" << traj_length_vec_[i] << "\t" << "nan" << endl;
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