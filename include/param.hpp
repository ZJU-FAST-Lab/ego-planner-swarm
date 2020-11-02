/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
* @Create Date:   2020-11-01 16:55:09
* @Last Modified by:   zuzu
* @Last Modified time: 2020-11-01 23:33:15
*/
#ifndef PARAM_HPP
#define PARAM_HPP
#include <iostream>
#include <ros/ros.h>

using namespace std;

namespace MultiTrajEvaluation{
	class Param {
	public:
		int agent_num;
		bool is_debug;
		string model_mesh_resource;
		string frame_id;
		string result_fn;
		double color_r, color_g, color_b, color_a, scale;

		int planner_type;
		bool setROSParam(const ros::NodeHandle &nh);

	private:
	};

bool Param::setROSParam(const ros::NodeHandle &nh) {
	nh.param<int>("agent_num", agent_num, 1);
	nh.param<bool>("debug_flag", is_debug, false);
	nh.param<int>("planner_type", planner_type, 0);
	nh.param<string>("mesh_resource", model_mesh_resource, string("package://odom_visualization/meshes/hummingbird.mesh"));
	
	nh.param<double>("color/r", color_r, 1.0);
	nh.param<double>("color/g", color_g, 0.0);
	nh.param<double>("color/b", color_b, 0.0);
	nh.param<double>("color/a", color_a, 1.0);
	nh.param<double>("robot_scale", scale, 2.0);    

	nh.param<string>("frame_id", frame_id, string("world") );    
	nh.param<string>("result_fn", result_fn, string(""));
	return true;
}
}
#endif