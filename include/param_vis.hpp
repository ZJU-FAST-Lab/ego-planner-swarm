/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#ifndef PARAM_VIS_HPP
#define PARAM_VIS_HPP
#include <iostream>
#include <ros/ros.h>

using namespace std;

namespace MultiPathVisualize {
	class Param {
	public:
		int agent_num;
		string mesh_resource;
		string frame_id;
		// double color_r, color_g, color_b, color_a, scale;
		vector<double> color_r, color_g, color_b, color_a;
		double scale;
		double stop_time;
		bool setROSParam(const ros::NodeHandle &nh);

	private:
	};

bool Param::setROSParam(const ros::NodeHandle &nh) {
	nh.param<int>("agent_num", agent_num, 1);

	color_r.resize(agent_num);
	color_g.resize(agent_num);
	color_b.resize(agent_num);
	color_a.resize(agent_num);

	nh.param<string>("mesh_resource", mesh_resource, string("package://odom_visualization/meshes/hummingbird.mesh"));
	nh.param<double>("color0/r", color_r[0], 1.0);
	nh.param<double>("color0/g", color_g[0], 0.0);
	nh.param<double>("color0/b", color_b[0], 0.0);
	nh.param<double>("color0/a", color_a[0], 1.0);

	nh.param<double>("color1/r", color_r[1], 1.0);
	nh.param<double>("color1/g", color_g[1], 0.0);
	nh.param<double>("color1/b", color_b[1], 0.0);
	nh.param<double>("color1/a", color_a[1], 1.0);
	
	nh.param<double>("color2/r", color_r[2], 1.0);
	nh.param<double>("color2/g", color_g[2], 0.0);
	nh.param<double>("color2/b", color_b[2], 0.0);
	nh.param<double>("color2/a", color_a[2], 1.0);

	nh.param<double>("robot_scale", scale, 2.0);    
	nh.param<double>("stop_time", stop_time, 0);
	nh.param<string>("frame_id", frame_id, string("world") );    
	return true;
}
}
#endif