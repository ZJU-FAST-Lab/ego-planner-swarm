/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#include <ros/ros.h>
#include "visualization_base.hpp"
#include "param_vis.hpp"

using MultiPathVisualize::VisualizationBase;
using MultiPathVisualize::Param;

int main(int argc, char *argv[])
{
	ROS_INFO("Multi Path visualization is Launched!!!");
	ros::init(argc, argv, "path_visualization_node");
	ros::NodeHandle nh("~");

	// load param
	Param param;
	if (!param.setROSParam(nh)) {
		return -1;
	}

	std::shared_ptr<VisualizationBase> multiPathVisualization_obj;
	multiPathVisualization_obj.reset(new VisualizationBase(nh, param));

	ros::Rate rate(100);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}