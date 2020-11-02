/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
* @Create Date:   2020-11-01 17:09:29
* @Last Modified by:   zuzu
* @Last Modified time: 2020-11-01 23:33:21
*/
#include <ros/ros.h>

#include "param.hpp"
#include "multi_traj_evaluator.hpp"

using namespace MultiTrajEvaluation;

int main(int argc, char *argv[])
{
	ROS_INFO("Multi Traj Evaluator is Launched!!!");
	ros::init(argc, argv, "traj_evaluation_node");
	ros::NodeHandle nh("~");
	
	// load param
	Param param;
	if (!param.setROSParam(nh)) {
		return -1;
	}
	std::shared_ptr<MultiTrajEvaluatorBase> multiTrajEvaluatorBase_obj;
	// MultiTrajEvaluatorBase multiTrajEvaluatorBase_obj(nh, param);
	multiTrajEvaluatorBase_obj.reset(new MultiTrajEvaluatorBase(nh, param));
	// Main Loop
	ros::Rate rate(20);
	while (ros::ok()) {

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}