/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
* @Create Date:   2020-11-01 17:09:29
* @Last Modified by:   JC Zhu
* @Last Modified time: 2020-11-02 14:42:01
*/
#include <ros/ros.h>

#include "param.hpp"
#include "multi_traj_evaluator.hpp"
#include "ego_swarm_evaluator.hpp"
#include "rbp_swarm_evaluator.hpp"
#include "dmpc_evaluator.hpp"
#include "dorca_evaluator.hpp"

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

	// ego 
	// std::shared_ptr<EgoSwarmEvaluator> multiTrajEvaluatorBase_obj;
	// multiTrajEvaluatorBase_obj.reset(new EgoSwarmEvaluator(nh, param));

	// rnp
	// std::shared_ptr<RbpSwarmEvaluator> multiTrajEvaluatorBase_obj;
	// multiTrajEvaluatorBase_obj.reset(new RbpSwarmEvaluator(nh, param));

	// dmpc
	// std::shared_ptr<DmpcEvaluator> multiTrajEvaluatorBase_obj;
	// multiTrajEvaluatorBase_obj.reset(new DmpcEvaluator(nh, param));

	// dorca
	std::shared_ptr<DorcaEvaluator> multiTrajEvaluatorBase_obj;
	multiTrajEvaluatorBase_obj.reset(new DorcaEvaluator(nh, param));

	ros::Rate rate(20);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}