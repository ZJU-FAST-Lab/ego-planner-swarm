/*
* @Author: JC_Zhu
* @Email:  jiangchaozhu@zju.edu.cn
*/
#include <ros/ros.h>

#include "param.hpp"
#include "evaluator_base.hpp"
#include "ego_evaluator.hpp"
#include "rbp_evaluator.hpp"
#include "dmpc_evaluator.hpp"
#include "dorca_evaluator.hpp"

using namespace MultiPathEvaluation;

int main(int argc, char *argv[])
{
	ROS_INFO("Multi Path Evaluator is Launched!!!");
	ros::init(argc, argv, "path_evaluation_node");
	ros::NodeHandle nh("~");
	
	// load param
	Param param;
	if (!param.setROSParam(nh)) {
		return -1;
	}

	if (param.planner_type == 0) {
		// ego 
		std::shared_ptr<EgoSwarmEvaluator> multiTrajEvaluatorBase_obj;
		multiTrajEvaluatorBase_obj.reset(new EgoSwarmEvaluator(nh, param));
	} else if (param.planner_type == 1) {
		// rnp
		std::shared_ptr<RbpSwarmEvaluator> multiTrajEvaluatorBase_obj;
		multiTrajEvaluatorBase_obj.reset(new RbpSwarmEvaluator(nh, param));
	} else if (param.planner_type == 2) {
		// dmpc
		std::shared_ptr<DmpcEvaluator> multiTrajEvaluatorBase_obj;
		multiTrajEvaluatorBase_obj.reset(new DmpcEvaluator(nh, param));
	} else {
		// dorca
		std::shared_ptr<DorcaEvaluator> multiTrajEvaluatorBase_obj;
		multiTrajEvaluatorBase_obj.reset(new DorcaEvaluator(nh, param));		
	}

	ros::Rate rate(20);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}