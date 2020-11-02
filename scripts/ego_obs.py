#! /usr/bin/python3 
# __author__ = "JC_Zhu"
# This script is used to generate roslaunch file for ego_swarm in obs test
# Usage:
# 	python3 ego_obs.py 10

import sys
from sys import version_info
import numpy as np
import os

map_width = 20
drone_dist = 2
height = 1.5
fname = "/home/zuzu/swarm10-24/src/planner/plan_manage/launch/swarm_obstest.launch"

def gen_launch_for_obs_test():
	""" generate roslaunch file for ego_swarm in obs scenario

	Args:
		argv: user inputs from termial, argv[1] is used as agent num


	"""
	# num = int(argv[1])
	num = 10

	x_pos_start = -drone_dist*(num/2-0.5)
	x_pos_array = np.linspace(x_pos_start, -x_pos_start, num)

	file = open(fname, "w")

	str_begin = "<launch>\n\
	<arg name=\"map_size_x\"	value=\"45\"/> \n\
	<arg name=\"map_size_y\"	value=\"45\"/> \n\
	<arg name=\"map_size_z\"	value=\" 5\"/> \n\
	<arg name=\"odom_topic\"	value=\"visual_slam/odom\" />\n\
	<arg name=\"agent_num\"		value=\"{agent_num}\"/>\n\
	<!-- <include file=\"$(find ego_planner)/launch/rviz.launch\"/> --> \n\
	<!--<node pkg =\"map_generator\" name =\"random_forest\" type =\"random_forest\" output=\"screen\"> \n\
		<param name=\"map/x_size\"				value=\"30\" />\n\
		<param name=\"map/y_size\"				value=\"38\" />\n\
		<param name=\"map/z_size\"				value=\"5\" />\n\
		<param name=\"map/resoluion\"			value=\"0.1\"/>		\n\
		<param name=\"map/is_save\"				value=\"false\"/>\n\
		<param name=\"map/is_load\"				value=\"false\"/>\n\
		<param name=\"map/pcd_fn\"				value=\"$(find map_generator)/map.pcd\"/>\n\
		<param name=\"ObstacleShape/seed\"		value=\"1\"/>\n\
		<param name=\"map/obs_num\"				value=\"200\"/>\n\
		<param name=\"ObstacleShape/lower_rad\" value=\"0.5\"/>\n\
		<param name=\"ObstacleShape/upper_rad\" value=\"0.7\"/>\n\
		<param name=\"ObstacleShape/lower_hei\" value=\"0.0\"/>\n\
		<param name=\"ObstacleShape/upper_hei\" value=\"3.0\"/>\n\
		<param name=\"map/circle_num\"			value=\"50\"/>\n\
		<param name=\"ObstacleShape/radius_l\"	value=\"0.7\"/>\n\
		<param name=\"ObstacleShape/radius_h\"	value=\"0.5\"/>\n\
		<param name=\"ObstacleShape/z_l\"		value=\"0.7\"/>\n\
		<param name=\"ObstacleShape/z_h\"	 	value=\"0.8\"/>\n\
		<param name=\"ObstacleShape/theta\"		value=\"0.5\"/>\n\
		<param name=\"sensing/radius\"			value=\"5.0\"/>\n\
		<param name=\"sensing/rate\"			value=\"10.0\"/>\n\
		<param name=\"min_distance\"			value=\"1.2\"/>\n\
	</node>-->\n".format(agent_num=num)
	file.write(str_begin)

	for i in range(num):
		str_for_agent = "	<include file=\"$(find ego_planner)/launch/run_in_sim.launch\">\n\
			<arg name=\"drone_id\"	value=\"{drone_id}\"/>\n\
			<arg name=\"agent_num\"	value=\"{agent_num}\"/>\n\
			<arg name=\"init_x\"		value=\"{init_x}\"/>\n\
			<arg name=\"init_y\"		value=\"{init_y}\"/>\n\
			<arg name=\"init_z\"		value=\"{init_z}\"/>\n\
			<arg name=\"target_x\"	value=\"{target_x}\"/>\n\
			<arg name=\"target_y\"	value=\"{target_y}\"/>\n\
			<arg name=\"target_z\"	value=\"{target_z}\"/>\n\
			<arg name=\"map_size_x\"	value=\"$(arg map_size_x)\"/>\n\
			<arg name=\"map_size_y\"	value=\"$(arg map_size_y)\"/>\n\
			<arg name=\"map_size_z\"	value=\"$(arg map_size_z)\"/>\n\
			<arg name=\"odom_topic\"	value=\"$(arg odom_topic)\"/>\n\
	</include>\n".format(drone_id=i, agent_num=num, init_x=x_pos_array[i], init_y=map_width, init_z=height, \
							target_x=-x_pos_array[i], target_y=-map_width, target_z=height)
		file.write(str_for_agent)

	str_end = "</launch>\n"
	file.write(str_end)
	file.close()
	print("version_info", version_info.major)

if __name__ == '__main__':
	# gen_launch_for_obs_test(sys.argv)
	gen_launch_for_obs_test()