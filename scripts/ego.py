import sys
import numpy as np
import os

map_width = 20
drone_dist = 2
height = 3 for
fname = "/home/zuzu/swarm10-24/src/planner/plan_manage/launch/swarm_scale.launch"
def main(argv):
	num = int(argv[1])
	str_begin = "<launch>\n\
	    <arg name=\"map_size_x\" value=\"55.0\"/> \n\
	    <arg name=\"map_size_y\" value=\"55.0\"/> \n\
	    <arg name=\"map_size_z\" value=\" 5.0\"/> \n\
	    <arg name=\"odom_topic\" value=\"visual_slam/odom\" />\n\
	    <arg name=\"agent_num\"  value=\"{agent_num}\"/>\n\
	    <!-- <include file=\"$(find ego_planner)/launch/rviz.launch\"/> --> \n\
	    <node pkg =\"map_generator\" name =\"random_forest\" type =\"random_forest\" output=\"screen\">   \n\
	        <param name=\"map/x_size\"              value=\"$(arg map_size_x)\" />\n\
	        <param name=\"map/y_size\"              value=\"$(arg map_size_y)\" />\n\
	        <param name=\"map/z_size\"              value=\"$(arg map_size_z)\" />\n\
	        <param name=\"map/resoluion\"           value=\"0.1\"/>        \n\
	        <param name=\"map/is_save\"              value=\"false\"/>\n\
	        <param name=\"map/is_load\"              value=\"false\"/>\n\
	        <param name=\"map/pcd_fn\"              value=\"$(find map_generator)/map.pcd\"/>\n\
	        <param name=\"ObstacleShape/seed\"      value=\"1\"/>\n\
	        <param name=\"map/obs_num\"             value=\"0\"/>\n\
	        <param name=\"ObstacleShape/lower_rad\" value=\"0.5\"/>\n\
	        <param name=\"ObstacleShape/upper_rad\" value=\"0.7\"/>\n\
	        <param name=\"ObstacleShape/lower_hei\" value=\"0.0\"/>\n\
	        <param name=\"ObstacleShape/upper_hei\" value=\"3.0\"/>\n\
	        <param name=\"map/circle_num\"          value=\"0\"/>\n\
	        <param name=\"ObstacleShape/radius_l\"  value=\"0.7\"/>\n\
	        <param name=\"ObstacleShape/radius_h\"  value=\"0.5\"/>\n\
	        <param name=\"ObstacleShape/z_l\"       value=\"0.7\"/>\n\
	        <param name=\"ObstacleShape/z_h\"       value=\"0.8\"/>\n\
	        <param name=\"ObstacleShape/theta\"     value=\"0.5\"/>\n\
	        <param name=\"sensing/radius\"          value=\"5.0\"/>\n\
	        <param name=\"sensing/rate\"            value=\"10.0\"/>\n\
	        <param name=\"min_distance\"            value=\"1.2\"/>\n\
	    </node>\n".format(agent_num=num)
	str_end = "</launch>\n"

	x_start = -drone_dist*(num/4-0.5)
	x_half_array1 = np.linspace(x_start, -x_start, num//2)
	x_half_array2 = np.linspace(x_start+drone_dist/2, -x_start+drone_dist/2, num//2)
	np.random.shuffle(x_half_array1)
	np.random.shuffle(x_half_array2)
	y_half_array1 = map_width*np.ones(num//2)
	# 打开文件，覆盖写入
	file = open(fname, "w")
	file.write(str_begin)
	# file.close()	
	for i in range(num):
		if i < num//2:
			str_for_agent = "    <include file=\"$(find ego_planner)/launch/run_in_sim.launch\">\n\
				<arg name=\"drone_id\"   value=\"{drone_id}\"/>\n\
				<arg name=\"agent_num\"  value=\"{agent_num}\"/>\n\
				<arg name=\"init_x\"     value=\"{init_x}\"/>\n\
				<arg name=\"init_y\"     value=\"{init_y}\"/>\n\
				<arg name=\"init_z\"     value=\"{init_z}\"/>\n\
				<arg name=\"target_x\"   value=\"{target_x}\"/>\n\
				<arg name=\"target_y\"   value=\"{target_y}\"/>\n\
				<arg name=\"target_z\"   value=\"{target_z}\"/>\n\
				<arg name=\"map_size_x\" value=\"$(arg map_size_x)\"/>\n\
				<arg name=\"map_size_y\" value=\"$(arg map_size_y)\"/>\n\
				<arg name=\"map_size_z\" value=\"$(arg map_size_z)\"/>\n\
				<arg name=\"odom_topic\" value=\"$(arg odom_topic)\"/>\n\
			</include>\n".format(drone_id=i, agent_num=num, init_x=x_half_array1[i], init_y=map_width, init_z=height, target_x=-x_half_array1[i], target_y=-map_width, target_z=height) 
			print("({x},{y},{z})".format(x=x_half_array1[i], y=map_width, z=height))
		else:
			str_for_agent = "    <include file=\"$(find ego_planner)/launch/run_in_sim.launch\">\n\
				<arg name=\"drone_id\"   value=\"{drone_id}\"/>\n\
				<arg name=\"agent_num\"  value=\"{agent_num}\"/>\n\
				<arg name=\"init_x\"     value=\"{init_x}\"/>\n\
				<arg name=\"init_y\"     value=\"{init_y}\"/>\n\
				<arg name=\"init_z\"     value=\"{init_z}\"/>\n\
				<arg name=\"target_x\"   value=\"{target_x}\"/>\n\
				<arg name=\"target_y\"   value=\"{target_y}\"/>\n\
				<arg name=\"target_z\"   value=\"{target_z}\"/>\n\
				<arg name=\"map_size_x\" value=\"$(arg map_size_x)\"/>\n\
				<arg name=\"map_size_y\" value=\"$(arg map_size_y)\"/>\n\
				<arg name=\"map_size_z\" value=\"$(arg map_size_z)\"/>\n\
				<arg name=\"odom_topic\" value=\"$(arg odom_topic)\"/>\n\
			</include>\n".format(drone_id=i, agent_num=num, \
								init_x=x_half_array2[i-num//2], init_y=-map_width, init_z=height,\
								target_x=-x_half_array2[i-num//2], target_y=map_width, target_z=height)			
			print("({x},{y},{z})".format(x=x_half_array2[i-num//2], y=-map_width, z=height))
		file.write(str_for_agent)
	file.write(str_end)
	file.close()

if __name__ == '__main__':
	main(sys.argv)