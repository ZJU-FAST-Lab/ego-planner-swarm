import sys
import numpy as np
import os


defaultUDP=14500
defaultLocalHost=14570
defaultMavLinkUDP=14600
defaultMavLinkTCP=4560

map_width = 20
drone_dist = 2
height = 3
radius = 10

launch_fn = "/home/zuzu/Documents/Docker/dorca/src/d-orca/Firmware/launch/dorca.launch"

def main(argv):
	num = int(argv[1])
	str_begin = "<?xml version=\"1.0\"?>\n\
<launch>\n\
	<!-- MAVROS posix SITL environment launch script -->\n\
	<!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->\n\
	<!-- vehicle model and world -->\n\
	<arg name=\"est\" default=\"ekf2\"/>\n\
	<arg name=\"vehicle\" default=\"iris\"/>\n\
	<arg name=\"world\" default=\"$(find mavlink_sitl_gazebo)/worlds/empty.world\"/>\n\
	<arg name=\"scenario_type\" default=\"0\"/>\n\
	<!-- gazebo configs -->\n\
	<arg name=\"gui\" default=\"true\"/>\n\
	<arg name=\"debug\" default=\"false\"/>\n\
	<arg name=\"verbose\" default=\"false\"/>\n\
	<arg name=\"paused\" default=\"false\"/>\n\
	<!-- Gazebo sim -->\n\
	<include file=\"$(find gazebo_ros)/launch/empty_world.launch\">\n\
		<arg name=\"gui\" value=\"$(arg gui)\"/>\n\
		<arg name=\"world_name\" value=\"$(arg world)\"/>\n\
		<arg name=\"debug\" value=\"$(arg debug)\"/>\n\
		<arg name=\"verbose\" value=\"$(arg verbose)\"/>\n\
		<arg name=\"paused\" value=\"$(arg paused)\"/>\n\
	</include>\n"
	str_end = "</launch>\n"

	x_start = -drone_dist*(num/4-0.5)
	x_half_array1 = np.linspace(x_start, -x_start, num//2)
	x_half_array2 = np.linspace(x_start+drone_dist/2, -x_start+drone_dist/2, num//2)
	np.random.shuffle(x_half_array1)
	np.random.shuffle(x_half_array2)
	y_half_array1 = map_width*np.ones(num//2)
	# 打开文件，覆盖写入
	file = open(launch_fn	, "w")
	file.write(str_begin)
	divInc=2*np.pi/num

	str_group1_begin = "	<group unless=\"$(arg scenario_type)\">\n";
	file.write(str_group1_begin)
	for j in range(num):
		str_for_agent = "\n\
		<!-- UAV{i} -->\n\
		<group ns=\"uav{i}\">\n\
			<!-- MAVROS and vehicle configs -->\n\
			<arg name=\"ID\" value=\"{i}\"/>\n\
			<arg name=\"fcu_url\" default=\"udp://:{udp}@localhost:{localhost}\"/>\n\
			<!-- PX4 SITL and vehicle spawn -->\n\
			<include file=\"$(find px4)/launch/single_vehicle_spawn.launch\">\n\
				<arg name=\"x\" value=\"{x}\"/>\n\
				<arg name=\"y\" value=\"{y}\"/>\n\
				<arg name=\"z\" value=\"0\"/>\n\
				<arg name=\"R\" value=\"0\"/>\n\
				<arg name=\"P\" value=\"0\"/>\n\
				<arg name=\"Y\" value=\"0\"/>\n\
				<arg name=\"vehicle\" value=\"$(arg vehicle)\"/>\n\
				<arg name=\"mavlink_udp_port\" value=\"{MavLinkUDP}\"/>\n\
				<arg name=\"mavlink_tcp_port\" value=\"{MavLinkTCP}\"/>\n\
				<arg name=\"ID\" value=\"$(arg ID)\"/>\n\
			</include>\n\
			<!-- MAVROS -->\n\
			<include file=\"$(find mavros)/launch/px4.launch\">\n\
				<arg name=\"fcu_url\" value=\"$(arg fcu_url)\"/>\n\
				<arg name=\"gcs_url\" value=\"\"/>\n\
				<arg name=\"tgt_system\" value=\"$(eval 1 + arg('ID'))\"/>\n\
				<arg name=\"tgt_component\" value=\"1\"/>\n\
			</include>\n\
		</group>\n".format(i=j+1, udp=defaultUDP+j+1, localhost=defaultLocalHost+j+1, x=np.round(radius*np.cos(divInc*(j+1)),5), y=np.round(radius*np.sin(divInc*(j+1)),5), MavLinkUDP=defaultMavLinkUDP+j+1, MavLinkTCP=defaultMavLinkTCP+j+1)
		
		file.write(str_for_agent)
	
	str_group1_end="	</group>\n"
	file.write(str_group1_end)
	
	# add group2 to launch
	str_group2_begin = "	<group if=\"$(arg scenario_type)\">\n";
	file.write(str_group2_begin)

	for j in range(num):
		if j < num//2:
			init_x = -3+drone_dist*j
			init_y = map_width/2
		else:
			init_x = -3+drone_dist*(j-num//2)
			init_y = -map_width/2

		str_for_agent = "\n\
		<!-- UAV$i -->\n\
		<group ns=\"uav{i}\">\n\
			 <!-- MAVROS and vehicle configs -->\n\
			 <arg name=\"ID\" value=\"{i}\"/>\n\
			 <arg name=\"fcu_url\" default=\"udp://:{udp}@localhost:{localhost}\"/>\n\
			 <!-- PX4 SITL and vehicle spawn -->\n\
			 <include file=\"$(find px4)/launch/single_vehicle_spawn.launch\">\n\
				 <arg name=\"x\" value=\"{x}\"/>\n\
				 <arg name=\"y\" value=\"{y}\"/>\n\
				 <arg name=\"z\" value=\"0\"/>\n\
				 <arg name=\"R\" value=\"0\"/>\n\
				 <arg name=\"P\" value=\"0\"/>\n\
				 <arg name=\"Y\" value=\"0\"/>\n\
				 <arg name=\"vehicle\" value=\"$(arg vehicle)\"/>\n\
				 <arg name=\"mavlink_udp_port\" value=\"{MavLinkUDP}\"/>\n\
				 <arg name=\"mavlink_tcp_port\" value=\"{MavLinkTCP}\"/>\n\
				 <arg name=\"ID\" value=\"$(arg ID)\"/>\n\
			 </include>\n\
			 <!-- MAVROS -->\n\
			 <include file=\"$(find mavros)/launch/px4.launch\">\n\
				 <arg name=\"fcu_url\" value=\"$(arg fcu_url)\"/>\n\
				 <arg name=\"gcs_url\" value=\"\"/>\n\
				 <arg name=\"tgt_system\" value=\"$(eval 1 + arg('ID'))\"/>\n\
				 <arg name=\"tgt_component\" value=\"1\"/>\n\
			 </include>\n\
		</group>\n".format(i=(j+1), udp=defaultUDP+j+1, localhost=defaultLocalHost+j+1, x=init_x, y=init_y, MavLinkUDP=defaultMavLinkUDP+j+1, MavLinkTCP=defaultMavLinkTCP+j+1)
		file.write(str_for_agent)
	str_group2_end="	</group>\n"
	file.write(str_group2_end)
	file.write(str_end)
	file.close()

if __name__ == '__main__':
	main(sys.argv)