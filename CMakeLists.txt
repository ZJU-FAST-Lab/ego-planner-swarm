cmake_minimum_required(VERSION 3.0.2)
project(path_evaluation)

add_compile_options(-std=c++11)

find_package(Armadillo REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	uav_utils
	pose_utils
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  rviz_visual_tools
	)

catkin_package()

include_directories(
  include
  ${ARMADILLO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/multi_path_evaluation_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   ${PCL_LIBRARIES}
   pose_utils
)

add_executable(path_visualization_node src/multi_path_visualization_node.cpp)

target_link_libraries(path_visualization_node
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   ${PCL_LIBRARIES}
   pose_utils
)