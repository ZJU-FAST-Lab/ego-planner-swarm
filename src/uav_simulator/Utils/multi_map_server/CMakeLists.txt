cmake_minimum_required(VERSION 3.8)
project(multi_map_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pose_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Armadillo REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/SparseMap3D.msg"
  "msg/VerticalOccupancyGridList.msg"

  DEPENDENCIES sensor_msgs geometry_msgs nav_msgs std_msgs rclcpp laser_geometry
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include  
  ${ARMADILLO_INCLUDE_DIRS}
)

add_executable(multi_map_visualization src/multi_map_visualization.cc)

ament_target_dependencies(multi_map_visualization 
  rclcpp  
  pose_utils
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(multi_map_visualization 
   ${ARMADILLO_LIBRARIES}
)

# 链接生成的msg
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(multi_map_visualization "${cpp_typesupport_target}")

install(TARGETS
  multi_map_visualization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
