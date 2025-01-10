cmake_minimum_required(VERSION 3.8)
project(so3_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cmake_utils REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)

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
  ${EIGEN3_INCLUDE_DIR}
  )

add_library(SO3Control src/SO3Control.cpp)
ament_target_dependencies(SO3Control 
  rclcpp
)
add_library(so3_control_component SHARED src/so3_control_component.cpp)

rclcpp_components_register_node(so3_control_component
  PLUGIN "SO3ControlComponent"
  EXECUTABLE so3_control_component_node)

ament_target_dependencies(so3_control_component 
  rclcpp
  nav_msgs
  rclcpp_components
  quadrotor_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(so3_control_component SO3Control )

add_executable(control_example src/control_example.cpp)
ament_target_dependencies(control_example 
  rclcpp
  quadrotor_msgs
)

install(TARGETS control_example
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_targets(export_so3_control_component)
# install(TARGETS so3_control_component SO3Control
#   EXPORT export_so3_control_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(
  TARGETS so3_control_component
  LIBRARY DESTINATION lib
)

# 安装config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
