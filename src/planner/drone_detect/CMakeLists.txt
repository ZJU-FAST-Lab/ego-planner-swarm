cmake_minimum_required(VERSION 3.8)
project(drone_detect)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(message_filters REQUIRED)

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
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(drone_detect
  src/${PROJECT_NAME}_node.cpp
  src/drone_detector.cpp
)
target_compile_features(drone_detect PUBLIC cxx_std_17)

ament_target_dependencies(drone_detect rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs cv_bridge Eigen3 OpenCV Boost message_filters)

# Install targets
install(
  TARGETS drone_detect
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install additional resources
install(
  DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Enable testing
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}-test
#     test/test_drone_detector.cpp
#   )
#   ament_target_dependencies(${PROJECT_NAME}-test rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs cv_bridge Eigen3 OpenCV Boost message_filters)
# endif()

ament_package()
