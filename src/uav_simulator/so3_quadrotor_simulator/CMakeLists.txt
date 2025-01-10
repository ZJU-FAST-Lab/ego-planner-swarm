cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(uav_utils REQUIRED)

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
  ${ARMADILLO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
ament_target_dependencies(quadrotor_dynamics rclcpp)

# 添加可执行文件
add_executable(so3_quadrotor_simulator src/quadrotor_simulator_so3.cpp)

# 链接依赖库
ament_target_dependencies(so3_quadrotor_simulator
  rclcpp
  nav_msgs
  sensor_msgs
  Eigen3
  quadrotor_msgs
  uav_utils
)
target_link_libraries(so3_quadrotor_simulator quadrotor_dynamics)

# 安装可执行文件
install(TARGETS so3_quadrotor_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# 安装config（存放rviz的config文件）
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
