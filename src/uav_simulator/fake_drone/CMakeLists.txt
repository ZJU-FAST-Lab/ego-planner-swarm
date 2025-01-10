cmake_minimum_required(VERSION 3.8)
project(poscmd_2_odom)

# 设置编译类型和编译选项
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
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

# 包含头文件路径
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

# 添加可执行文件
add_executable(poscmd_2_odom src/poscmd_2_odom.cpp)

# 链接依赖库
ament_target_dependencies(poscmd_2_odom
  rclcpp
  nav_msgs
  Eigen3
  quadrotor_msgs
)

# 安装可执行文件
install(TARGETS poscmd_2_odom
  DESTINATION lib/${PROJECT_NAME})

# 让ament自动管理
ament_package()