cmake_minimum_required(VERSION 3.8)
project(odom_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -O3 -g)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Armadillo REQUIRED)
find_package(pose_utils REQUIRED)
find_package(tf2_ros REQUIRED)


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

add_executable(odom_visualization src/odom_visualization.cpp)

ament_target_dependencies(odom_visualization 
                          rclcpp 
                          sensor_msgs
                          nav_msgs
                          visualization_msgs
                          quadrotor_msgs
                          tf2
                          # Armadillo
                          pose_utils
                          tf2_ros
                          )

target_link_libraries(odom_visualization
  # pose_utils
  # ${BLAS_LIBRARIES}
  # ${LAPACK_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

# 安装mesh
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

install(TARGETS odom_visualization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
