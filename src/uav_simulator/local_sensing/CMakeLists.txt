cmake_minimum_required(VERSION 3.8)
project(local_sensing)

# 设置编译选项
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ENABLE_CUDA false)
# set(ENABLE_CUDA true)


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

if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
  set(CUDA_NVCC_FLAGS 
#       -gencode arch=compute_20,code=sm_20;
#       -gencode arch=compute_20,code=sm_21;
#       -gencode arch=compute_30,code=sm_30;
#       -gencode arch=compute_35,code=sm_35;
#       -gencode arch=compute_50,code=sm_50;
#       -gencode arch=compute_52,code=sm_52; 
#       -gencode arch=compute_60,code=sm_60;
#       -gencode arch=compute_61,code=sm_61;
      # -gencode arch=compute_75,code=sm_75;
      -gencode arch=compute_89,code=sm_89;
  ) 

  set(CUDA_PROPAGATE_HOST_FLAGS OFF)

  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)
  find_package(Boost REQUIRED date_time)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(quadrotor_msgs REQUIRED)

  # 生成动态重配置（如果需要）
  # generate_dynamic_reconfigure_options(cfg/local_sensing_node.cfg)

  include_directories(
    SYSTEM 
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${image_transport_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
  )

  # CUDA 库
  cuda_add_library(depth_render_cuda SHARED src/depth_render.cu)

  install(TARGETS
    depth_render_cuda
    DESTINATION lib
  )

  ament_export_libraries(depth_render_cuda)

  # 添加可执行文件
  add_executable(pcl_render_node src/pcl_render_node.cpp)

  # 链接库
  ament_target_dependencies(pcl_render_node
    rclcpp
    cv_bridge
    image_transport
    pcl_ros
    sensor_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    OpenCV
    Boost
  )
  target_link_libraries(pcl_render_node depth_render_cuda)

else(ENABLE_CUDA)

  # 普通情况下的依赖
  find_package(Eigen3 REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(quadrotor_msgs REQUIRED)
  find_package(Boost REQUIRED date_time)

  include_directories(
    SYSTEM
    ${Eigen_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${quadrotor_msgs_INCLUDE_DIRS}
  )

  add_executable(pcl_render_node src/pointcloud_render_node.cpp)

  ament_target_dependencies(pcl_render_node
    rclcpp
    pcl_ros
    sensor_msgs
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    Boost
  )

endif(ENABLE_CUDA)

# Install setup
install(TARGETS pcl_render_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 params 文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
