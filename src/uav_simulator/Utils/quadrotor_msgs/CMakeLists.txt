cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 设置EIGEN3的头文件路径
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs rclcpp
)

# 将该包对应的依赖导出，其他包在调用是会自动依赖
ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs std_msgs rclcpp) 

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

include_directories(include) # 添加include目录
ament_export_include_directories(include)

# 声明库
add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

# 设置依赖
ament_target_dependencies(decode_msgs std_msgs geometry_msgs nav_msgs rclcpp) 
ament_target_dependencies(encode_msgs std_msgs geometry_msgs nav_msgs rclcpp) 

# 使用 rosidl_get_typesupport_target 获取类型支持目标并链接库
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(decode_msgs "${cpp_typesupport_target}")
target_link_libraries(encode_msgs "${cpp_typesupport_target}")


# 安装库文件
install(TARGETS
  decode_msgs
  encode_msgs
  # DESTINATION lib/${PROJECT_NAME} # 导致找不到的原因
  DESTINATION lib
)
# 让ament管理包
ament_export_libraries(encode_msgs decode_msgs)

ament_package()
