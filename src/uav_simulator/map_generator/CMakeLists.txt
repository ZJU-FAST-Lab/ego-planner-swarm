cmake_minimum_required(VERSION 3.8)
project(map_generator)

# 设置构建类型
set(CMAKE_BUILD_TYPE "Release")

# 设置C++标准（建议使用C++14或C++17）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 添加编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

# 仅在Release模式下添加优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -g)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)              
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# set(PCL_DIR "/usr/local/share/pcl-1.14")
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)


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

add_executable(random_forest src/random_forest_sensing.cpp)

ament_target_dependencies(random_forest rclcpp std_msgs geometry_msgs nav_msgs pcl_conversions Eigen3 PCL)
target_link_libraries(random_forest ${PCL_LIBRARIES})

install(TARGETS random_forest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
