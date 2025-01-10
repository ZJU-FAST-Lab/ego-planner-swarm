cmake_minimum_required(VERSION 3.8)
project(plan_env)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

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
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

add_library( plan_env SHARED
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )

ament_target_dependencies(plan_env std_msgs visualization_msgs geometry_msgs nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)

target_link_libraries( plan_env 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)  

add_executable(obj_generator
    src/obj_generator.cpp 
)
ament_target_dependencies(obj_generator std_msgs visualization_msgs geometry_msgs nav_msgs rclcpp Eigen3 PCL cv_bridge message_filters pcl_conversions)

install(TARGETS plan_env
        DESTINATION lib
)

install(TARGETS obj_generator
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(plan_env)

ament_package()
