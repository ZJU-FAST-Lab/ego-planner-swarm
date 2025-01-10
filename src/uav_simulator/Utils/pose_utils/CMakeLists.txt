cmake_minimum_required(VERSION 3.8)
project(pose_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)

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
  include/pose_utils
  ${ARMADILLO_INCLUDE_DIRS}
)

ament_export_include_directories(include)

add_library(pose_utils SHARED src/pose_utils.cpp)

target_link_libraries(pose_utils
  # ${LAPACK_LIBRARIES}
  # ${BLAS_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

ament_target_dependencies(pose_utils rclcpp )

# 安装库文件
install(TARGETS
  pose_utils
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(pose_utils)


ament_package()
