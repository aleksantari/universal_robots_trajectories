cmake_minimum_required(VERSION 3.8)
project(ur3_trajectory_senders)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# pose_utils library
add_library(pose_utils src/pose_utils.cpp)
target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(pose_utils
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# executable using pose_utils
add_executable(send_trajectory_action src/send_trajectory_action.cpp)
target_link_libraries(send_trajectory_action pose_utils)
ament_target_dependencies(send_trajectory_action
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  builtin_interfaces
)

# install targets
install(TARGETS
  send_trajectory_action
  pose_utils
  DESTINATION lib/${PROJECT_NAME}
)

# install include headers
install(DIRECTORY include/
  DESTINATION include/
)

# =============================
# Testing
# =============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_link_libraries(test_pose_utils pose_utils)
  ament_target_dependencies(test_pose_utils
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
  )
endif()

ament_package()
