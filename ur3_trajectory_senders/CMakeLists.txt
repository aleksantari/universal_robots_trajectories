cmake_minimum_required(VERSION 3.8)
project(ur3_trajectory_senders)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ——————————
# Find dependencies
# ——————————
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(ament_cmake_gtest REQUIRED)

# ——————————
# pose_utils library
# ——————————
add_library(pose_utils src/pose_utils.cpp)
ament_target_dependencies(pose_utils
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)
target_include_directories(pose_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ——————————
# interpolation_utils library
# ——————————
add_library(interpolation_utils src/interpolation_utils.cpp)
ament_target_dependencies(interpolation_utils
  rclcpp
  geometry_msgs
  Eigen3
)
target_include_directories(interpolation_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ——————————
# Executable: trajectory action client
# ——————————
add_executable(send_trajectory_action src/send_trajectory_action.cpp)
target_link_libraries(send_trajectory_action
  pose_utils
)
ament_target_dependencies(send_trajectory_action
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  builtin_interfaces
)

# ——————————
# Executable: capture + interpolate node
# ——————————
add_executable(capture_interpolate_node src/capture_interpolate_node.cpp)
target_link_libraries(capture_interpolate_node
  pose_utils
  interpolation_utils
)
ament_target_dependencies(capture_interpolate_node
  rclcpp
  std_srvs
  geometry_msgs
  Eigen3
  visualization_msgs
)

# ——————————
# Executable: IK solver node
# ——————————
add_executable(ik_solver_node src/ik_solver_node.cpp)
target_link_libraries(ik_solver_node
  interpolation_utils
)
ament_target_dependencies(ik_solver_node
  rclcpp
  std_srvs
  geometry_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
  tf2_eigen
)
target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


# ——————————
# Installation
# ——————————
install(TARGETS
  pose_utils
  interpolation_utils
  send_trajectory_action
  capture_interpolate_node
  ik_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# ——————————
# Testing
# ——————————
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
  interpolation_utils_test
  test/interpolation_utils_test.cpp
  src/interpolation_utils.cpp
  )
  ament_target_dependencies(
    interpolation_utils_test
    Eigen3
    rclcpp
    geometry_msgs
  )
  target_include_directories(interpolation_utils_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
