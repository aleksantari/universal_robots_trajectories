cmake_minimum_required(VERSION 3.8)
project(ur3_trajectory_senders)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)

# add executables
add_executable(send_trajectory_topic src/send_trajectory_topic.cpp)
ament_target_dependencies(send_trajectory_topic
  rclcpp
  trajectory_msgs

)

add_executable(send_trajectory_action src/send_trajectory_action.cpp)
ament_target_dependencies(send_trajectory_action
  rclcpp
  rclcpp_action
  trajectory_msgs
  control_msgs
  builtin_interfaces
)
# install targets
install(TARGETS
  send_trajectory_topic
  send_trajectory_action
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
