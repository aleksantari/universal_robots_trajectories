#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // initialize the node w/ a name
  auto node = rclcpp::Node::make_shared("topic_trajectory_publisher");

  // initilize published to controller topic
  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  // combose a joint trajectory message and fill it appropriately
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.joint_names =
  {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
    "wrist_3_joint"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};
  point.time_from_start.sec = 2;
  point.time_from_start.nanosec = 0;


  msg.points.push_back(point);
  publisher->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Published trajectory message");

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
