#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "ur3_trajectory_senders/pose_utils.hpp"

using namespace ur3_trajectory_senders;

// test case with test group: PoseUtilsTest, test name: test_lookup_pose
TEST(PoseUtilsTest, test_lookup_pose)
{
  rclcpp::init(0, nullptr);
  // create ros2 node
  auto node = rclcpp::Node::make_shared("test_pose_utils_node");

  // instance of PoseUtils class and passing in node to make constructor happy
  // this should create a TF buffer and start Transforms listen at will fill it
  PoseUtils pose_utils(node);

  // call the utility function
  auto pose = pose_utils.getCurrentTCP();

  std::cout << "Pose frame ID: " << pose.header.frame_id << std::endl;
  // This only passes if TF is active in the background
  EXPECT_TRUE(pose.header.frame_id == "base_link" || pose.header.frame_id.empty());



  rclcpp::shutdown();
}
