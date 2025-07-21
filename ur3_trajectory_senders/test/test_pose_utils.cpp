// test/test_pose_utils.cpp

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "ur3_trajectory_senders/pose_utils.hpp"

using namespace ur3_trajectory_senders;

// test group: PoseUtilsTest, test name: test_lookup_pose
TEST(PoseUtilsTest, test_lookup_pose)
{
  // Initialize ROS 2 (no args)
  rclcpp::init(0, nullptr);

  // Create a node (shared_ptr owned by test framework)
  auto node = rclcpp::Node::make_shared("test_pose_utils_node");

  // Pass raw pointer to PoseUtils, per the new signature
  PoseUtils pose_utils(node.get());

  // Call the utility function
  auto pose = pose_utils.getCurrentTCP();

  // Print for visibility (optional)
  std::cout << "Pose frame ID: " << pose.header.frame_id << std::endl;

  // We expect either a valid 'base_link' or an empty header if TF isn't up
  EXPECT_TRUE(pose.header.frame_id == "base_link" || pose.header.frame_id.empty());

  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
