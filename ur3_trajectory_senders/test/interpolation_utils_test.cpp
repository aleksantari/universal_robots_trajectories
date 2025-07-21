
// test/interpolation_utils_test.cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ur3_trajectory_senders/interpolation_utils.hpp"

using namespace ur3_trajectory_senders;

// Helper: create a PoseStamped at given Eigen position
geometry_msgs::msg::PoseStamped makePose(double x, double y, double z, const std::string &frame) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp = rclcpp::Clock().now();
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;  // identity orientation
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.0;
  p.pose.orientation.z = 0.0;
  return p;
}

TEST(InterpolationUtilsTest, FitSphereLS_CornerPoints) {
  // Four non-coplanar points on sphere center=(1,2,3), radius=5
  Eigen::Vector3d true_c(1.0, 2.0, 3.0);
  double true_r = 5.0;
  std::vector<geometry_msgs::msg::PoseStamped> samples;
  std::string frame = "world";
  // +X direction point
  samples.push_back(makePose(true_c.x() + true_r, true_c.y(),     true_c.z(),     frame));
  // +Y direction point
  samples.push_back(makePose(true_c.x(),     true_c.y() + true_r, true_c.z(),     frame));
  // +Z direction point
  samples.push_back(makePose(true_c.x(),     true_c.y(),          true_c.z() + true_r, frame));
  // -Z direction point
  samples.push_back(makePose(true_c.x(),     true_c.y(),          true_c.z() - true_r, frame));

  Eigen::Vector3d c_est;
  double r_est;
  bool ok = fitSphereLS(samples, c_est, r_est);
  EXPECT_TRUE(ok);
  EXPECT_NEAR(c_est.x(), true_c.x(), 1e-6);
  EXPECT_NEAR(c_est.y(), true_c.y(), 1e-6);
  EXPECT_NEAR(c_est.z(), true_c.z(), 1e-6);
  EXPECT_NEAR(r_est, true_r, 1e-6);
}

TEST(InterpolationUtilsTest, SampleSphere_UniformDistance) {
  // Sample M poses on a sphere and verify distance
  Eigen::Vector3d centre(0.0, 0.0, 0.0);
  double radius = 2.5;
  int M = 50;
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto poses = sampleSphere(centre, radius, "world", clock, M);

  ASSERT_EQ(static_cast<int>(poses.size()), M);
  // All sampled positions should lie at approx radius from centre
  for (const auto &p : poses) {
    double dx = p.pose.position.x - centre.x();
    double dy = p.pose.position.y - centre.y();
    double dz = p.pose.position.z - centre.z();
    double d = std::sqrt(dx*dx + dy*dy + dz*dz);
    EXPECT_NEAR(d, radius, 1e-3);
  }
}

