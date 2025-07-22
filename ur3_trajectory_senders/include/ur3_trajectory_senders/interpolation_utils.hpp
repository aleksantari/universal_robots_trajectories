#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <Eigen/Geometry>  // for Vector3d, Quaterniond
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace ur3_trajectory_senders {

/** Sphere fit: least‑squares centre & radius. */
bool fitSphereLS(const std::vector<geometry_msgs::msg::PoseStamped>& samples,
                 Eigen::Vector3d& centre,
                 double& radius);

/** Uniformly sample `M` viewing poses on the fitted sphere.
 *  Z‑axis faces centre; X/Y built from world +Y up reference.
 */
std::vector<geometry_msgs::msg::PoseStamped>
sampleSphere(const Eigen::Vector3d& centre,
             double radius,
             const std::string& frame_id,
             rclcpp::Clock::SharedPtr clock,
             int M = 10);

} // namespace