

#include "ur3_trajectory_senders/pose_utils.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
namespace ur3_trajectory_senders {

PoseUtils::PoseUtils(rclcpp::Node *node,
                     const std::string &base_frame,
                     const std::string &tcp_frame)
: node_(node), base_frame_(base_frame), tcp_frame_(tcp_frame)
{
  // Use node's clock for TF2 timing
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock_interface());
  // Create a TransformListener to fill the buffer
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, *node_);
}

geometry_msgs::msg::PoseStamped PoseUtils::getCurrentTCP() const
{
  geometry_msgs::msg::PoseStamped pose;
  // Initialize header
  pose.header.frame_id = base_frame_;
  pose.header.stamp = node_->now();

  try {
    // Wait for transform up to 100ms
    if (!tf_buffer_->canTransform(base_frame_, tcp_frame_, rclcpp::Time(0), 100ms)) {
      RCLCPP_WARN(node_->get_logger(), "Transform %s->%s not available after 100ms",
                  base_frame_.c_str(), tcp_frame_.c_str());
      return pose;
    }
    // Lookup the latest transform
    auto tf = tf_buffer_->lookupTransform(
      base_frame_, tcp_frame_, rclcpp::Time(0));

    // Fill pose from transform
    pose.header.stamp = tf.header.stamp;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
  }
  return pose;
}

} // namespace ur3_trajectory_senders