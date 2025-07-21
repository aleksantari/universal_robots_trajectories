#pragma once                                             //once include this file once per build
#include <geometry_msgs/msg/pose_stamped.hpp>            // its a pose but stamped with a header
#include <rclcpp/rclcpp.hpp>                             //
#include <tf2_ros/transform_listener.h>                  // object that subscribes to /tf and /tf_static and fills buffer automatically
#include <tf2_ros/buffer.h>                              // data structure that stores all the transform frames being broadcasted in the system
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>       // convert between TF2 objects and geometry_msgs

// wrapping class in a namespace to prevent name conflicts with other packages
namespace ur3_trajectory_senders {

/**
 * @class PoseUtils
 * @brief Helper for obtaining the end-effector (TCP) pose from TF2.
 */
class PoseUtils
{
public:
  /**
   * @brief Construct a new PoseUtils object
   * @param node Raw pointer to the ROS2 node for logging and clock.
   * @param base_frame The reference frame id (normally "base_link").
   * @param tcp_frame  The tool frame id (normally "tool0").
   */
  PoseUtils(rclcpp::Node *node,
            const std::string &base_frame = "base_link",
            const std::string &tcp_frame  = "tool0");

  /**
   * @brief Look up the latest transform and return the TCP pose
   * @return geometry_msgs::msg::PoseStamped
   */
  geometry_msgs::msg::PoseStamped getCurrentTCP() const;

private:
  rclcpp::Node *node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_frame_;
  std::string tcp_frame_;
};

} // namespace ur3_trajectory_senders