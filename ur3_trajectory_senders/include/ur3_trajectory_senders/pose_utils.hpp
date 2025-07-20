#pragma once                                             //once include this file once per build
#include <geometry_msgs/msg/pose_stamped.hpp>            // its a pose but stamped with a header
#include <rclcpp/rclcpp.hpp>                             // 
#include <tf2_ros/transform_listener.h>                  // object that subscribes to /tf and /tf_static and fills buffer automatically
#include <tf2_ros/buffer.h>                              // data structure that stores all the transform frames being broadcasted in the system
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>       // convert between TF2 objects and geometry_msgs

// wrapping class in a namespace to prevent name conflicts with other packages
namespace ur3_trajectory_senders {

class PoseUtils
{
public:
  // constructor 
  PoseUtils(const std::shared_ptr<rclcpp::Node>& node,
            const std::string& base_link = "base_link",
            const std::string& tcp_frame = "tool0");

  /**
   * @brief Get the current TCP pose in the world frame.
   * @return PoseStamped in base_link frame.
   */
  geometry_msgs::msg::PoseStamped getCurrentTCP();

private:        
  std::shared_ptr<rclcpp::Node> node_;                          // pointer calling my node for logging and clock access
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                  // 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string base_link_;
  std::string tcp_frame_;
};

} // namespace