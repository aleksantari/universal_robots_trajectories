// src/capture_interpolate_node.cpp

#include "ur3_trajectory_senders/pose_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using namespace ur3_trajectory_senders;
using std::placeholders::_1;
using std::placeholders::_2;

class PoseCaptureNode : public rclcpp::Node
{
public:
  PoseCaptureNode()
  : Node("pose_capture_node")
  {
    // --- Parameters for frame names ---
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("tcp_frame",  "tool0");

    auto base_frame = this->get_parameter("base_frame").as_string();
    auto tcp_frame  = this->get_parameter("tcp_frame").as_string();

    // --- Initialize PoseUtils with raw Node* ---
    pose_utils_ = std::make_unique<ur3_trajectory_senders::PoseUtils>(
      this, base_frame, tcp_frame);

    // --- Expose a Trigger service ---
    capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "capture_pose",
      std::bind(&PoseCaptureNode::captureCallback, this, _1, _2));

    RCLCPP_INFO(get_logger(),
      "PoseCaptureNode ready. Call:\n"
      "  ros2 service call /capture_pose std_srvs/srv/Trigger\n"
      "to capture the current TCP pose.");
  }

private:
  void captureCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response>      res)
  {
    // Query the latest TCP pose via TF2
    auto stamped_pose = pose_utils_->getCurrentTCP();

    // Store it for later interpolation
    captured_poses_.push_back(stamped_pose);

    // Log the result
    const auto &p = stamped_pose.pose.position;
    RCLCPP_INFO(get_logger(),
      "Captured pose [%zu]  frame='%s'  time=%.3f  x=%.3f  y=%.3f  z=%.3f",
      captured_poses_.size(),
      stamped_pose.header.frame_id.c_str(),
      stamped_pose.header.stamp.sec +
        stamped_pose.header.stamp.nanosec * 1e-9,
      p.x, p.y, p.z);

    res->success = true;
    res->message = "Pose captured successfully";
  }

  std::unique_ptr<PoseUtils>                             pose_utils_;
  std::vector<geometry_msgs::msg::PoseStamped>           captured_poses_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr     capture_srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
