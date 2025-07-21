#include "ur3_trajectory_senders/pose_utils.hpp"
#include "ur3_trajectory_senders/interpolation_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vector>

using std::placeholders::_1; using std::placeholders::_2;
using namespace ur3_trajectory_senders;

class CaptureInterpolateNode : public rclcpp::Node
{
public:
  CaptureInterpolateNode() : Node("capture_interpolate_node")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tcp_frame",  "tool0");
    declare_parameter<int>("samples", 100);

    pose_utils_ = std::make_unique<PoseUtils>(
      this,
      get_parameter("base_frame").as_string(),
      get_parameter("tcp_frame").as_string());

    cap_srv_ = create_service<std_srvs::srv::Trigger>(
      "capture_pose", std::bind(&CaptureInterpolateNode::onCapture, this, _1, _2));

    comp_srv_ = create_service<std_srvs::srv::Trigger>(
      "compute_views", std::bind(&CaptureInterpolateNode::onCompute, this, _1, _2));

    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/interpolated_poses", 10);

    RCLCPP_INFO(get_logger(), "CaptureInterpolateNode ready.");
  }

private:
  /* ---- capture ---- */
  void onCapture(std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    auto pose = pose_utils_->getCurrentTCP();
    poses_.push_back(pose);
    res->success = true;
    res->message = "Pose captured (" + std::to_string(poses_.size()) + ")";
  }

  /* ---- compute sphere & sample ---- */
  void onCompute(std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    Eigen::Vector3d centre; double radius;
    if (poses_.size() < 4 || !fitSphereLS(poses_, centre, radius)) {
      res->success = false; res->message = "Need ≥4 poses to fit sphere"; return;
    }

    const int M = this->get_parameter("samples").as_int();
    auto samples = sampleSphere(centre, radius, poses_.front().header.frame_id, get_clock(), M);

    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = poses_.front().header.frame_id;
    msg.header.stamp = now();
    for(auto& p: samples) msg.poses.push_back(p.pose);
    pub_->publish(msg);

    res->success = true;
    res->message = "Published " + std::to_string(samples.size()) + " poses on sphere";
  }

  std::unique_ptr<PoseUtils> pose_utils_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cap_srv_, comp_srv_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CaptureInterpolateNode>());
  rclcpp::shutdown();
  return 0;
}