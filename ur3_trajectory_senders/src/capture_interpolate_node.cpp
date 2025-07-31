#include "ur3_trajectory_senders/pose_utils.hpp"
#include "ur3_trajectory_senders/interpolation_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <vector>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace ur3_trajectory_senders;

// Helper to quickly create RGBA colors
static std_msgs::msg::ColorRGBA makeColor(float r,float g,float b,float a)
{
  std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
}

class CaptureInterpolateNode : public rclcpp::Node
{
public:
  CaptureInterpolateNode() : Node("capture_interpolate_node")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tcp_frame",  "tool0");
    declare_parameter<int>("samples", 10);

    pose_utils_ = std::make_unique<PoseUtils>(
      this,
      get_parameter("base_frame").as_string(),
      get_parameter("tcp_frame").as_string());

    cap_srv_ = create_service<std_srvs::srv::Trigger>(
      "capture_pose", std::bind(&CaptureInterpolateNode::onCapture,this,_1,_2));

    comp_srv_ = create_service<std_srvs::srv::Trigger>(
      "compute_views", std::bind(&CaptureInterpolateNode::onCompute,this,_1,_2));

    pub_sampled_ = create_publisher<geometry_msgs::msg::PoseArray>("/desired_poses", 10);
    pub_captured_ = create_publisher<geometry_msgs::msg::PoseArray>("/captured_poses", 10);
    marker_pub_   = create_publisher<visualization_msgs::msg::MarkerArray>(
                      "visualization_marker_array", 10);

    publishReferenceSphere();
    RCLCPP_INFO(get_logger(), "CaptureInterpolateNode ready.");
  }

private:
  // -------- reference sphere ------------ //
  void publishReferenceSphere()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker ref;
    ref.header.frame_id = get_parameter("base_frame").as_string();
    ref.header.stamp    = now();
    ref.ns   = "reference";
    ref.id   = 99;
    ref.type = visualization_msgs::msg::Marker::SPHERE;
    ref.action = visualization_msgs::msg::Marker::ADD;
    ref.pose.position.x = 0.3;
    ref.pose.position.y = 0.3;
    ref.pose.position.z = 0.3;
    ref.pose.orientation.w = 1.0;
    ref.scale.x = ref.scale.y = ref.scale.z = 0.40;
    ref.color   = makeColor(1,1,0,0.15);
    arr.markers.push_back(ref);
    marker_pub_->publish(arr);
  }

  // -------- capture pose --------------- //
  void onCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    auto pose = pose_utils_->getCurrentTCP();
    poses_.push_back(pose);

    // publish all captured poses as PoseArray (axes in RViz)
    geometry_msgs::msg::PoseArray pa;
    pa.header = pose.header;
    for(auto &p: poses_) pa.poses.push_back(p.pose);
    pub_captured_->publish(pa);

    RCLCPP_INFO(get_logger(),"Captured pose %zu [%.3f %.3f %.3f]",
                poses_.size(),
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    res->success = true;
    res->message = "Pose captured ("+std::to_string(poses_.size())+")";
  }

  // -------- compute arc & sample -------- //
  void onCompute(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (poses_.size() < 3) {
      res->success=false;
      res->message="Need ≥3 poses (start, mid, end)";
      return;
    }

    const int N = get_parameter("samples").as_int();
    const auto &start = poses_.front();
    const auto &end   = poses_.back();
    const auto &mid   = poses_[poses_.size()/2];

    std::vector<geometry_msgs::msg::PoseStamped> samples;
    try {
      samples = interpolateArc(start, mid, end, N);
    } catch(const std::exception &e) {
      res->success=false;
      res->message=std::string("Arc fit failed: ")+e.what();
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return;
    }

    // Publish sampled PoseArray
    geometry_msgs::msg::PoseArray out;
    out.header = start.header;
    for(auto &p: samples) out.poses.push_back(p.pose);
    pub_sampled_->publish(out);

    RCLCPP_INFO(get_logger(),"Interpolated %zu poses along arc", samples.size());
    res->success=true;
    res->message="Published "+std::to_string(samples.size())+" poses along arc";
  }

  // -------------- members --------------- //
  std::unique_ptr<PoseUtils> pose_utils_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cap_srv_, comp_srv_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_sampled_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_captured_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CaptureInterpolateNode>());
  rclcpp::shutdown();
  return 0;
}
