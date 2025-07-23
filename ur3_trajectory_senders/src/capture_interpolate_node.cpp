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
static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

// Capture, fit sphere, sample views, and visualise – with reference hemisphere
class CaptureInterpolateNode : public rclcpp::Node
{
public:
  CaptureInterpolateNode()
  : Node("capture_interpolate_node")
  {
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tcp_frame",  "tool0");
    declare_parameter<int>("samples", 10);

    // initialize pose utils object, tf listener begins to automatically fill tf buffer
    pose_utils_ = std::make_unique<PoseUtils>(
      this,
      get_parameter("base_frame").as_string(),
      get_parameter("tcp_frame").as_string());

    // service to capture a pose
    cap_srv_ = create_service<std_srvs::srv::Trigger>(
      "capture_pose", std::bind(&CaptureInterpolateNode::onCapture, this, _1, _2));

    // service to compute sphere & sample views
    comp_srv_ = create_service<std_srvs::srv::Trigger>(
      "compute_views", std::bind(&CaptureInterpolateNode::onCompute, this, _1, _2));

    // PoseArray publisher for downstream IK node
    pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/desired_poses", 10);
    // RViz visualisation publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker_array", 10);

    // publisher for *raw captured* poses (helps debugging)
    captured_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/captured_poses", 10);

    // publish a reference hemisphere (x=+0.3, r=0.15) so user has a visual guide
    publishReferenceSphere();

    RCLCPP_INFO(get_logger(), "CaptureInterpolateNode ready.");
  }

private:
  // --- helper to publish a reference sphere (centre 0.3,0,0 ; r=0.15) ---
  void publishReferenceSphere()
  {
    visualization_msgs::msg::MarkerArray ref_arr;
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
    // default orientation must be a valid quaternion
    ref.pose.orientation.w = 1.0;
    ref.pose.orientation.x = 0.0;
    ref.pose.orientation.y = 0.0;
    ref.pose.orientation.z = 0.0;
    ref.scale.x = ref.scale.y = ref.scale.z = 0.40; // diameter 0.15*2
    ref.color   = makeColor(1, 1, 0, 0.15);         // translucent yellow
    ref_arr.markers.push_back(ref);
    marker_pub_->publish(ref_arr);
    RCLCPP_INFO(get_logger(), "Reference sphere published at (0.3,0,0) r=0.15");
  }
  /* ---- capture ---- */
    /* ---- capture ---- */
  void onCapture(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    auto pose = pose_utils_->getCurrentTCP();
    poses_.push_back(pose);

    // publish updated captured PoseArray for easy echo / RViz
    geometry_msgs::msg::PoseArray cap_msg;
    cap_msg.header = pose.header;
    for (auto &p : poses_) cap_msg.poses.push_back(p.pose);
    captured_pub_->publish(cap_msg);

    RCLCPP_INFO(get_logger(), "Captured pose %zu at [%.3f %.3f %.3f]", poses_.size(),
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    res->success = true;
    res->message = "Pose captured (" + std::to_string(poses_.size()) + ")";
  }

  /* ---- compute sphere & sample ---- */
  void onCompute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    Eigen::Vector3d centre;
    double radius; // to be filled by fitSphereLS

    // ensure we have at least 4 poses
    if (poses_.size() < 4 || !fitSphereLS(poses_, centre, radius)) {
      res->success = false;
      res->message = "Need ≥4 poses to fit sphere";
      return;
    }

    const int M = this->get_parameter("samples").as_int();
    auto samples = sampleSphere(
      centre,
      radius,
      poses_.front().header.frame_id,
      get_clock(),
      M);

    // Publish PoseArray of sampled views
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = poses_.front().header.frame_id;
    msg.header.stamp = now();
    for (auto &p : samples) {
      msg.poses.push_back(p.pose);
    }
    pub_->publish(msg);

    // ---- RViz visualisation markers ----
    visualization_msgs::msg::MarkerArray marr;
    const std::string frame = poses_.front().header.frame_id;
    const rclcpp::Time stamp = now();

    // A. captured poses (red spheres)
    visualization_msgs::msg::Marker cap;
    cap.header.frame_id = frame;
    cap.header.stamp    = stamp;
    cap.ns   = "captured";
    cap.id   = 0;
    cap.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    cap.action = visualization_msgs::msg::Marker::ADD;
    cap.scale.x = cap.scale.y = cap.scale.z = 0.015; // 1.5cm
    cap.color = makeColor(1, 0, 0, 1);
    for (auto &p : poses_) {
      cap.points.push_back(p.pose.position);
    }
    marr.markers.push_back(cap);

    // B. sampled poses (green spheres)
    visualization_msgs::msg::Marker samp = cap;
    samp.ns = "sampled";
    samp.id = 1;
    samp.color = makeColor(0, 1, 0, 1);
    samp.points.clear();
    for (auto &p : samples) {
      samp.points.push_back(p.pose.position);
    }
    marr.markers.push_back(samp);

    // C. fitted sphere (translucent blue)
    visualization_msgs::msg::Marker sph;
    sph.header.frame_id = frame;
    sph.header.stamp    = stamp;
    sph.ns   = "sphere";
    sph.id   = 2;
    sph.type = visualization_msgs::msg::Marker::SPHERE;
    sph.action = visualization_msgs::msg::Marker::ADD;
    sph.pose.position.x = centre.x();
    sph.pose.position.y = centre.y();
    sph.pose.position.z = centre.z();
    sph.scale.x = sph.scale.y = sph.scale.z = radius * 2.0; // diameter
    sph.color   = makeColor(0, 0, 1, 0.3);
    marr.markers.push_back(sph);

        marker_pub_->publish(marr);

    RCLCPP_INFO(get_logger(),
      "Sphere centre [%.3f %.3f %.3f], radius %.3f, samples %d", centre.x(), centre.y(), centre.z(), radius, M);

    res->success = true;
    res->message = "Published " + std::to_string(samples.size()) + " poses on sphere";
  }

  std::unique_ptr<PoseUtils>                                  pose_utils_;
  std::vector<geometry_msgs::msg::PoseStamped>                poses_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr         cap_srv_, comp_srv_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr captured_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureInterpolateNode>());
  rclcpp::shutdown();
  return 0;
}
