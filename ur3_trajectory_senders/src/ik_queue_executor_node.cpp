#include "ur3_trajectory_senders/moveit_ik.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <deque>
#include <mutex>
#include <chrono>
using namespace std::chrono_literals;
using FollowJT = control_msgs::action::FollowJointTrajectory;
using ur3_trajectory_senders::MoveItIK;

class IKQueueExecutorNode : public rclcpp::Node
{
public:
  IKQueueExecutorNode() : Node("ik_queue_executor_node")
  {
    declare_parameter<double>("dt", 0.5);
    declare_parameter<std::string>("planning_group", "ur_manipulator");
    declare_parameter<std::string>("controller_ns", "/joint_trajectory_controller");

    /* late-init MoveIt IK to avoid bad_weak_ptr */
    init_timer_ = create_wall_timer(0ms, [this]() {
      init_timer_->cancel();
      ik_ = std::make_unique<MoveItIK>(shared_from_this(),
                                       get_parameter("planning_group").as_string());
      RCLCPP_INFO(get_logger(), "MoveIt IK initialised");
    });

    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/desired_poses", 10,
      std::bind(&IKQueueExecutorNode::onPoses, this, std::placeholders::_1));

    exec_srv_ = create_service<std_srvs::srv::Trigger>(
      "/execute_next",
      std::bind(&IKQueueExecutorNode::onExecute, this,
                std::placeholders::_1, std::placeholders::_2));

    /* action client */
    std::string ns = get_parameter("controller_ns").as_string();
    action_client_ = rclcpp_action::create_client<FollowJT>(
      this, ns + "/follow_joint_trajectory");

    RCLCPP_INFO(get_logger(), "IKQueueExecutorNode ready");
  }

private:
  /* ---------- subscriber: poses ➜ IK ➜ queue ---------- */
  void onPoses(const geometry_msgs::msg::PoseArray & msg)
  {
    if (!ik_) { RCLCPP_WARN(get_logger(), "IK not ready yet"); return; }

    const double dt = get_parameter("dt").as_double();
    size_t added = 0;

    std::lock_guard<std::mutex> lock(queue_mtx_);
    for (auto & pose : msg.poses)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header; ps.pose = pose;

      std::array<double,6> q;
      if (!ik_->solve(ps, q)) {
        RCLCPP_WARN(get_logger(), "IK failed – skipping pose");
        continue;
      }

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.insert(pt.positions.end(), q.begin(), q.end());
      pt.time_from_start = rclcpp::Duration::from_seconds(dt);  // 1-pt traj
      queue_.push_back(pt);
      ++added;
    }
    RCLCPP_INFO(get_logger(), "Queued %zu new trajectory points (queue size = %zu)",
                added, queue_.size());
  }

  /* ---------- service: pop queue & send action ---------- */
  void onExecute(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!action_client_->wait_for_action_server(1s)) {
      res->success = false;
      res->message = "Action server not available";
      return;
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      if (queue_.empty()) {
        res->success = false;
        res->message = "Queue empty";
        return;
      }
      pt = queue_.front();
      queue_.pop_front();
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
      "wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    traj.points.push_back(pt);
    traj.header.stamp = now();

    FollowJT::Goal goal;
    goal.trajectory = traj;

    auto send = action_client_->async_send_goal(goal);
    res->success = true;
    res->message = "Sent 1-point trajectory";
    RCLCPP_INFO(get_logger(), "Sent trajectory; remaining queue size = %zu", queue_.size());
  }

  /* -------------------------- fields -------------------------- */
  std::unique_ptr<MoveItIK> ik_;
  std::deque<trajectory_msgs::msg::JointTrajectoryPoint> queue_;
  std::mutex queue_mtx_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr exec_srv_;
  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKQueueExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
