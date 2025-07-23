#include "ur3_trajectory_senders/moveit_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <chrono>
using namespace std::chrono_literals;

using ur3_trajectory_senders::MoveItIK;

class IKSolverNode : public rclcpp::Node
{
public:
  IKSolverNode()
  : Node("ik_solver_node")
  {
    declare_parameter<double>("dt", 0.5);

    /* create IK after this object is wrapped in a shared_ptr */
    init_timer_ = create_wall_timer(0ms, [this](){
      init_timer_->cancel();                         // one-shot
      ik_ = std::make_unique<MoveItIK>(shared_from_this(), "ur_manipulator");
      RCLCPP_INFO(get_logger(), "MoveIt IK initialised");
    });

    sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/desired_poses", 10,
      std::bind(&IKSolverNode::onPoses, this, std::placeholders::_1));

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/planned_trajectory", 10);

    RCLCPP_INFO(get_logger(), "IKSolverNode ready (MoveIt core backend)");
  }

private:
  void onPoses(const geometry_msgs::msg::PoseArray & msg)
  {
    if (!ik_) { RCLCPP_WARN(get_logger(), "IK not ready yet"); return; }


    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
      "wrist_1_joint","wrist_2_joint","wrist_3_joint"};

    const double dt = get_parameter("dt").as_double();
    size_t good = 0;
    for (size_t i = 0; i < msg.poses.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose   = msg.poses[i];

      std::array<double,6> q;
      if (!ik_->solve(ps, q))
      {
        RCLCPP_WARN(get_logger(), "IK failed for pose %zu", i);
        continue;
      }
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.insert(pt.positions.end(), q.begin(), q.end());
      pt.time_from_start = rclcpp::Duration::from_seconds(good * dt);
      traj.points.push_back(pt);
      ++good;
    }
    if (good == 0)
    {
      RCLCPP_ERROR(get_logger(), "No IK solutions found – trajectory not published");
      return;
    }
    pub_->publish(traj);
    RCLCPP_INFO(get_logger(), "Published JointTrajectory with %zu points", good);
  }

  std::unique_ptr<MoveItIK> ik_;
  rclcpp::TimerBase::SharedPtr init_timer_; 
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKSolverNode>();          // shared_ptr first
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
