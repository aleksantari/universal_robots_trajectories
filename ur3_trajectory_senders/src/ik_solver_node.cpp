#include "ur3_trajectory_senders/moveit_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <chrono>
using namespace std::chrono_literals;

using ur3_trajectory_senders::MoveItIK;

class IKSolverNode : public rclcpp::Node
{
public:
  IKSolverNode() : Node("ik_solver_node")
  {
    declare_parameter<double>("dt", 0.5);

    // late‑init timer so shared_from_this() is valid
    init_timer_ = create_wall_timer(0ms, [this]() {
      init_timer_->cancel();
      ik_ = std::make_unique<MoveItIK>(shared_from_this(), "ur_manipulator");

      // Load RobotModel again for FK check (independent of IK helper)
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
          shared_from_this(), "robot_description");
      kmodel_ = robot_model_loader_->getModel();
      jmg_    = kmodel_->getJointModelGroup("ur_manipulator");
      RCLCPP_INFO(get_logger(), "MoveIt IK & FK initialised");
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
    if (!ik_ || !kmodel_) {
      RCLCPP_WARN(get_logger(), "IK/FK not ready yet");
      return;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    const double dt = get_parameter("dt").as_double();
    size_t good = 0;

    moveit::core::RobotState state(kmodel_);

    for (size_t i = 0; i < msg.poses.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose   = msg.poses[i];

      std::array<double, 6> q;
      if (!ik_->solve(ps, q)) {
        RCLCPP_WARN(get_logger(), "IK failed for pose %zu", i);
        continue;
      }

      // ------ FK verification ------
      std::vector<double> q_vec(q.begin(), q.end());
      state.setJointGroupPositions(jmg_, q_vec);
      state.updateLinkTransforms();
      Eigen::Isometry3d fk = state.getGlobalLinkTransform("tool0");

      Eigen::Vector3d p_des(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
      Eigen::Vector3d p_fk  = fk.translation();
      double pos_err = (p_fk - p_des).norm();

      Eigen::Quaterniond q_des(ps.pose.orientation.w, ps.pose.orientation.x,
                               ps.pose.orientation.y, ps.pose.orientation.z);
      Eigen::Quaterniond q_fk(fk.rotation());
      double ang_err = q_des.angularDistance(q_fk);

      RCLCPP_INFO(get_logger(),
                  "Pose %zu: pos_err %.4fm  ang_err %.2f°", i, pos_err, ang_err*180.0/M_PI);
      // -----------------------------

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.insert(pt.positions.end(), q.begin(), q.end());
      pt.time_from_start = rclcpp::Duration::from_seconds(good * dt);
      traj.points.push_back(pt);
      ++good;
    }

    if (good == 0) {
      RCLCPP_ERROR(get_logger(), "No IK solutions found – trajectory not published");
      return;
    }
    pub_->publish(traj);
    RCLCPP_INFO(get_logger(), "Published JointTrajectory with %zu points", good);
  }

  // late‑initialised components
  std::unique_ptr<MoveItIK> ik_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kmodel_;
  const moveit::core::JointModelGroup * jmg_ {nullptr};

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
