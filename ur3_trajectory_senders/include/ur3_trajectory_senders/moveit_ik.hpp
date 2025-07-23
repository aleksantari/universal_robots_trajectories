#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h> //loades the robot URDF + SDRF 
#include <moveit/robot_state/robot_state.h>               // holds the joint values and allows for FK/IK queries
#include <tf2_eigen/tf2_eigen.hpp>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ur3_trajectory_senders
{

class MoveItIK
{
public:
  /** node is needed for robot_model_loader ctor */
  MoveItIK(const rclcpp::Node::SharedPtr& node,
           const std::string & planning_group = "ur_manipulator")
  : robot_model_loader_(node, "robot_description"),             // loads UDRF via parameter
    kmodel_(robot_model_loader_.getModel()),                    // gets kinematic tree
    kstate_(kmodel_),                                           // builds current state object
    jmg_(kmodel_->getJointModelGroup(planning_group))           // fetches the arm joint grtoup
  {
    if (!kmodel_ || !jmg_)
      throw std::runtime_error("MoveItIK: failed to load robot model / joint group");
    kstate_.setToDefaultValues();
  }

  /** Solve IK. Fills out[6] on success */
  bool solve(const geometry_msgs::msg::PoseStamped & pose,
             std::array<double, 6> & out,
             double timeout = 0.05)
  {
    Eigen::Isometry3d target;
    tf2::fromMsg(pose.pose, target);

    if (!kstate_.setFromIK(jmg_, target, timeout))
      return false;

    std::vector<double> sol;
    kstate_.copyJointGroupPositions(jmg_, sol);                 // ✔ correct API
    std::copy(sol.begin(), sol.end(), out.begin());
    return true;
  }

private:
  robot_model_loader::RobotModelLoader          robot_model_loader_;
  moveit::core::RobotModelPtr                   kmodel_;
  moveit::core::RobotState                      kstate_;
  const moveit::core::JointModelGroup          *jmg_;
};

} // namespace
