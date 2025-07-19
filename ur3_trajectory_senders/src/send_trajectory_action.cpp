// send_trajectory_action.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <chrono>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class TrajectoryActionClient : public rclcpp::Node {
public:

  TrajectoryActionClient()
  : Node("action_trajectory_client") {
    //creating an action client with action type and action server name
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");
    // wait for server
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }


    // creating a goal message
    // the trajectory message is embedded in the goal message
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};
    point.time_from_start = builtin_interfaces::msg::Duration();
    point.time_from_start.sec = 2;
    goal_msg.trajectory.points.push_back(point);


    // send the goal asychronously
    // there is a 2 callbacks: for goal accept/rejected, for when result arrives
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      std::bind(&TrajectoryActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = 
      std::bind(&TrajectoryActionClient::result_callback, this, std::placeholders::_1);
    
    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;

  void goal_response_callback(GoalHandle::SharedPtr handle) {
    if (!handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
  }

  void result_callback(const GoalHandle::WrappedResult & result) {
    RCLCPP_INFO(this->get_logger(), "Trajectory result received: %d", static_cast<int>(result.code));
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryActionClient>());
  return 0;
}
