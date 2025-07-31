#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace ur3_trajectory_senders
{

/** Interpolate an arc given three poses (start, mid, end). */
std::vector<geometry_msgs::msg::PoseStamped>
interpolateArc(const geometry_msgs::msg::PoseStamped & start,
               const geometry_msgs::msg::PoseStamped & mid,
               const geometry_msgs::msg::PoseStamped & end,
               int samples);

} // namespace ur3_trajectory_senders
