#ifndef SWARM_SAR_BT__POSE_UTILS_HPP_
#define SWARM_SAR_BT__POSE_UTILS_HPP_

#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace swarm_sar_bt
{

// Looks up this robot's own pose in the global frame, the same
// global_frame -> robot_base_frame lookup Nav2 itself relies on
// (see bt_navigator's "global_frame" / "robot_base_frame" params).
inline std::optional<geometry_msgs::msg::PoseStamped> lookupRobotPose(
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame,
  const std::string & robot_base_frame,
  double transform_tolerance_s = 0.5)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = robot_base_frame;
  pose.pose.orientation.w = 1.0;

  try {
    auto transform = tf_buffer.lookupTransform(
      global_frame, robot_base_frame, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_s));
    geometry_msgs::msg::PoseStamped out;
    tf2::doTransform(pose, out, transform);
    out.header.frame_id = global_frame;
    return out;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

inline double distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__POSE_UTILS_HPP_
