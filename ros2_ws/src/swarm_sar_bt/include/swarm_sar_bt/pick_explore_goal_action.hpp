#ifndef SWARM_SAR_BT__PICK_EXPLORE_GOAL_ACTION_HPP_
#define SWARM_SAR_BT__PICK_EXPLORE_GOAL_ACTION_HPP_

#include <cmath>
#include <memory>
#include <random>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/pose_utils.hpp"
#include "tf2_ros/buffer.h"

namespace swarm_sar_bt
{

// Reached only when nothing is claimed and nothing is currently in sensor
// range: samples a random point within `radius` metres of this robot's own
// pose and writes it to "goal" for the NavigateToPose leaf that follows it
// in the tree. This is a placeholder for real frontier exploration - a
// production version would sample free space from this robot's own local
// costmap (nav2_costmap_2d) or an explore_lite-style frontier detector
// instead of sampling blindly; NavigateToPose failing on an unreachable
// sample simply fails that tick and a new point is sampled on the next one.
class PickExploreGoalAction : public BT::SyncActionNode
{
public:
  PickExploreGoalAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config), rng_(std::random_device{}())
  {
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    config.blackboard->get("global_frame", global_frame_);
    config.blackboard->get("robot_base_frame", robot_base_frame_);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("radius", 8.0, "sampling radius in meters")};
  }

  BT::NodeStatus tick() override
  {
    double radius = 8.0;
    getInput("radius", radius);

    auto robot_pose = lookupRobotPose(*tf_buffer_, global_frame_, robot_base_frame_);
    if (!robot_pose) {
      return BT::NodeStatus::FAILURE;
    }

    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> radius_dist(1.0, radius);
    const double angle = angle_dist(rng_);
    const double r = radius_dist(rng_);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = global_frame_;
    goal.pose.position.x = robot_pose->pose.position.x + r * std::cos(angle);
    goal.pose.position.y = robot_pose->pose.position.y + r * std::sin(angle);
    goal.pose.orientation.z = std::sin(angle / 2.0);
    goal.pose.orientation.w = std::cos(angle / 2.0);

    config().blackboard->set("goal", goal);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
  std::mt19937 rng_;
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__PICK_EXPLORE_GOAL_ACTION_HPP_
