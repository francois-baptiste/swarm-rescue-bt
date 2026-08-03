#ifndef SWARM_SAR_BT__IS_AT_VICTIM_CONDITION_HPP_
#define SWARM_SAR_BT__IS_AT_VICTIM_CONDITION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/pose_utils.hpp"
#include "tf2_ros/buffer.h"

namespace swarm_sar_bt
{

// SUCCESS if this robot's own pose (via TF, same global_frame /
// robot_base_frame convention as Nav2) is within `tolerance` meters of
// "claimed_victim_pose". Only meaningful after HasClaim has already
// succeeded earlier in the same Sequence.
class IsAtVictimCondition : public BT::ConditionNode
{
public:
  IsAtVictimCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    config.blackboard->get("global_frame", global_frame_);
    config.blackboard->get("robot_base_frame", robot_base_frame_);
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("tolerance", 0.5, "arrival distance in meters")};
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped victim_pose;
    if (!config().blackboard->get("claimed_victim_pose", victim_pose)) {
      return BT::NodeStatus::FAILURE;
    }
    double tolerance = 0.5;
    getInput("tolerance", tolerance);

    auto robot_pose = lookupRobotPose(*tf_buffer_, global_frame_, robot_base_frame_);
    if (!robot_pose) {
      return BT::NodeStatus::FAILURE;
    }
    return distance2D(robot_pose->pose.position, victim_pose.pose.position) <= tolerance ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__IS_AT_VICTIM_CONDITION_HPP_
