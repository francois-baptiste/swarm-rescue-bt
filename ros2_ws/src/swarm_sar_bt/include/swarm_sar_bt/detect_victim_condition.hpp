#ifndef SWARM_SAR_BT__DETECT_VICTIM_CONDITION_HPP_
#define SWARM_SAR_BT__DETECT_VICTIM_CONDITION_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/pose_utils.hpp"
#include "swarm_sar_bt/swarm_knowledge.hpp"
#include "tf2_ros/buffer.h"

namespace swarm_sar_bt
{

// SUCCESS if a not-actively-claimed victim is within sensor_range of this
// robot's own pose. This is the only place "what should I look at" gets
// decided, and it is decided purely from this robot's own TF pose plus
// whatever SwarmKnowledge has locally accumulated from the radio - no
// robot ever queries a global registry of victims or claims.
class DetectVictimCondition : public BT::ConditionNode
{
public:
  DetectVictimCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    knowledge_ = config.blackboard->get<std::shared_ptr<SwarmKnowledge>>("swarm_knowledge");
    config.blackboard->get("robot_id", robot_id_);
    config.blackboard->get("global_frame", global_frame_);
    config.blackboard->get("robot_base_frame", robot_base_frame_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("sensor_range", 3.0, "detection radius in meters"),
      BT::InputPort<double>("claim_timeout", 15.0, "seconds before another robot's claim is stale"),
      BT::OutputPort<int>("seen_victim_id"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("seen_victim_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    double sensor_range = 3.0;
    double claim_timeout = 15.0;
    getInput("sensor_range", sensor_range);
    getInput("claim_timeout", claim_timeout);

    auto robot_pose = lookupRobotPose(*tf_buffer_, global_frame_, robot_base_frame_);
    if (!robot_pose) {
      return BT::NodeStatus::FAILURE;
    }

    std::optional<swarm_sar_bt::msg::Victim> best;
    double best_dist = 0.0;
    for (const auto & victim : knowledge_->latestVictims()) {
      if (victim.rescued) {
        continue;
      }
      const double d = distance2D(robot_pose->pose.position, victim.pose);
      if (d > sensor_range) {
        continue;
      }
      if (knowledge_->isActivelyClaimedByOther(victim.id, robot_id_, claim_timeout)) {
        continue;
      }
      if (!best || d < best_dist) {
        best = victim;
        best_dist = d;
      }
    }

    if (!best) {
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped victim_pose;
    victim_pose.header.frame_id = global_frame_;
    victim_pose.header.stamp = node_->now();
    victim_pose.pose.position = best->pose;
    victim_pose.pose.orientation.w = 1.0;

    setOutput("seen_victim_id", best->id);
    setOutput("seen_victim_pose", victim_pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<SwarmKnowledge> knowledge_;
  int robot_id_{0};
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__DETECT_VICTIM_CONDITION_HPP_
