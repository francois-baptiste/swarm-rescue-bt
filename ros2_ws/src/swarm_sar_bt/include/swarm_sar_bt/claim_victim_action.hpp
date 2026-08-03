#ifndef SWARM_SAR_BT__CLAIM_VICTIM_ACTION_HPP_
#define SWARM_SAR_BT__CLAIM_VICTIM_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/swarm_knowledge.hpp"

namespace swarm_sar_bt
{

// Claims the victim DetectVictimCondition just found: broadcasts a
// VictimClaim on /swarm/claims (heard by every other robot, arbitrated by
// nobody) and writes "goal" on the blackboard so the Sequence's next leaf
// - the real Nav2 NavigateToPose node - can drive there. A SyncActionNode:
// this always completes within a single tick, it never blocks.
class ClaimVictimAction : public BT::SyncActionNode
{
public:
  ClaimVictimAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    knowledge_ = config.blackboard->get<std::shared_ptr<SwarmKnowledge>>("swarm_knowledge");
    config.blackboard->get("robot_id", robot_id_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("seen_victim_id"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("seen_victim_pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    int victim_id;
    geometry_msgs::msg::PoseStamped victim_pose;
    if (!getInput("seen_victim_id", victim_id) || !getInput("seen_victim_pose", victim_pose)) {
      return BT::NodeStatus::FAILURE;
    }

    knowledge_->publishClaim(robot_id_, victim_id, victim_pose);

    config().blackboard->set("claimed_victim_id", victim_id);
    config().blackboard->set("claimed_victim_pose", victim_pose);
    config().blackboard->set("goal", victim_pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<SwarmKnowledge> knowledge_;
  int robot_id_{0};
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__CLAIM_VICTIM_ACTION_HPP_
