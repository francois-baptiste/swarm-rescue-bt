#ifndef SWARM_SAR_BT__HAS_CLAIM_CONDITION_HPP_
#define SWARM_SAR_BT__HAS_CLAIM_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace swarm_sar_bt
{

// SUCCESS if this robot currently owns a claim on a victim
// ("claimed_victim_id" on the blackboard, set by ClaimVictimAction / -1
// initially). Purely a local blackboard read, no ROS interfaces needed -
// mirrors small single-purpose conditions like nav2's GoalUpdatedCondition.
class HasClaimCondition : public BT::ConditionNode
{
public:
  HasClaimCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    int claimed_victim_id = -1;
    config().blackboard->get("claimed_victim_id", claimed_victim_id);
    return claimed_victim_id >= 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__HAS_CLAIM_CONDITION_HPP_
