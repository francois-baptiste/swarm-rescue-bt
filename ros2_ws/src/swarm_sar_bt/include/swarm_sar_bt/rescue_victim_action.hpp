#ifndef SWARM_SAR_BT__RESCUE_VICTIM_ACTION_HPP_
#define SWARM_SAR_BT__RESCUE_VICTIM_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "swarm_sar_bt/swarm_knowledge.hpp"

namespace swarm_sar_bt
{

// Only reached once HasClaim and IsAtVictim have both already succeeded in
// the same Sequence. Broadcasts the rescue (every robot's SwarmKnowledge
// drops the claim on hearing it, freeing its BT to fall back to Explore)
// and clears this robot's own claim so it starts exploring again too.
class RescueVictimAction : public BT::SyncActionNode
{
public:
  RescueVictimAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    knowledge_ = config.blackboard->get<std::shared_ptr<SwarmKnowledge>>("swarm_knowledge");
    config.blackboard->get("robot_id", robot_id_);
  }

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    int victim_id = -1;
    if (!config().blackboard->get("claimed_victim_id", victim_id) || victim_id < 0) {
      return BT::NodeStatus::FAILURE;
    }

    knowledge_->publishRescue(robot_id_, victim_id);

    config().blackboard->set("claimed_victim_id", -1);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<SwarmKnowledge> knowledge_;
  int robot_id_{0};
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__RESCUE_VICTIM_ACTION_HPP_
