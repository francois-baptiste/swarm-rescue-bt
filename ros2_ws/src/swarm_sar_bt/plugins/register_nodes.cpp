// A shared library can only export one BT_RegisterNodesFromPlugin symbol
// (dlsym looks up exactly one entry point per .so - see bt_factory.h), so
// every node this plugin library provides is registered here in a single
// BT_REGISTER_NODES block, rather than one per node .cpp file.
#include "behaviortree_cpp_v3/bt_factory.h"

#include "swarm_sar_bt/claim_victim_action.hpp"
#include "swarm_sar_bt/detect_victim_condition.hpp"
#include "swarm_sar_bt/has_claim_condition.hpp"
#include "swarm_sar_bt/is_at_victim_condition.hpp"
#include "swarm_sar_bt/pick_explore_goal_action.hpp"
#include "swarm_sar_bt/rescue_victim_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<swarm_sar_bt::HasClaimCondition>("HasClaim");
  factory.registerNodeType<swarm_sar_bt::IsAtVictimCondition>("IsAtVictim");
  factory.registerNodeType<swarm_sar_bt::DetectVictimCondition>("DetectVictim");
  factory.registerNodeType<swarm_sar_bt::ClaimVictimAction>("ClaimVictim");
  factory.registerNodeType<swarm_sar_bt::PickExploreGoalAction>("PickExploreGoal");
  factory.registerNodeType<swarm_sar_bt::RescueVictimAction>("RescueVictim");
}
