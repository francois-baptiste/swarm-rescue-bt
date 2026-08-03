#include "swarm_sar_bt/claim_victim_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<swarm_sar_bt::ClaimVictimAction>("ClaimVictim");
}
