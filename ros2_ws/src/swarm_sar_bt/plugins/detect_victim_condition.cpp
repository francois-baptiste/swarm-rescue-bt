#include "swarm_sar_bt/detect_victim_condition.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<swarm_sar_bt::DetectVictimCondition>("DetectVictim");
}
