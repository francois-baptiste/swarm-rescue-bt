#include "swarm_sar_bt/pick_explore_goal_action.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<swarm_sar_bt::PickExploreGoalAction>("PickExploreGoal");
}
