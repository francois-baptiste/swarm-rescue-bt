// One instance of this executable runs per robot, inside that robot's own
// namespace. It is deliberately built the same way nav2_bt_navigator loads
// and ticks its navigation tree - it reuses nav2_behavior_tree's own
// BehaviorTreeEngine (registerFromPlugin over plugin_lib_names, then
// tickRoot() at a fixed rate) - except the tree here is a *mission* tree
// (search & rescue decision-making) that itself calls this robot's own
// Nav2 stack through the shipped NavigateToPose BT node, instead of being
// Nav2's own low-level navigate-with-replanning-and-recovery tree.
//
// Requires this robot's own `bt_navigator` (and therefore its
// `navigate_to_pose` action server) to already be up when this node
// starts, because BT.CPP v3 constructs every node in the XML - including
// the NavigateToPose leaves - as soon as the tree is built, and
// NavigateToPoseAction's constructor waits for its action server.
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/swarm_knowledge.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "mission_bt_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const int robot_id = node->declare_parameter("robot_id", 0);
  const std::string global_frame =
    node->declare_parameter("global_frame", std::string("map"));
  const std::string robot_base_frame =
    node->declare_parameter("robot_base_frame", std::string("base_link"));
  const std::string bt_xml_filename =
    node->declare_parameter("bt_xml_filename", std::string(""));
  const std::vector<std::string> plugin_lib_names =
    node->declare_parameter("plugin_lib_names", std::vector<std::string>{});
  const int bt_loop_duration_ms = node->declare_parameter("bt_loop_duration_ms", 200);
  const int server_timeout_ms = node->declare_parameter("server_timeout_ms", 20000);
  const int wait_for_service_timeout_ms =
    node->declare_parameter("wait_for_service_timeout_ms", 10000);

  if (bt_xml_filename.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Required parameter 'bt_xml_filename' was not set");
    rclcpp::shutdown();
    return 1;
  }

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  auto knowledge = std::make_shared<swarm_sar_bt::SwarmKnowledge>(node);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("tf_buffer", tf_buffer);
  blackboard->set("swarm_knowledge", knowledge);
  blackboard->set("robot_id", robot_id);
  blackboard->set("global_frame", global_frame);
  blackboard->set("robot_base_frame", robot_base_frame);
  blackboard->set("claimed_victim_id", -1);
  blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(bt_loop_duration_ms));
  blackboard->set<std::chrono::milliseconds>(
    "server_timeout", std::chrono::milliseconds(server_timeout_ms));
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_timeout_ms));

  nav2_behavior_tree::BehaviorTreeEngine engine(plugin_lib_names);

  BT::Tree tree;
  try {
    tree = engine.createTreeFromFile(bt_xml_filename, blackboard);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      node->get_logger(), "Failed to build tree from '%s': %s",
      bt_xml_filename.c_str(), ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(
    node->get_logger(), "mission_bt_node robot_id=%d ticking '%s' at %d ms",
    robot_id, bt_xml_filename.c_str(), bt_loop_duration_ms);

  engine.run(
    &tree,
    [&executor]() {executor.spin_some();},
    []() {return !rclcpp::ok();},
    std::chrono::milliseconds(bt_loop_duration_ms));

  rclcpp::shutdown();
  return 0;
}
