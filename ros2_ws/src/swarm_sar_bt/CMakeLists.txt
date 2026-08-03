cmake_minimum_required(VERSION 3.8)
project(swarm_sar_bt C CXX)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

nav2_package()

include_directories(include)

# ---------------------------------------------------------------------------
# Interfaces (same colocated-interfaces pattern used by many small nav2-
# adjacent packages: generate + link the C++ typesupport in this same
# CMakeLists via rosidl_get_typesupport_target).
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Victim.msg"
  "msg/VictimArray.msg"
  "msg/VictimClaim.msg"
  "msg/VictimRescue.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------------------------
# Custom BT.CPP plugin nodes, one shared library, loaded dynamically at
# runtime by mission_bt_node the same way nav2_behavior_tree loads its own
# plugin_lib_names entries (BT::SharedLibrary, not ROS pluginlib XML).
# ---------------------------------------------------------------------------
add_library(swarm_sar_bt_nodes SHARED
  plugins/has_claim_condition.cpp
  plugins/is_at_victim_condition.cpp
  plugins/detect_victim_condition.cpp
  plugins/claim_victim_action.cpp
  plugins/pick_explore_goal_action.cpp
  plugins/rescue_victim_action.cpp
)
ament_target_dependencies(swarm_sar_bt_nodes ${dependencies})
target_link_libraries(swarm_sar_bt_nodes "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# mission_bt_node: one instance runs per robot namespace. Loads
# sar_mission_tree.xml, registers plugin_lib_names (ours + Nav2's shipped
# nav2_navigate_to_pose_action_bt_node), ticks the tree.
# ---------------------------------------------------------------------------
add_executable(mission_bt_node src/mission_bt_node.cpp)
ament_target_dependencies(mission_bt_node ${dependencies} nav2_behavior_tree)
target_link_libraries(mission_bt_node "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# victim_ground_truth_node: publishes the scenario's victim list. Stands in
# for what Gazebo actors + each robot's own perception stack would provide.
# ---------------------------------------------------------------------------
add_executable(victim_ground_truth_node src/victim_ground_truth_node.cpp)
ament_target_dependencies(victim_ground_truth_node ${dependencies})
target_link_libraries(victim_ground_truth_node "${cpp_typesupport_target}")

install(TARGETS
  swarm_sar_bt_nodes
  DESTINATION lib
)

install(TARGETS
  mission_bt_node
  victim_ground_truth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY bt_xml config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(swarm_sar_bt_nodes)
ament_export_dependencies(${dependencies})

ament_package()
