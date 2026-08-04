# swarm_sar_bt

Decentralized search-and-rescue for a ground robot swarm, built as an
actual Nav2 package — BehaviorTree.CPP v3 plugin nodes loaded the way
`bt_navigator` loads its own, a mission tree written in Nav2's own BT XML
dialect that reuses Nav2's **shipped** `NavigateToPose` BT node, and a
launch file that reuses `nav2_bringup`'s own multi-robot bringup.

**Build status**: `colcon build` succeeds, verified end to end. Built
inside a container `FROM ghcr.io/ros-navigation/navigation2:humble` (the
official Nav2 CI image) plus the real `navigation2` source (`humble`
branch, cloned fresh from GitHub — the image's own bundled copy turned out
to be package.xml stubs with no source). `nav2_behavior_tree` and the rest
of `navigation2` (38 packages) built clean first, confirming the API
assumptions were correct. See `../../Dockerfile` for exactly how.

**Runtime status**: also verified end to end, without Gazebo -
`test/fake_navigate_to_pose_server.py` stubs out `navigate_to_pose` (real
NavigateToPose BT node, fake server) so `mission_bt_node` - real,
unmodified package code - can be exercised over real ROS2/DDS topics
between 3 separate node graphs; see `test/run_runtime_smoke_test.sh`.
Confirmed: BT ticking, sensor-range detection, claim broadcast, real
Nav2-node-driven navigation, rescue, and reclaim after a scripted robot
failure all work. This surfaced (and fixed) six real bugs the build alone
never would have caught:

1. `NodeOptions().automatically_declare_parameters_from_overrides(true)`
   collided with this file's own `declare_parameter()` calls
   (`ParameterAlreadyDeclaredException`) - removed.
2. `BT_REGISTER_NODES` expands to a `static` (non-exported, dead-stripped)
   function unless `BT_PLUGIN_EXPORT` is defined at compile time - added
   `target_compile_definitions(swarm_sar_bt_nodes PRIVATE BT_PLUGIN_EXPORT)`.
3. A shared library can only export one `BT_RegisterNodesFromPlugin`
   symbol; six separate plugin `.cpp` files each defining one doesn't
   compile once export actually works (`multiple definition of...`) -
   consolidated into `plugins/register_nodes.cpp`.
4. Plain `Fallback` resumes ticking at whichever child last returned
   RUNNING, so once a robot committed to `Explore` it would never
   re-check `DetectAndClaim` again until that leg finished - could walk
   right past a claimable victim. Root is now `ReactiveFallback` (same
   reason Nav2's own recovery tree uses it).
5. `nav2_behavior_tree::BehaviorTreeEngine::run()`'s `while(RUNNING)` loop
   is correct for a one-shot tree like `navigate_to_pose` but stops as
   soon as this *persistent* mission tree's root first returns SUCCESS -
   each robot did exactly one Explore leg and then the process went idle.
   Replaced with a plain `while(rclcpp::ok()) { tree.tickRoot(); ... }`.
6. `rclcpp::WallRate rate(std::chrono::milliseconds(x));` is a C++ "most
   vexing parse" (parsed as a function declaration) - fixed with `{}`.

**Known remaining limitation**: two robots that detect/claim the same
victim within the same DDS round-trip window (roughly 200ms-1s, before
either hears the other's `/swarm/claims` broadcast) can both end up
rescuing it, publishing more than one `/swarm/rescues` message for the
same `victim_id`. `SwarmKnowledge::isKnownRescued()` closes the *slow*
version of this race (stale `/victims_ground_truth`, republished only at
its own timer rate) but not this *fast* one. This is the same "eventual
consistency, first-arrival-wins" tradeoff the plain-Python prototype
documented from the start - a strict fix would need a per-victim
claim-acknowledgement handshake, which is a real design change, not a
bug fix, and deliberately out of scope for a "no arbiter" swarm.

## Target

ROS2 **Humble** (`behaviortree_cpp_v3`, plain `rclcpp::Node::SharedPtr` on
the BT blackboard's `"node"` key). Nav2's `master` branch has since moved
to BT.CPP v4 (`behaviortree_cpp`, blackboard `"node"` is a
`nav2::LifecycleNode::SharedPtr`) — if you're on a newer distro (Iron and
up), the plugin headers/registration macro stay the same shape but the
include paths and that one blackboard type need updating.

## Layout

```
package.xml, CMakeLists.txt      ament_cmake package; also generates its
                                  own msgs (rosidl_generate_interfaces +
                                  rosidl_get_typesupport_target, so the
                                  plugin lib and nodes can consume them
                                  from the same package)
msg/                              Victim, VictimArray, VictimClaim, VictimRescue
include/swarm_sar_bt/             BT node classes + swarm_knowledge.hpp
plugins/                          BT_REGISTER_NODES registration units,
                                  built into libswarm_sar_bt_nodes.so
src/mission_bt_node.cpp           per-robot executor (reuses nav2_behavior_tree
                                  ::BehaviorTreeEngine, same class bt_navigator
                                  itself uses to load/tick a tree)
src/victim_ground_truth_node.cpp  publishes the scenario's victims
bt_xml/sar_mission_tree.xml       the mission tree
config/                           mission_bt_node + scenario params
launch/swarm_sar_bringup.launch.py
```

## Architecture

One `mission_bt_node` runs per robot, in that robot's own namespace,
alongside that robot's own full Nav2 stack (brought up by
`nav2_bringup`'s `cloned_multi_tb3_simulation_launch.py`, included
as-is). Its tree:

```
ReactiveFallback (MissionRoot)
├── Sequence RescueIfArrived: HasClaim → IsAtVictim → RescueVictim
├── Sequence PursueClaim:     HasClaim → NavigateToPose            (Nav2's own node)
├── Sequence DetectAndClaim:  DetectVictim → ClaimVictim
└── Sequence Explore:         PickExploreGoal → NavigateToPose     (Nav2's own node)
```

Root is `ReactiveFallback`, not plain `Fallback`: see bug #4 below - a
plain `Fallback` would let a robot get "stuck" mid-Explore, unable to
notice a newly-detectable victim until that leg finished.

`<NavigateToPose goal="{goal}"/>` is loaded from
`nav2_navigate_to_pose_action_bt_node` — the exact shared library
`nav2_behavior_tree` ships — so those two leaves are real Nav2 code acting
as an action *client* to this robot's own `bt_navigator`. Everything else
is this package's `swarm_sar_bt_nodes` plugin library.

Coordination is entirely peer-to-peer: `HasClaim`/`IsAtVictim` only read
this robot's own blackboard; `DetectVictim`/`ClaimVictim`/`RescueVictim`
go through `SwarmKnowledge` (`include/swarm_sar_bt/swarm_knowledge.hpp`),
which owns the `/swarm/claims` and `/swarm/rescues` publishers/subscribers
and nothing else — it records what has been heard and lets each BT decide
for itself, with no arbiter, over real DDS topics. A claim with no
matching rescue within `claim_timeout` seconds is treated as abandoned by
every *other* robot independently — that's the fault-tolerance mechanic
that kicks in if a robot fails mid-mission.

## Building (once ROS2 Humble + Nav2 + `turtlebot3_gazebo` are installed)

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
ros2 launch swarm_sar_bt swarm_sar_bringup.launch.py
```

Brings up Gazebo with 3 robots (namespaced `robot0`/`robot1`/`robot2`,
poses from the `robots` launch argument), each robot's own Nav2 stack,
`victim_ground_truth_node`, and — after `bringup_delay` seconds (default
15s, tune it if your machine brings Nav2 up slower or faster) — one
`mission_bt_node` per robot.

```bash
ros2 topic echo /swarm/claims
ros2 topic echo /swarm/rescues
```

## Running the runtime smoke test (no Gazebo needed)

```bash
docker build -t swarm_sar_bt_dev -f Dockerfile .
docker run --rm \
  -v "$(pwd)/src/swarm_sar_bt/test:/opt/overlay_ws/src/swarm_sar_bt/test:ro" \
  swarm_sar_bt_dev bash -c \
  "source /opt/ros/humble/setup.bash && source /opt/overlay_ws/install/setup.bash && \
   bash /opt/overlay_ws/src/swarm_sar_bt/test/run_runtime_smoke_test.sh"
```

Bind-mounting `test/` over the image's baked-in copy means editing the
test scripts doesn't require a rebuild. See `test/run_runtime_smoke_test.sh`
for what it wires up and `test/fake_navigate_to_pose_server.py` for the
stub `navigate_to_pose` action server.

## Known simplifications / what a production version would change

- **Robot model & world**: `cloned_multi_tb3_simulation_launch.py` spawns
  TurtleBot3 Waffle by default, not a purpose-built ground rover — swap
  the `world`/robot model the same way you would for any other Nav2 TB3
  project. Nothing in `swarm_sar_bt` itself is TB3-specific.
- **Victim perception**: `victim_ground_truth_node` publishes hand-set
  ground-truth positions instead of a real detector or Gazebo actors +
  sensor plugin. `DetectVictimCondition` only ever reads this topic
  through the same sensor-range check a real detector's output would go
  through, so swapping the source is a matter of replacing this one node.
- **Exploration**: `PickExploreGoalAction` samples a random point in a
  radius rather than doing real frontier exploration off a costmap. A
  bad sample just fails that BT tick and gets resampled next tick — it
  never wedges the tree — but a production swarm would want
  `nav2_costmap_2d` free-space sampling or an `explore_lite`-style
  frontier detector here instead.
- **mission_bt_node startup ordering**: BT.CPP v3 constructs every node in
  the XML as soon as the tree is built — including both `NavigateToPose`
  leaves — and `NavigateToPoseAction`'s constructor blocks waiting for
  that robot's `navigate_to_pose` action server. The launch file papers
  over this with a flat `bringup_delay` (`TimerAction`) instead of a
  lifecycle-state event handler; fine for a demo, not for production.
