# swarm-rescue-bt

Decentralized multi-robot search & rescue on top of Nav2: a mission-level
behavior tree per robot (BehaviorTree.CPP v3, same taxonomy as Nav2's own
`nav2_behavior_tree`) that reuses Nav2's shipped `NavigateToPose` BT node
for navigation, while coordinating the swarm peer-to-peer over ROS2 topics
- no central arbiter anywhere in the stack.

The package lives in [`ros2_ws/src/swarm_sar_bt/`](ros2_ws/src/swarm_sar_bt/)
— see that README for the architecture, build/run instructions, and known
simplifications (what's verified vs. not, what a production version would
still need to change).

---

*[Version française](README.fr.md)*
