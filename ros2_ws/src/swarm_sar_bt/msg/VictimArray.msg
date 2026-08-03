# Ground-truth victim list, published by victim_ground_truth_node.
# Stands in for what would normally come from each robot's own perception
# stack (camera/thermal detector) plus Gazebo actor ground truth in sim.
std_msgs/Header header
Victim[] victims
