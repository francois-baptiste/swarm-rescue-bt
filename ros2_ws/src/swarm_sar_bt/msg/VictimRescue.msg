# Broadcast on /swarm/rescues once RescueVictimAction completes. Lets every
# robot that had this victim in its known_claims drop it, freeing up its
# behavior tree to fall back to Explore.
std_msgs/Header header
int32 victim_id
int32 robot_id
