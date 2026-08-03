# Ground-truth description of a single victim to be found by the swarm.
int32 id
geometry_msgs/Point pose
bool rescued
int32 rescued_by   # robot_id, -1 if not rescued
