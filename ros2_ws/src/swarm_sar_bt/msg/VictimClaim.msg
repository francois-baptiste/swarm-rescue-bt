# Broadcast on /swarm/claims when a robot's ClaimVictim BT node fires.
# There is no subscriber that arbitrates this message: every robot's
# IsClaimStaleCondition / DetectVictimCondition reads the same topic and
# independently decides whether to respect or override the claim.
std_msgs/Header header
int32 victim_id
int32 robot_id
geometry_msgs/PoseStamped victim_pose
