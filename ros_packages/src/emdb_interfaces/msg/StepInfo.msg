# emdb_interfaces/msg/StepInfo.msg
# Reward/done signal published in lockstep with Observation (same episode_id/step_id).
std_msgs/Header header
uint64 episode_id
uint64 step_id
float64 reward
bool terminated
bool truncated
bool success
