# emdb_interfaces/msg/Observation.msg
# Flattened robosuite/RoboCasa obs_dict, published each time the sim steps.
std_msgs/Header header
uint64 episode_id
uint64 step_id
ObservationEntry[] entries
