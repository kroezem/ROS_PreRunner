# Controller-agnostic speed annotation for one committed nav_msgs/Path.
std_msgs/Header header
builtin_interfaces/Time committed_path_stamp
uint64 path_hash
uint32 pose_count
float64 goal_arclength_m
float64 preset_ceiling_mps
float64 creep_speed_mps
runner_interfaces/PathSpeedProfilePoint[] points
