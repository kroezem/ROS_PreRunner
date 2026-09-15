# Read-only execution progress for one accepted committed path.
uint8 EXECUTION_TRACKING=0
uint8 EXECUTION_WAITING_FOR_STATIONARY=1
uint8 EXECUTION_COMPLETE=2

std_msgs/Header header
builtin_interfaces/Time committed_path_stamp
uint64 path_hash
uint32 pose_count
uint32 current_segment
uint32 global_path_index
float64 global_path_offset_m
uint8 execution_state
