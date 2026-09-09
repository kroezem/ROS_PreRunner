# Authoritative command-source and autonomy-interlock snapshot.
uint8 AUTHORITY_NONE=0
uint8 AUTHORITY_DUALSENSE=1
uint8 AUTHORITY_PADDOCK_MANUAL=2
uint8 AUTHORITY_PADDOCK_AUTONOMY=3

builtin_interfaces/Time stamp
uint8 authority
string client_id
string lease_id
bool dualsense_active
bool run_held
bool autonomy_permitted
bool autonomy_goal_selected
# Authoritative selected goal retained by command_authority. Meaningful only
# when autonomy_goal_selected is true; always map frame in v1.3.
string goal_frame
string goal_map
float64 goal_x
float64 goal_y
float64 goal_yaw
bool autonomy_action_active
bool brake_intent
bool lease_fresh
float32 lease_age_sec
bool raw_autonomy_fresh
float32 raw_autonomy_age_sec
bool manual_input_fresh
float32 manual_input_age_sec
float64 manual_applied_speed_mps
float64 manual_applied_steering
uint64 last_control_sequence
string reason
bool stop_state_fresh
bool stop_healthy
bool stop_applied
bool stop_clear
string stop_boot_id
uint64 stop_generation
string stop_reason
