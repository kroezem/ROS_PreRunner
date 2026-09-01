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
bool autonomy_action_active
string reason
