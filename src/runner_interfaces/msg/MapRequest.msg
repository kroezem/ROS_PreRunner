# Authorized map-session operation from the current Paddock control lease holder.
uint8 OP_NEW_MAP=0
uint8 OP_SAVE_MAP=1
uint8 OP_SELECT_MAP=2
uint8 OP_DELETE_MAP=3

builtin_interfaces/Time stamp
uint64 request_id
string lease_id
uint8 operation
# Bounded safe basename for OP_SAVE_MAP, OP_SELECT_MAP, and OP_DELETE_MAP;
# ignored for OP_NEW_MAP.
string name
# Mapping session the operation must correspond to. Required for OP_SAVE_MAP so a
# save cannot silently capture a newer or stale session; ignored otherwise.
string session_id
