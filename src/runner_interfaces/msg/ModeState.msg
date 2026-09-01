# Authoritative, queryable Paddock mode snapshot.
uint8 MODE_IDLE=0
uint8 MODE_MAPPING=1
uint8 MODE_AUTONOMY=2

# Runtime lifecycle is separate so the UI still has exactly three modes.
uint8 STATUS_STABLE=0
uint8 STATUS_TRANSITIONING=1
uint8 STATUS_FAULT=2

builtin_interfaces/Time stamp
uint8 mode
uint8 status
# Last accepted ModeRequest request_id; zero before any request is accepted.
uint64 accepted_request_id
# Explicit map selected for autonomy. Empty means that none is selected.
string active_autonomy_map
# Human-readable transition or fault evidence; empty while stable.
string detail
