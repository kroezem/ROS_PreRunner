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

# Monotonic generation of the current managed application. Increments on every
# successful application start (MAPPING or AUTONOMY) and on NEW MAP. Lets
# consumers reject status/results that arrive after a runtime change.
uint64 runtime_epoch
# Identity of the current mapping SLAM session. Non-empty only while MAPPING is
# the actual runtime; a fresh value each MAPPING start and each NEW MAP.
string mapping_session_id
# Continuously refreshed capability readiness for the actual runtime. Never true
# while status is TRANSITIONING or FAULT.
bool ready
# First actionable inhibit reason when not ready; empty when ready.
string readiness_reason
