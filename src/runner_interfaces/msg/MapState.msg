# Map-session executor state: mapping session lifecycle, save operation, catalog.
uint8 SESSION_NONE=0
uint8 SESSION_STARTING=1
uint8 SESSION_READY=2
uint8 SESSION_SAVING=3
uint8 SESSION_SAVED=4
uint8 SESSION_FAILED=5

uint8 SAVE_IDLE=0
uint8 SAVE_RUNNING=1
uint8 SAVE_SUCCEEDED=2
uint8 SAVE_FAILED=3

uint8 DELETE_IDLE=0
uint8 DELETE_SUCCEEDED=1
uint8 DELETE_FAILED=2

uint8 RESET_IDLE=0
uint8 RESET_RUNNING=1
uint8 RESET_SUCCEEDED=2
uint8 RESET_FAILED=3

builtin_interfaces/Time stamp

# Current mapping session identity, mirrored from ModeState. Empty when the
# actual runtime is not MAPPING.
string session_id
# Runtime generation the executor last observed from ModeState.
uint64 runtime_epoch
bool mapping_active
uint8 session_phase
# True once current-session map evidence exists (a /map update after the session
# began) and mapping readiness holds. A retained old-session raster never sets it.
bool session_ready
# True while the current session holds unsaved map content.
bool unsaved
string saved_name
string saved_revision

# Last completed or in-flight SAVE MAP operation.
uint8 save_state
uint64 save_request_id
# Actionable stage/reason for the last save (success or failure).
string save_detail

# Complete and incomplete bundles discovered under the map root.
MapCatalogEntry[] catalog

# Typed selection state for the AUTONOMY map (the Stage 4 operational setting).
string selected_map_requested
string selected_map_applied
string selected_map_reason

# Last completed DELETE MAP operation. Deletion is synchronous in the map
# executor, so there is no in-flight state.
uint8 delete_state
uint64 delete_request_id
string delete_name
string delete_detail

# Last NEW MAP operation. RUNNING spans motion revocation, confirmed stationary
# wait, mapping-stack replacement, and fresh-session readiness.
uint8 reset_state
uint64 reset_request_id
string reset_detail
