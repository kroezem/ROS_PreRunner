# One map bundle discovered under the map root.
string name
# Manifest revision id (short hash of the four core artifacts); empty for a
# legacy bundle saved before manifests existed.
string revision
# True only when the four core artifacts exist, are non-empty, the YAML parses,
# its referenced raster exists, and the raster header is plausible.
bool complete
# True when this bundle is the current AUTONOMY map selection.
bool selected
# Mapping session that produced the bundle; empty when unknown/legacy.
string session_id
# Reason the bundle is not complete; empty when complete.
string reason
# Raster metadata echoed from the YAML/PGM for display; zero when unknown.
float64 resolution
uint32 width
uint32 height
