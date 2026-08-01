# One origin-to-owner reconciliation result for a longitudinal parameter.
uint8 TYPE_BOOL=1
uint8 TYPE_INTEGER=2
uint8 TYPE_DOUBLE=3

uint8 DIVERGENCE_UNKNOWN=0
uint8 DIVERGENCE_MATCH=1
uint8 DIVERGENCE_DIFFERENT=2

string key
string node_name
string parameter_name
uint8 value_type
bool origin_bool
int64 origin_integer
float64 origin_double
bool observed_bool
int64 observed_integer
float64 observed_double
bool available
uint8 divergence
string detail
