# D-37 — Fixed-cardinality SLAM scan stream

**Status:** Ratified and implemented for Phase B
**Default geometry:** 503 endpoint-inclusive bins over `0..2*pi`

## Decision

The LD19 raw `/scan` stream is preserved exactly and is branched into two
purpose-specific scan-processing paths:

```text
LD19 /scan
  +-> rf2o_scan_canonicalizer -> /scan_rf2o -> RF2O
  |
  +-> scan_rebinner -> /scan_slam -> slam_toolbox
```

Both slam_toolbox mapping and localization consume `/scan_slam`. New maps must
not be created unless the rebinner is active and the fixed stream has first
been verified.

## Evidence and rationale

The LD19 publishes a fixed angular extent from `0` through `2*pi`, including
both endpoints, but its range count varies in practice (approximately
495–509). Its increment is consequently `2*pi / (N - 1)`. Karto registers the
first laser geometry it sees and hard-rejects later scans whose range count
differs. It neither truncates nor pads rejected scans, and a rejected scan can
consume and lose a localization seed.

The existing `house_good_v1` posegraph expects 503 readings, so 503 is the
default. Fresh mapping has also registered different counts on different
runs, demonstrating that this cannot be fixed only for localization: mapping
and localization must share the same deterministic geometry.

Raw `/scan` remains unchanged so the driver output stays observable and has
one publisher: the LD19. RF2O remains on `/scan_rf2o` because its established
angular-origin canonicalization is independent of Karto cardinality and is not
being generalized or retuned by this decision.

The rebinner maps each source beam to the nearest target angle. If multiple
beams collide, it keeps the beam whose actual source angle is closest to the
target centre, with the lower source index breaking exact ties. This preserves
angular-sampling semantics. Range interpolation is rejected because it invents
measurements; array truncation is rejected because it changes angular coverage
and makes the result depend on input cardinality. Empty target bins are explicit
no-returns (`inf` range and zero intensity).

The operational gate is strict: verify `/scan_slam` has constant 503-element
range and intensity arrays and constant endpoint-inclusive angle metadata
before creating any new map.
