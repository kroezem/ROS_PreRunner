# Runner Nav2 commitment nodes

Stage D1 asks the existing planner server for a Dubins (`ForwardGridBased`)
candidate first. Only failure to produce a certified forward candidate opens
the Reeds-Shepp (`ReverseGridBased`) fallback. Both plugins retain the vendored
Smac per-request immutable costmap snapshot; there is no second planner server
or costmap owner.

Every result passes `CertifyCandidatePath` before Stage C. Certification checks
finite geometry, curvature over a noise-resistant 0.15 m window, robust
forward/reverse segments, and a footprint-aware LETHAL sweep through the
planner server's global-costmap validity service. The curvature ceiling is
`2.1236 * 0.95 = 2.01742 m^-1`: Runner's physical limit with a configurable
five-percent certification margin. Edges below 0.01 m are ignored and an edge
must project by at least 0.5 onto the vehicle heading to classify direction.

The configurable 0.10 m minimum cusp-adjacent segment is derived from two
physical 0.20 s reversal intervals at Runner's 0.25 m/s minimum sustainable
motion: one interval represents the stop/stationarity/latch cost and one is the
minimum useful post-latch motion assumption (`2 * 0.20 s * 0.25 m/s`). This is
a conservative D1 geometric contract, not a claim that the latch itself moves
the vehicle 0.05 m.

Stage C then repeats current-live-costmap validation as a distinct freshness
boundary immediately before copying the candidate to the committed `path`
blackboard entry. A certification or freshness failure cannot replace the
previously committed path.

While following a committed path, `PersistentPathValid` checks only the next
1.25 m from tracked progress. It asks the existing footprint-aware
`is_path_valid` service for LETHAL-only validity and requests a replan only
after three consecutive invalid observations. Goal changes and the existing
bounded Stage-B controller recovery remain independent replan authorities.

`corridor_length`, `required_observations`, and
`max_progress_search_distance` are behavior-tree ports, not frozen policy
constants. Commitment events are published on
`/navigation/path_commitment_state` for the `navigation_debug` profile.

The geometry result is a separate data structure so Stage D2 can extend a
certified route with an execution profile. D1 does not calculate or enforce
speed ceilings.
