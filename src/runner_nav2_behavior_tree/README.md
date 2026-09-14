# Runner Nav2 commitment nodes

Stage C treats every Smac result as a candidate. The behavior trees validate
that candidate once against the planner server's current live global costmap
before copying it to the committed `path` blackboard entry. A freshness failure
cannot replace the previously committed path.

While following a committed path, `PersistentPathValid` checks only the next
1.25 m from tracked progress. It asks the existing footprint-aware
`is_path_valid` service for LETHAL-only validity and requests a replan only
after three consecutive invalid observations. Goal changes and the existing
bounded Stage-B controller recovery remain independent replan authorities.

`corridor_length`, `required_observations`, and
`max_progress_search_distance` are behavior-tree ports, not frozen policy
constants. Commitment events are published on
`/navigation/path_commitment_state` for the `navigation_debug` profile.

After candidate validation and commitment, `GeneratePathSpeedProfile` reads
the current controller preset and the already-published full global costmap.
It publishes one keyed, transient-local `/navigation/path_speed_profile`;
retained paths do not run the generator again. Profile generation is strictly
annotative: unavailable inputs select a conservative creep profile and never
reject or alter the committed route.
