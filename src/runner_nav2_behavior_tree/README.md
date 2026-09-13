# Runner Nav2 commitment nodes

Stage C treats every Smac result as a candidate. The behavior trees validate
that candidate once against the planner server's current live global costmap
before copying it to the committed `path` blackboard entry.

While following a committed path, `PersistentPathValid` checks only the next
1.25 m from tracked progress. It asks the existing footprint-aware
`is_path_valid` service for LETHAL-only validity and requests a replan only
after three consecutive invalid observations. Goal changes and the existing
bounded Stage-B controller recovery remain independent replan authorities.

`corridor_length`, `required_observations`, and
`max_progress_search_distance` are behavior-tree ports, not frozen policy
constants. Commitment events are published on
`/navigation/path_commitment_state` for the `navigation_debug` profile.
