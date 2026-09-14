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

After candidate validation and before commitment, `GeneratePathSpeedProfile` reads
the current controller preset and the already-published full global costmap.
It hands the exact path/profile pair to the controller through an acknowledged
service before `FollowPath` may dispatch the path, then publishes the keyed,
transient-local `/navigation/path_speed_profile` for observation. Retained paths
do not run the generator again. Profile generation is strictly annotative:
unavailable shaping inputs select a conservative creep profile; an unavailable
or rejected controller handoff prevents dispatch of an unpaired fresh path.

The speed law itself (clearance tiers, hysteresis, braking, and recovery —
see `runner_path_speed_profile::ProfileConfig`) is declared as
`speed_policy.*` parameters directly on the shared `bt_navigator` node rather
than as BT ports, so Paddock (or `ros2 param set`) can change them live; the
next fresh commit picks up whatever is current at tick time. They are not
BT-XML-overridable.
