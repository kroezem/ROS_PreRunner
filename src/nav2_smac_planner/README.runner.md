# Runner overlay contract

This package vendors Navigation2 `nav2_smac_planner` 1.3.12, matching the
installed ROS 2 Jazzy package. Runner changes only Hybrid-A* costmap ownership:

- each planning request copies the live master costmap under its mutex;
- the live mutex is released before search begins;
- collision checking, Hybrid-A* search, optional downsampling, and smoothing
  use only that request-owned immutable snapshot;
- request teardown restores internal pointers before the snapshot is destroyed.

A returned path is valid relative to the planning-start snapshot. Candidate-B
Stage C must separately validate it against current global evidence before
committing it; this overlay does not add that validation or commitment policy.
