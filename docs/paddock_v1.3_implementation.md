# Runner v1.3 implementation status

## Stage 0 — contract installed

Ratified specification and amendments reconciled with 6a7c961. Documentation only.

## Stage 1 — private scaffold isolation

Unfinished authority outputs use /paddock/private/cmd_vel_auto and
/paddock/private/cmd_vel_paddock. Neither is a production mux input.
This default also applies to direct executable starts, not just systemd remaps.
Adapter remains the transitional sole /cmd_vel_auto writer when launched.
Legacy keyboard/Foxglove paths remain transitional until the gated cutover.
This is not v1.3 autonomous authority or global STOP implementation.

Validation: runner_paddock colcon build passed; existing authority plumbing tests
4/4 passed. Restarted installed authority, started the existing managed AUTONOMY
unit with studio, then stopped it. Live publisher counts:
- /cmd_vel_auto: 1, drive_adapter
- /cmd_vel_teleop: 1, runner_teleop
- /cmd_vel: 1, twist_mux
- /paddock/private/cmd_vel_auto: 1, runner_command_authority; 0 subscribers

Observed final Twist was all zero; traction duty_cycle 0 ns (steering 1500000 ns).
No mission dispatched or traction connected. Application stopped after smoke check.
