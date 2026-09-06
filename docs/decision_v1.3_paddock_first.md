# Runner v1.3 — Paddock-first decision amendments

**Status:** Ratified by Matti, 7 September 2026; implementation staged.
**Contract:** [Runner v1.3](runner_spec_v1.3.md), especially sections 7, 16 and 18.
**Baseline:** 6a7c9611a3a910d93b607dcd8523707915f9fcaf, unchanged on reconciliation.

Historical D-numbers are reused across specifications. Version and decision title
are authoritative identifiers; numbers are secondary. This amendment does not
renumber history.

## Paddock-first orthogonal runtime and motion authority

Runtime IDLE/MAPPING/AUTONOMY is separate from STOP/IDLE/MANUAL/AUTONOMOUS.
Paddock owns normal intent; local safety remains downstream. This replaces v1.2
Paddock section 9 transitional operational authority and readiness assumptions.

## Global persistent STOP and local control lifetime

Ratified Q1: persistent local STOP executor uses existing mux lock/zero facilities.
STOP dominates all sources, survives failures, and clears only deliberately with
acknowledgement. Failed/unknown enforcement inhibits; known-clear operation does
not depend on web/authority. Concrete failure evidence must precede any mux change.
One persistent joy/teleop/mux tier replaces application constructors, amending
v1.2 Runnable launch completeness (D-29); persistent actuator ownership is retained.

## Supervised RUN, mission generations and cancellation

Ratified Q2: release immediately revokes motion and asynchronously cancels Nav2.
Retain logical continuation only; future deliberate RUN follows terminal/quiescent
state. One real mission runtime replaces competing keyboard/Foxglove authority.
This supersedes v1.0 Keyboard autonomy is a 600 s Pi-side latch (D-71), not v1.2
Persistent motor hardware ownership (also D-71).

## Single-writer conversion and shared remote speed controller

Nav2 -> /cmd_vel_nav -> drive_adapter -> /cmd_vel_auto_raw -> authority ->
/cmd_vel_auto -> existing mux -> /cmd_vel -> motor.
Manual bounded speed/steering uses the shared converter, then authority-owned
/cmd_vel_paddock. Local teleop alone writes /cmd_vel_teleop. STOP executor alone
writes stop zero/lock inputs. The mux alone writes /cmd_vel; motor owns actuators.
This amends v1.2 Drive adapter (D-55), retaining frozen longitudinal control (D-84).

## Supported operational configuration and complete mapping sessions

Ratified Q3: manual and mapping ceiling 0.30 m/s; autonomous desired 0.45 m/s,
maximum 0.60 m/s; moving floor 0.25 m/s. No tuning changes. Overrides are scoped
to session/runtime. Typed readback replaces arbitrary parameter mutation.
NEW MAP creates a fresh session; SAVE exposes only complete validated bundles.
Section 16 of the contract lists all retained/amended historical subjects.

## Reconciliation and validation level

Source baseline matches the draft. Current graph differs from the earlier live
snapshot: no application mux is running; the unfinished authority alone publishes
/cmd_vel_auto, so the collision is latent until application startup. Traction PWM
read zero. Documentation checked against current paths; no backend completion or
physical driving validation is claimed.
