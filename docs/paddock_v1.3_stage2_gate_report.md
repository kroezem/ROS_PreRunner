# Runner v1.3 implementation report — 7 September 2026

## CONFIRMED

Completed Stage 0 and Stage 1. Stage 2 was investigated with a temporary STOP
executor and the installed twist_mux; its restart/ownership acceptance gate did
not pass. No production STOP cutover or subsequent backend/frontend cutover was
performed.

### Specification and decisions

docs/runner_spec_v1.3.md is committed. The draft baseline and the Pi's original
HEAD both matched 6a7c9611a3a910d93b607dcd8523707915f9fcaf. Q1/Q2/Q3 are ratified
in section 18 and the separate version-qualified decision amendment. README
links v1.3 as the current target contract. Historical specifications remain.
README/map documentation now acknowledges the committed studio bundle without
claiming its current physical localization quality is validated.

### Collision isolation

The unfinished authority now defaults to:
- /paddock/private/cmd_vel_auto
- /paddock/private/cmd_vel_paddock

Neither is configured as a production mux input. The installed authority was
restarted and its private autonomy output had one publisher and zero subscribers.

During the existing managed AUTONOMY application smoke check, the exact counts
were:

| Topic | Publishers | Owner |
|---|---:|---|
| /cmd_vel_auto | 1 | drive_adapter |
| /cmd_vel_teleop | 1 | runner_teleop |
| /cmd_vel | 1 | twist_mux |
| /paddock/private/cmd_vel_auto | 1 | runner_command_authority; no subscribers |

This remains the **transitional** graph:
Nav2 -> drive_adapter -> /cmd_vel_auto -> twist_mux -> /cmd_vel -> motor.
Local teleop -> /cmd_vel_teleop -> same mux.
v1.3 raw/supervised authority ownership is not yet implemented.

The live state at the beginning differed from the earlier architecture review:
no application/mux was running. Therefore there was no live two-publisher
collision at that instant; the collision would have returned on application
startup. The source-default isolation removes that accidental collision.

### STOP probe: measured results and failed gate

The temporary probe used the installed mux, with one final /cmd_vel publisher,
a STOP-zero input at priority 255/timeout 0.10 s, a global lock at priority
200/timeout 0.15 s, and local input at priority 100/timeout 0.15 s. The temporary
executor used 20 ms enforcement callbacks and 100 ms outgoing DDS lifespan.
Application units were stopped before this test; no additional concurrent mux
was added. Synthetic local/neutral Joy input exercised the software path with
traction disconnected; this was not a physical DualSense test.

Representative output:

    known-clear local input permitted
    executor death: last nonzero after 0.120551 s
    PWM: traction 0 ns; steering 1500000 ns
    persisted known-clear restored without authority dependency
    STOP request to first final zero 0.003293 s
    STOP persists across SIGKILL; stale generation and boot clear rejected
    /cmd_vel publishers 1
    /cmd_vel_stop publishers 3
    /cmd_vel_teleop publishers 1

An earlier probe observed last nonzero 0.142026 s after executor death.
**Worst observed stale nonzero duration was therefore 142.026 ms**, limited to
this executor-death software-path test. PWM was sampled zero after the 0.8 s
post-failure observation window; exact PWM-zero latency was not measured.
The 3.293 ms result is one observed STOP-request-to-final-Twist-zero sample,
not a guaranteed bound, browser latency, or physical stopping distance.

Failures:
1. At 3 s initial discovery, the client had not received the expected state;
   a later 5 s observation succeeded.
2. With a short post-restart observation, a STOP service request timed out
   after 3 s. A later probe waiting 5 s after restart completed that request.
3. After rapid SIGKILL/restarts, ROS discovery reported **three STOP publisher
   endpoints**, not the required one.
4. The probe printed PASS for its behavioral assertions but merely printed,
   rather than asserted, endpoint counts. That printed PASS is explicitly
   retracted as an overall acceptance result.

Those results do not pass Stage 2. They do not establish that stock twist_mux
cannot support the approved architecture. No mux implementation change is
justified by this evidence alone.

The unaccepted prototype was removed from production source and its executable
entry point. Local copies of the prototype and probe remain alongside this
report. No persistent STOP service or mux configuration was installed.

## HYPOTHESES / UNVERIFIED

- Stale DDS discovery entries from hard-killed processes may explain the three
  reported STOP endpoints. This was not proven during the failing snapshot.
- Service rediscovery may explain the post-restart request timeout. A longer
  wait passing does not prove a safe production retry/acknowledgement contract.
- No physical traction, stopping distance, steering response, or reverse driving
  validation was performed.
- Browser freshness, mission generations, delayed Nav2 Twist rejection,
  transactional map saving, configuration readback and runtime readiness were
  not validated by these STOP tests.
- The temporary prototype did not complete the v1.3 authority lease/session,
  browser protocol or full STOP-clear/rearm integration.

## OPEN QUESTIONS

No new product-policy choice is established. Q1/Q2/Q3 remain ratified.
The unresolved engineering gate is truthful restart health/acknowledgement and
exact endpoint handling before allowing STOP clear or a production cutover.
Do not substitute a longer sleep for the required lifecycle contract.

## COMMITS

- afbabe5 — ratified v1.3 docs and qualified decision amendments;
  source-reconciled documentation only; pushed to origin/main.
- 47863b7 — private default outputs for unfinished authority;
  package build, four existing tests and live endpoint smoke check;
  pushed to origin/main.
- A subsequent documentation-only commit records this failed Stage 2 gate.

Destination explicitly approved by Matti:
git@github.com:kroezem/ROS_PreRunner.git, main.

## DIFF SUMMARY

- Documentation: current contract, ratified amendments, corrected map claims,
  discoverability and implementation evidence.
- Authority: two default output topics moved to a private namespace.
- Existing plumbing test: expected private topic names updated.
- No motor, encoder, PWM, serial, TF, localization or controller tuning changes.
- No production STOP executor, persistent local-control migration, real mission
  runtime, manual conversion or browser command interface added.

## VALIDATION

Accepted Stage 1:
- colcon build --packages-select runner_paddock: passed.
- pytest src/runner_paddock/test/test_command_authority_node.py -q: 4 passed.
- ros2 topic info -v on each command endpoint: counts shown above.
- Managed AUTONOMY startup with studio, zero-command observation, then shutdown.
  This was an ownership smoke check, not a Nav2 readiness acceptance test.
- git diff --check before commits.

Temporary STOP exploration:
- Missing durable state, deliberate STOP/clear, ongoing local input, executor
  SIGKILL, restart from persisted clear and persisted STOP, stale boot/generation
  clear requests, endpoint counts and PWM readback.
- Failed overall gate as documented above.

Rollback:
Removing generated experimental interfaces required a clean interface build
cache; the first incremental rollback build retained obsolete service-generated
code and failed. Final clean rebuild passed for runner_interfaces and runner_paddock (36.2 s);
all four existing plumbing tests passed again. Experimental generated interfaces
and executable were removed. Both application units are inactive. Final /cmd_vel
publisher count is zero, with motor_driver its sole subscriber; the private
authority output has one publisher and zero subscribers. Traction PWM is 0 ns;
steering PWM is 1500000 ns. No test mux, STOP executor or mission remains active.

## REMAINING WORK

Stage 2 is incomplete. Resolve its restart/discovery gate and finish typed
authority, lease/session, stale/replay, STOP acknowledgement and rearm contracts.
Then proceed through persistent local control, truthful runtime/mapping/config,
one real navigation runtime, Paddock manual, and atomic autonomy cutover.

Backend Definition of Done: **NOT PASSED**.
Frontend work: **NOT STARTED**.
Frontend functional acceptance: **NOT PASSED / NOT RUN**.
Physical traction validation: **PENDING**.
Future exploration remains outside this implementation scope.

TL;DR: v1.3 docs and collision isolation are pushed. The STOP prototype produced
useful timing results but failed its restart/endpoint gate, so it was not deployed.
The full backend and frontend are not complete.
