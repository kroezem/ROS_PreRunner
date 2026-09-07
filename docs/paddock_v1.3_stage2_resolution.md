# Runner v1.3 Stage 2 STOP restart-gate resolution

Validation date: 2026-09-07. This is software validation with the traction
battery physically disconnected.

## Confirmed

The three STOP publisher endpoints in the failed probe were Fast DDS discovery
entries from three sequential, dead executor processes. The isolated run mapped
them as follows:

| Generation | PID | Boot ID prefix | DDS writer GUID |
| --- | ---: | --- | --- |
| 1 | 253006 | `d6e...` | `010f88774edca0db0000000000001303` |
| 2 | 253023 | `f13...` | `010f88775fdcc6d40000000000001303` |
| 3 | 253040 | `b73...` | `010f887770dc94310000000000001303` |

Each PID was absent from the process table before its successor started. A
long-lived observer still counted all three endpoints, but received heartbeats
only from the current boot. After all three processes were dead, raw endpoint
count was 3 at five seconds, 2 at ten seconds, and 0 at twenty seconds. Dead
endpoints could not publish.

The service timeout was caused by a stale Fast DDS participant association. A
persistent client continued to report the dead service as ready. Destroying and
recreating the client, and even creating another node in the same ROS context,
still timed out. A client in a fresh process immediately reached the current
server and received its current boot ID. This establishes that service readiness
was discovery readiness, not proof of a live server, and explains the timeout.

Systemd did not overlap live executor generations. With the real `ros2 run`
wrapper shape, the old wrapper and executor child left the service cgroup before
the replacement generation started. The production gate also observed no
intersection between old and new cgroup membership. The two steady PIDs in the
cgroup are one `ros2 run` wrapper and its one executor child, not two executor
generations.

The live-ownership acceptance test therefore uses process and boot evidence,
rather than raw graph count: one systemd cgroup, one executor child, one fresh
boot heartbeat, acknowledgements matching that boot and request ID, and no
fresh heartbeat from an older boot. Raw graph entries are diagnostic only and
must converge to zero after their processes die.

## Correction

STOP command transport now uses short-lived, boot-qualified, idempotent request
messages with explicit acknowledgements in the executor state stream. This
removes reliance on a persistent service client's stale association while
retaining the existing STOP semantics. The persistent executor stores STOP
atomically, publishes the highest-priority velocity zero and mux lock, and is
the only live STOP generation. The existing `twist_mux` remains the sole
`/cmd_vel` writer.

No graph introspection runs in the enforcement timer. STOP application is
acknowledged from durable state, active lock, and a fresh observed final zero.

## Restart-gate evidence

- Isolated STOP request to final zero: 1.281 ms.
- Executor hard-failure worst last nonzero: 141.688 ms.
- Persisted STOP restored with a new boot ID after executor restart.
- Replayed clear for an old boot and generation was rejected.
- A post-restart request was acknowledged by the current boot; retry was
  idempotent.
- Known-clear survived authority failure without depending on Paddock/web.
- Application restart preserved STOP.
- Production `/cmd_vel` graph contained one endpoint with the existing mux
  identity, and the process table contained one live mux process.
- Final PWM: traction `0`, steering `1500000`.
- `runner_interfaces`, `runner_paddock`, and `runner_bringup` built successfully.
- `runner_paddock`: 82 tests passed.

## Hypotheses

None required for the Stage 2 decision. The internal Fast DDS cache mechanism
that delays disposal was not characterized beyond its externally observed
behavior because the dead endpoints were proven inert and converged within a
bounded interval.

## Open questions

None for Stage 2. Later v1.3 stages remain out of scope.
