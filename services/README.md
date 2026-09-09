# Runner system services

These units run the Foxglove bridge, battery monitor, Raspberry Pi telemetry,
motor hardware owner, wheel encoder hardware owner, Paddock web backend, and
the persistent Paddock mode supervisor and local-control tier independently of application modes. The
battery, telemetry, motor, and encoder nodes are deliberately not added to
mode composites.

## Stage 3 persistent local control

`runner-local-control.service` owns exactly one `joy_node`, keyboard bridge,
`runner_teleop`, and the existing `twist_mux` in IDLE, MAPPING, and AUTONOMY.
The STOP executor remains its separate persistent service. Application launches
contain no local-control or mux nodes. Inactive teleop publishes status but no
velocity; release emits a bounded brake before becoming silent. The mux
precedence is STOP (255/lock 200), DualSense (100), supervised Paddock manual
(75), then supervised autonomy (50).

## Stage 4 application-mode supervision

`runner-mode-supervisor.service` is the only owner of application-mode
start/stop operations. It consumes the typed `/paddock/mode_request` interface
and publishes authoritative, transient-local `/paddock/mode_state`. Its
separate `status` field reports `STABLE`, `TRANSITIONING`, or `FAULT`; the UI
mode remains exactly `IDLE`, `MAPPING`, or `AUTONOMY`.

The supervisor runs unprivileged as `User=matti`, like the rest of ROS/DDS,
and manages the two fixed mode units over the systemd D-Bus API
(`org.freedesktop.systemd1.Manager` `StartUnit`/`StopUnit`) rather than a
setuid or sudo path. Authorization comes from a narrow polkit rule,
`/etc/polkit-1/rules.d/49-runner-mode-units.rules`, that grants
`subject.user == "matti"` the `org.freedesktop.systemd1.manage-units` action
only when `unit` is `runner-mode-mapping.service` or
`runner-mode-autonomy.service` and `verb` is `start` or `stop`; no other verb
(`restart`, `reload`, `enable`, `disable`, ...), unit, or subject is granted,
so matti cannot manage an unrelated system unit this way and still needs an
interactive admin password for anything outside this pair. The rule does not
touch PWM/motor/hardware privilege services.

`runner-command-authority.service` keeps the existing Stage 2 lease and
command-grant supervisor alive so only the current typed Paddock lease can
request a mode. It now follows authoritative `ModeState`; `TRANSITIONING` and
`FAULT` immediately reduce its effective mode to IDLE and revoke grants. It
is the sole normal supervised writer to `/cmd_vel_auto` and
`/cmd_vel_paddock`; neither it nor the browser writes `/cmd_vel` directly.
`runner-drive-adapter.service` persistently owns the one shared speed PI and
converts both Nav2 SI Twist and authority-bounded browser manual demand.

The supervisor always publishes `TRANSITIONING`, stops both fixed mode units,
waits for empty cgroups and a graph with no mode resources, then starts and
checks the requested mode. A failed start is stopped through systemd and is
reported as `IDLE/FAULT`. On restart it derives state from both units and the
ROS graph. Conflicting, partial, failed, or unmanaged mode graphs fail closed
instead of being guessed as a mode.

The two application units have mutual `Conflicts=`, `KillMode=control-group`,
and no `[Install]` section, so they cannot be enabled at boot:

- `runner-mode-mapping.service`: `map.launch.py`, containing one sensor/static
  TF tier, one estimation tier, and mapping slam_toolbox.
- `runner-mode-autonomy.service`: `autonomy.launch.py`, containing the same
  common owners once, localization slam_toolbox remapped to `/slam_map`,
  map_server as the sole `/map` publisher, Nav2, and
  `runner_navigation_runtime` (Stage 5) as the sole Nav2 mission/action owner.

## Stage 5 navigation runtime

`runner_navigation_runtime` (`ros2 run runner_bringup navigation_runtime`,
started inside `nav2.launch.py`, application-tier) is the only component that
owns Nav2 `NavigateToPose`/`NavigateThroughPoses` action clients. It consumes
authorized `/paddock/navigation_request` from the command authority and
publishes a truthful `/paddock/navigation_state` lifecycle; it never reports an
active action before Nav2 has accepted a goal, and a stale action generation or
runtime epoch can never overwrite current mission state. The retired
`foxglove_goal_bridge` node and its direct goal/keyboard ingress are gone. This
stage does not add a mux input, does not authorize motion, and leaves traction
disconnected.

AUTONOMY requests carry `autonomy_map` in `ModeRequest`. Before starting, the
supervisor rejects path-like names and verifies all four artifacts: `.data`,
`.posegraph`, `.yaml`, and the occupancy image referenced by the YAML. The
validated basename is atomically written to
`/run/runner-paddock/autonomy-map`; the fixed autonomy unit validates it again
before execing the launch. No shell interpolation is used for the basename.

Install the Stage 4 polkit rule before enabling the supervisor, since it runs
as matti and needs it to start/stop the two fixed mode units:

```sh
sudo install -m 0644 -o root -g root \
  /home/matti/runner_ws/services/49-runner-mode-units.rules \
  /etc/polkit-1/rules.d/49-runner-mode-units.rules
sudo systemctl restart polkit.service
sudo apt-get install -y python3-dbus
```

Install the Stage 4 units as authoritative symlinks, reload systemd, and enable
only the supervisor:

```sh
sudo ln -s /home/matti/runner_ws/services/runner-mode-mapping.service /etc/systemd/system/runner-mode-mapping.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-autonomy.service /etc/systemd/system/runner-mode-autonomy.service
sudo ln -s /home/matti/runner_ws/services/runner-command-authority.service /etc/systemd/system/runner-command-authority.service
sudo ln -s /home/matti/runner_ws/services/runner-local-control.service /etc/systemd/system/runner-local-control.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-supervisor.service /etc/systemd/system/runner-mode-supervisor.service
sudo systemctl daemon-reload
sudo systemctl enable --now runner-command-authority.service runner-local-control.service runner-mode-supervisor.service
```

Do not run `systemctl disable` on the two static mode units: because their
authoritative files are manually linked into `/etc/systemd/system`, `disable`
would remove those links. Their lack of an `[Install]` section means the
`linked` state is already non-boot-enabled. Verify there are no mode-unit links
under any target's `.wants/` or `.requires/` directory.

Do not launch `map.launch.py`, `localize.launch.py`, `nav2.launch.py`, or
`autonomy.launch.py` manually on a deployed robot. An unmanaged mode graph is
intentionally reported as `IDLE/FAULT`; the supervisor will not use `pkill` or
claim an unknown process tree.

`runner-paddock-web.service` runs the Paddock operator backend
(`ros2 run runner_paddock web`) as the persistent-operator-tier browser
gateway: it subscribes to established robot state and serves a same-origin
WebSocket, and it is the **sole browser-side writer** of
`/paddock/control_event` (lease / RUN / STOP / CLEAR STOP / goal / heartbeat),
`/paddock/mode_request` (runtime selection) and `/paddock/map_request` (NEW /
SAVE / SELECT / DELETE map). DELETE is lease-scoped, executor-validated, and
rejects the selected or active-autonomy map. It holds at most one control lease
at a time (one controller, any number of observers); it manufactures no
renewals — an intent is published only in direct response to a fresh browser
message, so a silent browser lets the Pi-side lease expire. It still runs
unprivileged as `matti`,
binds `127.0.0.1` only (`PADDOCK_WEB_HOST`/`PADDOCK_WEB_PORT` in the unit),
runs one uvicorn worker with no reload, and has no `systemctl`/sudoers grant —
it cannot touch the hardware tier and can only *request* mode changes through
the lease-checked mode supervisor / map executor / command authority. `KillSignal=SIGINT` gives uvicorn the same
graceful shutdown path as an interactive Ctrl-C, after which the
`/runner_paddock_web_state` node leaves the graph. Runtime dependencies
(`python3-fastapi python3-uvicorn python3-websockets`, declared in
`package.xml`) are installed from the Ubuntu archive via apt/rosdep — there is
no pip target or `PYTHONPATH` shim.

**Reaching Paddock: Tailscale Serve, not a LAN port (Stage 3C).** The backend
only ever binds `127.0.0.1:8000`; it is not reachable from the LAN or from
the tailnet IP directly. The network-facing boundary is Tailscale Serve,
configured tailnet-only (no Funnel — never public internet):

```sh
tailscale serve --bg --https=443 http://127.0.0.1:8000
```

**Operator URL:** `https://makro-runner.taila47bfc.ts.net/` — same-origin
HTTP and the `/ws` WebSocket (as `wss://`) both work through the proxy.
Reaching it requires being on the tailnet; MagicDNS and HTTPS certificates
must be enabled for the tailnet in the admin console
(https://login.tailscale.com/admin/dns) before `tailscale serve` will accept
`--https`.

`--bg` persists the config in `tailscaled`'s own state and is restored
automatically across `tailscaled` restarts and reboots — it is independent
of `runner-paddock-web.service` and does not need to be re-run after a
Paddock service restart. Recovery / check commands:

```sh
tailscale serve status               # human-readable: proxy target, tailnet-only vs funnel
tailscale serve status --json        # machine-readable, confirms no Funnel/AllowFunnel key
curl -s -o /dev/null -w '%{http_code}\n' https://makro-runner.taila47bfc.ts.net/
sudo tailscale cert makro-runner.taila47bfc.ts.net   # force a cert refresh if HTTPS breaks
tailscale serve --bg --https=443 http://127.0.0.1:8000   # idempotent re-apply if config is lost
tailscale serve --https=443 off      # tear down the proxy entirely
```

`runner-motor.service` is the sole continuous owner of the motor and steering
PWM channels. The existing `runner-pwm-setup.service` remains the temporary
boot-time exporter and permission preparer; systemd requires it to complete
before starting the motor service. The motor service drives the Cytron MD13S
with GPIO12 hardware PWM at 20 kHz and GPIO23 DIR, requested exclusively from
the `pinctrl-rp1` GPIO chip by label. GPIO13 remains the 50 Hz steering PWM.
The node writes sysfs directly and never unexports either PWM channel.

`runner-encoder.service` is the sole continuous owner of GPIO 22. Keeping it
alive independently of application launches ensures that
`/wheel/encoder_state` remains available to the motor reversal gate. If encoder
state is absent or never supplies a post-request stationary sample,
`motor_node` remains fail-closed at zero duty (active brake), logs the pending
gate condition, and does not change the hardware DIR line.

### Coherent install / deploy

`services/install.sh` is the single source of truth for the systemd layout.
It symlinks **every** tracked unit into `/etc/systemd/system` (replacing any
stale hand-copied file), installs `setup-runner-pwm` to `/usr/local/sbin` and
the narrow `49-runner-mode-units.rules` polkit rule, runs `daemon-reload`, and
`enable`s the persistent tier — but never the two `runner-mode-*` units, which
`runner-mode-supervisor` owns.

```sh
cd /home/matti/runner_ws
git pull
colcon build --symlink-install          # message defs + nodes must be current
services/install.sh --check             # report drift, change nothing
sudo services/install.sh                # apply the layout + enable persistent tier
sudo services/install.sh --restart      # restart the operator/application tier in order
```

`--restart` cycles `runner-stop-enforcer → runner-command-authority →
runner-local-control → runner-mode-supervisor → runner-map-executor →
runner-paddock-web`. It deliberately does **not** touch `runner-motor` /
`runner-encoder`; restart those yourself, with traction power disconnected, if
their binaries changed. A reboot achieves the same coherent bring-up.

The persistent tier that `install.sh` enables:

| Unit | Role |
|---|---|
| `runner-pwm-setup.service` | oneshot PWM export + perms (before motor) |
| `runner-motor.service` | sole motor/steering PWM owner |
| `runner-encoder.service` | sole GPIO 22 encoder owner |
| `runner-battery` / `runner-telemetry` / `runner-foxglove` | telemetry + diag |
| `runner-stop-enforcer.service` | persistent global STOP executor + mux lock |
| `runner-local-control.service` | joy + keyboard bridge + teleop + the one twist_mux |
| `runner-command-authority.service` | sole supervised `/cmd_vel_auto` writer, lease + RUN |
| `runner-mode-supervisor.service` | sole start/stop owner of the mode units |
| `runner-map-executor.service` | `/paddock/map_request` → `/paddock/map_state` |
| `runner-paddock-web.service` | browser intent gateway |

Manual equivalent (only if not using the script):

```sh
for u in runner-pwm-setup runner-motor runner-encoder runner-battery \
         runner-telemetry runner-foxglove runner-stop-enforcer \
         runner-local-control runner-command-authority runner-mode-supervisor \
         runner-map-executor runner-paddock-web runner-mode-mapping \
         runner-mode-autonomy; do
  sudo ln -sfn "/home/matti/runner_ws/services/$u.service" "/etc/systemd/system/$u.service"
done
sudo install -m 0755 /home/matti/runner_ws/services/setup-runner-pwm /usr/local/sbin/setup-runner-pwm
sudo install -m 0644 /home/matti/runner_ws/services/49-runner-mode-units.rules /etc/polkit-1/rules.d/
sudo systemctl daemon-reload
sudo systemctl enable runner-pwm-setup runner-motor runner-encoder \
  runner-battery runner-telemetry runner-foxglove runner-stop-enforcer \
  runner-local-control runner-command-authority runner-mode-supervisor \
  runner-map-executor runner-paddock-web
```

Do not pass `runner-mode-mapping` / `runner-mode-autonomy` to `enable` or
`disable`: they carry no `[Install]` section, so `enable` is a no-op and
`disable` only deletes the symlink you just created. See the note above.

`runner-paddock-web.service` also needs its Python runtime dependencies
present for `/usr/bin/python3` (one time, from the Ubuntu archive):

```sh
sudo apt-get install -y python3-fastapi python3-uvicorn python3-websockets
```

All three are stock Ubuntu 24.04 archive packages and are the exact rosdep
keys declared in `src/runner_paddock/package.xml`; `rosdep install` resolves
to the same apt packages once `rosdep` is initialized. `python3-httpx` is an
archive package too and is only needed to run the package test suite.

Before enabling any unit, verify that it sources both
`/opt/ros/jazzy/setup.bash` and `/home/matti/runner_ws/install/setup.bash`, and
that the workspace overlay is current.

Enable and start the services manually (or just use `install.sh` above):

```sh
sudo systemctl enable --now runner-foxglove.service
sudo systemctl enable --now runner-battery.service
sudo systemctl enable --now runner-telemetry.service
sudo systemctl enable --now runner-stop-enforcer.service
sudo systemctl enable --now runner-map-executor.service
sudo systemctl enable --now runner-encoder.service
sudo systemctl enable --now runner-paddock-web.service
sudo systemctl enable --now runner-command-authority.service
sudo systemctl enable --now runner-local-control.service
sudo systemctl enable --now runner-mode-supervisor.service
```

With traction power disconnected, enable and start the motor owner separately:

```sh
sudo systemctl enable --now runner-motor.service
systemctl status runner-motor.service
```

Application composites publish commands but do not manage motor hardware. On
composite shutdown, `/cmd_vel` publication stops while `motor_node` remains
alive. D-09 is the primary stop path: after 200 ms without a command, the motor
watchdog writes motor duty zero, which is MD13S active brake, and publishes
the hardware-latched direction. Do not stop the motor service as part of
normal composite teardown.

Check the encoder owner and its motor-gate publication with:

```sh
systemctl status runner-encoder.service
ros2 topic info /wheel/encoder_state
ros2 topic echo /wheel/encoder_state --once
```

After rebuilding `runner_motor`, restart its persistent owner with traction
power disconnected so it loads the updated workspace installation:

```sh
sudo systemctl restart runner-motor.service
sudo systemctl restart runner-encoder.service
systemctl status runner-motor.service
journalctl -u runner-encoder.service -n 50 --no-pager
journalctl -u runner-motor.service -n 50 --no-pager
```

With traction power disconnected, the running service should report GPIO23's
consumer as `runner_motor_dir`; GPIO12 should read `period=50000`,
`duty_cycle=0`, `enable=1`, and `polarity=normal`; GPIO13 should retain
`period=20000000`. Wheels-off-ground validation with traction connected is a
separate required hardware step.
