# Runner system services

These units run the Foxglove bridge, battery monitor, Raspberry Pi telemetry,
motor hardware owner, wheel encoder hardware owner, Paddock web backend, and
the persistent Paddock mode supervisor independently of application modes. The
battery, telemetry, motor, and encoder nodes are deliberately not added to
mode composites.

## Stage 4 application-mode supervision

`runner-mode-supervisor.service` is the only owner of application-mode
start/stop operations. It consumes the typed `/paddock/mode_request` interface
and publishes authoritative, transient-local `/paddock/mode_state`. Its
separate `status` field reports `STABLE`, `TRANSITIONING`, or `FAULT`; the UI
mode remains exactly `IDLE`, `MAPPING`, or `AUTONOMY`.

`runner-command-authority.service` keeps the existing Stage 2 lease and
command-grant supervisor alive so only the current typed Paddock lease can
request a mode. It now follows authoritative `ModeState`; `TRANSITIONING` and
`FAULT` immediately reduce its effective mode to IDLE and revoke grants. It
remains offline from the production mux: `/cmd_vel_paddock` and supervised
autonomy are not added to `twist_mux.yaml`, and no command priority changes.

The supervisor always publishes `TRANSITIONING`, stops both fixed mode units,
waits for empty cgroups and a graph with no mode resources, then starts and
checks the requested mode. A failed start is stopped through systemd and is
reported as `IDLE/FAULT`. On restart it derives state from both units and the
ROS graph. Conflicting, partial, failed, or unmanaged mode graphs fail closed
instead of being guessed as a mode.

The two application units have mutual `Conflicts=`, `KillMode=control-group`,
and no `[Install]` section, so they cannot be enabled at boot:

- `runner-mode-mapping.service`: `map.launch.py`, containing one sensor/static
  TF tier, one estimation tier, mapping slam_toolbox, and the existing
  keyboard/DualSense teleop plus twist_mux path.
- `runner-mode-autonomy.service`: `autonomy.launch.py`, containing the same
  common owners once, localization slam_toolbox remapped to `/slam_map`,
  map_server as the sole `/map` publisher, Nav2, physical teleop, drive
  adapter, and twist_mux.

AUTONOMY requests carry `autonomy_map` in `ModeRequest`. Before starting, the
supervisor rejects path-like names and verifies all four artifacts: `.data`,
`.posegraph`, `.yaml`, and the occupancy image referenced by the YAML. The
validated basename is atomically written to
`/run/runner-paddock/autonomy-map`; the fixed autonomy unit validates it again
before execing the launch. No shell interpolation is used for the basename.

Install the Stage 4 units as authoritative symlinks, reload systemd, and enable
only the supervisor:

```sh
sudo ln -s /home/matti/runner_ws/services/runner-mode-mapping.service /etc/systemd/system/runner-mode-mapping.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-autonomy.service /etc/systemd/system/runner-mode-autonomy.service
sudo ln -s /home/matti/runner_ws/services/runner-command-authority.service /etc/systemd/system/runner-command-authority.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-supervisor.service /etc/systemd/system/runner-mode-supervisor.service
sudo systemctl daemon-reload
sudo systemctl enable --now runner-command-authority.service runner-mode-supervisor.service
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

`runner-paddock-web.service` runs the Stage 3 Paddock backend
(`ros2 run runner_paddock web`) as an application-tier, read-only observer: it
subscribes to established robot state and serves a same-origin WebSocket, and
publishes nothing to ROS. It runs unprivileged as `matti`, binds `127.0.0.1`
only (`PADDOCK_WEB_HOST`/`PADDOCK_WEB_PORT` in the unit), runs one uvicorn
worker with no reload, and has no `systemctl`/sudoers grant — it cannot touch
the hardware tier or switch modes. `KillSignal=SIGINT` gives uvicorn the same
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

Install the units as symlinks so the tracked copies remain authoritative:

```sh
sudo ln -s /home/matti/runner_ws/services/runner-foxglove.service /etc/systemd/system/runner-foxglove.service
sudo ln -s /home/matti/runner_ws/services/runner-battery.service /etc/systemd/system/runner-battery.service
sudo ln -s /home/matti/runner_ws/services/runner-telemetry.service /etc/systemd/system/runner-telemetry.service
sudo ln -s /home/matti/runner_ws/services/runner-motor.service /etc/systemd/system/runner-motor.service
sudo ln -s /home/matti/runner_ws/services/runner-encoder.service /etc/systemd/system/runner-encoder.service
sudo ln -s /home/matti/runner_ws/services/runner-paddock-web.service /etc/systemd/system/runner-paddock-web.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-mapping.service /etc/systemd/system/runner-mode-mapping.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-autonomy.service /etc/systemd/system/runner-mode-autonomy.service
sudo ln -s /home/matti/runner_ws/services/runner-command-authority.service /etc/systemd/system/runner-command-authority.service
sudo ln -s /home/matti/runner_ws/services/runner-mode-supervisor.service /etc/systemd/system/runner-mode-supervisor.service
sudo systemctl daemon-reload
```

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

Enable and start the services manually:

```sh
sudo systemctl enable --now runner-foxglove.service
sudo systemctl enable --now runner-battery.service
sudo systemctl enable --now runner-telemetry.service
sudo systemctl enable --now runner-encoder.service
sudo systemctl enable --now runner-paddock-web.service
sudo systemctl enable --now runner-command-authority.service
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
