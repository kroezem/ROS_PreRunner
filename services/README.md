# Runner system services

These units run the Foxglove bridge, battery monitor, Raspberry Pi telemetry,
and motor hardware owner independently of the application launch stacks. The
battery, telemetry, and motor nodes are deliberately not added to composites.

`runner-motor.service` is the sole continuous owner of the motor and steering
PWM channels. The existing `runner-pwm-setup.service` remains the temporary
boot-time exporter and permission preparer; systemd requires it to complete
before starting the motor service. The motor service drives the Cytron MD13S
with GPIO12 hardware PWM at 20 kHz and GPIO23 DIR, requested exclusively from
the `pinctrl-rp1` GPIO chip by label. GPIO13 remains the 50 Hz steering PWM.
The node writes sysfs directly and never unexports either PWM channel.

Install the units as symlinks so the tracked copies remain authoritative:

```sh
sudo ln -s /home/matti/runner_ws/services/runner-foxglove.service /etc/systemd/system/runner-foxglove.service
sudo ln -s /home/matti/runner_ws/services/runner-battery.service /etc/systemd/system/runner-battery.service
sudo ln -s /home/matti/runner_ws/services/runner-telemetry.service /etc/systemd/system/runner-telemetry.service
sudo ln -s /home/matti/runner_ws/services/runner-motor.service /etc/systemd/system/runner-motor.service
sudo systemctl daemon-reload
```

Before enabling any unit, verify that it sources both
`/opt/ros/jazzy/setup.bash` and `/home/matti/runner_ws/install/setup.bash`, and
that the workspace overlay is current.

Enable and start the services manually:

```sh
sudo systemctl enable --now runner-foxglove.service
sudo systemctl enable --now runner-battery.service
sudo systemctl enable --now runner-telemetry.service
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
direction zero. Do not stop the motor service as part of normal composite
teardown.

After rebuilding `runner_motor`, restart its persistent owner with traction
power disconnected so it loads the updated workspace installation:

```sh
sudo systemctl restart runner-motor.service
systemctl status runner-motor.service
journalctl -u runner-motor.service -n 50 --no-pager
```

With traction power disconnected, the running service should report GPIO23's
consumer as `runner_motor_dir`; GPIO12 should read `period=50000`,
`duty_cycle=0`, `enable=1`, and `polarity=normal`; GPIO13 should retain
`period=20000000`. Wheels-off-ground validation with traction connected is a
separate required hardware step.
