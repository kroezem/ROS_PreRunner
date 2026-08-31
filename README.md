# Runner

Runner is a self-contained 1/18-scale autonomous vehicle built from a LaTrax Prerunner RC platform. It uses a Raspberry Pi 5 running Ubuntu 24.04 and ROS 2 Jazzy, with all sensing and computation performed onboard.

The project is an experimental platform for autonomous navigation, localization, control, and hardware/software integration on a small Ackermann-steered vehicle.

Project page: https://makro.ca/prerunner/

## Current Status

| Capability                           | Status  |
| ------------------------------------ | ------- |
| ROS 2 hardware bring-up              | Working |
| LiDAR and IMU integration            | Working |
| Wheel speed sensing                  | Working |
| Teleoperation                        | Working |
| SLAM                                 | Working |
| Fixed-map localization               | Working |
| Point-to-point navigation            | Working |
| Multi-waypoint autonomous navigation | Working |
| Autonomous exploration               | Planned |
| Higher-speed autonomous driving      | Planned |

The vehicle currently performs SLAM and autonomous point-to-point navigation through indoor environments. The next development stage is autonomous exploration of previously unknown environments, followed by progressively increasing vehicle speed to investigate the limits imposed by localization, control, and tire traction.

## Hardware

| Component                 | Role                                    |
| ------------------------- | --------------------------------------- |
| Raspberry Pi 5 8 GB       | Onboard compute                         |
| LD19 2D LiDAR             | Mapping, localization, obstacle sensing |
| BNO085 IMU                | Orientation and inertial sensing        |
| Hall-effect wheel encoder | Wheel speed and motion state            |
| MD13S motor driver        | Traction motor control                  |
| Steering servo            | Ackermann steering                      |
| X1201 UPS                 | Compute power and battery telemetry     |
| LaTrax Prerunner          | 1/18-scale vehicle platform             |

All primary sensing and compute are onboard. The platform does not require motion capture, external localization anchors, or offboard processing.

## Software Architecture

The system is split into ROS 2 packages with separate ownership of sensing, estimation, control, navigation, telemetry, and hardware resources.

```text
LD19 LiDAR ------> scan processing ------> SLAM / localization
                                            |
BNO085 IMU --------> estimation -----------+
                                            |
Wheel encoder ------> odometry ------------+
                                            |
                                            v
                                      robot pose
                                            |
                                            v
                                         Nav2
                                            |
                                            v
                                      drive adapter
                                            |
                                            v
                                        twist mux
                                            |
                                            v
                                      motor + steering
```

Localization and navigation use ROS 2 TF, slam_toolbox, an EKF-based odometry pipeline, RF2O laser odometry, and Navigation2.

Nav2 provides global planning and path following while custom Runner nodes handle vehicle-specific command conversion, sensing, actuation, telemetry, teleoperation, and safety behavior.

## Runner Packages

| Package                | Purpose                                                                      |
| ---------------------- | ---------------------------------------------------------------------------- |
| `runner_bringup`       | Launch, scan processing, transforms, localization and navigation integration |
| `runner_drive_adapter` | Converts Nav2 vehicle commands into bounded vehicle actuation commands       |
| `runner_motor`         | Motor direction/PWM and steering servo control                               |
| `runner_encoder`       | Hall-effect wheel sensing and motion state                                   |
| `runner_imu`           | BNO085 ROS 2 interface                                                       |
| `runner_teleop`        | Manual control interfaces                                                    |
| `runner_telemetry`     | System telemetry                                                             |
| `runner_battery`       | UPS battery telemetry                                                        |
| `runner_interfaces`    | Custom ROS 2 messages                                                        |

## Third-Party and Modified Packages

The workspace also contains external ROS packages required by the platform.

| Package                                  | Use                                                |
| ---------------------------------------- | -------------------------------------------------- |
| `ldlidar_stl_ros2`                       | LD19 LiDAR driver                                  |
| `rf2o_laser_odometry`                    | Laser odometry, maintained as a vendored fork      |
| `nav2_regulated_pure_pursuit_controller` | Nav2 controller with Runner-specific modifications |

Vendored packages retain their upstream attribution and licensing. Runner-specific changes to modified upstream packages are documented within those packages.

## Testing and Validation

A major focus of the project is making failures observable and validating changes against the physical system.

The custom control stack includes unit and integration tests for command conversion, saturation, timing, direction handling, sensor freshness, controller behavior, and safety constraints.

Hardware behavior is validated through repeatable physical tests and ROS bag/MCAP analysis. Examples include motor characterization, encoder characterization, localization comparison, controller response, turning geometry, and failure-mode testing.

The project follows a diagnostic-first workflow: changes are made after identifying a measurable failure mode, then verified using recorded data or hardware testing.

## Repository Structure

```text
.
|-- analysis/     Experimental analysis and reports
|-- docs/         Architecture, design decisions and validation notes
|-- maps/         SLAM maps used during development
|-- scripts/      Bring-up and map-management utilities
|-- services/     Persistent hardware systemd services
|-- src/          ROS 2 packages
`-- tools/        Development and analysis tools
```

The most complete architecture and implementation notes are available under `docs/`.

## Next Steps

| Stage                    | Goal                                                                                        |
| ------------------------ | ------------------------------------------------------------------------------------------- |
| Exploration              | Autonomously map and explore unknown indoor environments                                    |
| Higher-speed autonomy    | Increase navigation speed while maintaining localization and control stability              |
| Dynamic characterization | Explore localization, controller and traction limits as vehicle dynamics become significant |
| Advanced control         | Investigate more capable path-following and racing-oriented control strategies              |
