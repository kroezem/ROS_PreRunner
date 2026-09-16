// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#ifndef RUNNER_PATH_SPEED_PROFILE__PATH_SPEED_PROFILE_HPP_
#define RUNNER_PATH_SPEED_PROFILE__PATH_SPEED_PROFILE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "runner_interfaces/msg/path_speed_profile.hpp"

namespace runner_path_speed_profile
{

struct ProfileConfig
{
  double minimum_traversal_speed{0.25};
  double constrained_speed_scaling{0.20};
  // The single authoritative reference ceiling constrained-speed scaling
  // normalizes against (today, ABSURD's preset maximum). Explicit and
  // tunable so a future change to that preset's ceiling cannot silently
  // desynchronize this law from it.
  double scaling_reference_speed{2.0};
  double tight_clearance{0.05};
  double open_clearance{0.70};
  // 0 = linear, 1 = power, 2 = smoothstep(power(x, shape)).
  double clearance_curve_family{2.0};
  double clearance_curve_shape{1.0};
  double approach_time_s{0.0};
  double curvature_window{0.40};
  double max_lateral_acceleration{0.35};
  double footprint_front{0.230};
  double footprint_rear{0.060};
  double footprint_half_width{0.0825};
  double braking_linear{1.6};
  double braking_constant{0.27};
  // Anticipatory distance added to backward braking reachability for
  // steering/vehicle response lag. This augments, rather than recalibrates,
  // the braking model above.
  double reaction_time_s{0.40};
  // Forward recovery uses a(v) = gain * v + floor, deliberately independent
  // of the braking model above. At the former 0.25 m/s creep speed this keeps
  // the previous 1.0 m/s^2 recovery while releasing faster at higher speed.
  double recovery_acceleration_gain{1.6};
  double recovery_acceleration_floor{0.60};
  double minimum_pose_step{1e-4};
  double direction_projection_threshold{0.5};
};

uint64_t pathIdentity(const nav_msgs::msg::Path & path);

std::vector<double> costmapClearance(
  const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D & costmap,
  double footprint_front, double footprint_rear, double footprint_half_width);

double clearanceSpeed(
  double clearance, double preset_ceiling, const ProfileConfig & config);

bool validConfig(const ProfileConfig & config);

runner_interfaces::msg::PathSpeedProfile makeProfile(
  const nav_msgs::msg::Path & path, const std::vector<double> & clearance,
  double preset_ceiling, const ProfileConfig & config);

bool validateProfile(
  const runner_interfaces::msg::PathSpeedProfile & profile,
  const nav_msgs::msg::Path & path, std::string & reason);

double sampleCeiling(
  const runner_interfaces::msg::PathSpeedProfile & profile, double arclength);

}  // namespace runner_path_speed_profile

#endif  // RUNNER_PATH_SPEED_PROFILE__PATH_SPEED_PROFILE_HPP_
