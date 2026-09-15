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
  double creep_speed{0.40};
  // Clearance beyond footprint_radius at which the continuous clearance
  // ceiling is halfway from creep_speed to the active preset ceiling.
  double clearance_half_speed{0.05};
  double curvature_window{0.40};
  double max_lateral_acceleration{0.35};
  double footprint_radius{0.2444};
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
  double footprint_radius);

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
