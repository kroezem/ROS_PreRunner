// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include "runner_path_speed_profile/path_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "runner_interfaces/msg/path_speed_profile_point.hpp"

namespace runner_path_speed_profile
{
namespace
{

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

void hashBytes(uint64_t & hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= kFnvPrime;
  }
}

void hashDouble(uint64_t & hash, double value)
{
  if (value == 0.0) {
    value = 0.0;  // Give -0.0 and +0.0 the same identity.
  }
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    const auto byte = static_cast<unsigned char>((bits >> shift) & 0xffu);
    hashBytes(hash, &byte, sizeof(byte));
  }
}

double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(
    b.pose.position.x - a.pose.position.x,
    b.pose.position.y - a.pose.position.y);
}

double yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double angleDifference(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

std::vector<double> arclengths(const nav_msgs::msg::Path & path)
{
  std::vector<double> s(path.poses.size(), 0.0);
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    s[i] = s[i - 1] + poseDistance(path.poses[i - 1], path.poses[i]);
  }
  return s;
}

// Felzenszwalb/Huttenlocher squared Euclidean distance transform in one dimension.
void distanceTransform1D(
  const std::vector<double> & input, std::vector<double> & output)
{
  const int n = static_cast<int>(input.size());
  std::vector<int> sites(n);
  std::vector<double> boundaries(n + 1);
  int k = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double boundary = 0.0;
    while (true) {
      const int p = sites[k];
      boundary = ((input[q] + q * q) - (input[p] + p * p)) /
        (2.0 * (q - p));
      if (boundary > boundaries[k] || k == 0) {
        break;
      }
      --k;
    }
    ++k;
    sites[k] = q;
    boundaries[k] = boundary;
    boundaries[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (boundaries[k + 1] < q) {
      ++k;
    }
    const double delta = q - sites[k];
    output[q] = delta * delta + input[sites[k]];
  }
}

double stoppingPotential(double speed, const ProfileConfig & config)
{
  if (speed <= 0.0) {
    return 0.0;
  }
  const double k = config.braking_linear;
  const double c = config.braking_constant;
  return speed / k - c / (k * k) * std::log((k * speed + c) / c);
}

double brakingReachabilityPotential(double speed, const ProfileConfig & config)
{
  return stoppingPotential(speed, config) + config.reaction_time_s * speed;
}

double reachableSpeed(
  double from_speed, double distance, double ceiling,
  const ProfileConfig & config)
{
  const double target = brakingReachabilityPotential(from_speed, config) +
    std::max(0.0, distance);
  if (brakingReachabilityPotential(ceiling, config) <= target) {
    return ceiling;
  }
  double low = 0.0;
  double high = ceiling;
  for (int iteration = 0; iteration < 48; ++iteration) {
    const double middle = 0.5 * (low + high);
    if (brakingReachabilityPotential(middle, config) <= target) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return low;
}

// Forward recovery is deliberately not the inverse of reachableSpeed()
// above. Its independently tuned acceleration rises with speed, avoiding a
// flat high-speed recovery cap without granting authority above the raw
// clearance/curvature/cusp/goal ceiling supplied by the caller.
double reachableSpeedByAcceleration(
  double from_speed, double distance, double ceiling, double acceleration_gain,
  double acceleration_floor)
{
  if (distance <= 0.0) {
    return std::min(from_speed, ceiling);
  }
  // Integrating v dv/ds = gain * v + floor gives distance as a monotonic
  // function of the reachable speed. Solve it by bisection to retain the
  // exact raw ceiling as the upper bound.
  const auto recovery_distance = [&](double speed) {
      const double from_term = acceleration_gain * from_speed + acceleration_floor;
      const double speed_term = acceleration_gain * speed + acceleration_floor;
      return (speed - from_speed) / acceleration_gain -
             acceleration_floor / (acceleration_gain * acceleration_gain) *
             std::log(speed_term / from_term);
    };
  if (from_speed >= ceiling || recovery_distance(ceiling) <= distance) {
    return ceiling;
  }
  double low = from_speed;
  double high = ceiling;
  for (int iteration = 0; iteration < 48; ++iteration) {
    const double middle = 0.5 * (low + high);
    if (recovery_distance(middle) <= distance) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return low;
}

// Per-pose continuous clearance ceiling, before curvature and longitudinal
// passes. Invalid or missing clearance fails conservatively to zero clearance.
std::vector<double> clearanceSpeeds(
  const std::vector<double> & clearance, std::size_t pose_count,
  double preset_ceiling, const ProfileConfig & config)
{
  std::vector<double> speeds(pose_count);
  for (std::size_t i = 0; i < pose_count; ++i) {
    const double measured = clearance.size() == pose_count ? clearance[i] : 0.0;
    speeds[i] = clearanceSpeed(measured, preset_ceiling, config);
  }
  return speeds;
}

bool validConfigImpl(const ProfileConfig & c)
{
  return std::isfinite(c.minimum_traversal_speed) && c.minimum_traversal_speed > 0.0 &&
         std::isfinite(c.tight_clearance) && c.tight_clearance >= 0.0 &&
         std::isfinite(c.open_clearance) && c.open_clearance > c.tight_clearance &&
         std::isfinite(c.clearance_curve_family) && c.clearance_curve_family >= 0.0 &&
         c.clearance_curve_family <= 2.0 &&
         std::floor(c.clearance_curve_family) == c.clearance_curve_family &&
         std::isfinite(c.clearance_curve_shape) && c.clearance_curve_shape > 0.0 &&
         std::isfinite(c.approach_time_s) && c.approach_time_s >= 0.0 &&
         std::isfinite(c.curvature_window) && c.curvature_window > 0.0 &&
         std::isfinite(c.max_lateral_acceleration) && c.max_lateral_acceleration > 0.0 &&
         std::isfinite(c.footprint_front) && c.footprint_front > 0.0 &&
         std::isfinite(c.footprint_rear) && c.footprint_rear >= 0.0 &&
         std::isfinite(c.footprint_half_width) && c.footprint_half_width > 0.0 &&
         std::isfinite(c.braking_linear) && c.braking_linear > 0.0 &&
         std::isfinite(c.braking_constant) && c.braking_constant > 0.0 &&
         std::isfinite(c.reaction_time_s) && c.reaction_time_s >= 0.0 &&
         std::isfinite(c.recovery_acceleration_gain) &&
         c.recovery_acceleration_gain > 0.0 &&
         std::isfinite(c.recovery_acceleration_floor) &&
         c.recovery_acceleration_floor > 0.0 &&
         std::isfinite(c.minimum_pose_step) && c.minimum_pose_step > 0.0 &&
         std::isfinite(c.direction_projection_threshold) &&
         c.direction_projection_threshold > 0.0 && c.direction_projection_threshold < 1.0;
}

}  // namespace

uint64_t pathIdentity(const nav_msgs::msg::Path & path)
{
  uint64_t hash = kFnvOffset;
  hashBytes(hash, path.header.frame_id.data(), path.header.frame_id.size());
  const uint64_t count = path.poses.size();
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    const auto byte = static_cast<unsigned char>((count >> shift) & 0xffu);
    hashBytes(hash, &byte, sizeof(byte));
  }
  for (const auto & stamped_pose : path.poses) {
    const auto & p = stamped_pose.pose.position;
    const auto & q = stamped_pose.pose.orientation;
    hashDouble(hash, p.x);
    hashDouble(hash, p.y);
    hashDouble(hash, p.z);
    hashDouble(hash, q.x);
    hashDouble(hash, q.y);
    hashDouble(hash, q.z);
    hashDouble(hash, q.w);
  }
  return hash;
}

std::vector<double> costmapClearance(
  const nav_msgs::msg::Path & path,
  nav2_costmap_2d::Costmap2D & costmap,
  double footprint_front, double footprint_rear, double footprint_half_width)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
  const unsigned int width = costmap.getSizeInCellsX();
  const unsigned int height = costmap.getSizeInCellsY();
  if (width == 0u || height == 0u || !std::isfinite(footprint_front) ||
    !std::isfinite(footprint_rear) || !std::isfinite(footprint_half_width))
  {
    return {};
  }
  const double far = static_cast<double>(width + height) * (width + height);
  std::vector<double> grid(width * height, far);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const auto cost = costmap.getCost(x, y);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) {
        grid[y * width + x] = 0.0;
      }
    }
  }

  std::vector<double> input(std::max(width, height));
  std::vector<double> output(input.size());
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      input[x] = grid[y * width + x];
    }
    distanceTransform1D(
      std::vector<double>(input.begin(), input.begin() + width), output);
    for (unsigned int x = 0; x < width; ++x) {
      grid[y * width + x] = output[x];
    }
  }
  for (unsigned int x = 0; x < width; ++x) {
    for (unsigned int y = 0; y < height; ++y) {
      input[y] = grid[y * width + x];
    }
    distanceTransform1D(
      std::vector<double>(input.begin(), input.begin() + height), output);
    for (unsigned int y = 0; y < height; ++y) {
      grid[y * width + x] = output[y];
    }
  }

  std::vector<double> clearance(path.poses.size(), 0.0);
  const double resolution = costmap.getResolution();
  const double cell_radius = resolution * std::sqrt(0.5);
  const std::vector<std::pair<double, double>> boundary = {
    {footprint_front, footprint_half_width}, {footprint_front, 0.0},
    {footprint_front, -footprint_half_width}, {0.5 * (footprint_front - footprint_rear), -footprint_half_width},
    {-footprint_rear, -footprint_half_width}, {-footprint_rear, 0.0},
    {-footprint_rear, footprint_half_width}, {0.5 * (footprint_front - footprint_rear), footprint_half_width}};
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double heading = yaw(path.poses[i].pose.orientation);
    const double cos_heading = std::cos(heading);
    const double sin_heading = std::sin(heading);
    double margin = std::numeric_limits<double>::infinity();
    for (const auto & sample : boundary) {
      const double wx = p.x + cos_heading * sample.first - sin_heading * sample.second;
      const double wy = p.y + sin_heading * sample.first + cos_heading * sample.second;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap.worldToMap(wx, wy, mx, my)) {
        margin = 0.0;
        break;
      }
      margin = std::min(margin, std::sqrt(grid[my * width + mx]) * resolution);
    }
    clearance[i] = std::max(0.0, margin - cell_radius);
  }
  return clearance;
}

bool validConfig(const ProfileConfig & config)
{
  return validConfigImpl(config);
}

double clearanceSpeed(
  double clearance, double preset_ceiling, const ProfileConfig & config)
{
  const double preset = std::max(preset_ceiling, config.minimum_traversal_speed);
  const double bottom = std::clamp(config.minimum_traversal_speed, 0.0, preset);
  const double measured = std::isfinite(clearance) ? clearance : 0.0;
  const double x = std::clamp(
    (measured - config.tight_clearance) /
    (config.open_clearance - config.tight_clearance), 0.0, 1.0);
  double shaped = x;
  if (config.clearance_curve_family >= 1.0) {
    shaped = std::pow(x, config.clearance_curve_shape);
  }
  if (config.clearance_curve_family == 2.0) {
    shaped = shaped * shaped * (3.0 - 2.0 * shaped);
  }
  return bottom + (preset - bottom) * shaped;
}

runner_interfaces::msg::PathSpeedProfile makeProfile(
  const nav_msgs::msg::Path & path, const std::vector<double> & clearance,
  double preset_ceiling, const ProfileConfig & requested_config)
{
  using Point = runner_interfaces::msg::PathSpeedProfilePoint;
  ProfileConfig config = validConfigImpl(requested_config) ? requested_config : ProfileConfig{};
  if (!std::isfinite(preset_ceiling) || preset_ceiling < config.minimum_traversal_speed) {
    preset_ceiling = config.minimum_traversal_speed;
  }

  runner_interfaces::msg::PathSpeedProfile profile;
  profile.header.frame_id = path.header.frame_id;
  profile.committed_path_stamp = path.header.stamp;
  profile.path_hash = pathIdentity(path);
  profile.pose_count = static_cast<uint32_t>(path.poses.size());
  profile.preset_ceiling_mps = preset_ceiling;
  profile.creep_speed_mps = clearanceSpeed(0.0, preset_ceiling, config);
  if (path.poses.empty()) {
    return profile;
  }

  const auto s = arclengths(path);
  profile.goal_arclength_m = s.back();
  profile.points.resize(path.poses.size());
  std::vector<double> ceiling(path.poses.size(), preset_ceiling);
  std::vector<uint8_t> state(path.poses.size(), Point::CUSP_NONE);

  // Boundary-aware windows expand to the opposite side near either endpoint.
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    std::size_t begin = i;
    std::size_t end = i;
    while (s[end] - s[begin] < config.curvature_window &&
      (begin > 0u || end + 1u < path.poses.size()))
    {
      const double left_span = begin > 0u ? s[i] - s[begin - 1u] :
        std::numeric_limits<double>::infinity();
      const double right_span = end + 1u < path.poses.size() ? s[end + 1u] - s[i] :
        std::numeric_limits<double>::infinity();
      if (left_span <= right_span) {
        --begin;
      } else {
        ++end;
      }
    }
    const double span = s[end] - s[begin];
    if (span >= config.minimum_pose_step) {
      const double curvature = std::abs(angleDifference(
          yaw(path.poses[end].pose.orientation),
          yaw(path.poses[begin].pose.orientation))) / span;
      if (curvature > 1e-9) {
        ceiling[i] = std::min(
          ceiling[i], std::sqrt(config.max_lateral_acceleration / curvature));
      }
    }
  }

  const auto clearance_speed = clearanceSpeeds(
    clearance, path.poses.size(), preset_ceiling, config);
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    ceiling[i] = std::min(ceiling[i], clearance_speed[i]);
    ceiling[i] = std::clamp(ceiling[i], profile.creep_speed_mps, preset_ceiling);
  }

  std::vector<int> direction(path.poses.size() > 1u ? path.poses.size() - 1u : 0u, 0);
  for (std::size_t i = 0; i < direction.size(); ++i) {
    const auto & before = path.poses[i];
    const auto & after = path.poses[i + 1u];
    const double dx = after.pose.position.x - before.pose.position.x;
    const double dy = after.pose.position.y - before.pose.position.y;
    const double length = std::hypot(dx, dy);
    if (length < config.minimum_pose_step) {
      continue;
    }
    const double projection =
      (dx * std::cos(yaw(before.pose.orientation)) +
      dy * std::sin(yaw(before.pose.orientation))) / length;
    if (std::abs(projection) >= config.direction_projection_threshold) {
      direction[i] = projection > 0.0 ? 1 : -1;
    }
  }
  for (std::size_t i = 1; i < direction.size(); ++i) {
    if (direction[i] == 0) {
      direction[i] = direction[i - 1u];
    }
  }
  for (std::size_t i = direction.size(); i-- > 1u; ) {
    if (direction[i - 1u] == 0) {
      direction[i - 1u] = direction[i];
    }
  }
  for (std::size_t i = 1; i < direction.size(); ++i) {
    if (direction[i - 1u] != 0 && direction[i] != 0 &&
      direction[i - 1u] != direction[i])
    {
      ceiling[i] = 0.0;
      state[i] = Point::CUSP_STOP;
      if (i > 0u && state[i - 1u] == Point::CUSP_NONE) {
        state[i - 1u] = Point::CUSP_APPROACH;
      }
      if (i + 1u < state.size()) {
        state[i + 1u] = Point::CUSP_DEPARTURE;
      }
    }
  }
  ceiling.back() = 0.0;

  // One backwards O(n) anticipation pass. A decreasing future raw target
  // becomes active for target*approach_time_s metres before its entry.
  if (config.approach_time_s > 0.0) {
    double target = preset_ceiling;
    double remaining = 0.0;
    for (std::size_t i = ceiling.size() - 1u; i > 0u; --i) {
      if (ceiling[i] < target) {
        target = ceiling[i];
        remaining = std::max(remaining, target * config.approach_time_s);
      }
      if (remaining > 0.0 && target > 0.0) {
        ceiling[i - 1u] = std::min(ceiling[i - 1u], target);
        remaining -= s[i] - s[i - 1u];
      }
      if (remaining <= 0.0) {
        // No active plateau: return to the open-corridor baseline so the
        // next pose's raw ceiling is compared against an unconstrained
        // reference, not pre-synced to itself (which would silently
        // defeat the trigger above for every following pose).
        target = preset_ceiling;
      }
    }
  }

  // Forward pass: release back toward the raw ceiling with the independent
  // speed-dependent recovery model. std::min preserves that raw authority.
  for (std::size_t i = 1; i < ceiling.size(); ++i) {
    ceiling[i] = std::min(
      ceiling[i], reachableSpeedByAcceleration(
        ceiling[i - 1u], s[i] - s[i - 1u], preset_ceiling,
        config.recovery_acceleration_gain, config.recovery_acceleration_floor));
  }
  // Backward pass: the adopted a(v)=1.6v+0.27 braking model integrated into
  // a stopping-distance potential, plus reaction-time travel. The augmented
  // potential remains transitive across poses and starts each downstream
  // speed reduction earlier without changing the braking calibration.
  for (std::size_t i = ceiling.size() - 1u; i > 0u; --i) {
    ceiling[i - 1u] = std::min(
      ceiling[i - 1u], reachableSpeed(ceiling[i], s[i] - s[i - 1u],
      preset_ceiling, config));
  }

  for (std::size_t i = 0; i < ceiling.size(); ++i) {
    if (ceiling[i] > 0.0) {
      ceiling[i] = std::max(ceiling[i], profile.creep_speed_mps);
    }
    profile.points[i].arclength_m = s[i];
    profile.points[i].speed_ceiling_mps = ceiling[i];
    profile.points[i].cusp_state = state[i];
  }
  profile.points.back().speed_ceiling_mps = 0.0;
  return profile;
}

bool validateProfile(
  const runner_interfaces::msg::PathSpeedProfile & profile,
  const nav_msgs::msg::Path & path, std::string & reason)
{
  if (profile.pose_count != path.poses.size() || profile.points.size() != path.poses.size()) {
    reason = "pose_count_mismatch";
    return false;
  }
  if (profile.path_hash != pathIdentity(path)) {
    reason = "path_hash_mismatch";
    return false;
  }
  if (profile.committed_path_stamp.sec != path.header.stamp.sec ||
    profile.committed_path_stamp.nanosec != path.header.stamp.nanosec)
  {
    reason = "path_stamp_mismatch";
    return false;
  }
  if (profile.points.empty() || profile.points.front().arclength_m != 0.0 ||
    profile.header.frame_id != path.header.frame_id ||
    !std::isfinite(profile.goal_arclength_m) || profile.goal_arclength_m < 0.0 ||
    !std::isfinite(profile.creep_speed_mps) || profile.creep_speed_mps <= 0.0 ||
    !std::isfinite(profile.preset_ceiling_mps) ||
    profile.preset_ceiling_mps < profile.creep_speed_mps)
  {
    reason = "profile_bounds_invalid";
    return false;
  }
  const auto expected_arclength = arclengths(path);
  double prior = -1.0;
  for (std::size_t i = 0; i < profile.points.size(); ++i) {
    const auto & point = profile.points[i];
    const bool speed_not_finite = !std::isfinite(point.speed_ceiling_mps);
    const bool speed_negative = point.speed_ceiling_mps < 0.0;
    const bool speed_over_preset =
      point.speed_ceiling_mps > profile.preset_ceiling_mps + 1e-6;
    const bool speed_below_creep = point.speed_ceiling_mps > 0.0 &&
      point.speed_ceiling_mps + 1e-6 < profile.creep_speed_mps;
    const bool speed_invalid =
      speed_not_finite || speed_negative || speed_over_preset || speed_below_creep;
    if (!std::isfinite(point.arclength_m) || point.arclength_m < prior ||
      std::abs(point.arclength_m - expected_arclength[i]) > 1e-6 ||
      speed_invalid ||
      point.cusp_state > runner_interfaces::msg::PathSpeedProfilePoint::CUSP_DEPARTURE ||
      (point.cusp_state == runner_interfaces::msg::PathSpeedProfilePoint::CUSP_STOP &&
      point.speed_ceiling_mps != 0.0))
    {
      reason = "profile_point_invalid";
      return false;
    }
    prior = point.arclength_m;
  }
  if (std::abs(prior - profile.goal_arclength_m) > 1e-6 ||
    profile.points.back().speed_ceiling_mps != 0.0)
  {
    reason = "profile_goal_invalid";
    return false;
  }
  reason = "matched";
  return true;
}

double sampleCeiling(
  const runner_interfaces::msg::PathSpeedProfile & profile, double arclength)
{
  if (profile.points.empty() || !std::isfinite(arclength)) {
    return 0.0;
  }
  arclength = std::clamp(arclength, 0.0, profile.goal_arclength_m);
  const auto upper = std::upper_bound(
    profile.points.begin(), profile.points.end(), arclength,
    [](double value, const auto & point) {return value < point.arclength_m;});
  if (upper == profile.points.begin()) {
    return upper->speed_ceiling_mps;
  }
  if (upper == profile.points.end()) {
    return profile.points.back().speed_ceiling_mps;
  }
  const auto & after = *upper;
  const auto & before = *std::prev(upper);
  const double span = after.arclength_m - before.arclength_m;
  if (span <= 0.0) {
    return std::min(before.speed_ceiling_mps, after.speed_ceiling_mps);
  }
  if (before.speed_ceiling_mps == 0.0) {
    return arclength == before.arclength_m ? 0.0 : after.speed_ceiling_mps;
  }
  if (after.speed_ceiling_mps == 0.0) {
    return arclength == after.arclength_m ? 0.0 : before.speed_ceiling_mps;
  }
  const double ratio = (arclength - before.arclength_m) / span;
  return before.speed_ceiling_mps +
         ratio * (after.speed_ceiling_mps - before.speed_ceiling_mps);
}

}  // namespace runner_path_speed_profile
