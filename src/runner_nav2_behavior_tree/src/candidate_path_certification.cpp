// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include "runner_nav2_behavior_tree/candidate_path_certification.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace runner_nav2_behavior_tree
{

namespace
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const double siny_cosp = 2.0 * (
    orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy_cosp = 1.0 - 2.0 * (
    orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double poseDistance(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return std::hypot(
    rhs.pose.position.x - lhs.pose.position.x,
    rhs.pose.position.y - lhs.pose.position.y);
}

bool validConfig(const CandidatePathCertificationConfig & config)
{
  return std::isfinite(config.physical_curvature_limit) &&
         config.physical_curvature_limit > 0.0 &&
         std::isfinite(config.curvature_margin) &&
         config.curvature_margin > 0.0 && config.curvature_margin <= 1.0 &&
         std::isfinite(config.curvature_window) && config.curvature_window > 0.0 &&
         std::isfinite(config.minimum_reversal_segment) &&
         config.minimum_reversal_segment > 0.0 &&
         std::isfinite(config.minimum_pose_step) && config.minimum_pose_step > 0.0 &&
         std::isfinite(config.direction_projection_threshold) &&
         config.direction_projection_threshold > 0.0 &&
         config.direction_projection_threshold < 1.0;
}

}  // namespace

CandidatePathCertificationResult certifyCandidateGeometry(
  const nav_msgs::msg::Path & path,
  const CandidatePathCertificationConfig & config)
{
  CandidatePathCertificationResult result;
  if (!validConfig(config)) {
    result.reason = "certification_config_invalid";
    return result;
  }
  if (path.poses.empty()) {
    result.reason = "candidate_empty";
    return result;
  }
  for (const auto & pose : path.poses) {
    const auto & position = pose.pose.position;
    const auto & orientation = pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      result.reason = "candidate_non_finite";
      return result;
    }
  }
  if (path.poses.size() == 1u) {
    result.accepted = true;
    result.reason = "certified_forward";
    return result;
  }

  // Curvature is measured over a short arc-length window rather than one
  // discretized pose edge. This prevents normal Smac angle-bin and pose noise
  // from looking like an impossible instantaneous steering demand.
  const double maximum_curvature =
    config.physical_curvature_limit * config.curvature_margin;
  for (std::size_t start = 0; start + 1 < path.poses.size(); ++start) {
    double arc_length = 0.0;
    std::size_t end = start;
    while (end + 1 < path.poses.size() && arc_length < config.curvature_window) {
      arc_length += poseDistance(path.poses[end], path.poses[end + 1]);
      ++end;
    }
    if (arc_length < config.minimum_pose_step) {
      continue;
    }
    const double heading_change = std::abs(normalizeAngle(
      yaw(path.poses[end].pose.orientation) -
      yaw(path.poses[start].pose.orientation)));
    if (heading_change / arc_length > maximum_curvature + 1e-6) {
      result.reason = "curvature_exceeds_runner_limit";
      return result;
    }
  }

  struct DirectedEdge
  {
    int direction;
    double length;
  };
  std::vector<DirectedEdge> edges;
  edges.reserve(path.poses.size() - 1u);
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & before = path.poses[index - 1];
    const auto & after = path.poses[index];
    const double dx = after.pose.position.x - before.pose.position.x;
    const double dy = after.pose.position.y - before.pose.position.y;
    const double length = std::hypot(dx, dy);
    if (length < config.minimum_pose_step) {
      continue;
    }
    const double heading = yaw(before.pose.orientation);
    const double projection =
      (dx * std::cos(heading) + dy * std::sin(heading)) / length;
    if (std::abs(projection) < config.direction_projection_threshold) {
      // One ambiguous edge is expected around a discretized cusp. Fold it
      // into its neighbour instead of manufacturing a direction change.
      if (!edges.empty()) {
        edges.back().length += length;
      }
      continue;
    }
    const int direction = projection > 0.0 ? 1 : -1;
    if (!edges.empty() && edges.back().direction == direction) {
      edges.back().length += length;
    } else {
      edges.push_back({direction, length});
    }
  }

  if (edges.empty()) {
    result.reason = "candidate_direction_unclassifiable";
    return result;
  }
  result.contains_reverse = std::any_of(
    edges.begin(), edges.end(), [](const DirectedEdge & edge) {return edge.direction < 0;});

  if (edges.size() > 1u || result.contains_reverse) {
    for (const auto & edge : edges) {
      if (edge.length + 1e-6 < config.minimum_reversal_segment) {
        result.reason = "reversal_segment_too_short";
        return result;
      }
    }
  }

  result.accepted = true;
  result.reason = result.contains_reverse ? "certified_reverse" : "certified_forward";
  return result;
}

}  // namespace runner_nav2_behavior_tree
