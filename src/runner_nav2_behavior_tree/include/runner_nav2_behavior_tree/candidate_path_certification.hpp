// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#ifndef RUNNER_NAV2_BEHAVIOR_TREE__CANDIDATE_PATH_CERTIFICATION_HPP_
#define RUNNER_NAV2_BEHAVIOR_TREE__CANDIDATE_PATH_CERTIFICATION_HPP_

#include <string>

#include "nav_msgs/msg/path.hpp"

namespace runner_nav2_behavior_tree
{

struct CandidatePathCertificationConfig
{
  double physical_curvature_limit{2.1236};
  double curvature_margin{0.95};
  double curvature_window{0.15};
  double minimum_reversal_segment{0.10};
  double minimum_pose_step{0.01};
  double direction_projection_threshold{0.5};
};

struct CandidatePathCertificationResult
{
  bool accepted{false};
  bool contains_reverse{false};
  std::string reason;
};

/** Deterministic geometry portion of Runner candidate certification.
 *
 * The result intentionally leaves room for Stage D2 to attach a route
 * execution profile without changing the certification boundary.
 */
CandidatePathCertificationResult certifyCandidateGeometry(
  const nav_msgs::msg::Path & path,
  const CandidatePathCertificationConfig & config);

}  // namespace runner_nav2_behavior_tree

#endif  // RUNNER_NAV2_BEHAVIOR_TREE__CANDIDATE_PATH_CERTIFICATION_HPP_
