// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include <cmath>

#include "gtest/gtest.h"
#include "runner_nav2_behavior_tree/candidate_path_certification.hpp"

namespace
{

void appendPose(nav_msgs::msg::Path & path, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  path.poses.push_back(pose);
}

}  // namespace

TEST(CandidatePathCertification, RejectsImpossibleCurvature)
{
  nav_msgs::msg::Path path;
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.05, 0.0, 0.4);
  appendPose(path, 0.10, 0.02, 0.8);
  appendPose(path, 0.14, 0.05, 1.2);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "curvature_exceeds_runner_limit");
}

TEST(CandidatePathCertification, RejectsPathologicalShortReversalSegment)
{
  nav_msgs::msg::Path path;
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.20, 0.0, 0.0);
  appendPose(path, 0.15, 0.0, 0.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "reversal_segment_too_short");
}

TEST(CandidatePathCertification, AcceptsMeaningfulReverseManeuver)
{
  nav_msgs::msg::Path path;
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.20, 0.0, 0.0);
  appendPose(path, 0.40, 0.0, 0.0);
  appendPose(path, 0.25, 0.0, 0.0);
  appendPose(path, 0.10, 0.0, 0.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.contains_reverse);
  EXPECT_EQ(result.reason, "certified_reverse");
}

TEST(CandidatePathCertification, RejectsShortInitialReverseManeuver)
{
  nav_msgs::msg::Path path;
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, -0.05, 0.0, 0.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "reversal_segment_too_short");
}

TEST(CandidatePathCertification, AcceptsQuantizedTerminalHeadingCorrection)
{
  // Reproduces the D1 false-rejection: a dead-straight path (five 0.0712 m
  // edges, matching a real Smac Hybrid-A* output spacing) with a single
  // 10-degree heading correction concentrated entirely in the final edge --
  // the signature of Smac snapping to the goal's discretized heading bin.
  // The single terminal edge alone reads as 0.1745 rad / 0.0712 m
  // = 2.45 rad/m, over the 2.1236 * 0.95 = 2.017 rad/m limit, even though
  // the same 10-degree turn spread over the intended ~0.15 m window is a
  // gentle ~0.82-1.2 rad/m -- well within both the certification margin and
  // Smac's own 0.60 m minimum-turning-radius (1.667 rad/m) guarantee.
  nav_msgs::msg::Path path;
  appendPose(path, 0.0000, 0.0, 0.0);
  appendPose(path, 0.0712, 0.0, 0.0);
  appendPose(path, 0.1424, 0.0, 0.0);
  appendPose(path, 0.2136, 0.0, 0.0);
  appendPose(path, 0.2848, 0.0, 0.0);
  appendPose(path, 0.3560, 0.0, 10.0 * M_PI / 180.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, "certified_forward");
}

TEST(CandidatePathCertification, RejectsGenuinelyExcessiveTerminalCurvature)
{
  // A boundary-adjacent turn that is NOT a single quantization artifact --
  // every one of the last few edges keeps turning sharply, so widening the
  // window backward cannot dilute it below the limit the way it legitimately
  // does for a single terminal snap. This must still be rejected after the
  // boundary-aware fix.
  nav_msgs::msg::Path path;
  appendPose(path, 0.00, 0.00, 0.0);
  appendPose(path, 0.05, 0.00, 0.0);
  appendPose(path, 0.10, 0.01, 30.0 * M_PI / 180.0);
  appendPose(path, 0.15, 0.03, 60.0 * M_PI / 180.0);
  appendPose(path, 0.19, 0.06, 90.0 * M_PI / 180.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "curvature_exceeds_runner_limit");
}

TEST(CandidatePathCertification, InteriorExcessiveCurvatureStillRejected)
{
  // The same magnitude of turning as RejectsImpossibleCurvature, but placed
  // in the middle of a longer path with plenty of poses on both sides.
  // Interior windows always reach curvature_window by extending forward
  // alone, so the boundary-aware backward extension must never engage here;
  // this proves interior behavior is unchanged by the fix.
  nav_msgs::msg::Path path;
  appendPose(path, -0.20, 0.0, 0.0);
  appendPose(path, -0.15, 0.0, 0.0);
  appendPose(path, -0.10, 0.0, 0.0);
  appendPose(path, -0.05, 0.0, 0.0);
  appendPose(path, 0.00, 0.0, 0.0);
  appendPose(path, 0.05, 0.0, 0.4);
  appendPose(path, 0.10, 0.02, 0.8);
  appendPose(path, 0.14, 0.05, 1.2);
  appendPose(path, 0.19, 0.05, 1.2);
  appendPose(path, 0.24, 0.05, 1.2);
  appendPose(path, 0.29, 0.05, 1.2);
  appendPose(path, 0.34, 0.05, 1.2);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "curvature_exceeds_runner_limit");
}

TEST(CandidatePathCertification, IgnoresPoseNoiseAtDirectionChange)
{
  nav_msgs::msg::Path path;
  appendPose(path, 0.0, 0.0, 0.0);
  appendPose(path, 0.20, 0.0, 0.0);
  appendPose(path, 0.199, 0.001, 0.0);
  appendPose(path, 0.05, 0.0, 0.0);

  const auto result = runner_nav2_behavior_tree::certifyCandidateGeometry(path, {});
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.contains_reverse);
}
