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
