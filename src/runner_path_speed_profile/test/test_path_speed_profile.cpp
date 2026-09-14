// Copyright 2026 matti
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "runner_path_speed_profile/path_speed_profile.hpp"

namespace
{

void appendPose(nav_msgs::msg::Path & path, double x, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  path.poses.push_back(pose);
}

nav_msgs::msg::Path cuspPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 12;
  for (int i = 0; i <= 10; ++i) {
    appendPose(path, 0.1 * i);
  }
  for (int i = 9; i >= 0; --i) {
    appendPose(path, 0.1 * i);
  }
  return path;
}

}  // namespace

TEST(PathSpeedProfile, CoversPathAndBrakesIntoCuspAndGoal)
{
  const auto path = cuspPath();
  runner_path_speed_profile::ProfileConfig config;
  const auto profile = runner_path_speed_profile::makeProfile(
    path, std::vector<double>(path.poses.size(), 1.0), 1.5, config);

  ASSERT_EQ(profile.points.size(), path.poses.size());
  EXPECT_DOUBLE_EQ(profile.points.front().arclength_m, 0.0);
  EXPECT_DOUBLE_EQ(profile.points.back().arclength_m, profile.goal_arclength_m);
  EXPECT_DOUBLE_EQ(profile.points.back().speed_ceiling_mps, 0.0);
  for (std::size_t i = 1; i < profile.points.size(); ++i) {
    EXPECT_GE(profile.points[i].arclength_m, profile.points[i - 1].arclength_m);
  }
  for (const auto & point : profile.points) {
    EXPECT_TRUE(point.speed_ceiling_mps == 0.0 ||
      point.speed_ceiling_mps >= config.creep_speed);
  }

  const auto cusp = std::find_if(
    profile.points.begin(), profile.points.end(), [](const auto & point) {
      return point.cusp_state == point.CUSP_STOP;
    });
  ASSERT_NE(cusp, profile.points.end());
  EXPECT_DOUBLE_EQ(cusp->speed_ceiling_mps, 0.0);
  ASSERT_GE(std::distance(profile.points.begin(), cusp), 2);
  EXPECT_LT(std::prev(cusp, 2)->speed_ceiling_mps, profile.preset_ceiling_mps);
  EXPECT_LT(std::prev(profile.points.end(), 2)->speed_ceiling_mps, profile.preset_ceiling_mps);
}

TEST(PathSpeedProfile, IdentityRejectsStaleGeometryAndStamp)
{
  auto path = cuspPath();
  const auto profile = runner_path_speed_profile::makeProfile(
    path, std::vector<double>(path.poses.size(), 1.0), 1.0, {});
  std::string reason;
  EXPECT_TRUE(runner_path_speed_profile::validateProfile(profile, path, reason));

  path.poses[3].pose.position.y = 0.01;
  EXPECT_FALSE(runner_path_speed_profile::validateProfile(profile, path, reason));
  EXPECT_EQ(reason, "path_hash_mismatch");
  path.poses[3].pose.position.y = 0.0;
  path.header.stamp.nanosec = 1;
  EXPECT_FALSE(runner_path_speed_profile::validateProfile(profile, path, reason));
  EXPECT_EQ(reason, "path_stamp_mismatch");

  auto malformed = profile;
  malformed.committed_path_stamp = path.header.stamp;
  malformed.points[2].speed_ceiling_mps = 0.10;
  EXPECT_FALSE(runner_path_speed_profile::validateProfile(malformed, path, reason));
  EXPECT_EQ(reason, "profile_point_invalid");
}

TEST(PathSpeedProfile, ClearanceShapesWithoutCreatingNonStopZero)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  appendPose(path, 0.0);
  appendPose(path, 0.5);
  appendPose(path, 1.0);
  appendPose(path, 1.5);
  const auto profile = runner_path_speed_profile::makeProfile(
    path, {1.0, 0.0, 1.0, 1.0}, 1.0, {});
  EXPECT_DOUBLE_EQ(profile.points[1].speed_ceiling_mps, 0.25);
  EXPECT_GT(profile.points[0].speed_ceiling_mps, 0.25);
  EXPECT_DOUBLE_EQ(profile.points.back().speed_ceiling_mps, 0.0);
}

TEST(PathSpeedProfile, SamplesBelowRppNormalFloorAndAtExactZero)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  appendPose(path, 0.0);
  appendPose(path, 0.2);
  appendPose(path, 0.4);
  const auto profile = runner_path_speed_profile::makeProfile(
    path, std::vector<double>(3, 0.0), 0.45, {});
  EXPECT_DOUBLE_EQ(runner_path_speed_profile::sampleCeiling(profile, 0.0), 0.25);
  EXPECT_LT(runner_path_speed_profile::sampleCeiling(profile, 0.3), 0.30);
  EXPECT_DOUBLE_EQ(runner_path_speed_profile::sampleCeiling(profile, 0.4), 0.0);
}

TEST(PathSpeedProfile, ReadsLiveCostmapClearance)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, 0.0, 0.0, 0);
  costmap.setCost(10, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav_msgs::msg::Path path;
  appendPose(path, 1.05);
  path.poses.back().pose.position.y = 1.05;
  appendPose(path, 0.05);
  path.poses.back().pose.position.y = 0.05;
  const auto clearance = runner_path_speed_profile::costmapClearance(path, costmap, 0.0);
  ASSERT_EQ(clearance.size(), 2u);
  EXPECT_DOUBLE_EQ(clearance.front(), 0.0);
  EXPECT_GT(clearance.back(), 1.0);
}
