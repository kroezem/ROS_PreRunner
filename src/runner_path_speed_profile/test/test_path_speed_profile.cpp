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

TEST(PathSpeedProfile, ClearanceStepsBetweenExactTiersWithNoRamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  // Spaced far enough apart that braking/recovery kinematics do not bind,
  // isolating the clearance tiering itself.
  for (int i = 0; i < 5; ++i) {
    appendPose(path, 2.0 * i);
  }
  runner_path_speed_profile::ProfileConfig config;
  // clearances: fully tight, just below open, fully open, just above tight
  const std::vector<double> clearance = {
    0.0, config.open_clearance - 0.01, 1.0, config.passable_clearance + 0.01, 1.0};
  const auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, config);
  EXPECT_DOUBLE_EQ(profile.points[0].speed_ceiling_mps, config.creep_speed);
  EXPECT_DOUBLE_EQ(profile.points[1].speed_ceiling_mps, config.caution_speed);
  EXPECT_DOUBLE_EQ(profile.points[3].speed_ceiling_mps, config.caution_speed);
  // No intermediate value between creep and caution, or caution and open,
  // is ever produced by clearance alone.
  for (const auto & point : profile.points) {
    const double v = point.speed_ceiling_mps;
    EXPECT_TRUE(
      v == 0.0 || v == config.creep_speed || v == config.caution_speed ||
      v > config.caution_speed);
  }
}

TEST(PathSpeedProfile, HysteresisAbsorbsAShortOpenSpikeInsideACautionCorridor)
{
  // Reproduces the bag-observed sawtooth precursor: a single noisy pose
  // reading "open" surrounded on both (equal) sides by "caution" must not
  // leak through as a momentary speed-up. Poses are spaced well clear of
  // the goal so end-of-path braking cannot also explain a lower reading.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 7; ++i) {
    appendPose(path, 0.2 * i);  // spike span (0.2 m) still < min_tier_run_length
  }
  runner_path_speed_profile::ProfileConfig config;
  const std::vector<double> clearance = {
    0.20, 0.20, 1.0, 0.20, 0.20, 0.20, 0.20};
  const auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, config);
  EXPECT_DOUBLE_EQ(profile.points[2].speed_ceiling_mps, config.caution_speed);
}

TEST(PathSpeedProfile, HysteresisNeverErasesAGenuineIsolatedCreepHazard)
{
  // A real single-pose clearance drop inside an otherwise open corridor
  // must survive hysteresis exactly, even though it is a short run: only
  // less-restrictive short runs get smoothed away, never the reverse.
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 5; ++i) {
    appendPose(path, 0.5 * i);
  }
  runner_path_speed_profile::ProfileConfig config;
  const std::vector<double> clearance = {1.0, 1.0, 0.0, 1.0, 1.0};
  const auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, config);
  EXPECT_DOUBLE_EQ(profile.points[2].speed_ceiling_mps, config.creep_speed);
}

TEST(PathSpeedProfile, ForwardRecoveryIsFasterThanBackwardBraking)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 6; ++i) {
    appendPose(path, 0.3 * i);
  }
  runner_path_speed_profile::ProfileConfig config;
  config.recovery_acceleration = 5.0;  // deliberately much faster than braking
  // Force a full creep zone at the very start (post-constraint recovery)
  // and a symmetric approach to a stop at the very end (pre-stop braking)
  // over an equal number of poses/distance, then compare how far the
  // ceiling has climbed vs. how far it has fallen.
  const std::vector<double> clearance = {0.0, 1.0, 1.0, 1.0, 1.0, 0.0};
  const auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, config);
  const double recovered_by_pose_2 = profile.points[2].speed_ceiling_mps;
  const double braked_by_pose_3 = profile.points[3].speed_ceiling_mps;
  // Recovery from creep_speed at pose 1 should climb faster over the same
  // 0.3 m step than braking descends approaching the symmetric endpoint.
  EXPECT_GT(recovered_by_pose_2 - config.creep_speed, 0.0);
  EXPECT_GT(recovered_by_pose_2, braked_by_pose_3);
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
