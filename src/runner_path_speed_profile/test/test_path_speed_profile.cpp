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
      point.speed_ceiling_mps >= profile.creep_speed_mps);
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
  EXPECT_DOUBLE_EQ(profile.points[1].speed_ceiling_mps, profile.creep_speed_mps);
  EXPECT_GT(profile.points[0].speed_ceiling_mps, profile.creep_speed_mps);
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
  EXPECT_DOUBLE_EQ(
    runner_path_speed_profile::sampleCeiling(profile, 0.0), profile.creep_speed_mps);
  EXPECT_LT(runner_path_speed_profile::sampleCeiling(profile, 0.3), 0.45);
  EXPECT_DOUBLE_EQ(runner_path_speed_profile::sampleCeiling(profile, 0.4), 0.0);
}

TEST(PathSpeedProfile, ClearanceLawIsContinuousMonotonicAndBounded)
{
  runner_path_speed_profile::ProfileConfig config;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  appendPose(path, 0.0);
  appendPose(path, 100.0);  // Keep goal braking from binding at pose zero.

  const double preset = 1.5;
  const auto speed_at = [&](double clearance) {
      return runner_path_speed_profile::makeProfile(
        path, {clearance, clearance}, preset, config).points.front().speed_ceiling_mps;
    };
  const double bottom = runner_path_speed_profile::clearanceSpeed(0.0, preset, config);
  EXPECT_DOUBLE_EQ(speed_at(config.tight_clearance), bottom);
  EXPECT_DOUBLE_EQ(speed_at(config.open_clearance), preset);
  EXPECT_LT(speed_at(0.10), speed_at(0.30));
  EXPECT_LT(speed_at(0.30), speed_at(0.60));
  EXPECT_LE(speed_at(0.60), preset);
}

TEST(PathSpeedProfile, ClearanceLawDependsOnPresetAndPreservesExactStops)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  appendPose(path, 0.0);
  appendPose(path, 100.0);
  runner_path_speed_profile::ProfileConfig config;
  const std::vector<double> clearance(2, config.open_clearance);
  const auto timid = runner_path_speed_profile::makeProfile(path, clearance, 0.45, config);
  const auto insane = runner_path_speed_profile::makeProfile(path, clearance, 1.5, config);
  EXPECT_DOUBLE_EQ(timid.points.front().speed_ceiling_mps, 0.45);
  EXPECT_DOUBLE_EQ(insane.points.front().speed_ceiling_mps, 1.5);
  EXPECT_GT(insane.points.front().speed_ceiling_mps, timid.points.front().speed_ceiling_mps);
  EXPECT_DOUBLE_EQ(timid.points.back().speed_ceiling_mps, 0.0);
  EXPECT_DOUBLE_EQ(insane.points.back().speed_ceiling_mps, 0.0);
}

TEST(PathSpeedProfile, RecoveryAccelerationIncreasesWithSpeed)
{
  runner_path_speed_profile::ProfileConfig config;
  const auto recovery_after_step = [&](double starting_speed) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      for (int i = 0; i <= 100; ++i) {
        appendPose(path, 0.1 * i);
      }
      const double preset = 1.5;
      config.clearance_curve_family = 0.0;
      const double bottom = runner_path_speed_profile::clearanceSpeed(0.0, preset, config);
      const double initial_clearance = config.tight_clearance +
        (config.open_clearance - config.tight_clearance) *
        (starting_speed - bottom) / (preset - bottom);
      std::vector<double> clearance(path.poses.size(), 100.0);
      clearance.front() = initial_clearance;
      return runner_path_speed_profile::makeProfile(
        path, clearance, preset, config).points[1].speed_ceiling_mps;
    };

  const double low_start = runner_path_speed_profile::clearanceSpeed(0.0, 1.5, config);
  const double medium_start = 0.8;
  const double low_recovered = recovery_after_step(low_start);
  const double medium_recovered = recovery_after_step(medium_start);
  const double low_effective_acceleration =
    (low_recovered * low_recovered - low_start * low_start) / 0.2;
  const double medium_effective_acceleration =
    (medium_recovered * medium_recovered - medium_start * medium_start) / 0.2;

  EXPECT_NEAR(
    config.recovery_acceleration_gain * 0.25 +
    config.recovery_acceleration_floor, 1.0, 1e-12);
  EXPECT_GT(medium_effective_acceleration, low_effective_acceleration);
  EXPECT_GT(medium_effective_acceleration, 1.0);
}

TEST(PathSpeedProfile, RecoveryNeverExceedsRawCeiling)
{
  runner_path_speed_profile::ProfileConfig config;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 100; ++i) {
    appendPose(path, 0.1 * i);
  }
  const double preset = 1.5;
  const double raw_target = 0.55;
  config.clearance_curve_family = 0.0;
  const double bottom = runner_path_speed_profile::clearanceSpeed(0.0, preset, config);
  const double target_clearance = config.tight_clearance +
    (config.open_clearance - config.tight_clearance) *
    (raw_target - bottom) / (preset - bottom);
  std::vector<double> clearance(path.poses.size(), 100.0);
  clearance.front() = 0.0;
  clearance[1] = target_clearance;

  const auto profile = runner_path_speed_profile::makeProfile(
    path, clearance, preset, config);
  EXPECT_LE(profile.points[1].speed_ceiling_mps, raw_target);
  EXPECT_NEAR(profile.points[1].speed_ceiling_mps, raw_target, 1e-12);
}

TEST(PathSpeedProfile, ReactionTimeStartsBackwardRestrictionEarlier)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
    appendPose(path, 0.1 * i);
  }
  runner_path_speed_profile::ProfileConfig instantaneous;
  instantaneous.reaction_time_s = 0.0;
  runner_path_speed_profile::ProfileConfig anticipated = instantaneous;
  anticipated.reaction_time_s = 0.40;
  const std::vector<double> clearance(path.poses.size(), 100.0);
  const auto without_margin = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, instantaneous);
  const auto with_margin = runner_path_speed_profile::makeProfile(
    path, clearance, 2.0, anticipated);

  EXPECT_LT(
    with_margin.points[10].speed_ceiling_mps,
    without_margin.points[10].speed_ceiling_mps);
  EXPECT_DOUBLE_EQ(with_margin.points.back().speed_ceiling_mps, 0.0);
  EXPECT_GE(with_margin.points[10].speed_ceiling_mps, with_margin.creep_speed_mps);
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
  const auto clearance = runner_path_speed_profile::costmapClearance(
    path, costmap, 0.01, 0.0, 0.01);
  ASSERT_EQ(clearance.size(), 2u);
  EXPECT_DOUBLE_EQ(clearance.front(), 0.0);
  EXPECT_GT(clearance.back(), 1.0);
}

TEST(PathSpeedProfile, CurveFamiliesEndpointsShapesAndScaling)
{
  runner_path_speed_profile::ProfileConfig config;
  const double preset = 1.0;
  for (double family : {0.0, 1.0, 2.0}) {
    config.clearance_curve_family = family;
    EXPECT_DOUBLE_EQ(
      runner_path_speed_profile::clearanceSpeed(config.open_clearance, preset, config),
      preset);
  }
  config.clearance_curve_family = 1.0;
  config.clearance_curve_shape = 2.0;
  const double midpoint = 0.5 * (config.tight_clearance + config.open_clearance);
  EXPECT_LT(
    runner_path_speed_profile::clearanceSpeed(midpoint, preset, config),
    runner_path_speed_profile::clearanceSpeed(
      midpoint, preset, runner_path_speed_profile::ProfileConfig{}));
  EXPECT_DOUBLE_EQ(
    runner_path_speed_profile::clearanceSpeed(0.0, 0.45, config),
    config.minimum_traversal_speed);
}

TEST(PathSpeedProfile, RejectsInvalidThresholds)
{
  runner_path_speed_profile::ProfileConfig config;
  config.open_clearance = config.tight_clearance;
  EXPECT_FALSE(runner_path_speed_profile::validConfig(config));
  config.open_clearance = 0.7;
  config.clearance_curve_family = 3.0;
  EXPECT_FALSE(runner_path_speed_profile::validConfig(config));
}

TEST(PathSpeedProfile, ApproachTimeZeroIsNoOpButNonzeroPlateausEarlier)
{
  // Long and flat enough that goal-approach braking (from the forced
  // zero at the last pose) has fully recovered well before the mid-path
  // clearance dip, isolating the anticipation pass under test.
  nav_msgs::msg::Path path;
  for (int i = 0; i < 200; ++i) appendPose(path, 0.1 * i);
  std::vector<double> clearance(path.poses.size(), 1.0);
  clearance[100] = 0.0;

  runner_path_speed_profile::ProfileConfig zero;
  zero.approach_time_s = 0.0;
  runner_path_speed_profile::ProfileConfig zero_again = zero;

  // Two independently-built zero-approach-time configs must match exactly:
  // the anticipation pass is skipped entirely by its own guard.
  EXPECT_EQ(
    runner_path_speed_profile::makeProfile(path, clearance, 1.0, zero),
    runner_path_speed_profile::makeProfile(path, clearance, 1.0, zero_again));

  // A meaningfully nonzero approach time must actually change the profile:
  // the constrained target must become active earlier than physical
  // braking alone requires.
  runner_path_speed_profile::ProfileConfig anticipated = zero;
  // Large enough that its required lead distance exceeds what physical
  // braking alone needs to decelerate into the dip (~0.7-0.8 m here), so
  // the anticipation pass is the one actually doing the extra work.
  anticipated.approach_time_s = 5.0;
  const auto zero_profile = runner_path_speed_profile::makeProfile(
    path, clearance, 1.0, zero);
  const auto anticipated_profile = runner_path_speed_profile::makeProfile(
    path, clearance, 1.0, anticipated);
  bool differs = false;
  for (std::size_t i = 0; i < zero_profile.points.size(); ++i) {
    if (std::abs(
        zero_profile.points[i].speed_ceiling_mps -
        anticipated_profile.points[i].speed_ceiling_mps) > 1e-9)
    {
      differs = true;
      break;
    }
  }
  EXPECT_TRUE(differs) <<
    "nonzero approach_time_s must plateau the ceiling earlier than the "
    "zero-approach-time profile somewhere along the path";
  for (std::size_t i = 0; i < zero_profile.points.size(); ++i) {
    EXPECT_LE(
      anticipated_profile.points[i].speed_ceiling_mps,
      zero_profile.points[i].speed_ceiling_mps + 1e-9) <<
      "approach-time anticipation must never raise the ceiling above the "
      "zero-approach-time (purely physical) profile at index " << i;
  }
}

TEST(PathSpeedProfile, OrientedFootprintChangesBoundaryMargin)
{
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, 0);
  costmap.setCost(25, 20, nav2_costmap_2d::LETHAL_OBSTACLE);
  nav_msgs::msg::Path path;
  appendPose(path, 1.0, 0.0);
  path.poses.back().pose.position.y = 1.0;
  appendPose(path, 1.0, M_PI_2);
  path.poses.back().pose.position.y = 1.0;
  const auto clearance = runner_path_speed_profile::costmapClearance(
    path, costmap, 0.25, 0.05, 0.08);
  ASSERT_EQ(clearance.size(), 2u);
  EXPECT_LT(clearance[0], clearance[1]);
}
