#include <gtest/gtest.h>

#include <cmath>

#include "rf2o_laser_odometry/BaseFrameMotion.hpp"

namespace
{

using Pose3d = Eigen::Isometry3d;

Pose3d poseWithYaw(const double yaw)
{
  Pose3d pose = Pose3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

Pose3d laserIncrementForBaseMotion(
  const Pose3d & base_to_laser, const double base_x, const double base_y,
  const double base_yaw)
{
  Pose3d base_increment = poseWithYaw(base_yaw);
  base_increment.translation().x() = base_x;
  base_increment.translation().y() = base_y;
  return base_to_laser.inverse() * base_increment * base_to_laser;
}

Pose3d recoveredBaseIncrement(
  const Pose3d & base_to_laser, const Pose3d & laser_increment)
{
  const Pose3d old_laser_pose = base_to_laser;
  const Pose3d new_laser_pose = old_laser_pose * laser_increment;
  return rf2o::baseFrameIncrement(old_laser_pose, new_laser_pose, base_to_laser);
}

void expectForwardAndReverseSigns(const double extrinsic_yaw)
{
  const Pose3d base_to_laser = poseWithYaw(extrinsic_yaw);

  const Pose3d forward = recoveredBaseIncrement(
    base_to_laser, laserIncrementForBaseMotion(base_to_laser, 0.8, 0.0, 0.0));
  const Pose3d reverse = recoveredBaseIncrement(
    base_to_laser, laserIncrementForBaseMotion(base_to_laser, -0.6, 0.0, 0.0));

  EXPECT_GT(forward.translation().x(), 0.0);
  EXPECT_NEAR(forward.translation().x(), 0.8, 1e-12);
  EXPECT_LT(reverse.translation().x(), 0.0);
  EXPECT_NEAR(reverse.translation().x(), -0.6, 1e-12);
}

TEST(BaseFrameTwist, ForwardAndReverseWithZeroYawExtrinsic)
{
  expectForwardAndReverseSigns(0.0);
}

TEST(BaseFrameTwist, ForwardAndReverseWithPiYawExtrinsic)
{
  expectForwardAndReverseSigns(std::acos(-1.0));
}

TEST(BaseFrameTwist, NontrivialYawRotatesLinearMotionIntoBaseFrame)
{
  const double extrinsic_yaw = 0.73;
  const Pose3d base_to_laser = poseWithYaw(extrinsic_yaw);
  const Pose3d laser_increment =
    laserIncrementForBaseMotion(base_to_laser, 0.9, -0.2, 0.0);
  const Pose3d base_increment =
    recoveredBaseIncrement(base_to_laser, laser_increment);

  EXPECT_NEAR(base_increment.translation().x(), 0.9, 1e-12);
  EXPECT_NEAR(base_increment.translation().y(), -0.2, 1e-12);
}

TEST(BaseFrameTwist, PureYawExtrinsicPreservesAngularDirection)
{
  const Pose3d base_to_laser = poseWithYaw(0.73);
  const Pose3d laser_increment =
    laserIncrementForBaseMotion(base_to_laser, 0.0, 0.0, -0.31);
  const Pose3d base_increment =
    recoveredBaseIncrement(base_to_laser, laser_increment);

  EXPECT_NEAR(
    std::atan2(base_increment.rotation()(1, 0), base_increment.rotation()(0, 0)),
    -0.31, 1e-12);
}

}  // namespace
