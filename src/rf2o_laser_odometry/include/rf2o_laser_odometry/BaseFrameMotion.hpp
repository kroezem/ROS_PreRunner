#ifndef RF2O_LASER_ODOMETRY__BASE_FRAME_MOTION_HPP_
#define RF2O_LASER_ODOMETRY__BASE_FRAME_MOTION_HPP_

#include <eigen3/Eigen/Geometry>

namespace rf2o
{

inline Eigen::Isometry3d baseFrameIncrement(
  const Eigen::Isometry3d & old_laser_pose,
  const Eigen::Isometry3d & new_laser_pose,
  const Eigen::Isometry3d & base_to_laser)
{
  const Eigen::Isometry3d old_base_pose = old_laser_pose * base_to_laser.inverse();
  const Eigen::Isometry3d new_base_pose = new_laser_pose * base_to_laser.inverse();
  return old_base_pose.inverse() * new_base_pose;
}

}  // namespace rf2o

#endif  // RF2O_LASER_ODOMETRY__BASE_FRAME_MOTION_HPP_
