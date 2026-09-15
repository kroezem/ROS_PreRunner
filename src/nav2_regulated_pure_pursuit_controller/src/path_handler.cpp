// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <cmath>

#include "nav2_regulated_pure_pursuit_controller/path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

using nav2_util::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  tf2::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros)
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

void PathHandler::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  path_offset_ = 0.0;
  global_path_index_ = 0u;
  current_segment_ = 0u;
  segment_terminal_indices_.clear();
}

void PathHandler::setSegmentTerminalIndices(
  const std::vector<std::size_t> & terminal_indices)
{
  segment_terminal_indices_ = terminal_indices;
  current_segment_ = 0u;
}

bool PathHandler::atSegmentTerminal() const
{
  return current_segment_ < segment_terminal_indices_.size() &&
         global_path_index_ >= segment_terminal_indices_[current_segment_];
}

bool PathHandler::advanceSegment()
{
  if (!atSegmentTerminal() || current_segment_ + 1u >= segment_terminal_indices_.size()) {
    return false;
  }
  // The cusp belongs to both adjacent segments. Move the scalar progress just
  // beyond its exact-zero sample while retaining the cusp pose for geometry.
  path_offset_ = std::nextafter(path_offset_, std::numeric_limits<double>::infinity());
  ++current_segment_;
  return true;
}

nav_msgs::msg::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool reject_unit_path)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("Received plan with zero length");
  }

  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw nav2_core::InvalidPath("Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::ControllerTFError("Unable to transform robot pose into global plan's frame");
  }

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

  const bool has_active_cusp =
    current_segment_ + 1u < segment_terminal_indices_.size();
  if (has_active_cusp) {
    const auto remaining_to_terminal =
      segment_terminal_indices_[current_segment_] - global_path_index_;
    const auto segment_upper_bound = global_plan_.poses.begin() +
      std::min(global_plan_.poses.size(), remaining_to_terminal + 1u);
    closest_pose_upper_bound = std::min(closest_pose_upper_bound, segment_upper_bound);
  }

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (!has_active_cusp && global_plan_.poses.begin() != closest_pose_upper_bound &&
    global_plan_.poses.size() > 1 &&
    transformation_begin == std::prev(closest_pose_upper_bound))
  {
    transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
  }

  // We'll discard points on the plan that are outside the local costmap
  const double max_costmap_extent = getCostmapMaxExtent();
  auto eligible_end = global_plan_.poses.end();
  if (has_active_cusp) {
    const auto remaining_to_terminal =
      segment_terminal_indices_[current_segment_] - global_path_index_;
    eligible_end = global_plan_.poses.begin() +
      std::min(global_plan_.poses.size(), remaining_to_terminal + 1u);
  }
  auto transformation_end = std::find_if(
    transformation_begin, eligible_end,
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw nav2_core::ControllerTFError("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  if (transformed_plan.poses.size() == 1u && has_active_cusp) {
    transformed_plan.poses.push_back(transformed_plan.poses.front());
  }
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  for (auto iterator = global_plan_.poses.begin(); iterator != transformation_begin; ++iterator) {
    const auto next = std::next(iterator);
    if (next != global_plan_.poses.end()) {
      path_offset_ += euclidean_distance(*iterator, *next);
    }
  }
  global_path_index_ += static_cast<std::size_t>(
    std::distance(global_plan_.poses.begin(), transformation_begin));
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PathHandler::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_regulated_pure_pursuit_controller
