// Copyright (c) 2026 Runner project contributors
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

#include <memory>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulation_functions.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

using nav2_costmap_2d::Costmap2D;
using nav2_regulated_pure_pursuit_controller::heuristics::costAlongPath;

void setCostAtWorld(Costmap2D & costmap, const double wx, const double wy, const unsigned char cost)
{
  unsigned int mx, my;
  ASSERT_TRUE(costmap.worldToMap(wx, wy, mx, my));
  costmap.setCost(mx, my, cost);
}

nav_msgs::msg::Path makeStraightPath()
{
  nav_msgs::msg::Path path;
  for (const double x : {0.1, 0.3, 0.5, 0.7, 0.9}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::Pose makeRobotPose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 1.0;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, M_PI_2);
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

geometry_msgs::msg::Point makeLookahead(const double x)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  return point;
}

TEST(PathCostTest, ReturnsMaximumCostAlongPathThroughLookahead)
{
  Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto robot_pose = makeRobotPose();
  const auto path = makeStraightPath();

  // With the robot rotated 90 degrees, local +x maps to world +y.
  setCostAtWorld(costmap, 1.0, 1.20, 80);
  setCostAtWorld(costmap, 1.0, 1.45, 200);
  setCostAtWorld(costmap, 1.0, 1.55, 120);
  setCostAtWorld(costmap, 1.0, 1.85, 252);  // Beyond the 0.6 m carrot.
  setCostAtWorld(costmap, 1.20, 1.45, 253);  // High cost, but off the path.

  EXPECT_DOUBLE_EQ(
    costAlongPath(0.0, robot_pose, path, makeLookahead(0.6), &costmap), 200.0);
}

TEST(PathCostTest, IncludesCurrentPoseCost)
{
  Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto robot_pose = makeRobotPose();
  const auto path = makeStraightPath();
  setCostAtWorld(costmap, 1.0, 1.30, 50);

  EXPECT_DOUBLE_EQ(
    costAlongPath(90.0, robot_pose, path, makeLookahead(0.6), &costmap), 90.0);
}

TEST(PathCostTest, FreePathPreservesFreeSpaceCost)
{
  Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

  EXPECT_DOUBLE_EQ(
    costAlongPath(
      0.0, makeRobotPose(), makeStraightPath(), makeLookahead(0.6), &costmap),
    static_cast<double>(nav2_costmap_2d::FREE_SPACE));
}

TEST(PathCostTest, UnknownDoesNotMaskKnownInflationCost)
{
  Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto robot_pose = makeRobotPose();
  const auto path = makeStraightPath();
  setCostAtWorld(costmap, 1.0, 1.20, 140);
  setCostAtWorld(costmap, 1.0, 1.40, nav2_costmap_2d::NO_INFORMATION);

  EXPECT_DOUBLE_EQ(
    costAlongPath(
      static_cast<double>(nav2_costmap_2d::NO_INFORMATION),
      robot_pose, path, makeLookahead(0.6), &costmap),
    140.0);
}

}  // namespace
