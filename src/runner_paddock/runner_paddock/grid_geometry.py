# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Planar geometry helpers for placing rolling grids in the map frame."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarPose:
    """A finite SE(2) pose."""

    x: float
    y: float
    yaw: float


def normalized_yaw(yaw: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def compose(parent_from_child: PlanarPose, child_pose: PlanarPose) -> PlanarPose:
    """Express ``child_pose`` in ``parent`` coordinates."""
    values = (*parent_from_child.__dict__.values(), *child_pose.__dict__.values())
    if not all(math.isfinite(value) for value in values):
        raise ValueError('planar pose contains a non-finite value')
    cosine = math.cos(parent_from_child.yaw)
    sine = math.sin(parent_from_child.yaw)
    return PlanarPose(
        x=(parent_from_child.x + cosine * child_pose.x
           - sine * child_pose.y),
        y=(parent_from_child.y + sine * child_pose.x
           + cosine * child_pose.y),
        yaw=normalized_yaw(parent_from_child.yaw + child_pose.yaw),
    )
