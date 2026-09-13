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

"""Focused tests for the Stage-A1 navigation replay geometry."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import pytest

from runner_bringup.navigation_bag_analyzer import _summary
from runner_bringup.navigation_bag_analyzer import footprint_lethal_cells
from runner_bringup.navigation_bag_analyzer import path_static_overlap
from runner_bringup.navigation_bag_analyzer import stopping_window_bound


def _grid():
    grid = OccupancyGrid()
    grid.info.resolution = 0.05
    grid.info.width = 20
    grid.info.height = 20
    grid.data = [0] * 400
    return grid


def _pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def test_footprint_raster_detects_lethal_perimeter_cell():
    grid = _grid()
    grid.data[10 * grid.info.width + 14] = 100
    assert footprint_lethal_cells(grid, 0.5, 0.5, 0.0) == {(14, 10)}


def test_static_overlap_ignores_measured_start_but_checks_future_path():
    grid = _grid()
    grid.data[10 * grid.info.width + 14] = 100
    start_only = Path(poses=[_pose(0.5, 0.5)])
    crossing = Path(poses=[_pose(0.1, 0.1), _pose(0.5, 0.5)])
    assert path_static_overlap(start_only, grid) == 0
    assert path_static_overlap(crossing, grid) == 1


def test_i11_window_bound_covers_insane_ceiling_and_footprint():
    bound = stopping_window_bound()
    assert bound['deceleration_mps2'] == pytest.approx(2.67)
    assert bound['stopping_distance_m'] == pytest.approx(0.721348, abs=1e-6)
    assert bound['required_width_m'] == pytest.approx(3.531393, abs=1e-6)
    assert 4 >= bound['required_width_m']


def test_summary_handles_a_bag_without_navigation_attempts():
    report = {
        'bag': 'smoke', 'duration_sec': 1.0,
        'static_plan_overlap': {'overlapping_plans': 0, 'plans': 0},
        'dispatch_execution_lethality': {
            'lethal_dispatches': 0, 'available_dispatches': 0,
        },
        'attempts': {'total': 0, 'outcomes': {}, 'classes': {}},
        'replanning': {
            'total_replans': 0, 'distance_metres': 0.0,
            'replans_per_metre': None,
        },
        'identical_redispatch_storms': {'count': 0, 'generations': []},
        'lethal_flicker': {
            'planning': {'available': False, 'frames': 0},
            'execution': {'available': False, 'frames': 0},
        },
        'control_loop_misses': {'count': 0},
    }
    assert '0 replans / 0.000 m = n/a' in _summary(report)
