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

"""Quantitative local-costmap/global-map alignment checks."""

import math

import pytest

from runner_paddock.grid_geometry import compose, PlanarPose


def test_local_grid_origin_composes_into_global_map_frame():
    map_from_odom = PlanarPose(10.0, -2.0, math.pi / 2.0)
    odom_from_grid = PlanarPose(1.0, 2.0, 0.25)

    map_from_grid = compose(map_from_odom, odom_from_grid)

    assert map_from_grid.x == pytest.approx(8.0, abs=1e-12)
    assert map_from_grid.y == pytest.approx(-1.0, abs=1e-12)
    assert map_from_grid.yaw == pytest.approx(math.pi / 2.0 + 0.25, abs=1e-12)

    # A known point in the rolling grid must land at the same map coordinate
    # whether transforms are applied successively or via the composed origin.
    grid_point = PlanarPose(0.7, -0.3, 0.0)
    successive = compose(map_from_odom, compose(odom_from_grid, grid_point))
    composed = compose(map_from_grid, grid_point)
    assert composed.x == pytest.approx(successive.x, abs=1e-12)
    assert composed.y == pytest.approx(successive.y, abs=1e-12)
