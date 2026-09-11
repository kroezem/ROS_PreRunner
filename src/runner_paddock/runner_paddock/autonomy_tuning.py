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

"""Authoritative schema and presets for live autonomy speed tuning."""

from dataclasses import dataclass
import math


CONTROLLER_OWNER = 'controller'
ADAPTER_OWNER = 'adapter'


@dataclass(frozen=True)
class TuningParameter:
    """Map one browser field to its sole ROS parameter owner."""

    owner: str
    node_name: str
    parameter_name: str


PARAMETERS = {
    'desired_linear_vel': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.desired_linear_vel',
    ),
    'regulated_linear_scaling_min_speed': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.regulated_linear_scaling_min_speed',
    ),
    'cost_scaling_dist': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.cost_scaling_dist',
    ),
    'cost_scaling_gain': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.cost_scaling_gain',
    ),
    'regulated_linear_scaling_min_radius': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.regulated_linear_scaling_min_radius',
    ),
    'min_lookahead_dist': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.min_lookahead_dist',
    ),
    'max_lookahead_dist': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.max_lookahead_dist',
    ),
    'lookahead_time': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.lookahead_time',
    ),
    'max_allowed_time_to_collision_up_to_carrot': TuningParameter(
        CONTROLLER_OWNER, '/controller_server',
        'FollowPath.max_allowed_time_to_collision_up_to_carrot',
    ),
    'maximum_commanded_speed': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'maximum_commanded_speed',
    ),
    'proportional_gain': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'proportional_gain',
    ),
    'integral_gain': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'integral_gain',
    ),
    'feedforward_effort_per_speed': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'feedforward_effort_per_speed',
    ),
    'feedforward_effort_intercept': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'feedforward_effort_intercept',
    ),
    'output_max': TuningParameter(
        ADAPTER_OWNER, '/drive_adapter', 'output_max',
    ),
}

TIMID = {
    'desired_linear_vel': 0.45,
    'maximum_commanded_speed': 0.60,
    'regulated_linear_scaling_min_speed': 0.30,
    'cost_scaling_dist': 0.45,
    'cost_scaling_gain': 1.0,
    'regulated_linear_scaling_min_radius': 0.75,
    'min_lookahead_dist': 0.30,
    'max_lookahead_dist': 0.80,
    'lookahead_time': 1.0,
    'max_allowed_time_to_collision_up_to_carrot': 0.15,
    'proportional_gain': 0.05,
    'integral_gain': 0.01,
    'feedforward_effort_per_speed': 0.1188,
    'feedforward_effort_intercept': 0.0174,
    'output_max': 0.14,
}

CONFIDENT = {
    **TIMID,
    'desired_linear_vel': 1.00,
    'maximum_commanded_speed': 1.00,
    'regulated_linear_scaling_min_speed': 0.40,
    'cost_scaling_dist': 0.60,
    'max_allowed_time_to_collision_up_to_carrot': 0.60,
}

PRESETS = {'timid': TIMID, 'confident': CONFIDENT}


def values_for_owner(values: dict[str, float], owner: str) -> dict[str, float]:
    """Return ROS parameter names and values for one atomic owner write."""
    return {
        spec.parameter_name: values[field]
        for field, spec in PARAMETERS.items()
        if spec.owner == owner
    }


def validate_values(values: dict) -> dict[str, float]:
    """Validate a complete browser tuning snapshot and cross-field bounds."""
    if set(values) != set(PARAMETERS):
        missing = sorted(set(PARAMETERS) - set(values))
        extra = sorted(set(values) - set(PARAMETERS))
        raise ValueError(f'tuning fields mismatch; missing={missing}, extra={extra}')
    normalized = {}
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a finite number')
        normalized[name] = float(value)
        if not math.isfinite(normalized[name]):
            raise ValueError(f'{name} must be a finite number')

    positive = set(PARAMETERS) - {
        'integral_gain', 'feedforward_effort_intercept',
    }
    for name in positive:
        if normalized[name] <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    if normalized['integral_gain'] < 0.0:
        raise ValueError('integral_gain must be nonnegative')
    if normalized['cost_scaling_gain'] > 1.0:
        raise ValueError('cost_scaling_gain must not exceed 1.0')
    if normalized['regulated_linear_scaling_min_speed'] > normalized[
        'desired_linear_vel'
    ]:
        raise ValueError('regulated minimum must not exceed nominal speed')
    if normalized['desired_linear_vel'] > normalized[
        'maximum_commanded_speed'
    ]:
        raise ValueError('nominal speed must not exceed adapter ceiling')
    if normalized['min_lookahead_dist'] > normalized['max_lookahead_dist']:
        raise ValueError('minimum lookahead must not exceed maximum lookahead')
    maximum_feedforward = (
        normalized['feedforward_effort_per_speed']
        * normalized['maximum_commanded_speed']
        + normalized['feedforward_effort_intercept']
    )
    minimum_feedforward = (
        normalized['feedforward_effort_per_speed'] * 0.25
        + normalized['feedforward_effort_intercept']
    )
    if min(maximum_feedforward, minimum_feedforward) < 0.0:
        raise ValueError('feedforward must be nonnegative in the command range')
    if normalized['output_max'] > 0.14:
        raise ValueError('output_max must not exceed 0.14')
    if normalized['output_max'] < maximum_feedforward:
        raise ValueError('output_max must reach maximum feedforward')
    return normalized


def matching_preset(values: dict[str, float]) -> str:
    """Classify only a complete exact live readback as a named preset."""
    if set(values) != set(PARAMETERS):
        return 'custom'
    for name, preset in PRESETS.items():
        if all(values[field] == expected for field, expected in preset.items()):
            return name
    return 'custom'
