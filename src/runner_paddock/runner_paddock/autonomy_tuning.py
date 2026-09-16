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
import os
from pathlib import Path
import tempfile

from runner_interfaces.msg import AutonomyTuningPolicy

CONTROLLER_OWNER = 'controller'
ADAPTER_OWNER = 'adapter'
NAVIGATOR_OWNER = 'navigator'
PLANNER_OWNER = 'planner'
ABSOLUTE_BOUNDS = {
    'maximum_commanded_speed': (
        AutonomyTuningPolicy.MAXIMUM_COMMANDED_SPEED
    ),
    'output_max': AutonomyTuningPolicy.MAXIMUM_OUTPUT_AUTHORITY,
}
OVERRIDE_PATH = Path(os.environ.get(
    'PADDOCK_SPEED_POLICY_OVERRIDE',
    '/home/matti/.config/runner/speed_profile_overrides.yaml',
))


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
    # D2 committed-path speed law. Declared on the shared bt_navigator node
    # (see runner_nav2_behavior_tree::GeneratePathSpeedProfile); a fresh
    # commit picks up whatever value is current at that moment, no rebuild
    # or BT edit needed. desired_linear_vel (above) remains the authoritative
    # preset ceiling approached by the continuous clearance law.
    'minimum_traversal_speed': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.minimum_traversal_speed',
    ),
    'constrained_speed_scaling': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.constrained_speed_scaling',
    ),
    # The single authoritative reference ceiling constrained_speed_scaling
    # normalizes fully-scaled bottoms against (today, ABSURD's maximum).
    # Explicit so C++ and this preset table cannot silently desynchronize
    # from a future change to that preset ceiling.
    'scaling_reference_speed': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.scaling_reference_speed',
    ),
    'tight_clearance': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.tight_clearance',
    ),
    'open_clearance': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.open_clearance',
    ),
    'clearance_curve_family': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.clearance_curve_family',
    ),
    'clearance_curve_shape': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.clearance_curve_shape',
    ),
    'approach_time_s': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.approach_time_s',
    ),
    'curvature_window': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.curvature_window',
    ),
    'max_lateral_acceleration': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.max_lateral_acceleration',
    ),
    'footprint_front': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.footprint_front',
    ),
    'footprint_rear': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.footprint_rear',
    ),
    'footprint_half_width': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.footprint_half_width',
    ),
    'braking_linear': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.braking_linear',
    ),
    'braking_constant': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.braking_constant',
    ),
    'reaction_time_s': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator', 'speed_policy.reaction_time_s',
    ),
    'recovery_acceleration_gain': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.recovery_acceleration_gain',
    ),
    'recovery_acceleration_floor': TuningParameter(
        NAVIGATOR_OWNER, '/bt_navigator',
        'speed_policy.recovery_acceleration_floor',
    ),
    'cost_penalty': TuningParameter(
        PLANNER_OWNER, '/planner_server', 'GridBased.cost_penalty',
    ),
}

# The one authoritative ABSURD ceiling. Both ABSURD's own
# desired_linear_vel and every preset's scaling_reference_speed (D2's
# fully-scaled-bottom normalization point) derive from this single
# constant, so changing it cannot silently desynchronize the two.
ABSURD_MAXIMUM_SPEED_MPS = 2.00

TIMID = {
    'desired_linear_vel': 0.45,
    'maximum_commanded_speed': 0.60,
    'regulated_linear_scaling_min_speed': 0.30,
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
    # D2 speed law: unchanged across driving-aggressiveness presets. The
    # presets scale the preset ceiling and reaction distances; the
    # clearance and recovery shape stay put.
    'minimum_traversal_speed': 0.25,
    'constrained_speed_scaling': 0.20,
    'scaling_reference_speed': ABSURD_MAXIMUM_SPEED_MPS,
    'tight_clearance': 0.05,
    'open_clearance': 0.70,
    'clearance_curve_family': 2.0,
    'clearance_curve_shape': 1.0,
    'approach_time_s': 0.0,
    'curvature_window': 0.40,
    'max_lateral_acceleration': 0.35,
    'footprint_front': 0.230,
    'footprint_rear': 0.060,
    'footprint_half_width': 0.0825,
    'braking_linear': 1.6,
    'braking_constant': 0.27,
    'reaction_time_s': 0.40,
    'recovery_acceleration_gain': 1.6,
    'recovery_acceleration_floor': 0.60,
    'cost_penalty': 2.0,
}

CONFIDENT = {
    **TIMID,
    'desired_linear_vel': 1.00,
    'maximum_commanded_speed': 1.00,
    'regulated_linear_scaling_min_speed': 0.40,
    'max_allowed_time_to_collision_up_to_carrot': 0.60,
}

INSANE = {
    **CONFIDENT,
    'desired_linear_vel': 1.50,
    'maximum_commanded_speed': 1.50,
    'output_max': 0.22,
}

ABSURD = {
    **CONFIDENT,
    'desired_linear_vel': ABSURD_MAXIMUM_SPEED_MPS,
    'maximum_commanded_speed': ABSURD_MAXIMUM_SPEED_MPS,
    'output_max': 0.28,
}

PRESETS = {
    'timid': TIMID, 'confident': CONFIDENT, 'insane': INSANE,
    'absurd': ABSURD,
}


def values_for_owner(values: dict[str, float], owner: str) -> dict[str, float]:
    """Return ROS parameter names and values for one atomic owner write."""
    return {
        spec.parameter_name: values[field]
        for field, spec in PARAMETERS.items()
        if spec.owner == owner
    }


def persist_override(values: dict, path: Path = OVERRIDE_PATH) -> Path:
    """Atomically persist only D2 and planner-owned validated parameters."""
    normalized = validate_values(values)
    sections = (
        ('planner_server', values_for_owner(normalized, PLANNER_OWNER)),
        ('bt_navigator', values_for_owner(normalized, NAVIGATOR_OWNER)),
    )
    lines = []
    for node, parameters in sections:
        lines.extend((f'{node}:', '  ros__parameters:'))
        for name, value in parameters.items():
            lines.append(f'    {name}: {value:.12g}')
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=path.parent,
        prefix=f'.{path.name}.', delete=False,
    ) as stream:
        temporary = Path(stream.name)
        stream.write('\n'.join(lines) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    return path


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
        'integral_gain', 'feedforward_effort_intercept', 'reaction_time_s',
        'approach_time_s', 'constrained_speed_scaling',
        'clearance_curve_family', 'tight_clearance', 'footprint_rear',
    }
    for name in positive:
        if normalized[name] <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
    if normalized['integral_gain'] < 0.0:
        raise ValueError('integral_gain must be nonnegative')
    for name in ('reaction_time_s', 'approach_time_s', 'tight_clearance', 'footprint_rear'):
        if normalized[name] < 0.0:
            raise ValueError(f'{name} must be nonnegative')
    if not 0.0 <= normalized['constrained_speed_scaling'] <= 1.0:
        raise ValueError('constrained_speed_scaling must be between zero and one')
    if normalized['clearance_curve_family'] not in (0.0, 1.0, 2.0):
        raise ValueError('clearance_curve_family must be linear, power, or smoothstep')
    if normalized['open_clearance'] <= normalized['tight_clearance']:
        raise ValueError('open_clearance must exceed tight_clearance')
    if normalized['regulated_linear_scaling_min_speed'] > normalized[
        'desired_linear_vel'
    ]:
        raise ValueError('regulated minimum must not exceed nominal speed')
    if normalized['desired_linear_vel'] > normalized[
        'maximum_commanded_speed'
    ]:
        raise ValueError('nominal speed must not exceed adapter ceiling')
    if normalized['maximum_commanded_speed'] > ABSOLUTE_BOUNDS[
        'maximum_commanded_speed'
    ]:
        raise ValueError(
            'maximum_commanded_speed must not exceed '
            f'{ABSOLUTE_BOUNDS["maximum_commanded_speed"]}'
        )
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
    if normalized['output_max'] > ABSOLUTE_BOUNDS['output_max']:
        raise ValueError(
            f'output_max must not exceed {ABSOLUTE_BOUNDS["output_max"]}'
        )
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
