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

# Adapter PID/feedforward calibration. Paddock's Engineering panel remains
# the sole UI home for these; they are never part of a named Speed Profile.
ENGINEERING_FIELDS = frozenset({
    'proportional_gain', 'integral_gain', 'feedforward_effort_per_speed',
    'feedforward_effort_intercept', 'output_max',
})

# Every operator-facing field Speed Profile owns: the controller's speed
# ceiling/lookahead behavior plus the complete D2 committed-path speed law.
# GridBased.cost_penalty (Planner Settings) and the adapter's PID/feedforward
# calibration (Engineering) are deliberately excluded from a named profile.
SPEED_PROFILE_FIELDS = frozenset(PARAMETERS) - ENGINEERING_FIELDS - {
    'cost_penalty',
}

# Deliberately permissive stand-ins for every field a Speed Profile does not
# own (Engineering, Planner Settings), used only to complete a snapshot
# before running it through the one shared validator. None of them can
# itself trigger a rejection, so a loaded profile's validity depends only on
# its own fields, never on live Engineering drift since it was saved.
NEUTRAL_COMPLETION_VALUES = {
    'proportional_gain': 0.001,
    'integral_gain': 0.0,
    'feedforward_effort_per_speed': 1e-6,
    'feedforward_effort_intercept': 0.0,
    'output_max': ABSOLUTE_BOUNDS['output_max'],
    'cost_penalty': 2.0,
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
        'approach_time_s',
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


def validate_speed_profile_snapshot(
    snapshot: dict, current_values: dict
) -> dict[str, float]:
    """
    Validate a Speed-Profile-only snapshot against the one shared schema.

    A named Speed Profile only covers ``SPEED_PROFILE_FIELDS``. To validate
    it with the exact same rules ``validate_values`` applies to a live Apply
    (no second schema), complete it with the caller's live values for every
    field outside Speed Profile's ownership (Engineering, Planner Settings),
    validate the whole snapshot, then return just the Speed Profile fields.
    """
    if set(snapshot) != SPEED_PROFILE_FIELDS:
        missing = sorted(SPEED_PROFILE_FIELDS - set(snapshot))
        extra = sorted(set(snapshot) - SPEED_PROFILE_FIELDS)
        raise ValueError(
            f'speed profile fields mismatch; missing={missing}, extra={extra}'
        )
    completed = {**current_values, **snapshot}
    normalized = validate_values(completed)
    return {field: normalized[field] for field in SPEED_PROFILE_FIELDS}
