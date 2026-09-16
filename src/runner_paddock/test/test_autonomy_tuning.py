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

"""Tests for the authoritative adaptive autonomy tuning schema."""

import pytest

from runner_interfaces.msg import AutonomyTuningPolicy
from runner_paddock.autonomy_tuning import (
    ABSOLUTE_BOUNDS,
    ADAPTER_OWNER,
    CONTROLLER_OWNER,
    ENGINEERING_FIELDS,
    NAVIGATOR_OWNER,
    NEUTRAL_COMPLETION_VALUES,
    PARAMETERS,
    persist_override,
    PLANNER_OWNER,
    SPEED_PROFILE_FIELDS,
    validate_speed_profile_snapshot,
    validate_values,
    values_for_owner,
)

# A complete, valid live tuning snapshot -- today's launched values (see
# runner_drive_adapter/config/speed_envelope.yaml and
# runner_bringup/config/nav2_params.yaml).
REFERENCE = {
    'desired_linear_vel': 1.00,
    'maximum_commanded_speed': 1.00,
    'regulated_linear_scaling_min_speed': 0.40,
    'regulated_linear_scaling_min_radius': 0.75,
    'min_lookahead_dist': 0.30,
    'max_lookahead_dist': 0.80,
    'lookahead_time': 1.0,
    'max_allowed_time_to_collision_up_to_carrot': 0.60,
    'proportional_gain': 0.05,
    'integral_gain': 0.01,
    'feedforward_effort_per_speed': 0.1188,
    'feedforward_effort_intercept': 0.0174,
    'output_max': 0.14,
    'minimum_traversal_speed': 0.25,
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
assert set(REFERENCE) == set(PARAMETERS)


def test_absolute_bounds_come_from_shared_policy_contract():
    assert ABSOLUTE_BOUNDS == {
        'maximum_commanded_speed': (
            AutonomyTuningPolicy.MAXIMUM_COMMANDED_SPEED
        ),
        'output_max': AutonomyTuningPolicy.MAXIMUM_OUTPUT_AUTHORITY,
    }


def test_saved_override_contains_only_validated_runtime_policy(tmp_path):
    target = persist_override(dict(REFERENCE), tmp_path / 'override.yaml')
    content = target.read_text()
    assert 'planner_server:' in content
    assert 'GridBased.cost_penalty: 2' in content
    assert 'bt_navigator:' in content
    assert 'speed_policy.tight_clearance: 0.05' in content
    assert 'FollowPath.desired_linear_vel' not in content
    with pytest.raises(ValueError):
        persist_override(
            {**REFERENCE, 'open_clearance': REFERENCE['tight_clearance']},
            tmp_path / 'invalid.yaml',
        )


def test_no_preset_system_remains():
    import runner_paddock.autonomy_tuning as module
    for name in ('TIMID', 'CONFIDENT', 'INSANE', 'ABSURD', 'PRESETS', 'matching_preset'):
        assert not hasattr(module, name), f'{name} should have been removed'


def test_speed_profile_fields_exclude_engineering_and_planner():
    assert SPEED_PROFILE_FIELDS == set(PARAMETERS) - ENGINEERING_FIELDS - {'cost_penalty'}
    assert 'desired_linear_vel' in SPEED_PROFILE_FIELDS
    assert 'minimum_traversal_speed' in SPEED_PROFILE_FIELDS
    assert 'constrained_speed_scaling' not in PARAMETERS
    assert 'scaling_reference_speed' not in PARAMETERS
    for field in ENGINEERING_FIELDS:
        assert field not in SPEED_PROFILE_FIELDS
    assert 'cost_penalty' not in SPEED_PROFILE_FIELDS


def test_owner_partition_covers_every_field():
    values = validate_values(dict(REFERENCE))
    controller = values_for_owner(values, CONTROLLER_OWNER)
    adapter = values_for_owner(values, ADAPTER_OWNER)
    navigator = values_for_owner(values, NAVIGATOR_OWNER)
    planner = values_for_owner(values, PLANNER_OWNER)
    assert set(controller) == {
        spec.parameter_name for spec in PARAMETERS.values()
        if spec.owner == CONTROLLER_OWNER
    }
    assert set(adapter) == {
        spec.parameter_name for spec in PARAMETERS.values()
        if spec.owner == ADAPTER_OWNER
    }
    assert set(navigator) == {
        spec.parameter_name for spec in PARAMETERS.values()
        if spec.owner == NAVIGATOR_OWNER
    }
    assert planner == {'GridBased.cost_penalty': 2.0}
    assert navigator['speed_policy.tight_clearance'] == 0.05
    assert navigator['speed_policy.recovery_acceleration_gain'] == 1.6
    assert navigator['speed_policy.recovery_acceleration_floor'] == 0.60
    assert navigator['speed_policy.reaction_time_s'] == 0.40
    assert '/speed_limit' not in str(PARAMETERS)
    assert 'cost_scaling_dist' not in PARAMETERS
    assert 'cost_scaling_gain' not in PARAMETERS


@pytest.mark.parametrize(
    'changes',
    [
        {'desired_linear_vel': 1.01},
        {'regulated_linear_scaling_min_speed': 1.01},
        {'min_lookahead_dist': 0.81},
        {'open_clearance': 0.0},
        {'reaction_time_s': -0.01},
        {'output_max': 0.13},
    ],
)
def test_incoherent_complete_edits_are_rejected(changes):
    values = {**REFERENCE, **changes}
    with pytest.raises(ValueError):
        validate_values(values)


def test_incomplete_or_nonfinite_edits_are_rejected():
    incomplete = dict(REFERENCE)
    del incomplete['lookahead_time']
    with pytest.raises(ValueError, match='fields mismatch'):
        validate_values(incomplete)
    with pytest.raises(ValueError, match='finite'):
        validate_values({**REFERENCE, 'lookahead_time': float('nan')})


@pytest.mark.parametrize(
    ('changes', 'message'),
    [
        ({'desired_linear_vel': 2.01, 'maximum_commanded_speed': 2.01},
         'maximum_commanded_speed must not exceed 2.0'),
        ({'output_max': 0.301}, 'output_max must not exceed 0.3'),
    ],
)
def test_experimental_absolute_ceilings_are_enforced(changes, message):
    fast = {**REFERENCE, 'desired_linear_vel': 1.50, 'maximum_commanded_speed': 1.50}
    with pytest.raises(ValueError, match=message):
        validate_values({**fast, **changes})


def test_output_limit_must_still_reach_feedforward_requirement():
    fast = {**REFERENCE, 'desired_linear_vel': 1.50, 'maximum_commanded_speed': 1.50}
    with pytest.raises(ValueError, match='must reach maximum feedforward'):
        validate_values({**fast, 'output_max': 0.19})


# -- validate_speed_profile_snapshot: the shared validator, reused ---------

def _snapshot(**overrides):
    base = {field: REFERENCE[field] for field in SPEED_PROFILE_FIELDS}
    base.update(overrides)
    return base


def test_speed_profile_snapshot_validates_against_live_engineering_values():
    validated = validate_speed_profile_snapshot(_snapshot(), REFERENCE)
    assert set(validated) == SPEED_PROFILE_FIELDS
    assert validated['desired_linear_vel'] == 1.00


def test_speed_profile_snapshot_rejects_wrong_field_set():
    incomplete = _snapshot()
    del incomplete['desired_linear_vel']
    with pytest.raises(ValueError, match='fields mismatch'):
        validate_speed_profile_snapshot(incomplete, REFERENCE)
    extra = {**_snapshot(), 'cost_penalty': 2.0}
    with pytest.raises(ValueError, match='fields mismatch'):
        validate_speed_profile_snapshot(extra, REFERENCE)


def test_speed_profile_snapshot_rejects_internal_incoherence():
    with pytest.raises(ValueError):
        validate_speed_profile_snapshot(
            _snapshot(open_clearance=REFERENCE['tight_clearance']), REFERENCE,
        )


def test_speed_profile_snapshot_with_neutral_completion_ignores_engineering_drift():
    # A saved profile with a high maximum speed must still load even if the
    # live adapter's output_max/feedforward would otherwise reject it -- the
    # neutral completion values can never themselves cause a rejection.
    fast = _snapshot(desired_linear_vel=1.9, maximum_commanded_speed=1.9)
    validated = validate_speed_profile_snapshot(fast, NEUTRAL_COMPLETION_VALUES)
    assert validated['desired_linear_vel'] == 1.9


def test_neutral_completion_values_cover_exactly_the_non_speed_profile_fields():
    assert set(NEUTRAL_COMPLETION_VALUES) == set(PARAMETERS) - SPEED_PROFILE_FIELDS
