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
    ABSURD,
    ADAPTER_OWNER,
    CONFIDENT,
    CONTROLLER_OWNER,
    INSANE,
    matching_preset,
    NAVIGATOR_OWNER,
    PARAMETERS,
    persist_override,
    TIMID,
    validate_values,
    values_for_owner,
)


def test_absolute_bounds_come_from_shared_policy_contract():
    assert ABSOLUTE_BOUNDS == {
        'maximum_commanded_speed': (
            AutonomyTuningPolicy.MAXIMUM_COMMANDED_SPEED
        ),
        'output_max': AutonomyTuningPolicy.MAXIMUM_OUTPUT_AUTHORITY,
    }


def test_saved_override_contains_only_validated_runtime_policy(tmp_path):
    target = persist_override(dict(CONFIDENT), tmp_path / 'override.yaml')
    content = target.read_text()
    assert 'planner_server:' in content
    assert 'GridBased.cost_penalty: 2' in content
    assert 'bt_navigator:' in content
    assert 'speed_policy.tight_clearance: 0.05' in content
    assert 'FollowPath.desired_linear_vel' not in content
    with pytest.raises(ValueError):
        persist_override(
            {**CONFIDENT, 'open_clearance': CONFIDENT['tight_clearance']},
            tmp_path / 'invalid.yaml',
        )


def test_timid_exactly_reproduces_committed_conservative_policy():
    assert TIMID['desired_linear_vel'] == 0.45
    assert TIMID['minimum_traversal_speed'] == 0.25
    assert TIMID['tight_clearance'] < TIMID['open_clearance']
    assert matching_preset(validate_values(dict(TIMID))) == 'timid'


def test_confident_v1_exact_values_and_owner_partition():
    assert CONFIDENT == {
        **TIMID,
        'desired_linear_vel': 1.00,
        'maximum_commanded_speed': 1.00,
        'regulated_linear_scaling_min_speed': 0.40,
        'max_allowed_time_to_collision_up_to_carrot': 0.60,
    }
    values = validate_values(dict(CONFIDENT))
    assert matching_preset(values) == 'confident'
    controller = values_for_owner(values, CONTROLLER_OWNER)
    adapter = values_for_owner(values, ADAPTER_OWNER)
    navigator = values_for_owner(values, NAVIGATOR_OWNER)
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
    assert navigator['speed_policy.tight_clearance'] == 0.05
    assert navigator['speed_policy.recovery_acceleration_gain'] == 1.6
    assert navigator['speed_policy.recovery_acceleration_floor'] == 0.60
    assert navigator['speed_policy.reaction_time_s'] == 0.40
    assert '/speed_limit' not in str(PARAMETERS)
    assert 'cost_scaling_dist' not in PARAMETERS
    assert 'cost_scaling_gain' not in PARAMETERS


def test_insane_exactly_copies_confident_except_experimental_limits():
    assert INSANE == {
        **CONFIDENT,
        'desired_linear_vel': 1.50,
        'maximum_commanded_speed': 1.50,
        'output_max': 0.22,
    }
    values = validate_values(dict(INSANE))
    assert matching_preset(values) == 'insane'
    assert values_for_owner(values, CONTROLLER_OWNER)[
        'FollowPath.desired_linear_vel'
    ] == 1.50
    adapter = values_for_owner(values, ADAPTER_OWNER)
    assert adapter['maximum_commanded_speed'] == 1.50
    assert adapter['output_max'] == 0.22


def test_absurd_exactly_copies_confident_except_authorized_limits():
    assert ABSURD == {
        **CONFIDENT,
        'desired_linear_vel': 2.00,
        'maximum_commanded_speed': 2.00,
        'output_max': 0.28,
    }
    values = validate_values(dict(ABSURD))
    assert matching_preset(values) == 'absurd'
    assert values['maximum_commanded_speed'] == ABSOLUTE_BOUNDS[
        'maximum_commanded_speed'
    ]
    assert values['output_max'] < ABSOLUTE_BOUNDS['output_max']


def test_manual_edit_classifies_live_values_as_custom():
    values = dict(CONFIDENT)
    values['lookahead_time'] = 1.01
    assert matching_preset(validate_values(values)) == 'custom'


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
    values = {**CONFIDENT, **changes}
    with pytest.raises(ValueError):
        validate_values(values)


def test_incomplete_or_nonfinite_edits_are_rejected():
    incomplete = dict(TIMID)
    del incomplete['lookahead_time']
    with pytest.raises(ValueError, match='fields mismatch'):
        validate_values(incomplete)
    with pytest.raises(ValueError, match='finite'):
        validate_values({**TIMID, 'lookahead_time': float('nan')})


@pytest.mark.parametrize(
    ('changes', 'message'),
    [
        ({'desired_linear_vel': 2.01, 'maximum_commanded_speed': 2.01},
         'maximum_commanded_speed must not exceed 2.0'),
        ({'output_max': 0.301}, 'output_max must not exceed 0.3'),
    ],
)
def test_experimental_absolute_ceilings_are_enforced(changes, message):
    with pytest.raises(ValueError, match=message):
        validate_values({**INSANE, **changes})


def test_output_limit_must_still_reach_feedforward_requirement():
    with pytest.raises(ValueError, match='must reach maximum feedforward'):
        validate_values({**INSANE, 'output_max': 0.19})
