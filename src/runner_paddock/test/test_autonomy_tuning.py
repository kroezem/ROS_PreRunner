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

from runner_paddock.autonomy_tuning import (
    ADAPTER_OWNER,
    CONFIDENT,
    CONTROLLER_OWNER,
    matching_preset,
    PARAMETERS,
    TIMID,
    validate_values,
    values_for_owner,
)


def test_timid_exactly_reproduces_committed_conservative_policy():
    assert TIMID == {
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
    assert matching_preset(validate_values(dict(TIMID))) == 'timid'


def test_confident_v1_exact_values_and_owner_partition():
    assert CONFIDENT == {
        **TIMID,
        'desired_linear_vel': 1.00,
        'maximum_commanded_speed': 1.00,
        'regulated_linear_scaling_min_speed': 0.40,
        'cost_scaling_dist': 0.60,
        'max_allowed_time_to_collision_up_to_carrot': 0.60,
    }
    values = validate_values(dict(CONFIDENT))
    assert matching_preset(values) == 'confident'
    controller = values_for_owner(values, CONTROLLER_OWNER)
    adapter = values_for_owner(values, ADAPTER_OWNER)
    assert set(controller) == {
        spec.parameter_name for spec in PARAMETERS.values()
        if spec.owner == CONTROLLER_OWNER
    }
    assert set(adapter) == {
        spec.parameter_name for spec in PARAMETERS.values()
        if spec.owner == ADAPTER_OWNER
    }
    assert '/speed_limit' not in str(PARAMETERS)


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
        {'cost_scaling_gain': 1.01},
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
