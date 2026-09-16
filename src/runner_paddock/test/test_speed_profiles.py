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

"""Tests for named Speed Profile snapshots (save/load/delete)."""

import pytest

from runner_paddock.autonomy_tuning import NEUTRAL_COMPLETION_VALUES
from runner_paddock.autonomy_tuning import SPEED_PROFILE_FIELDS
from runner_paddock.speed_profiles import BASELINE_NAME
from runner_paddock.speed_profiles import BASELINE_VALUES
from runner_paddock.speed_profiles import delete_profile
from runner_paddock.speed_profiles import list_profiles
from runner_paddock.speed_profiles import load_profile
from runner_paddock.speed_profiles import save_profile

REFERENCE = dict(BASELINE_VALUES)
# A complete, valid live snapshot (Speed Profile fields plus Engineering and
# Planner Settings), needed wherever save_profile's `current_values` param
# completes a Speed-Profile-only snapshot for the one shared validator.
FULL_LIVE_VALUES = {
    **BASELINE_VALUES,
    'proportional_gain': 0.05,
    'integral_gain': 0.01,
    'feedforward_effort_per_speed': 0.1188,
    'feedforward_effort_intercept': 0.0174,
    'output_max': 0.14,
    'cost_penalty': 2.0,
}


def _path(tmp_path):
    return tmp_path / 'speed_profiles.json'


def test_baseline_is_always_listed_and_loadable_and_flagged_immutable(tmp_path):
    path = _path(tmp_path)
    entries = list_profiles(path)
    assert entries == [{'name': BASELINE_NAME, 'baseline': True}]

    loaded = load_profile(BASELINE_NAME, path)
    assert loaded == BASELINE_VALUES


def test_baseline_cannot_be_saved_over_or_deleted(tmp_path):
    path = _path(tmp_path)
    with pytest.raises(ValueError, match='immutable'):
        save_profile(BASELINE_NAME, REFERENCE, FULL_LIVE_VALUES, path)
    with pytest.raises(ValueError, match='immutable'):
        delete_profile(BASELINE_NAME, path)


def test_save_then_load_round_trips_the_complete_snapshot(tmp_path):
    path = _path(tmp_path)
    snapshot = {**REFERENCE, 'minimum_traversal_speed': 0.4, 'tight_clearance': 0.1}
    saved = save_profile('Aggressive', snapshot, FULL_LIVE_VALUES, path)
    assert saved['minimum_traversal_speed'] == 0.4

    entries = list_profiles(path)
    assert {'name': 'Aggressive', 'baseline': False} in entries
    assert {'name': BASELINE_NAME, 'baseline': True} in entries

    loaded = load_profile('Aggressive', path)
    assert loaded == saved


def test_save_validates_against_the_one_shared_schema(tmp_path):
    path = _path(tmp_path)
    broken = {**REFERENCE, 'open_clearance': REFERENCE['tight_clearance']}
    with pytest.raises(ValueError):
        save_profile('Broken', broken, FULL_LIVE_VALUES, path)
    assert list_profiles(path) == [{'name': BASELINE_NAME, 'baseline': True}]


def test_save_rejects_empty_name(tmp_path):
    path = _path(tmp_path)
    with pytest.raises(ValueError):
        save_profile('   ', REFERENCE, FULL_LIVE_VALUES, path)


def test_delete_removes_a_user_profile_but_not_the_baseline(tmp_path):
    path = _path(tmp_path)
    save_profile('Temp', REFERENCE, FULL_LIVE_VALUES, path)
    assert any(entry['name'] == 'Temp' for entry in list_profiles(path))

    delete_profile('Temp', path)
    assert not any(entry['name'] == 'Temp' for entry in list_profiles(path))


def test_delete_unknown_profile_raises(tmp_path):
    path = _path(tmp_path)
    with pytest.raises(KeyError):
        delete_profile('does-not-exist', path)


def test_load_unknown_profile_raises(tmp_path):
    path = _path(tmp_path)
    with pytest.raises(KeyError):
        load_profile('does-not-exist', path)


def test_saving_never_persists_engineering_or_planner_fields(tmp_path):
    """A named profile is Speed Profile-only, never cost_penalty/Engineering."""
    path = _path(tmp_path)
    live = {**FULL_LIVE_VALUES, 'cost_penalty': 99.0, 'output_max': 0.28}
    saved = save_profile('Custom', REFERENCE, live, path)
    assert set(saved) == SPEED_PROFILE_FIELDS
    assert 'cost_penalty' not in saved
    assert 'output_max' not in saved


def test_load_is_immune_to_live_engineering_drift(tmp_path):
    """
    A saved profile must load even if live Engineering values have drifted.

    Load never talks to ROS, so it has no way to observe that drift.
    """
    path = _path(tmp_path)
    fast = {**REFERENCE, 'desired_linear_vel': 1.9, 'maximum_commanded_speed': 1.9}
    save_profile('Fast', fast, {**FULL_LIVE_VALUES, 'output_max': 0.28}, path)

    loaded = load_profile('Fast', path)
    assert loaded['desired_linear_vel'] == 1.9


def test_a_corrupted_saved_entry_is_rejected_on_load(tmp_path):
    import json
    path = _path(tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'profiles': {'Bad': {'desired_linear_vel': 1.0}}}))
    with pytest.raises(ValueError, match='corrupt'):
        load_profile('Bad', path)


def test_neutral_completion_values_exactly_fill_the_gap_left_by_a_profile():
    assert set(NEUTRAL_COMPLETION_VALUES) | SPEED_PROFILE_FIELDS == set(BASELINE_VALUES) | set(
        NEUTRAL_COMPLETION_VALUES
    )
    assert not (set(NEUTRAL_COMPLETION_VALUES) & SPEED_PROFILE_FIELDS)
