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

"""
Named Speed Profile snapshots, persisted outside the ROS parameter tree.

An operator can save, load, and delete complete driving-behavior presets by
name. Loading or deleting a saved profile never touches live ROS state --
these are pure local-file operations. Both save and load funnel every
snapshot through the one authoritative :func:`autonomy_tuning.validate_values`
schema (via :func:`autonomy_tuning.validate_speed_profile_snapshot`), so
there is no second policy schema to drift out of sync with Apply.
"""

import json
import os
from pathlib import Path
import tempfile

from runner_paddock.autonomy_tuning import NEUTRAL_COMPLETION_VALUES
from runner_paddock.autonomy_tuning import SPEED_PROFILE_FIELDS
from runner_paddock.autonomy_tuning import validate_speed_profile_snapshot

PROFILES_PATH = Path(os.environ.get(
    'PADDOCK_SPEED_PROFILES_PATH',
    '/home/matti/.config/runner/speed_profiles.json',
))

BASELINE_NAME = 'Default Baseline'

# The values launched today (see runner_drive_adapter/config/speed_envelope
# .yaml and runner_bringup/config/nav2_params.yaml) -- an always-available,
# never-overwritten reference profile.
BASELINE_VALUES = {
    'desired_linear_vel': 1.00,
    'maximum_commanded_speed': 1.00,
    'regulated_linear_scaling_min_speed': 0.40,
    'regulated_linear_scaling_min_radius': 0.75,
    'min_lookahead_dist': 0.30,
    'max_lookahead_dist': 0.80,
    'lookahead_time': 1.0,
    'max_allowed_time_to_collision_up_to_carrot': 0.60,
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
}
assert set(BASELINE_VALUES) == SPEED_PROFILE_FIELDS


def _read_store(path: Path) -> dict:
    try:
        raw = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return {}
    except (OSError, ValueError):
        return {}
    profiles = raw.get('profiles') if isinstance(raw, dict) else None
    return profiles if isinstance(profiles, dict) else {}


def _write_store(profiles: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps({'profiles': profiles}, indent=2, sort_keys=True)
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=path.parent,
        prefix=f'.{path.name}.', delete=False,
    ) as stream:
        temporary = Path(stream.name)
        stream.write(payload + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def list_profiles(path: Path = PROFILES_PATH) -> list[dict]:
    """List every saved profile plus the immutable baseline, by name."""
    names = sorted(_read_store(path))
    return (
        [{'name': BASELINE_NAME, 'baseline': True}]
        + [{'name': name, 'baseline': False} for name in names]
    )


def load_profile(name: str, path: Path = PROFILES_PATH) -> dict[str, float]:
    """Return one validated, complete Speed Profile snapshot by name."""
    if name == BASELINE_NAME:
        raw = BASELINE_VALUES
    else:
        store = _read_store(path)
        if name not in store:
            raise KeyError(name)
        raw = store[name]
        if not isinstance(raw, dict) or set(raw) != SPEED_PROFILE_FIELDS:
            raise ValueError(f'saved profile {name!r} is corrupt')
    return validate_speed_profile_snapshot(raw, NEUTRAL_COMPLETION_VALUES)


def save_profile(
    name: str, snapshot: dict, current_values: dict,
    path: Path = PROFILES_PATH,
) -> dict[str, float]:
    """Validate and atomically persist one named Speed Profile snapshot."""
    name = name.strip()
    if not name:
        raise ValueError('profile name must not be empty')
    if name == BASELINE_NAME:
        raise ValueError(f'{BASELINE_NAME!r} is immutable and cannot be saved over')
    validated = validate_speed_profile_snapshot(snapshot, current_values)
    store = _read_store(path)
    store[name] = validated
    _write_store(store, path)
    return validated


def delete_profile(name: str, path: Path = PROFILES_PATH) -> None:
    """Delete one saved profile by name; the baseline can never be deleted."""
    if name == BASELINE_NAME:
        raise ValueError(f'{BASELINE_NAME!r} is immutable and cannot be deleted')
    store = _read_store(path)
    if name not in store:
        raise KeyError(name)
    del store[name]
    _write_store(store, path)
