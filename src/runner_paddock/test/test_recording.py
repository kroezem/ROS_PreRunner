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

"""Focused tests for persistent MCAP recorder process ownership."""

from datetime import datetime, timezone
import json

import pytest

import runner_paddock.recording as recording
from runner_paddock.recording import RecordingExecutor
from runner_paddock.recording import RUNNER_DEBUG_TOPICS
from runner_paddock.recording import safe_recording_name


class FakeProcess:
    """Minimal healthy subprocess stand-in."""

    pid = 4242

    def poll(self):
        return None


def test_safe_name_generates_default_and_rejects_traversal():
    instant = datetime(2026, 9, 9, 12, 34, 56, tzinfo=timezone.utc)
    assert safe_recording_name('', instant) == 'Runner_20260909_123456'
    assert safe_recording_name('run-1.mcap') == 'run-1.mcap'
    for bad in ('../escape', '/absolute', 'has space', '.hidden'):
        with pytest.raises(ValueError):
            safe_recording_name(bad)


def test_curated_profile_covers_command_authority_and_diagnosis_topics():
    required = {
        '/cmd_vel_nav', '/cmd_vel_auto_raw', '/cmd_vel_auto',
        '/paddock/manual_demand', '/cmd_vel_paddock_manual_raw',
        '/cmd_vel_paddock', '/cmd_vel_teleop', '/cmd_vel',
        '/drive_adapter/state_typed', '/paddock/command_authority_state',
        '/paddock/config_request', '/paddock/config_state',
        '/paddock/navigation_state', '/paddock/mode_state',
        '/paddock/stop_state', '/teleop/control_state',
        '/wheel/encoder_state', '/motor/direction', '/odometry/filtered',
        '/tf', '/tf_static', '/scan', '/map',
        '/global_costmap/costmap', '/local_costmap/costmap', '/plan',
        '/system/telemetry',
    }
    assert required <= set(RUNNER_DEBUG_TOPICS)


def test_start_is_single_owner_mcap_and_refuses_overwrite(tmp_path):
    commands = []

    def popen(command, **kwargs):
        commands.append((command, kwargs))
        return FakeProcess()

    runtime = tmp_path / 'runtime.json'
    executor = RecordingExecutor(
        tmp_path / 'bags', runtime_path=runtime, popen=popen
    )
    executor.start(7, 'debug_run', 'runner_debug')

    command = commands[0][0]
    assert command[:6] == [
        'ros2', 'bag', 'record', '--storage', 'mcap', '--output'
    ]
    assert '--topics' in command
    assert '/cmd_vel' in command
    assert executor.pid == 4242
    assert json.loads(runtime.read_text())['name'] == 'debug_run'
    with pytest.raises(ValueError, match='already active'):
        executor.start(8, 'second', 'runner_debug')


def test_catalog_reads_rosbag_metadata_and_delete_is_explicit(tmp_path):
    root = tmp_path / 'bags'
    bag = root / 'finished'
    bag.mkdir(parents=True)
    (bag / 'finished_0.mcap').write_bytes(b'MCAP data')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n    nanoseconds: 2500000000\n'
        '  starting_time:\n    nanoseconds_since_epoch: 123000000000\n',
        encoding='utf-8',
    )
    (bag / '.paddock-recording.json').write_text(
        '{"profile":"runner_debug"}', encoding='utf-8'
    )
    executor = RecordingExecutor(root, runtime_path=tmp_path / 'runtime.json')

    entry = executor.catalog()[0]
    assert entry.name == 'finished'
    assert entry.duration_sec == pytest.approx(2.5)
    assert entry.size_bytes > 0
    executor.delete(9, 'finished')
    assert not bag.exists()


def test_catalog_reuses_metadata_until_directory_changes(tmp_path, monkeypatch):
    root = tmp_path / 'bags'
    bag = root / 'finished'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n    nanoseconds: 1\n'
        '  starting_time:\n    nanoseconds_since_epoch: 2\n',
        encoding='utf-8',
    )
    (bag / 'finished_0.mcap').write_bytes(b'initial data')
    calls = 0
    original = recording.read_recording_info

    def counted(path):
        nonlocal calls
        calls += 1
        return original(path)

    monkeypatch.setattr(recording, 'read_recording_info', counted)
    executor = RecordingExecutor(root, runtime_path=tmp_path / 'runtime.json')
    assert len(executor.catalog()) == 1
    assert len(executor.catalog()) == 1
    assert calls == 1

    # Active MCAP growth must not force reparsing every finalized bag.
    (bag / 'finished_0.mcap').write_bytes(b'growing data')
    executor.catalog()
    assert calls == 1

    metadata = bag / 'metadata.yaml'
    metadata.write_text(metadata.read_text() + '# finalized update\n')
    executor.catalog()
    assert calls == 2
