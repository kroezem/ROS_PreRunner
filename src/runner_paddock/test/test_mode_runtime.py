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

"""Tests for fail-closed systemd mode orchestration without ROS/systemd."""

from collections import Counter
from pathlib import Path

import pytest

from runner_paddock.mode_runtime import AUTONOMY_ONLY_NODES
from runner_paddock.mode_runtime import AUTONOMY_UNIT
from runner_paddock.mode_runtime import COMMON_NODES
from runner_paddock.mode_runtime import Lifecycle
from runner_paddock.mode_runtime import MAPPING_UNIT
from runner_paddock.mode_runtime import ModeRuntime
from runner_paddock.mode_runtime import UnitState
from runner_paddock.mode_runtime import validate_map_bundle
from runner_paddock.state_machine import Mode


class FakeSystemd:
    """Model fixed units and cgroup teardown for orchestration tests."""

    def __init__(self, graph):
        self.graph = graph
        self.units = {
            MAPPING_UNIT: UnitState('inactive', 'dead'),
            AUTONOMY_UNIT: UnitState('inactive', 'dead'),
        }
        self.operations = []
        self.fail_start = False

    def state(self, unit):
        return self.units[unit]

    def stop(self, unit):
        self.operations.append(('stop', unit))
        was_active = self.units[unit].active
        self.units[unit] = UnitState('inactive', 'dead')
        if was_active and all(not state.active for state in self.units.values()):
            self.graph.clear()

    def start(self, unit):
        self.operations.append(('start', unit))
        if self.fail_start:
            raise RuntimeError('injected start failure')
        other = AUTONOMY_UNIT if unit == MAPPING_UNIT else MAPPING_UNIT
        self.units[other] = UnitState('inactive', 'dead')
        self.units[unit] = UnitState(
            'active', 'running', 123, f'/system.slice/{unit}'
        )
        self.graph.update({name: 1 for name in COMMON_NODES})
        if unit == AUTONOMY_UNIT:
            self.graph.update({name: 1 for name in AUTONOMY_ONLY_NODES})


def complete_map(directory: Path, name='studio'):
    """Create a complete four-file bundle in a temporary directory."""
    (directory / f'{name}.posegraph').write_text('posegraph')
    (directory / f'{name}.data').write_text('data')
    (directory / f'{name}.pgm').write_text('image')
    (directory / f'{name}.yaml').write_text(f'image: {name}.pgm\n')


def runtime(tmp_path, graph=None):
    """Build a fast runtime and collect every published lifecycle state."""
    graph = Counter() if graph is None else graph
    systemd = FakeSystemd(graph)
    published = []
    value = ModeRuntime(
        systemd,
        lambda: graph,
        published.append,
        map_directory=tmp_path,
        map_file=tmp_path / 'run' / 'autonomy-map',
        transition_timeout=0.05,
        poll_period=0.001,
    )
    return value, systemd, graph, published


@pytest.mark.parametrize('name', ['', '../studio', 'a/b', 'a\\b', '..'])
def test_map_basename_rejects_paths(name, tmp_path):
    with pytest.raises(ValueError):
        validate_map_bundle(name, map_directory=tmp_path)


def test_map_requires_yaml_referenced_fourth_file(tmp_path):
    complete_map(tmp_path)
    (tmp_path / 'studio.pgm').unlink()

    with pytest.raises(ValueError, match='incomplete autonomy map bundle'):
        validate_map_bundle('studio', map_directory=tmp_path)


def test_mapping_idle_autonomy_idle_is_serial_and_exclusive(tmp_path):
    complete_map(tmp_path)
    value, systemd, graph, published = runtime(tmp_path)

    mapping = value.transition(Mode.MAPPING, 1)
    idle = value.transition(Mode.IDLE, 2)
    autonomy = value.transition(Mode.AUTONOMY, 3, autonomy_map='studio')
    final = value.transition(Mode.IDLE, 4)

    assert mapping.mode == Mode.MAPPING
    assert idle.mode == Mode.IDLE
    assert autonomy.mode == Mode.AUTONOMY
    assert autonomy.active_autonomy_map == 'studio'
    assert final == value.state
    assert final.mode == Mode.IDLE
    assert not graph
    assert all(not state.active for state in systemd.units.values())
    assert [state.lifecycle for state in published] == [
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
    ]


def test_start_failure_cleans_partial_graph_and_faults_idle(tmp_path):
    value, systemd, graph, _published = runtime(tmp_path)
    systemd.fail_start = True

    result = value.transition(Mode.MAPPING, 1)

    assert result.mode == Mode.IDLE
    assert result.lifecycle == Lifecycle.FAULT
    assert 'injected start failure' in result.detail
    assert not graph
    assert all(not state.active for state in systemd.units.values())


def test_readiness_rejects_wrong_one_owner_publishers(tmp_path):
    graph = Counter()
    systemd = FakeSystemd(graph)
    value = ModeRuntime(
        systemd,
        lambda: graph,
        lambda _state: None,
        map_directory=tmp_path,
        map_file=tmp_path / 'autonomy-map',
        transition_timeout=0.005,
        poll_period=0.001,
        ownership_ready=lambda _mode: False,
    )

    result = value.transition(Mode.MAPPING, 1)

    assert result.mode == Mode.IDLE
    assert result.lifecycle == Lifecycle.FAULT
    assert 'readiness' in result.detail
    assert not graph


def test_newer_idempotent_request_acknowledges_without_restarting(tmp_path):
    value, systemd, _graph, _published = runtime(tmp_path)
    value.transition(Mode.MAPPING, 1)
    operations = list(systemd.operations)

    result = value.transition(Mode.MAPPING, 2)

    assert result.mode == Mode.MAPPING
    assert result.accepted_request_id == 2
    assert systemd.operations == operations


def test_reconcile_recovers_one_actual_mode(tmp_path):
    complete_map(tmp_path)
    value, systemd, _graph, _published = runtime(tmp_path)
    value._write_map('studio')
    systemd.start(AUTONOMY_UNIT)

    result = value.reconcile()

    assert result.mode == Mode.AUTONOMY
    assert result.lifecycle == Lifecycle.STABLE
    assert result.active_autonomy_map == 'studio'


def test_reconcile_conflict_stops_both_and_faults(tmp_path):
    value, systemd, graph, _published = runtime(tmp_path)
    systemd.start(MAPPING_UNIT)
    systemd.units[AUTONOMY_UNIT] = UnitState(
        'active', 'running', 456, '/system.slice/autonomy'
    )

    result = value.reconcile()

    assert result.mode == Mode.IDLE
    assert result.lifecycle == Lifecycle.FAULT
    assert not graph
    assert all(not state.active for state in systemd.units.values())


def test_reconcile_unmanaged_graph_faults_without_guessing(tmp_path):
    graph = Counter({'/slam_toolbox': 1})
    value, _systemd, graph, _published = runtime(tmp_path, graph)

    result = value.reconcile()

    assert result.mode == Mode.IDLE
    assert result.lifecycle == Lifecycle.FAULT
    assert 'outside active mode units' in result.detail
    # Unknown processes are not killed by name.
    assert graph == Counter({'/slam_toolbox': 1})
