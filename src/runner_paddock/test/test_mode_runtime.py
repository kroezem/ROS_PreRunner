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
from itertools import groupby
from pathlib import Path

import pytest

from runner_paddock.mode_runtime import AUTONOMY_ONLY_NODES
from runner_paddock.mode_runtime import AUTONOMY_UNIT
from runner_paddock.mode_runtime import COMMON_NODES
from runner_paddock.mode_runtime import Lifecycle
from runner_paddock.mode_runtime import MAPPING_UNIT
from runner_paddock.mode_runtime import MODE_NODES
from runner_paddock.mode_runtime import ModeRuntime
from runner_paddock.mode_runtime import OP_NEW_MAP
from runner_paddock.mode_runtime import PERSISTENT_LOCAL_NODES
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
            for name in COMMON_NODES | AUTONOMY_ONLY_NODES:
                self.graph.pop(name, None)

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


def runtime(tmp_path, graph=None, capability_ready=None):
    """Build a fast runtime and collect every published lifecycle state."""
    graph = (
        Counter({name: 1 for name in PERSISTENT_LOCAL_NODES})
        if graph is None else graph
    )
    systemd = FakeSystemd(graph)
    published = []
    counter = [0]

    def next_session_id():
        counter[0] += 1
        return f'tok{counter[0]}'

    value = ModeRuntime(
        systemd,
        lambda: graph,
        published.append,
        map_directory=tmp_path,
        map_file=tmp_path / 'run' / 'autonomy-map',
        transition_timeout=0.05,
        poll_period=0.001,
        capability_ready=capability_ready,
        session_id_factory=next_session_id,
    )
    return value, systemd, graph, published


def test_drive_adapter_is_persistent_not_mode_scoped(tmp_path):
    assert '/drive_adapter' in PERSISTENT_LOCAL_NODES
    assert '/drive_adapter' not in AUTONOMY_ONLY_NODES
    assert '/drive_adapter' not in MODE_NODES

    value, _systemd, graph, _published = runtime(tmp_path)

    # Persistent control nodes are valid while no application mode unit owns
    # the graph and must not block mode-resource cleanup.
    assert value._resources_gone()
    idle = value.reconcile()
    assert idle.mode == Mode.IDLE
    assert idle.lifecycle == Lifecycle.STABLE

    mapping = value.transition(Mode.MAPPING, 1)
    assert mapping.mode == Mode.MAPPING
    assert mapping.lifecycle == Lifecycle.STABLE
    assert '/drive_adapter' in graph


def test_autonomy_requires_persistent_drive_adapter(tmp_path):
    complete_map(tmp_path)
    value, systemd, graph, _published = runtime(tmp_path)
    systemd.start(AUTONOMY_UNIT)
    graph.pop('/drive_adapter')

    assert value._structural_reason(Mode.AUTONOMY) == (
        'missing node(s): /drive_adapter'
    )


def test_runtime_epoch_increments_per_successful_application_start(tmp_path):
    complete_map(tmp_path)
    value, _systemd, _graph, _published = runtime(tmp_path)

    assert value.state.runtime_epoch == 0
    assert value.transition(Mode.MAPPING, 1).runtime_epoch == 1
    assert value.transition(Mode.IDLE, 2).runtime_epoch == 1
    assert value.transition(
        Mode.AUTONOMY, 3, autonomy_map='studio'
    ).runtime_epoch == 2


def test_mapping_session_id_lifecycle(tmp_path):
    complete_map(tmp_path)
    value, systemd, _graph, _published = runtime(tmp_path)

    first = value.transition(Mode.MAPPING, 1).mapping_session_id
    assert first
    # Idempotent re-selection never resets the session or restarts anything.
    operations = list(systemd.operations)
    same = value.transition(Mode.MAPPING, 2).mapping_session_id
    assert same == first
    assert systemd.operations == operations

    # NEW MAP forces a fresh session and a real restart.
    fresh = value.transition(
        Mode.MAPPING, 3, operation=OP_NEW_MAP
    )
    assert fresh.mapping_session_id != first
    assert fresh.runtime_epoch == 2
    assert systemd.operations != operations

    # Leaving MAPPING clears the session identity.
    assert value.transition(Mode.IDLE, 4).mapping_session_id == ''
    autonomy = value.transition(Mode.AUTONOMY, 5, autonomy_map='studio')
    assert autonomy.mapping_session_id == ''


def test_new_map_restart_verifies_old_owner_gone_first(tmp_path):
    value, systemd, graph, _published = runtime(tmp_path)
    value.transition(Mode.MAPPING, 1)
    boundary = len(systemd.operations)

    value.transition(Mode.MAPPING, 2, operation=OP_NEW_MAP)

    # stop precedes start within the NEW MAP restart.
    new_map_ops = systemd.operations[boundary:]
    assert new_map_ops[0][0] == 'stop'
    assert ('start', MAPPING_UNIT) in new_map_ops
    assert new_map_ops.index(('start', MAPPING_UNIT)) > 0


def test_capability_evidence_gates_readiness_and_surfaces_reason(tmp_path):
    gate = {'ready': False}
    value, _systemd, _graph, _published = runtime(
        tmp_path,
        capability_ready=lambda _mode: (
            (True, '') if gate['ready'] else (False, 'scan_slam is stale')
        ),
    )

    result = value.transition(Mode.MAPPING, 1)
    assert result.lifecycle == Lifecycle.FAULT
    assert 'readiness' in result.detail

    gate['ready'] = True
    ready = value.transition(Mode.MAPPING, 2)
    assert ready.lifecycle == Lifecycle.STABLE
    assert ready.ready
    assert ready.readiness_reason == ''

    # Continuous refresh reflects a later capability loss without a FAULT.
    gate['ready'] = False
    refreshed = value.refresh()
    assert refreshed.lifecycle == Lifecycle.STABLE
    assert not refreshed.ready
    assert refreshed.readiness_reason == 'scan_slam is stale'


@pytest.mark.parametrize('name', ['', '../studio', 'a/b', 'a\\b', '..'])
def test_map_basename_rejects_paths(name, tmp_path):
    with pytest.raises(ValueError):
        validate_map_bundle(name, map_directory=tmp_path)


def test_map_requires_yaml_referenced_fourth_file(tmp_path):
    complete_map(tmp_path)
    (tmp_path / 'studio.pgm').unlink()

    with pytest.raises(ValueError, match='incomplete autonomy map bundle'):
        validate_map_bundle('studio', map_directory=tmp_path)


def test_autonomy_without_a_selected_map_is_rejected_not_faulted(tmp_path):
    complete_map(tmp_path)
    value, systemd, _graph, _published = runtime(tmp_path)
    value.transition(Mode.MAPPING, 1)
    operations = list(systemd.operations)

    result = value.transition(Mode.AUTONOMY, 2, autonomy_map='')

    # Missing selection is an operator precondition, not a runtime fault: the
    # current runtime is untouched and no basename error surfaces.
    assert result.lifecycle != Lifecycle.FAULT
    assert result.mode == Mode.MAPPING
    assert 'map' in result.detail.lower()
    assert 'basename' not in result.detail.lower()
    assert systemd.operations == operations

    # A later, well-formed AUTONOMY request still works.
    ok = value.transition(Mode.AUTONOMY, 3, autonomy_map='studio')
    assert ok.mode == Mode.AUTONOMY


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
    assert graph == Counter({name: 1 for name in PERSISTENT_LOCAL_NODES})
    assert all(not state.active for state in systemd.units.values())
    lifecycle_changes = [
        lifecycle for lifecycle, _group in groupby(
            state.lifecycle for state in published
        )
    ]
    assert lifecycle_changes == [
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
        Lifecycle.TRANSITIONING, Lifecycle.STABLE,
    ]


def test_transition_publishes_truthful_runtime_phases(tmp_path):
    complete_map(tmp_path)
    value, _systemd, _graph, published = runtime(tmp_path)

    value.transition(Mode.AUTONOMY, 1, autonomy_map='studio')

    details = [state.detail for state in published]
    assert 'Stopping previous runtime — requesting service stop' in details
    assert 'Stopping previous runtime — waiting for ROS graph cleanup' in details
    assert 'Validating selected map — studio' in details
    assert any(
        detail.startswith('Starting AUTONOMY — requesting ') for detail in details
    )
    assert 'Starting AUTONOMY — waiting for runtime readiness' in details


def test_failed_unit_with_no_process_or_cgroup_is_clean(tmp_path):
    value, systemd, _graph, _published = runtime(tmp_path)
    systemd.units[AUTONOMY_UNIT] = UnitState(
        'failed', 'failed', main_pid=0, control_group=''
    )

    assert value._unit_cleanup_status() == (True, '')


def test_cgroup_blocker_detail_names_unit_pid_and_command(tmp_path):
    value, systemd, _graph, published = runtime(tmp_path)
    systemd.units[AUTONOMY_UNIT] = UnitState(
        'deactivating',
        'stop-sigterm',
        main_pid=1974,
        control_group='/system.slice/runner-mode-autonomy.service',
        cgroup_processes=((1974, 'ros2 run runner_paddock mode_launcher autonomy'),),
    )

    try:
        value._wait_for_unit_cleanup(value._transition_progress)
    except RuntimeError as error:
        detail = str(error)
    else:
        raise AssertionError('populated cgroup unexpectedly passed cleanup')

    assert 'runner-mode-autonomy.service deactivating/stop-sigterm' in detail
    assert 'cgroup /system.slice/runner-mode-autonomy.service' in detail
    assert 'PID 1974 ros2 run runner_paddock mode_launcher autonomy' in detail
    assert any('PID 1974' in state.detail for state in published)


def test_start_failure_cleans_partial_graph_and_faults_idle(tmp_path):
    value, systemd, graph, _published = runtime(tmp_path)
    systemd.fail_start = True

    result = value.transition(Mode.MAPPING, 1)

    assert result.mode == Mode.IDLE
    assert result.lifecycle == Lifecycle.FAULT
    assert 'injected start failure' in result.detail
    assert graph == Counter({name: 1 for name in PERSISTENT_LOCAL_NODES})
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
    assert graph == Counter({name: 1 for name in PERSISTENT_LOCAL_NODES})
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
