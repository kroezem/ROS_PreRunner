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

"""ROS-independent, systemd-backed Paddock application-mode runtime."""

from collections import Counter
from dataclasses import dataclass
from enum import IntEnum
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Callable, Mapping

from runner_paddock.state_machine import Mode


MAPPING_UNIT = 'runner-mode-mapping.service'
AUTONOMY_UNIT = 'runner-mode-autonomy.service'
MODE_UNITS = (MAPPING_UNIT, AUTONOMY_UNIT)
MAP_DIRECTORY = Path(os.environ.get(
    'PADDOCK_MAP_DIRECTORY', '/home/matti/runner_ws/maps'
))
AUTONOMY_MAP_FILE = Path(os.environ.get(
    'PADDOCK_AUTONOMY_MAP_FILE', '/run/runner-paddock/autonomy-map'
))
MAP_BASENAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')

COMMON_NODES = frozenset({
    '/LD19',
    '/base_link_to_base_laser',
    '/base_link_to_imu_link',
    '/bno085',
    '/ekf_node',
    '/rf2o_laser_odometry',
    '/rf2o_scan_canonicalizer',
    '/scan_rebinner',
    '/slam_toolbox',
    '/joy_node',
    '/keyboard_bridge',
    '/runner_teleop',
    '/twist_mux',
})
AUTONOMY_ONLY_NODES = frozenset({
    '/map_server',
    '/planner_server',
    '/controller_server',
    '/bt_navigator',
    '/bt_navigator_navigate_to_pose_rclcpp_node',
    '/bt_navigator_navigate_through_poses_rclcpp_node',
    '/global_costmap/global_costmap',
    '/local_costmap/local_costmap',
    '/lifecycle_manager_navigation',
    '/foxglove_goal_bridge',
    '/drive_adapter',
    '/speed_envelope_observer',
})
MODE_NODES = COMMON_NODES | AUTONOMY_ONLY_NODES


class Lifecycle(IntEnum):
    """Mode transition lifecycle; values mirror ModeState.STATUS_* constants."""

    STABLE = 0
    TRANSITIONING = 1
    FAULT = 2


@dataclass(frozen=True)
class UnitState:
    """Relevant systemd state for one mode unit."""

    active_state: str
    sub_state: str = ''
    main_pid: int = 0
    control_group: str = ''

    @property
    def active(self) -> bool:
        """Return whether systemd considers the unit fully active."""
        return self.active_state == 'active' and self.sub_state == 'running'

    @property
    def cleanly_inactive(self) -> bool:
        """Return whether the unit has no process or cgroup membership."""
        return (
            self.active_state == 'inactive'
            and self.main_pid == 0
            and not self.control_group
        )


@dataclass(frozen=True)
class RuntimeState:
    """One authoritative application-mode snapshot."""

    mode: Mode = Mode.IDLE
    lifecycle: Lifecycle = Lifecycle.STABLE
    accepted_request_id: int = 0
    active_autonomy_map: str = ''
    detail: str = ''


class SystemdManager:
    """Minimal systemctl adapter; systemd performs all process teardown."""

    def _run(self, *arguments: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            ('systemctl', *arguments),
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )

    def state(self, unit: str) -> UnitState:
        """Read state without treating inactive units as command failures."""
        result = self._run(
            'show', unit,
            '--property=ActiveState,SubState,MainPID,ControlGroup',
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f'cannot inspect {unit}: {detail}')
        values = dict(
            line.split('=', 1)
            for line in result.stdout.splitlines()
            if '=' in line
        )
        if not all(
            key in values
            for key in ('ActiveState', 'SubState', 'MainPID', 'ControlGroup')
        ):
            raise RuntimeError(f'incomplete systemd state for {unit}')
        return UnitState(
            active_state=values['ActiveState'],
            sub_state=values['SubState'],
            main_pid=int(values['MainPID'] or '0'),
            control_group=values['ControlGroup'],
        )

    def start(self, unit: str) -> None:
        """Start one fixed mode unit and wait for systemd's start job."""
        result = self._run('start', unit)
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f'cannot start {unit}: {detail}')

    def stop(self, unit: str) -> None:
        """Stop a unit; KillMode=control-group owns descendant teardown."""
        result = self._run('stop', unit)
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f'cannot stop {unit}: {detail}')


def validate_map_bundle(
    basename: str,
    *,
    map_directory: Path = MAP_DIRECTORY,
) -> tuple[Path, Path, Path, Path]:
    """Validate a basename and its posegraph/data/YAML/image bundle."""
    if (
        not basename
        or not MAP_BASENAME.fullmatch(basename)
        or '..' in basename
        or Path(basename).name != basename
    ):
        raise ValueError(
            'autonomy map must be a basename containing only letters, '
            'numbers, dot, underscore, or hyphen'
        )
    base = map_directory / basename
    posegraph = Path(f'{base}.posegraph')
    data = Path(f'{base}.data')
    yaml_file = Path(f'{base}.yaml')
    missing = [path for path in (posegraph, data, yaml_file) if not path.is_file()]
    image = None
    if not missing:
        for line in yaml_file.read_text(encoding='utf-8').splitlines():
            if line.strip().startswith('image:'):
                value = line.split(':', 1)[1].strip().strip('"\'')
                if value:
                    image = Path(value)
                    if not image.is_absolute():
                        image = yaml_file.parent / image
                break
        if image is None:
            raise ValueError(f'{yaml_file} has no image entry')
        if not image.is_file():
            missing.append(image)
    if missing:
        raise ValueError(
            'incomplete autonomy map bundle: '
            + ', '.join(str(path) for path in missing)
        )
    return posegraph, data, yaml_file, image


class ModeRuntime:
    """Serialize fail-closed transitions across fixed systemd units."""

    def __init__(
        self,
        systemd,
        graph_nodes: Callable[[], Mapping[str, int]],
        publish: Callable[[RuntimeState], None],
        *,
        map_directory: Path = MAP_DIRECTORY,
        map_file: Path = AUTONOMY_MAP_FILE,
        transition_timeout: float = 20.0,
        poll_period: float = 0.1,
        ownership_ready: Callable[[Mode], bool] | None = None,
    ) -> None:
        self.systemd = systemd
        self.graph_nodes = graph_nodes
        self.publish = publish
        self.map_directory = map_directory
        self.map_file = map_file
        self.transition_timeout = transition_timeout
        self.poll_period = poll_period
        self.ownership_ready = ownership_ready or (lambda _mode: True)
        self.state = RuntimeState()

    def _set(self, **changes) -> None:
        values = self.state.__dict__ | changes
        self.state = RuntimeState(**values)
        self.publish(self.state)

    def _wait(self, predicate: Callable[[], bool], description: str) -> None:
        deadline = time.monotonic() + self.transition_timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(self.poll_period)
        raise RuntimeError(f'timed out waiting for {description}')

    def _all_units_clean(self) -> bool:
        return all(self.systemd.state(unit).cleanly_inactive for unit in MODE_UNITS)

    def _resources_gone(self) -> bool:
        counts = Counter(self.graph_nodes())
        return not any(counts.get(node, 0) for node in MODE_NODES)

    def _ready(self, mode: Mode) -> bool:
        unit = MAPPING_UNIT if mode == Mode.MAPPING else AUTONOMY_UNIT
        if not self.systemd.state(unit).active:
            return False
        counts = Counter(self.graph_nodes())
        required = COMMON_NODES
        forbidden = AUTONOMY_ONLY_NODES if mode == Mode.MAPPING else frozenset()
        return (
            all(counts.get(node, 0) == 1 for node in required)
            and all(counts.get(node, 0) == 1 for node in (
                AUTONOMY_ONLY_NODES if mode == Mode.AUTONOMY else frozenset()
            ))
            and not any(counts.get(node, 0) for node in forbidden)
            and self.ownership_ready(mode)
        )

    def _write_map(self, basename: str) -> None:
        self.map_file.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        temporary = self.map_file.with_suffix('.tmp')
        temporary.write_text(f'{basename}\n', encoding='utf-8')
        os.chmod(temporary, 0o644)
        temporary.replace(self.map_file)

    def _stored_map(self) -> str:
        if not self.map_file.is_file():
            return ''
        return self.map_file.read_text(encoding='utf-8').strip()

    def _stop_all(self) -> None:
        # Always issue both stops: this also collapses ambiguous/conflicting state.
        errors = []
        for unit in MODE_UNITS:
            try:
                self.systemd.stop(unit)
            except RuntimeError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('; '.join(errors))
        self._wait(self._all_units_clean, 'empty mode cgroups')
        self._wait(self._resources_gone, 'mode-scoped ROS resources to disappear')

    def _fail(self, detail: str) -> RuntimeState:
        cleanup = ''
        try:
            self._stop_all()
        except RuntimeError as error:
            cleanup = f'; cleanup failed: {error}'
        self._set(
            mode=Mode.IDLE,
            lifecycle=Lifecycle.FAULT,
            detail=detail + cleanup,
        )
        return self.state

    def reconcile(self) -> RuntimeState:
        """Derive actual mode from systemd and reject partial/foreign graphs."""
        try:
            states = {unit: self.systemd.state(unit) for unit in MODE_UNITS}
            active = [unit for unit, state in states.items() if state.active]
            ambiguous = [
                unit for unit, state in states.items()
                if not state.active and not state.cleanly_inactive
            ]
            if ambiguous or len(active) > 1:
                return self._fail(
                    'ambiguous systemd mode state: '
                    + ', '.join(ambiguous or active)
                )
            if not active:
                if not self._resources_gone():
                    return self._fail(
                        'mode-scoped ROS resources exist outside active mode units'
                    )
                self._set(mode=Mode.IDLE, lifecycle=Lifecycle.STABLE, detail='')
                return self.state
            mode = (
                Mode.MAPPING if active[0] == MAPPING_UNIT else Mode.AUTONOMY
            )
            if not self._ready(mode):
                return self._fail(f'{mode.name} unit is active but graph is partial')
            selected_map = self.state.active_autonomy_map
            if mode == Mode.AUTONOMY:
                selected_map = self._stored_map()
                validate_map_bundle(selected_map, map_directory=self.map_directory)
            self._set(
                mode=mode,
                lifecycle=Lifecycle.STABLE,
                active_autonomy_map=selected_map,
                detail='',
            )
            return self.state
        except (OSError, RuntimeError, ValueError) as error:
            return self._fail(f'reconciliation failed: {error}')

    def transition(
        self,
        requested: Mode,
        request_id: int,
        *,
        autonomy_map: str = '',
    ) -> RuntimeState:
        """Stop fully, validate/start, verify, then publish actual mode."""
        if request_id <= self.state.accepted_request_id:
            return self.state
        if not isinstance(requested, Mode):
            return self.state
        same_selection = (
            requested != Mode.AUTONOMY
            or autonomy_map == self.state.active_autonomy_map
        )
        if (
            self.state.lifecycle == Lifecycle.STABLE
            and requested == self.state.mode
            and same_selection
        ):
            self._set(accepted_request_id=request_id)
            return self.state
        self._set(
            lifecycle=Lifecycle.TRANSITIONING,
            accepted_request_id=request_id,
            detail=f'transitioning to {requested.name}',
        )
        try:
            self._stop_all()
            if requested == Mode.IDLE:
                self._set(mode=Mode.IDLE, lifecycle=Lifecycle.STABLE, detail='')
                return self.state
            if requested == Mode.AUTONOMY:
                validate_map_bundle(
                    autonomy_map,
                    map_directory=self.map_directory,
                )
                self._write_map(autonomy_map)
                unit = AUTONOMY_UNIT
            else:
                unit = MAPPING_UNIT
            self.systemd.start(unit)
            self._wait(lambda: self._ready(requested), f'{requested.name} readiness')
            changes = {
                'mode': requested,
                'lifecycle': Lifecycle.STABLE,
                'detail': '',
            }
            if requested == Mode.AUTONOMY:
                changes['active_autonomy_map'] = autonomy_map
            self._set(**changes)
            return self.state
        except (OSError, RuntimeError, ValueError) as error:
            return self._fail(f'{requested.name} transition failed: {error}')
