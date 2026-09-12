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
import time
from typing import Callable, Mapping, Tuple
import uuid

import dbus

from runner_paddock.state_machine import Mode


# ModeRequest.operation constants (kept local to avoid a message import here).
OP_SELECT_RUNTIME = 0
OP_NEW_MAP = 1


SYSTEMD_BUS_NAME = 'org.freedesktop.systemd1'
SYSTEMD_OBJECT_PATH = '/org/freedesktop/systemd1'
SYSTEMD_MANAGER_IFACE = 'org.freedesktop.systemd1.Manager'
SYSTEMD_UNIT_IFACE = 'org.freedesktop.systemd1.Unit'
SYSTEMD_SERVICE_IFACE = 'org.freedesktop.systemd1.Service'
DBUS_PROPERTIES_IFACE = 'org.freedesktop.DBus.Properties'


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
})
PERSISTENT_LOCAL_NODES = frozenset({
    '/drive_adapter',
    '/joy_node',
    '/runner_teleop',
    '/twist_mux',
    '/runner_stop_enforcer',
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
    '/runner_navigation_runtime',
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
    cgroup_processes: tuple[tuple[int, str], ...] = ()

    @property
    def active(self) -> bool:
        """Return whether systemd considers the unit fully active."""
        return self.active_state == 'active' and self.sub_state == 'running'

    @property
    def cleanly_inactive(self) -> bool:
        """Return whether the unit has no process or cgroup membership."""
        return self.main_pid == 0 and not self.control_group


@dataclass(frozen=True)
class RuntimeState:
    """One authoritative application-mode snapshot."""

    mode: Mode = Mode.IDLE
    lifecycle: Lifecycle = Lifecycle.STABLE
    accepted_request_id: int = 0
    active_autonomy_map: str = ''
    detail: str = ''
    # Monotonic generation; increments on every successful application start
    # (MAPPING/AUTONOMY) and on NEW MAP. Distinguishes current from stale status.
    runtime_epoch: int = 0
    # Non-empty only while MAPPING is the actual runtime; a fresh value per
    # MAPPING start and per NEW MAP.
    mapping_session_id: str = ''
    # Continuously refreshed capability readiness for the actual runtime.
    ready: bool = False
    readiness_reason: str = 'IDLE'


class SystemdManager:
    """Minimal systemd D-Bus adapter; systemd performs all process teardown."""

    def __init__(self) -> None:
        self._bus = dbus.SystemBus()
        systemd = self._bus.get_object(SYSTEMD_BUS_NAME, SYSTEMD_OBJECT_PATH)
        self._manager = dbus.Interface(systemd, SYSTEMD_MANAGER_IFACE)

    def _unit_properties(self, unit: str) -> dbus.Interface:
        path = self._manager.LoadUnit(unit)
        proxy = self._bus.get_object(SYSTEMD_BUS_NAME, path)
        return dbus.Interface(proxy, DBUS_PROPERTIES_IFACE)

    def state(self, unit: str) -> UnitState:
        """Read state without treating inactive units as command failures."""
        try:
            properties = self._unit_properties(unit)
            active_state = str(properties.Get(SYSTEMD_UNIT_IFACE, 'ActiveState'))
            sub_state = str(properties.Get(SYSTEMD_UNIT_IFACE, 'SubState'))
            main_pid = int(properties.Get(SYSTEMD_SERVICE_IFACE, 'MainPID'))
            control_group = str(
                properties.Get(SYSTEMD_SERVICE_IFACE, 'ControlGroup')
            )
            cgroup_processes = self._cgroup_processes(control_group)
        except dbus.exceptions.DBusException as error:
            raise RuntimeError(f'cannot inspect {unit}: {error}') from error
        return UnitState(
            active_state=active_state,
            sub_state=sub_state,
            main_pid=main_pid,
            control_group=control_group,
            cgroup_processes=cgroup_processes,
        )

    @staticmethod
    def _cgroup_processes(control_group: str) -> tuple[tuple[int, str], ...]:
        """Describe current cgroup members for transition diagnostics."""
        if not control_group:
            return ()
        try:
            raw_pids = (
                Path('/sys/fs/cgroup')
                / control_group.lstrip('/')
                / 'cgroup.procs'
            ).read_text(encoding='ascii').splitlines()
        except OSError:
            return ()
        processes = []
        for raw_pid in raw_pids:
            try:
                pid = int(raw_pid)
                command = Path(f'/proc/{pid}/cmdline').read_bytes()
                command_text = command.replace(b'\0', b' ').decode(
                    errors='replace'
                ).strip()
            except (OSError, ValueError):
                continue
            processes.append((pid, command_text or '<unknown>'))
        return tuple(processes)

    def start(self, unit: str) -> None:
        """Start one fixed mode unit; systemd polkit-authorizes the call."""
        try:
            self._manager.StartUnit(unit, 'replace')
        except dbus.exceptions.DBusException as error:
            raise RuntimeError(f'cannot start {unit}: {error}') from error

    def stop(self, unit: str) -> None:
        """Stop a unit; KillMode=control-group owns descendant teardown."""
        try:
            self._manager.StopUnit(unit, 'replace')
        except dbus.exceptions.DBusException as error:
            raise RuntimeError(f'cannot stop {unit}: {error}') from error


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
        transition_timeout: float = 45.0,
        poll_period: float = 0.1,
        ownership_ready: Callable[[Mode], bool] | None = None,
        capability_ready: Callable[[Mode], Tuple[bool, str]] | None = None,
        begin_quiescence: Callable[[], None] | None = None,
        quiescence_ready: Callable[[], Tuple[bool, str]] | None = None,
        session_id_factory: Callable[[], str] | None = None,
    ) -> None:
        self.systemd = systemd
        self.graph_nodes = graph_nodes
        self.publish = publish
        self.map_directory = map_directory
        self.map_file = map_file
        self.transition_timeout = transition_timeout
        self.poll_period = poll_period
        self.ownership_ready = ownership_ready or (lambda _mode: True)
        # Capability-specific evidence supplied by the ROS node; ROS-free tests
        # default it to "ready" so structural orchestration stays isolated.
        self.capability_ready = capability_ready or (lambda _mode: (True, ''))
        self.begin_quiescence = begin_quiescence or (lambda: None)
        self.quiescence_ready = quiescence_ready or (lambda: (True, ''))
        self.session_id_factory = (
            session_id_factory or (lambda: uuid.uuid4().hex[:12])
        )
        self.state = RuntimeState(ready=True, readiness_reason='')

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

    def _unit_cleanup_status(self) -> tuple[bool, str]:
        """Return whether mode units are empty and identify every blocker."""
        blockers = []
        for unit in MODE_UNITS:
            state = self.systemd.state(unit)
            if state.cleanly_inactive:
                continue
            location = state.control_group or '<no cgroup reported>'
            processes = ', '.join(
                f'PID {pid} {command}'
                for pid, command in state.cgroup_processes
            )
            if not processes and state.main_pid:
                processes = f'MainPID {state.main_pid}'
            if not processes:
                processes = 'no process details available'
            blockers.append(
                f'{unit} {state.active_state}/{state.sub_state}, '
                f'cgroup {location}: {processes}'
            )
        return not blockers, '; '.join(blockers)

    def _wait_for_unit_cleanup(
        self, progress: Callable[[str], None] | None = None
    ) -> None:
        """Wait for empty mode cgroups while publishing changing blockers."""
        deadline = time.monotonic() + self.transition_timeout
        last_reason = ''
        while time.monotonic() < deadline:
            clean, reason = self._unit_cleanup_status()
            if clean:
                return
            if progress is not None and reason != last_reason:
                progress('Stopping previous runtime — waiting for ' + reason)
                last_reason = reason
            time.sleep(self.poll_period)
        suffix = f': {last_reason}' if last_reason else ''
        raise RuntimeError(f'timed out waiting for empty mode cgroups{suffix}')

    def _resources_gone(self) -> bool:
        counts = Counter(self.graph_nodes())
        return not any(counts.get(node, 0) for node in MODE_NODES)

    def _structural_reason(self, mode: Mode) -> str:
        """Return '' when systemd/graph ownership matches ``mode`` exactly."""
        unit = MAPPING_UNIT if mode == Mode.MAPPING else AUTONOMY_UNIT
        if not self.systemd.state(unit).active:
            return f'{unit} is not active/running'
        counts = Counter(self.graph_nodes())
        forbidden = AUTONOMY_ONLY_NODES if mode == Mode.MAPPING else frozenset()
        required = set(COMMON_NODES) | set(PERSISTENT_LOCAL_NODES)
        if mode == Mode.AUTONOMY:
            required |= set(AUTONOMY_ONLY_NODES)
        # Presence, not exact name count: a duplicate node *name* is DDS
        # discovery residue, not proof of a duplicate owner (v1.3 s2). The
        # authoritative single-owner enforcement is per topic/edge in
        # ownership_ready(); the forbidden set still blocks mapping/autonomy
        # node coexistence, which is the real ownership-conflict risk.
        missing = sorted(node for node in required if counts.get(node, 0) < 1)
        if missing:
            return f'missing node(s): {", ".join(missing)}'
        present_forbidden = sorted(
            node for node in forbidden if counts.get(node, 0)
        )
        if present_forbidden:
            return f'unexpected node(s): {", ".join(present_forbidden)}'
        if not self.ownership_ready(mode):
            return 'single-writer ownership check failed'
        return ''

    def _ready(self, mode: Mode) -> bool:
        return not self._structural_reason(mode)

    def _fully_ready(self, mode: Mode) -> Tuple[bool, str]:
        """Structural ownership plus capability-specific runtime evidence."""
        reason = self._structural_reason(mode)
        if reason:
            return False, reason
        ok, capability_reason = self.capability_ready(mode)
        return ok, ('' if ok else (capability_reason or 'capability not ready'))

    def refresh(self) -> RuntimeState:
        """Re-evaluate continuous readiness without changing the lifecycle."""
        if self.state.lifecycle != Lifecycle.STABLE:
            if self.state.ready:
                self._set(ready=False)
            return self.state
        if self.state.mode == Mode.IDLE:
            if not self.state.ready or self.state.readiness_reason:
                self._set(ready=True, readiness_reason='')
            return self.state
        ok, reason = self._fully_ready(self.state.mode)
        if ok != self.state.ready or reason != self.state.readiness_reason:
            self._set(ready=ok, readiness_reason=reason)
        return self.state

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

    def _new_session_id(self) -> str:
        return f'{self.state.runtime_epoch + 1}-{self.session_id_factory()}'

    def _stop_all(self, progress: Callable[[str], None] | None = None) -> None:
        # Always issue both stops: this also collapses ambiguous/conflicting state.
        if progress is not None:
            progress('Stopping previous runtime — requesting service stop')
        errors = []
        for unit in MODE_UNITS:
            try:
                self.systemd.stop(unit)
            except RuntimeError as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('; '.join(errors))
        self._wait_for_unit_cleanup(progress)
        if progress is not None:
            progress('Stopping previous runtime — waiting for ROS graph cleanup')
        self._wait(self._resources_gone, 'mode-scoped ROS resources to disappear')

    def _transition_progress(self, detail: str) -> None:
        """Publish one truthful, non-percentage transition phase."""
        if detail != self.state.detail:
            self._set(detail=detail, readiness_reason=detail)

    def _wait_for_readiness(self, mode: Mode) -> None:
        """Wait for runtime evidence while publishing its changing blocker."""
        deadline = time.monotonic() + self.transition_timeout
        last_reason = ''
        while time.monotonic() < deadline:
            ok, reason = self._fully_ready(mode)
            if ok:
                return
            if reason != last_reason:
                self._transition_progress(
                    f'Starting {mode.name} — {reason}'
                )
                last_reason = reason
            time.sleep(self.poll_period)
        suffix = f': {last_reason}' if last_reason else ''
        raise RuntimeError(
            f'timed out waiting for {mode.name} readiness{suffix}'
        )

    def _wait_for_quiescence(self) -> None:
        """Wait for authority revocation and later stationary evidence."""
        deadline = time.monotonic() + self.transition_timeout
        last_reason = ''
        while time.monotonic() < deadline:
            ok, reason = self.quiescence_ready()
            if ok:
                return
            if reason != last_reason:
                self._transition_progress(
                    'Resetting MAPPING — ' + (
                        reason or 'waiting for motion to become safe'
                    )
                )
                last_reason = reason
            time.sleep(self.poll_period)
        suffix = f': {last_reason}' if last_reason else ''
        raise RuntimeError(
            f'timed out waiting for confirmed stationary state{suffix}'
        )

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
            mapping_session_id='',
            ready=False,
            readiness_reason=detail,
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
                self._set(
                    mode=Mode.IDLE,
                    lifecycle=Lifecycle.STABLE,
                    detail='',
                    mapping_session_id='',
                    ready=True,
                    readiness_reason='',
                )
                return self.state
            mode = (
                Mode.MAPPING if active[0] == MAPPING_UNIT else Mode.AUTONOMY
            )
            if not self._ready(mode):
                # A freshly active unit may still be discovering its graph.
                # Give structural ownership a bounded grace period before
                # concluding the graph is genuinely partial and tearing it down.
                try:
                    self._wait(
                        lambda: self._ready(mode),
                        f'{mode.name} graph discovery',
                    )
                except RuntimeError:
                    return self._fail(
                        f'{mode.name} unit is active but graph is partial: '
                        f'{self._structural_reason(mode)}'
                    )
            selected_map = self.state.active_autonomy_map
            if mode == Mode.AUTONOMY:
                selected_map = self._stored_map()
                validate_map_bundle(selected_map, map_directory=self.map_directory)
            ok, reason = self._fully_ready(mode)
            session_id = (
                self._new_session_id() if mode == Mode.MAPPING else ''
            )
            self._set(
                mode=mode,
                lifecycle=Lifecycle.STABLE,
                active_autonomy_map=selected_map,
                detail='',
                runtime_epoch=max(self.state.runtime_epoch, 1),
                mapping_session_id=session_id,
                ready=ok,
                readiness_reason=reason,
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
        operation: int = OP_SELECT_RUNTIME,
    ) -> RuntimeState:
        """
        Stop fully, validate/start, verify, then publish actual mode.

        ``operation`` OP_NEW_MAP (only with ``requested`` MAPPING) forces a fresh
        mapping SLAM session even when MAPPING is already the stable runtime.
        """
        if request_id <= self.state.accepted_request_id:
            return self.state
        if not isinstance(requested, Mode):
            return self.state
        if requested == Mode.AUTONOMY and not autonomy_map:
            # Operator precondition, not a runtime fault: AUTONOMY needs a map
            # selected first. Acknowledge the request but reject it in place --
            # do not stop the current runtime or enter FAULT over a missing
            # selection. The gateway/UI keeps AUTONOMY unavailable until a
            # completed map is selected; this is the backstop.
            self._set(
                accepted_request_id=request_id,
                detail='AUTONOMY needs a selected map',
            )
            return self.state
        new_map = operation == OP_NEW_MAP and requested == Mode.MAPPING
        if new_map and not (
            self.state.lifecycle == Lifecycle.STABLE
            and self.state.mode == Mode.MAPPING
            and bool(self.state.mapping_session_id)
        ):
            self._set(
                accepted_request_id=request_id,
                detail='NEW MAP rejected: requires an active stable MAPPING session',
            )
            return self.state
        same_selection = (
            requested != Mode.AUTONOMY
            or autonomy_map == self.state.active_autonomy_map
        )
        if (
            not new_map
            and self.state.lifecycle == Lifecycle.STABLE
            and requested == self.state.mode
            and same_selection
        ):
            # Idempotent re-selection: never restarts, never resets a map.
            self._set(accepted_request_id=request_id)
            return self.state
        detail = (
            'starting fresh mapping session'
            if new_map else f'transitioning to {requested.name}'
        )
        previous = self.state
        if new_map:
            self.begin_quiescence()
        self._set(
            lifecycle=Lifecycle.TRANSITIONING,
            accepted_request_id=request_id,
            detail=detail,
            ready=False,
            readiness_reason=detail,
            mapping_session_id='',
        )
        if new_map:
            try:
                self._wait_for_quiescence()
            except RuntimeError as error:
                ok, reason = self._fully_ready(Mode.MAPPING)
                self._set(
                    mode=previous.mode,
                    lifecycle=Lifecycle.STABLE,
                    detail=f'NEW MAP rejected: {error}',
                    runtime_epoch=previous.runtime_epoch,
                    mapping_session_id=previous.mapping_session_id,
                    ready=ok,
                    readiness_reason=reason,
                )
                return self.state
        try:
            self._stop_all(self._transition_progress)
            if requested == Mode.IDLE:
                self._set(
                    mode=Mode.IDLE,
                    lifecycle=Lifecycle.STABLE,
                    detail='',
                    mapping_session_id='',
                    ready=True,
                    readiness_reason='',
                )
                return self.state
            if requested == Mode.AUTONOMY:
                self._transition_progress(
                    f'Validating selected map — {autonomy_map}'
                )
                validate_map_bundle(
                    autonomy_map,
                    map_directory=self.map_directory,
                )
                self._write_map(autonomy_map)
                unit = AUTONOMY_UNIT
            else:
                unit = MAPPING_UNIT
            self._transition_progress(
                f'Starting {requested.name} — requesting {unit}'
            )
            self.systemd.start(unit)
            self._transition_progress(
                f'Starting {requested.name} — waiting for runtime readiness'
            )
            self._wait_for_readiness(requested)
            ok, reason = self._fully_ready(requested)
            epoch = self.state.runtime_epoch + 1
            session_id = (
                f'{epoch}-{self.session_id_factory()}'
                if requested == Mode.MAPPING else ''
            )
            changes = {
                'mode': requested,
                'lifecycle': Lifecycle.STABLE,
                'detail': '',
                'runtime_epoch': epoch,
                'mapping_session_id': session_id,
                'ready': ok,
                'readiness_reason': reason,
            }
            if requested == Mode.AUTONOMY:
                changes['active_autonomy_map'] = autonomy_map
            self._set(**changes)
            return self.state
        except (OSError, RuntimeError, ValueError) as error:
            return self._fail(f'{requested.name} transition failed: {error}')
