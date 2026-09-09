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

"""Persistent rosbag2 MCAP process ownership and recording catalog."""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from typing import Callable

import yaml


RUNNER_DEBUG_TOPICS = (
    '/cmd_vel_nav',
    '/cmd_vel_auto_raw',
    '/cmd_vel_auto',
    '/paddock/manual_demand',
    '/cmd_vel_paddock_manual_raw',
    '/cmd_vel_paddock',
    '/cmd_vel_teleop',
    '/cmd_vel',
    '/drive_adapter/state',
    '/drive_adapter/state_typed',
    '/paddock/control_event',
    '/paddock/control_lease',
    '/paddock/command_authority_state',
    '/paddock/navigation_request',
    '/paddock/navigation_state',
    '/paddock/mode_request',
    '/paddock/mode_state',
    '/paddock/stop_state',
    '/paddock/map_state',
    '/paddock/recording_state',
    '/teleop/control_state',
    '/teleop/active_mode',
    '/teleop/fixed_throttle_setpoint',
    '/wheel/encoder_state',
    '/wheel/odom',
    '/motor/direction',
    '/odometry/filtered',
    '/tf',
    '/tf_static',
    '/scan',
    '/scan_slam',
    '/scan_rf2o',
    '/map',
    '/slam_map',
    '/global_costmap/costmap',
    '/local_costmap/costmap',
    '/plan',
    '/imu/data',
    '/imu/read_errors',
    '/battery',
    '/system/telemetry',
    '/speed_envelope/status',
    '/diagnostics',
)
PROFILES = {
    'runner_debug': RUNNER_DEBUG_TOPICS,
    'everything': None,
}
SAFE_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,95}$')


@dataclass(frozen=True)
class RecordingInfo:
    """One active or finalized recording entry."""

    name: str
    profile: str
    start_time_ns: int
    duration_sec: float
    size_bytes: int
    output_path: str


def safe_recording_name(name: str, now: datetime | None = None) -> str:
    """Return a safe basename or generate a meaningful timestamped one."""
    value = name.strip()
    if not value:
        instant = now or datetime.now(timezone.utc).astimezone()
        return instant.strftime('Runner_%Y%m%d_%H%M%S')
    if not SAFE_NAME.fullmatch(value):
        raise ValueError(
            'name must start with a letter or digit and contain only '
            'letters, digits, dot, underscore, or dash'
        )
    return value


def directory_size(path: Path) -> int:
    """Return the byte size of regular files directly within one bag."""
    try:
        return sum(
            item.stat().st_size for item in path.iterdir() if item.is_file()
        )
    except OSError:
        return 0


def read_recording_info(path: Path) -> RecordingInfo | None:
    """Read finalized rosbag2 metadata without trusting arbitrary paths."""
    metadata = path / 'metadata.yaml'
    if not path.is_dir() or not metadata.is_file():
        return None
    try:
        document = yaml.safe_load(metadata.read_text(encoding='utf-8'))
        info = document['rosbag2_bagfile_information']
        started = int(info['starting_time']['nanoseconds_since_epoch'])
        duration = int(info['duration']['nanoseconds']) / 1e9
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError):
        return None
    profile = ''
    manifest = path / '.paddock-recording.json'
    try:
        profile = str(json.loads(manifest.read_text())['profile'])
    except (KeyError, OSError, TypeError, ValueError):
        pass
    return RecordingInfo(
        name=path.name,
        profile=profile,
        start_time_ns=started,
        duration_sec=duration,
        size_bytes=directory_size(path),
        output_path=str(path),
    )


class RecordingExecutor:
    """Own exactly one recorder process independently of browser sessions."""

    IDLE = 0
    STARTING = 1
    RECORDING = 2
    STOPPING = 3
    FAILED = 4

    def __init__(
        self,
        root: Path,
        *,
        runtime_path: Path = Path('/run/runner-paddock/recording.json'),
        popen: Callable = subprocess.Popen,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self.root = root.resolve()
        self.runtime_path = runtime_path
        self._popen = popen
        self._monotonic = monotonic
        self._wall_time_ns = wall_time_ns
        self.state = self.IDLE
        self.accepted_request_id = 0
        self.name = ''
        self.profile = ''
        self.output_path: Path | None = None
        self.start_time_ns = 0
        self.started_at = 0.0
        self.detail = 'idle'
        self.pid = 0
        self.process = None
        self.stop_requested_at: float | None = None
        self.root.mkdir(parents=True, exist_ok=True)
        self._reconcile_runtime_record()

    def _command(self, output: Path, profile: str) -> list[str]:
        command = [
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '--output', str(output), '--disable-keyboard-controls',
        ]
        topics = PROFILES[profile]
        if topics is None:
            command.append('--all-topics')
        else:
            command.extend(['--topics', *topics])
        return command

    def start(self, request_id: int, name: str, profile: str) -> None:
        """Start one recorder after basename/profile/overwrite validation."""
        self.poll()
        if self.state in (self.STARTING, self.RECORDING, self.STOPPING):
            raise ValueError('a recording is already active')
        profile_key = profile.strip().lower() or 'runner_debug'
        if profile_key not in PROFILES:
            raise ValueError('profile must be runner_debug or everything')
        basename = safe_recording_name(name)
        output = self.root / basename
        if output.exists():
            raise ValueError('recording name already exists; overwrite refused')
        self.state = self.STARTING
        self.accepted_request_id = request_id
        self.name = basename
        self.profile = profile_key
        self.output_path = output
        self.start_time_ns = self._wall_time_ns()
        self.started_at = self._monotonic()
        self.detail = 'starting rosbag2 MCAP recorder'
        try:
            process = self._popen(
                self._command(output, profile_key),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as error:
            self.state = self.FAILED
            self.detail = f'failed to launch recorder: {error}'
            self._persist()
            return
        self.process = process
        self.pid = int(process.pid)
        self._persist()

    def stop(self, request_id: int) -> None:
        """Request a clean rosbag2 SIGINT finalization."""
        self.poll()
        if self.state not in (self.STARTING, self.RECORDING):
            raise ValueError('no recording is active')
        self.accepted_request_id = request_id
        self.state = self.STOPPING
        self.detail = 'waiting for rosbag2 to finalize MCAP'
        self.stop_requested_at = self._monotonic()
        self._signal(signal.SIGINT)
        self._persist()

    def delete(self, request_id: int, name: str) -> None:
        """Delete one inactive bag directory after strict target validation."""
        basename = safe_recording_name(name)
        if self.output_path is not None and basename == self.output_path.name:
            if self.state in (self.STARTING, self.RECORDING, self.STOPPING):
                raise ValueError('cannot delete the active recording')
        target = self.root / basename
        if not target.is_dir() or read_recording_info(target) is None:
            raise ValueError('recording does not exist or is not finalized')
        for item in target.iterdir():
            if item.is_file() or item.is_symlink():
                item.unlink()
            else:
                raise ValueError('recording contains an unexpected directory')
        target.rmdir()
        self.accepted_request_id = request_id
        self.detail = f'deleted {basename}'

    def poll(self) -> None:
        """Reconcile state against the actual recorder process."""
        if self.state not in (self.STARTING, self.RECORDING, self.STOPPING):
            return
        return_code = self._poll_process()
        if return_code is None:
            self._write_manifest()
            if self.state == self.STARTING:
                self.state = self.RECORDING
                self.detail = 'recording'
            elif self.state == self.STOPPING and self.stop_requested_at is not None:
                age = self._monotonic() - self.stop_requested_at
                if age > 15.0:
                    self._signal(signal.SIGKILL)
                elif age > 10.0:
                    self._signal(signal.SIGTERM)
            return
        was_stopping = self.state == self.STOPPING
        complete = (
            self.output_path is not None
            and read_recording_info(self.output_path) is not None
        )
        self.pid = 0
        self.process = None
        self.stop_requested_at = None
        if was_stopping and complete:
            self.state = self.IDLE
            self.detail = 'recording finalized'
            self._clear_runtime()
        else:
            self.state = self.FAILED
            self.detail = (
                f'recorder exited with code {return_code}; '
                f'metadata {"present" if complete else "missing"}'
            )
            self._persist()

    def _write_manifest(self) -> None:
        if self.output_path is None or not self.output_path.is_dir():
            return
        try:
            (self.output_path / '.paddock-recording.json').write_text(
                json.dumps({'profile': self.profile}), encoding='utf-8'
            )
        except OSError:
            pass

    def elapsed_sec(self) -> float:
        """Return monotonic active elapsed time."""
        if self.state not in (self.STARTING, self.RECORDING, self.STOPPING):
            return 0.0
        return max(0.0, self._monotonic() - self.started_at)

    def size_bytes(self) -> int:
        """Return current active bag size cheaply."""
        return directory_size(self.output_path) if self.output_path else 0

    def catalog(self) -> tuple[RecordingInfo, ...]:
        """List finalized MCAP recordings newest first."""
        entries = filter(None, (read_recording_info(path) for path in self.root.iterdir()))
        return tuple(sorted(entries, key=lambda item: item.start_time_ns, reverse=True))

    def process_healthy(self) -> bool:
        """Report process reality, not requested state."""
        return (
            self.state in (self.STARTING, self.RECORDING, self.STOPPING)
            and self._poll_process() is None
        )

    def _poll_process(self):
        if self.process is not None:
            return self.process.poll()
        if self.pid <= 0:
            return 1
        try:
            os.kill(self.pid, 0)
            # An adopted recorder can briefly be a zombie until its previous
            # parent is reaped. kill(pid, 0) still succeeds for zombies, but
            # they are no longer healthy recorder processes.
            stat = Path(f'/proc/{self.pid}/stat').read_text(encoding='ascii')
            if stat.rsplit(')', 1)[1].strip().split()[0] == 'Z':
                return 0
        except OSError:
            return 1
        return None

    def _signal(self, sig: signal.Signals) -> None:
        try:
            os.killpg(self.pid, sig)
        except (OSError, ProcessLookupError):
            pass

    def _persist(self) -> None:
        try:
            self.runtime_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.runtime_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({
                'pid': self.pid,
                'name': self.name,
                'profile': self.profile,
                'output_path': str(self.output_path or ''),
                'start_time_ns': self.start_time_ns,
            }), encoding='utf-8')
            temporary.replace(self.runtime_path)
        except OSError:
            pass

    def _clear_runtime(self) -> None:
        try:
            self.runtime_path.unlink(missing_ok=True)
        except OSError:
            pass

    def _reconcile_runtime_record(self) -> None:
        try:
            record = json.loads(self.runtime_path.read_text(encoding='utf-8'))
            pid = int(record['pid'])
            output = Path(record['output_path']).resolve()
            command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ')
            if (
                output.parent != self.root
                or b'ros2 bag record' not in command
                or str(output).encode() not in command
            ):
                raise ValueError('runtime record does not match a recorder')
            os.kill(pid, 0)
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
            self._clear_runtime()
            return
        self.pid = pid
        self.name = str(record['name'])
        self.profile = str(record['profile'])
        self.output_path = output
        self.start_time_ns = int(record['start_time_ns'])
        elapsed_wall = max(0.0, (self._wall_time_ns() - self.start_time_ns) / 1e9)
        self.started_at = self._monotonic() - elapsed_wall
        self.state = self.RECORDING
        self.detail = 'adopted existing rosbag2 recorder'
