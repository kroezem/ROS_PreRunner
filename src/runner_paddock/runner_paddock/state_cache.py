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

"""Thread-safe latest-state storage shared by ROS and the web event loop."""

from copy import deepcopy
from dataclasses import dataclass
import threading
import time
from typing import Any, Callable, Optional


@dataclass
class _Entry:
    """One latest-wins source value and its local reception time."""

    value: Optional[dict[str, Any]] = None
    received_at: Optional[float] = None
    revision: int = 0


class StateCache:
    """Hold immutable-by-convention JSON values behind one short lock."""

    _GRID_SOURCES = ('map', 'global_costmap', 'local_costmap')
    _LARGE_SOURCES = (*_GRID_SOURCES, 'plan')

    _EXPIRY_SEC = {
        'pose': 0.5,
        'mode': 0.5,
        'command_authority': 0.5,
        'control_lease': 0.5,
        'stop_state': 1.0,
        'local_control': 0.5,
        'adapter_state': 0.5,
        'encoder_state': 0.2,
        'map_state': 3.0,
        'navigation_state': 2.0,
        'plan': 2.0,
        'global_costmap': 2.0,
        'local_costmap': 1.0,
        'recording_state': 2.0,
        'system_telemetry': 2.5,
        'battery': 2.5,
    }

    _SMALL_SOURCES = (
        'pose', 'mode', 'command_authority', 'control_lease', 'stop_state',
        'local_control', 'adapter_state', 'gateway', 'config', 'map_state',
        'navigation_state', 'encoder_state', 'initial_pose',
        'recording_state',
        'system_telemetry', 'battery', 'obstacle_processing',
        'autonomy_tuning',
    )

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._entries = {
            'pose': _Entry(),
            'mode': _Entry(),
            'command_authority': _Entry(),
            'control_lease': _Entry(),
            'stop_state': _Entry(),
            'local_control': _Entry(),
            'adapter_state': _Entry(),
            'encoder_state': _Entry(),
            'initial_pose': _Entry(),
            'gateway': _Entry(),
            'config': _Entry(),
            'map_state': _Entry(),
            'navigation_state': _Entry(),
            'recording_state': _Entry(),
            'system_telemetry': _Entry(),
            'battery': _Entry(),
            'obstacle_processing': _Entry(),
            'autonomy_tuning': _Entry(),
            'map': _Entry(),
            'plan': _Entry(),
            'global_costmap': _Entry(),
            'local_costmap': _Entry(),
        }

    def update(self, source: str, value: dict[str, Any]) -> bool:
        """Replace a source and return whether its transmitted value changed."""
        if source not in self._entries:
            raise KeyError(source)
        now = self._clock()
        # ROS callbacks build replacement-only large payloads. Retaining their
        # reference avoids walking every grid merely to make an identical copy.
        copied = value if source in self._GRID_SOURCES else deepcopy(value)
        with self._lock:
            entry = self._entries[source]
            changed = entry.value != copied
            entry.value = copied
            entry.received_at = now
            if changed:
                entry.revision += 1
            return changed

    def invalidate(self, source: str) -> bool:
        """Discard one cached source and revision its transmitted tombstone."""
        if source not in self._entries:
            raise KeyError(source)
        with self._lock:
            entry = self._entries[source]
            if entry.value is None:
                return False
            entry.value = None
            entry.received_at = None
            entry.revision += 1
            return True

    def state_snapshot(self) -> dict[str, Any]:
        """Copy small state plus local age/freshness and aggregate health."""
        now = self._clock()
        with self._lock:
            values = {
                name: deepcopy(self._entries[name].value)
                for name in self._SMALL_SOURCES
            }
            sources = {
                name: self._source_health(name, entry, now)
                for name, entry in self._entries.items()
            }

        required = (sources['pose'], sources['map'])
        if not any(item['available'] for item in required):
            health = 'starting'
        elif all(item['fresh'] for item in required):
            health = 'ok'
        else:
            health = 'degraded'
        return {
            **values,
            'health': {
                'status': health,
                'sources': sources,
            },
        }

    def state_revision(self) -> tuple[tuple[int, ...], tuple[bool, ...]]:
        """
        Return the cheap semantic key for the next small-state frame.

        Payload revisions cover received value changes.  Freshness is derived
        directly from reception times so silence changes this key at an expiry
        boundary without requiring a full copied snapshot.
        """
        now = self._clock()
        with self._lock:
            revisions = tuple(
                self._entries[name].revision for name in self._entries
            )
            freshness = tuple(
                self._source_is_fresh(name, entry, now)
                for name, entry in self._entries.items()
            )
        return revisions, freshness

    def large_snapshot(
        self, source: str
    ) -> tuple[int, Optional[dict[str, Any]]]:
        """
        Return immutable-by-convention large grid/path data and its revision.

        ROS callbacks replace complete dictionaries and never mutate a stored
        value. The stable reference avoids copying a potentially large grid
        while holding the cache lock.
        """
        if source not in self._LARGE_SOURCES:
            raise KeyError(source)
        with self._lock:
            entry = self._entries[source]
            return entry.revision, entry.value

    def _source_health(
        self, name: str, entry: _Entry, now: float
    ) -> dict[str, Any]:
        available = entry.value is not None and entry.received_at is not None
        age = None if entry.received_at is None else max(
            0.0, now - entry.received_at
        )
        return {
            'available': available,
            'fresh': self._source_is_fresh(name, entry, now),
            'age_sec': age,
            'revision': entry.revision,
        }

    def _source_is_fresh(
        self, name: str, entry: _Entry, now: float
    ) -> bool:
        if entry.value is None or entry.received_at is None:
            return False
        expiry = self._EXPIRY_SEC.get(name)
        return expiry is None or now - entry.received_at <= expiry
