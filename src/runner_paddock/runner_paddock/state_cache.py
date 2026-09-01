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

    _EXPIRY_SEC = {
        'pose': 0.5,
        'mode': 0.5,
        'command_authority': 0.5,
        'plan': 2.0,
    }

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._entries = {
            'pose': _Entry(),
            'mode': _Entry(),
            'command_authority': _Entry(),
            'map': _Entry(),
            'plan': _Entry(),
        }

    def update(self, source: str, value: dict[str, Any]) -> bool:
        """Replace a source and return whether its transmitted value changed."""
        if source not in self._entries:
            raise KeyError(source)
        now = self._clock()
        copied = deepcopy(value)
        with self._lock:
            entry = self._entries[source]
            changed = entry.value != copied
            entry.value = copied
            entry.received_at = now
            if changed:
                entry.revision += 1
            return changed

    def state_snapshot(self) -> dict[str, Any]:
        """Copy small state plus local age/freshness and aggregate health."""
        now = self._clock()
        with self._lock:
            values = {
                name: deepcopy(self._entries[name].value)
                for name in ('pose', 'mode', 'command_authority')
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

    def large_snapshot(
        self, source: str
    ) -> tuple[int, Optional[dict[str, Any]]]:
        """
        Return immutable-by-convention map/plan data and its revision.

        ROS callbacks replace complete dictionaries and never mutate a stored
        value. The stable reference avoids copying a potentially large grid
        while holding the cache lock.
        """
        if source not in ('map', 'plan'):
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
        expiry = self._EXPIRY_SEC.get(name)
        fresh = available and (expiry is None or age <= expiry)
        return {
            'available': available,
            'fresh': fresh,
            'age_sec': age,
            'revision': entry.revision,
        }
