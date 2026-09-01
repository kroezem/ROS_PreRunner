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

"""Bounded latest-wins fan-out from the ROS cache to web clients."""

import asyncio
import logging

from runner_paddock.protocol import encode_message
from runner_paddock.state_cache import StateCache


LOGGER = logging.getLogger(__name__)
STREAM_HZ = 10.0
_FRAME_KINDS = ('map', 'plan', 'state')


class ClientConnection:
    """At most one pending frame of each kind for one read-only client."""

    def __init__(self) -> None:
        self._pending: dict[str, str] = {}
        self._available = asyncio.Event()
        self._closed = False
        self.revisions = {'map': 0, 'plan': 0}

    @property
    def pending_count(self) -> int:
        """Return the bounded number of frames waiting for this client."""
        return len(self._pending)

    def offer(self, kind: str, frame: str) -> None:
        """Replace the pending frame of one kind without ever blocking."""
        if kind not in _FRAME_KINDS:
            raise KeyError(kind)
        self._pending[kind] = frame
        self._available.set()

    def close(self) -> None:
        """Wake a parked reader so its sender task can unwind on shutdown."""
        self._closed = True
        self._available.set()

    async def next_frame(self) -> str:
        """Wait for and remove the oldest pending frame kind."""
        await self._available.wait()
        if not self._pending:
            raise RuntimeError('client connection closed')
        kind = next(iter(self._pending))
        frame = self._pending.pop(kind)
        if not self._pending and not self._closed:
            self._available.clear()
        return frame


class ClientHub:
    """Manage independent bounded queues for any number of readers."""

    def __init__(self) -> None:
        self._clients: set[ClientConnection] = set()

    @property
    def client_count(self) -> int:
        """Return the number of currently registered readers."""
        return len(self._clients)

    def register(self) -> ClientConnection:
        """Register a reader on the asyncio/WebSocket thread."""
        client = ClientConnection()
        self._clients.add(client)
        return client

    def unregister(self, client: ClientConnection) -> None:
        """Forget a reader; unregistering twice is harmless."""
        self._clients.discard(client)

    def close(self) -> None:
        """Wake every registered reader so its sender task can exit."""
        for client in self._clients:
            client.close()

    def publish(self, cache: StateCache) -> None:
        """Offer one state tick and only unseen map/plan revisions."""
        if not self._clients:
            return

        try:
            state_frame = encode_message('state', **cache.state_snapshot())
        except (TypeError, ValueError) as error:
            LOGGER.warning('Rejected invalid state snapshot: %s', error)
        else:
            for client in self._clients:
                client.offer('state', state_frame)

        for kind in ('map', 'plan'):
            revision, value = cache.large_snapshot(kind)
            recipients = [
                client for client in self._clients
                if value is not None and client.revisions[kind] != revision
            ]
            if not recipients:
                continue
            try:
                frame = encode_message(
                    kind, revision=revision, **value
                )
            except (TypeError, ValueError) as error:
                LOGGER.warning('Rejected invalid %s snapshot: %s', kind, error)
                continue
            for client in recipients:
                client.offer(kind, frame)
                client.revisions[kind] = revision


class StateStreamer:
    """Sample the cache at a fixed cadence on the asyncio event loop."""

    def __init__(
        self, cache: StateCache, hub: ClientHub, rate_hz: float = STREAM_HZ
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')
        self._cache = cache
        self._hub = hub
        self._period = 1.0 / rate_hz
        self._task: asyncio.Task[None] | None = None

    def start(self) -> None:
        """Start the single cache sampling task."""
        if self._task is not None:
            raise RuntimeError('state streamer is already running')
        self._task = asyncio.create_task(
            self._run(), name='paddock-state-stream'
        )

    async def stop(self) -> None:
        """Cancel and await the cache sampling task."""
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        self._task = None

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time()
        while True:
            self._hub.publish(self._cache)
            deadline += self._period
            await asyncio.sleep(max(0.0, deadline - loop.time()))
