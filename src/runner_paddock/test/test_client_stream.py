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

"""Tests for bounded, change-aware multi-client fan-out."""

import asyncio
import json

from runner_paddock.client_stream import ClientConnection
from runner_paddock.client_stream import ClientHub
from runner_paddock.state_cache import StateCache


def _frame_type(frame):
    return json.loads(frame)['type']


def test_slow_client_is_bounded_and_each_kind_is_latest_wins():
    async def scenario():
        client = ClientConnection()
        for index in range(1000):
            client.offer('state', f'state-{index}')
            client.offer('map', f'map-{index}')
            client.offer('plan', f'plan-{index}')
        assert client.pending_count == 3
        assert {
            await client.next_frame(),
            await client.next_frame(),
            await client.next_frame(),
        } == {'state-999', 'map-999', 'plan-999'}

    asyncio.run(scenario())


def test_map_and_plan_send_on_change_and_new_clients_get_latest():
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('map', {'frame_id': 'map', 'data': [0]})
        cache.update('plan', {'frame_id': 'map', 'poses': []})
        hub = ClientHub()
        first = hub.register()

        hub.publish(cache)
        assert first.pending_count == 3
        types = {_frame_type(await first.next_frame()) for _ in range(3)}
        assert types == {'state', 'map', 'plan'}

        hub.publish(cache)
        assert first.pending_count == 1
        assert _frame_type(await first.next_frame()) == 'state'

        second = hub.register()
        hub.publish(cache)
        assert first.pending_count == 1
        assert second.pending_count == 3

        cache.update('map', {'frame_id': 'map', 'data': [100]})
        hub.publish(cache)
        assert first.pending_count == 2
        assert second.pending_count == 3
        assert first.pending_count <= 3
        assert second.pending_count <= 3

        hub.unregister(first)
        hub.unregister(second)
        assert hub.client_count == 0

    asyncio.run(scenario())
