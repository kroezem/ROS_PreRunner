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

import runner_paddock.client_stream as client_stream

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
            client.offer('global_costmap', f'global_costmap-{index}')
            client.offer('local_costmap', f'local_costmap-{index}')
            client.offer('plan', f'plan-{index}')
            assert client.pending_count == 5
        assert {
            await client.next_frame(),
            await client.next_frame(),
            await client.next_frame(),
            await client.next_frame(),
            await client.next_frame(),
        } == {
            'state-999', 'map-999', 'global_costmap-999',
            'local_costmap-999', 'plan-999',
        }

    asyncio.run(scenario())


def test_map_and_plan_send_on_change_and_new_clients_get_latest():
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('map', {'frame_id': 'map', 'data': [0]})
        cache.update('global_costmap', {'frame_id': 'map', 'data': [100]})
        cache.update('plan', {'frame_id': 'map', 'poses': []})
        hub = ClientHub()
        first = hub.register()

        hub.publish(cache)
        assert first.pending_count == 4
        types = {_frame_type(await first.next_frame()) for _ in range(4)}
        assert types == {'state', 'map', 'global_costmap', 'plan'}

        hub.publish(cache)
        assert first.pending_count == 0

        second = hub.register()
        hub.publish(cache)
        assert first.pending_count == 0
        assert second.pending_count == 4

        cache.update('map', {'frame_id': 'map', 'data': [100]})
        hub.publish(cache)
        assert first.pending_count == 2
        assert second.pending_count == 5
        assert first.pending_count <= 5
        assert second.pending_count <= 5

        hub.unregister(first)
        hub.unregister(second)
        assert hub.client_count == 0

    asyncio.run(scenario())


def test_unchanged_state_is_encoded_once_and_reused_for_new_client(monkeypatch):
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        hub = ClientHub()
        first = hub.register()
        calls = 0
        real_encode = client_stream.encode_message

        def counted_encode(*args, **kwargs):
            nonlocal calls
            if args[0] == 'state':
                calls += 1
            return real_encode(*args, **kwargs)

        monkeypatch.setattr(client_stream, 'encode_message', counted_encode)
        hub.publish(cache)
        hub.publish(cache)
        assert calls == 1
        assert first.pending_count == 1

        second = hub.register()
        hub.publish(cache)
        assert calls == 1
        assert second.pending_count == 1
        assert await first.next_frame() == await second.next_frame()

    asyncio.run(scenario())


def test_silent_expiry_and_recovery_each_emit_state():
    async def scenario():
        now = [1.0]
        cache = StateCache(clock=lambda: now[0])
        pose = {'x': 1.0}
        cache.update('pose', pose)
        hub = ClientHub()
        client = hub.register()

        hub.publish(cache)
        initial = json.loads(await client.next_frame())
        assert initial['health']['sources']['pose']['fresh']

        now[0] += 0.4
        hub.publish(cache)
        assert client.pending_count == 0

        now[0] += 0.2
        hub.publish(cache)
        stale = json.loads(await client.next_frame())
        assert stale['type'] == 'state_update'
        assert stale['section'] == 'pose'
        assert not stale['source_health']['fresh']

        assert not cache.update('pose', pose)
        hub.publish(cache)
        recovered = json.loads(await client.next_frame())
        assert recovered['section'] == 'pose'
        assert recovered['source_health']['fresh']

    asyncio.run(scenario())


def test_pose_change_only_rebuilds_and_encodes_pose_section(monkeypatch):
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('pose', {'x': 1.0})
        cache.update('mode', {'mode': 2})
        hub = ClientHub()
        client = hub.register()
        hub.publish(cache)
        await client.next_frame()
        second = hub.register()
        hub.publish(cache)
        await second.next_frame()

        snapshots = []
        encoded = []
        real_snapshot = cache.state_section_snapshot
        real_encode = client_stream.encode_message

        def counted_snapshot(source):
            snapshots.append(source)
            return real_snapshot(source)

        def counted_encode(kind, **fields):
            encoded.append((kind, fields.get('section')))
            return real_encode(kind, **fields)

        monkeypatch.setattr(cache, 'state_section_snapshot', counted_snapshot)
        monkeypatch.setattr(client_stream, 'encode_message', counted_encode)

        cache.update('pose', {'x': 2.0})
        hub.publish(cache)
        frame = json.loads(await client.next_frame())
        second_frame = json.loads(await second.next_frame())

        assert snapshots == ['pose']
        assert encoded == [('state_update', 'pose')]
        assert frame['section'] == 'pose'
        assert frame['value'] == {'x': 2.0}
        assert second_frame == frame

    asyncio.run(scenario())


def test_reconnect_gets_complete_state_not_partial_update():
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('pose', {'x': 1.0})
        cache.update('mode', {'mode': 2})
        hub = ClientHub()
        first = hub.register()
        hub.publish(cache)
        await first.next_frame()

        cache.update('pose', {'x': 2.0})
        hub.publish(cache)
        assert json.loads(await first.next_frame())['type'] == 'state_update'

        second = hub.register()
        hub.publish(cache)
        frame = json.loads(await second.next_frame())
        assert frame['type'] == 'state'
        assert frame['pose'] == {'x': 2.0}
        assert frame['mode'] == {'mode': 2}
        assert frame['health']['sources']['pose']['fresh']

    asyncio.run(scenario())


def test_state_frame_contains_authoritative_mode_stop_and_config():
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('mode', {'mode': 1, 'status': 0, 'ready': True})
        cache.update('stop_state', {'stopped': False, 'healthy': True})
        cache.update('config', {
            'field': 'manual_max_speed_mps', 'applied_value': 0.6,
        })
        hub = ClientHub()
        client = hub.register()

        hub.publish(cache)

        frame = json.loads(await client.next_frame())
        assert frame['type'] == 'state'
        assert frame['mode'] == {'mode': 1, 'status': 0, 'ready': True}
        assert frame['stop_state'] == {'stopped': False, 'healthy': True}
        assert frame['config']['applied_value'] == 0.6

    asyncio.run(scenario())


def test_map_invalidation_sends_revisioned_tombstone():
    async def scenario():
        cache = StateCache(clock=lambda: 1.0)
        cache.update('map', {'frame_id': 'map', 'data': [100]})
        hub = ClientHub()
        client = hub.register()
        hub.publish(cache)
        initial = [json.loads(await client.next_frame()) for _ in range(2)]
        assert {frame['type'] for frame in initial} == {'state', 'map'}

        cache.invalidate('map')
        hub.publish(cache)
        frames = [json.loads(await client.next_frame()) for _ in range(2)]
        tombstone = next(frame for frame in frames if frame['type'] == 'map')
        assert tombstone['cleared'] is True
        assert tombstone['revision'] == 2

    asyncio.run(scenario())
