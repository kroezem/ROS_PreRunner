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

"""End-to-end tests for the HTTP shell and read-only WebSocket stream."""

import time

from fastapi.testclient import TestClient

from runner_paddock.state_cache import StateCache
from runner_paddock.web_app import create_app


class FakeRuntime:
    """Record application lifecycle without starting ROS in HTTP tests."""

    def __init__(self):
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


def _initial_cache():
    cache = StateCache()
    cache.update('map', {'frame_id': 'map', 'data': [0, 100, -1]})
    cache.update('plan', {'frame_id': 'map', 'poses': []})
    return cache


def test_static_shell_lifecycle_and_two_read_only_clients():
    runtime = FakeRuntime()
    app = create_app(cache=_initial_cache(), runtime=runtime)

    with TestClient(app) as client:
        response = client.get('/')
        assert response.status_code == 200
        assert 'Read-only robot state' in response.text
        assert runtime.started

        with client.websocket_connect('/ws') as first:
            with client.websocket_connect('/ws') as second:
                first_types = {
                    first.receive_json()['type'] for _ in range(3)
                }
                second_types = {
                    second.receive_json()['type'] for _ in range(3)
                }
                assert first_types == {'state', 'map', 'plan'}
                assert second_types == {'state', 'map', 'plan'}

                for websocket in (first, second):
                    frames = [websocket.receive_json() for _ in range(3)]
                    assert {frame['type'] for frame in frames} == {'state'}
                    assert all(
                        frame['protocol_version'] == 1 for frame in frames
                    )

    assert runtime.stopped


def test_state_frames_arrive_at_approximately_ten_hz():
    app = create_app(cache=StateCache(), runtime=FakeRuntime())
    with TestClient(app) as client:
        with client.websocket_connect('/ws') as websocket:
            websocket.receive_json()
            started = time.monotonic()
            for _ in range(4):
                assert websocket.receive_json()['type'] == 'state'
            elapsed = time.monotonic() - started

    assert 0.20 <= elapsed <= 0.80
