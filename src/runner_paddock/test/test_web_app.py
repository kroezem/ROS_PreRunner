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
from runner_paddock.web_app import STATIC_DIRECTORY


class FakeRuntime:
    """Record application lifecycle without starting ROS in HTTP tests."""

    def __init__(self):
        self.started = False
        self.stopped = False
        self.actions = []
        self.disconnected = []
        self._owner = None

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def submit(self, conn_id, action):
        self.actions.append((conn_id, action))
        name = action.get('action') if isinstance(action, dict) else None
        if name == 'acquire':
            if self._owner is None:
                self._owner = conn_id
                return {'accepted': True, 'reason': 'acquired',
                        'role': 'controller'}
            role = 'controller' if self._owner == conn_id else 'observer'
            return {'accepted': self._owner == conn_id,
                    'reason': 'lease', 'role': role}
        role = 'controller' if self._owner == conn_id else 'observer'
        return {'accepted': role == 'controller', 'reason': name or 'noop',
                'role': role}

    def disconnect(self, conn_id):
        self.disconnected.append(conn_id)
        if self._owner == conn_id:
            self._owner = None


def _initial_cache():
    cache = StateCache()
    cache.update('map', {'frame_id': 'map', 'data': [0, 100, -1]})
    cache.update('global_costmap', {'frame_id': 'map', 'data': [0, 100, -1]})
    cache.update('plan', {'frame_id': 'map', 'poses': []})
    return cache


def _drain_until(websocket, wanted, limit=40):
    for _ in range(limit):
        frame = websocket.receive_json()
        if frame['type'] == wanted:
            return frame
    raise AssertionError(f'no {wanted} frame within {limit} frames')


def test_frontend_assets_are_packaged_at_runtime_location():
    expected = {
        'index.html',
        'app.js',
        'joystick_geometry.js',
        'map_geometry.js',
        'style.css',
        'service-worker.js',
    }
    assert expected <= {
        path.name for path in STATIC_DIRECTORY.iterdir() if path.is_file()
    }


def test_static_shell_lifecycle_and_two_clients():
    runtime = FakeRuntime()
    app = create_app(cache=_initial_cache(), runtime=runtime)

    with TestClient(app) as client:
        response = client.get('/')
        assert response.status_code == 200
        assert 'operator console' in response.text
        assert 'data-view="control"' in response.text
        assert 'data-view="configure"' in response.text
        assert 'id="view-control"' in response.text
        assert 'id="view-configure"' in response.text
        assert 'maximum-scale=1' in response.text
        assert 'role="tablist"' in response.text
        assert 'data-map-mode="view"' in response.text
        assert 'id="btn-initial-pose-mode"' in response.text
        assert 'id="btn-confirm-initial-pose"' in response.text
        assert 'id="layer-visible-global_costmap"' in response.text
        assert 'id="btn-clear-obstacles"' in response.text
        assert 'id="btn-global-obstacles-off"' in response.text
        assert 'id="btn-local-obstacles-on"' in response.text
        assert 'obstacle_layer.enabled' in response.text
        assert 'id="system-cpu-load"' in response.text
        assert 'id="system-battery-voltage"' in response.text
        assert 'id="manual-max-speed-input"' in response.text
        assert 'id="m-reset"' in response.text
        assert 'id="mapping-speed-target"' in response.text
        assert 'id="mapping-speed-actual"' in response.text
        assert 'id="mapping-speed-target-mark"' in response.text
        assert 'id="display-map-preview"' in response.text
        assert 'id="btn-cancel"' not in response.text
        assert response.text.index('Saved map catalog') < response.text.index(
            'Browser manual drive'
        )
        assert 'data-tab="autonomy"' in response.text
        assert 'id="delete-dialog"' in response.text
        assert 'id="btn-record"' in response.text
        assert 'data-tab="recording"' in response.text
        assert 'id="delete-recording-dialog"' in response.text
        assert runtime.started

        with client.websocket_connect('/ws') as first:
            with client.websocket_connect('/ws') as second:
                first_types = {
                    first.receive_json()['type'] for _ in range(4)
                }
                second_types = {
                    second.receive_json()['type'] for _ in range(4)
                }
                assert first_types == {
                    'state', 'map', 'global_costmap', 'plan'
                }
                assert second_types == {
                    'state', 'map', 'global_costmap', 'plan'
                }

    assert runtime.stopped
    assert len(runtime.disconnected) == 2


def test_ws_action_round_trip_and_lease_role():
    runtime = FakeRuntime()
    app = create_app(cache=_initial_cache(), runtime=runtime)
    with TestClient(app) as client:
        with client.websocket_connect('/ws') as first:
            first.send_json({'action': 'acquire'})
            ack = _drain_until(first, 'ack')
            assert ack['accepted'] and ack['role'] == 'controller'
            assert ack['name'] == 'acquire'

            first.send_json({'action': 'stop'})
            assert _drain_until(first, 'ack')['accepted']

            first.send_json({'action': 'select_map', 'name': 'studio'})
            select_ack = _drain_until(first, 'ack')
            assert select_ack['accepted']
            assert select_ack['name'] == 'select_map'
            assert runtime.actions[-1][1] == {
                'action': 'select_map', 'name': 'studio'
            }

            with client.websocket_connect('/ws') as second:
                second.send_json({'action': 'acquire'})
                ack2 = _drain_until(second, 'ack')
                assert ack2['role'] == 'observer' and not ack2['accepted']

                second.send_json({'action': 'stop'})
                assert not _drain_until(second, 'ack')['accepted']

    # Both connections released; the owner's disconnect frees the lease.
    assert runtime._owner is None


def test_ws_malformed_action_is_rejected_not_fatal():
    runtime = FakeRuntime()
    app = create_app(cache=_initial_cache(), runtime=runtime)
    with TestClient(app) as client:
        with client.websocket_connect('/ws') as socket:
            socket.send_text('not json')
            ack = _drain_until(socket, 'ack')
            assert not ack['accepted']
            assert 'malformed' in ack['reason']


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
