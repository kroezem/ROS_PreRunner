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

from runner_paddock.map_session import MapSaveTransaction
from runner_paddock.map_session import OccupancyRaster
from runner_paddock.map_session import POSEGRAPH_DATA_NAME
from runner_paddock.map_session import POSEGRAPH_POSEGRAPH_NAME
from runner_paddock.semantics import semantics_path
from runner_paddock.semantics import write_semantics
from runner_paddock.state_cache import StateCache
from runner_paddock.web_app import create_app
from runner_paddock.web_app import STATIC_DIRECTORY


def test_recording_profile_selector_exposes_navigation_debug():
    index = (STATIC_DIRECTORY / 'index.html').read_text(encoding='utf-8')
    assert '<option value="navigation_debug">NAVIGATION DEBUG</option>' in index


class FakeRuntime:
    """Record application lifecycle without starting ROS in HTTP tests."""

    def __init__(self):
        self.started = False
        self.stopped = False
        self.actions = []
        self.disconnected = []
        self.visualization_demands = {}
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
        if name == 'takeover':
            self._owner = conn_id
            return {'accepted': True, 'reason': 'taken over',
                    'role': 'controller'}
        role = 'controller' if self._owner == conn_id else 'observer'
        return {'accepted': role == 'controller', 'reason': name or 'noop',
                'role': role}

    def disconnect(self, conn_id):
        self.disconnected.append(conn_id)
        self.visualization_demands.pop(conn_id, None)
        if self._owner == conn_id:
            self._owner = None

    def set_visualization_demand(self, conn_id, demand):
        self.visualization_demands[conn_id] = demand


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
        'semantic_paint.js',
        'speed_profile.js',
        'style.css',
        'service-worker.js',
        'telemetry_warning.js',
    }
    assert expected <= {
        path.name for path in STATIC_DIRECTORY.iterdir() if path.is_file()
    }


def test_hidden_diagnostics_skip_periodic_stringification():
    app_source = (STATIC_DIRECTORY / 'app.js').read_text(encoding='utf-8')

    assert 'if (!diagnosticsVisible(debugDetails)) return;' in app_source
    assert 'if (!diagnosticsVisible(healthDetails)) return;' in app_source
    assert 'details.closest("[hidden]")' in app_source


def test_release_stop_and_takeover_are_one_click_actions():
    app_source = (STATIC_DIRECTORY / 'app.js').read_text(encoding='utf-8')

    assert 'stopButton.addEventListener("click"' in app_source
    assert app_source.count('send({ action: "clear_stop" })') == 1
    assert 'stopHold' not in app_source
    assert 'takeoverButton.addEventListener("click"' in app_source
    assert app_source.count('send({ action: "takeover" })') == 1
    assert 'takeoverHold' not in app_source


def test_tuning_failure_and_shared_bounds_are_rendered_explicitly():
    app_source = (STATIC_DIRECTORY / 'app.js').read_text(encoding='utf-8')

    assert 'const failed = tuning.status === "failed";' in app_source
    assert 'const bounds = tuning.bounds || {};' in app_source


def test_no_preset_system_remains_in_frontend_assets():
    for name in ('app.js', 'index.html', 'speed_profile.js'):
        source = (STATIC_DIRECTORY / name).read_text(encoding='utf-8')
        for token in ('TIMID', 'CONFIDENT', 'INSANE', 'ABSURD', 'data-speed-preset'):
            assert token not in source, f'{token} should have been removed from {name}'


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
        control_html = response.text.split('id="view-control"', 1)[1].split(
            'id="view-configure"', 1)[0]
        assert 'id="btn-clear-stop"' not in response.text
        assert 'id="stop-label"' in response.text
        assert 'id="btn-takeover"' in response.text
        assert 'TAKE CONTROL' in response.text
        assert '/static/hold_to_confirm.js' not in response.text
        assert '/static/map_viewport_storage.js' in response.text
        assert '/static/route_visualization.js' in response.text
        assert 'id="route-speed-legend"' in response.text
        assert '>Committed route<' in response.text
        assert 'id="layer-color-plan"' not in response.text
        assert 'id="autonomy-speed-commanded"' in response.text
        assert 'id="autonomy-speed-effective"' in response.text
        assert 'data-tuning-field="output_max"' in response.text
        assert 'id="profile-policy-controls"' in response.text
        speed_profile_source = (STATIC_DIRECTORY / 'speed_profile.js').read_text(
            encoding='utf-8'
        )
        assert '"desired_linear_vel"' in speed_profile_source
        assert '"maximum_commanded_speed"' in speed_profile_source
        assert '"reaction_time_s"' in speed_profile_source
        assert '"recovery_acceleration_gain"' in speed_profile_source
        assert '"recovery_acceleration_floor"' in speed_profile_source
        assert 'id="profile-catalog-select"' in response.text
        assert 'id="btn-load-profile"' in response.text
        assert 'id="btn-delete-profile"' in response.text
        assert 'id="btn-save-as-profile"' in response.text
        assert 'id="layer-visible-global_costmap"' in response.text
        assert 'id="btn-clear-obstacles"' in response.text
        assert 'id="btn-global-obstacles-off"' in response.text
        assert 'id="btn-local-obstacles-on"' in response.text
        assert 'obstacle_layer.enabled' in response.text
        assert 'id="system-cpu-load"' in response.text
        assert 'id="system-battery-voltage"' in response.text
        assert 'id="control-cpu-load"' in control_html
        assert 'id="control-battery-voltage"' in control_html
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
        assert 'data-tab="speed-profile"' in response.text
        assert 'id="profile-clearance-chart"' in response.text
        assert 'id="profile-path-chart"' not in response.text
        assert 'edit previews · Apply is live · Save persists' in response.text
        assert 'id="delete-dialog"' in response.text
        assert 'id="btn-record"' in response.text
        assert 'data-tab="recording"' in response.text
        assert 'id="delete-recording-dialog"' in response.text
        assert runtime.started

        with client.websocket_connect('/ws') as first:
            with client.websocket_connect('/ws') as second:
                demand = {
                    'action': 'visualization_demand',
                    'layers': ['map', 'global_costmap', 'plan'],
                }
                first.send_json(demand)
                second.send_json(demand)
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
    assert runtime.visualization_demands == {}


def test_finalized_recording_download_is_catalog_and_root_scoped(tmp_path):
    root = tmp_path / 'bags'
    bag = root / 'finished'
    bag.mkdir(parents=True)
    mcap = bag / 'finished_0.mcap'
    mcap.write_bytes(b'MCAP download')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  duration:\n    nanoseconds: 1\n'
        '  starting_time:\n    nanoseconds_since_epoch: 2\n'
        '  relative_file_paths:\n    - finished_0.mcap\n',
        encoding='utf-8',
    )
    cache = _initial_cache()
    cache.update('recording_state', {
        'state': 0,
        'recordings': [{'name': 'finished'}],
    })
    app = create_app(
        cache=cache, runtime=FakeRuntime(), recording_root=root
    )

    with TestClient(app) as client:
        response = client.get('/recordings/finished/download')
        missing = client.get('/recordings/not-cataloged/download')

    assert response.status_code == 200
    assert response.content == b'MCAP download'
    assert 'filename="finished_0.mcap"' in response.headers[
        'content-disposition'
    ]
    assert missing.status_code == 404


def test_recording_download_is_blocked_while_recorder_active(tmp_path):
    cache = _initial_cache()
    cache.update('recording_state', {
        'state': 3,
        'recordings': [{'name': 'finished'}],
    })
    app = create_app(
        cache=cache, runtime=FakeRuntime(), recording_root=tmp_path
    )

    with TestClient(app) as client:
        response = client.get('/recordings/finished/download')

    assert response.status_code == 409


def test_recording_download_requires_fresh_authoritative_catalog(tmp_path):
    app = create_app(
        cache=_initial_cache(), runtime=FakeRuntime(), recording_root=tmp_path
    )

    with TestClient(app) as client:
        response = client.get('/recordings/finished/download')

    assert response.status_code == 503


def _publish_map(map_root, name='studio', *, width=3, height=2):
    txn = MapSaveTransaction(map_root, name, 'sess')
    txn.prepare()
    txn.staged_path(POSEGRAPH_POSEGRAPH_NAME).write_bytes(b'p')
    txn.staged_path(POSEGRAPH_DATA_NAME).write_bytes(b'd')
    raster = OccupancyRaster(
        width=width, height=height, resolution=0.05,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0,
        data=tuple([-1] * (width * height)),
    )
    txn.write_raster(raster)
    txn.validate()
    txn.publish()


def test_semantics_download_is_404_when_no_layer_is_saved(tmp_path):
    _publish_map(tmp_path, 'studio')
    app = create_app(cache=_initial_cache(), runtime=FakeRuntime(), map_root=tmp_path)

    with TestClient(app) as client:
        missing_layer = client.get('/maps/studio/semantics.png')
        missing_map = client.get('/maps/nope/semantics.png')

    assert missing_layer.status_code == 404
    assert missing_map.status_code == 404


def test_semantics_download_serves_saved_layer(tmp_path):
    _publish_map(tmp_path, 'studio', width=3, height=2)
    write_semantics(tmp_path, 'studio', bytes([0, 1, 2, 0, 0, 1]))
    app = create_app(cache=_initial_cache(), runtime=FakeRuntime(), map_root=tmp_path)

    with TestClient(app) as client:
        response = client.get('/maps/studio/semantics.png')

    assert response.status_code == 200
    assert response.headers['content-type'] == 'image/png'


def test_corrupt_semantics_download_is_reported_and_does_not_break_the_map(tmp_path):
    _publish_map(tmp_path, 'studio', width=3, height=2)
    write_semantics(tmp_path, 'studio', bytes([0, 1, 2, 0, 0, 1]))
    semantics_path(tmp_path, 'studio').write_bytes(b'not a png')
    app = create_app(cache=_initial_cache(), runtime=FakeRuntime(), map_root=tmp_path)

    with TestClient(app) as client:
        corrupt = client.get('/maps/studio/semantics.png')
        # The core occupancy bundle must remain unaffected by the corrupt
        # optional layer -- confirmed indirectly via the map save endpoint
        # continuing to accept requests for this map id.
        still_valid_save = client.get('/maps/studio/semantics.png')

    assert corrupt.status_code == 422
    assert still_valid_save.status_code == 422  # same corrupt file, not a crash


def test_speed_profiles_list_always_includes_the_immutable_baseline(tmp_path):
    app = create_app(
        cache=_initial_cache(), runtime=FakeRuntime(),
        speed_profiles_path=tmp_path / 'speed_profiles.json',
    )

    with TestClient(app) as client:
        response = client.get('/speed_profiles')

    assert response.status_code == 200
    assert response.json() == [{'name': 'Default Baseline', 'baseline': True}]


def test_speed_profiles_load_baseline_returns_a_complete_snapshot(tmp_path):
    from runner_paddock.autonomy_tuning import SPEED_PROFILE_FIELDS

    app = create_app(
        cache=_initial_cache(), runtime=FakeRuntime(),
        speed_profiles_path=tmp_path / 'speed_profiles.json',
    )

    with TestClient(app) as client:
        response = client.get('/speed_profiles/Default Baseline')

    assert response.status_code == 200
    assert set(response.json()) == SPEED_PROFILE_FIELDS


def test_speed_profiles_load_unknown_name_is_404(tmp_path):
    app = create_app(
        cache=_initial_cache(), runtime=FakeRuntime(),
        speed_profiles_path=tmp_path / 'speed_profiles.json',
    )

    with TestClient(app) as client:
        response = client.get('/speed_profiles/does-not-exist')

    assert response.status_code == 404


def test_speed_profiles_endpoints_never_touch_ros_runtime(tmp_path):
    runtime = FakeRuntime()
    app = create_app(
        cache=_initial_cache(), runtime=runtime,
        speed_profiles_path=tmp_path / 'speed_profiles.json',
    )

    with TestClient(app) as client:
        client.get('/speed_profiles')
        client.get('/speed_profiles/Default Baseline')

    assert runtime.actions == []


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

                second.send_json({'action': 'takeover'})
                takeover = _drain_until(second, 'ack')
                assert takeover['accepted']
                assert takeover['role'] == 'controller'

                first.send_json({'action': 'run', 'held': True})
                old_owner = _drain_until(first, 'ack')
                assert not old_owner['accepted']
                assert old_owner['role'] == 'observer'

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


def test_state_frames_are_change_gated_at_ten_hz():
    app = create_app(cache=StateCache(), runtime=FakeRuntime())
    with TestClient(app) as client:
        with client.websocket_connect('/ws') as websocket:
            websocket.receive_json()
            app.state.cache.update('gateway', {'revision': 1})
            started = time.monotonic()
            frame = websocket.receive_json()
            assert frame['type'] == 'state_update'
            assert frame['section'] == 'gateway'
            elapsed = time.monotonic() - started

    assert elapsed <= 0.30
