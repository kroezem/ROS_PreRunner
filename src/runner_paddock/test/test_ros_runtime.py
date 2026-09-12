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

"""Lifecycle test for the dedicated ROS executor thread."""

import math
import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid
import pytest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from runner_interfaces.msg import ConfigState
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import StopState
from runner_interfaces.msg import SystemTelemetry
from runner_paddock.autonomy_tuning import (
    ADAPTER_OWNER,
    CONFIDENT,
    CONTROLLER_OWNER,
    PARAMETERS as TUNING_PARAMETERS,
    TIMID,
)
from runner_paddock.gateway import AutonomyTuningIntent
from runner_paddock.gateway import GatewayResult
from runner_paddock.gateway import InitialPoseIntent
from runner_paddock.gateway import ObstacleProcessingIntent
from runner_paddock.ros_runtime import RosRuntime
from runner_paddock.ros_state_node import _grid, RosStateNode
from runner_paddock.state_cache import StateCache
from sensor_msgs.msg import BatteryState


class _Future:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return object()


class _ResponseFuture:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class _ParameterClient:
    def __init__(self, response, ready=True):
        self.response = response
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return _ResponseFuture(self.response)


class _ClearClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = 0

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        self.calls += 1
        return _Future()


class _InitialPosePublisher:
    def __init__(self, subscribers=1):
        self.subscribers = subscribers
        self.messages = []

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    def now(self):
        return SimpleNamespace(
            nanoseconds=12_000_000_034,
            to_msg=lambda: Time(sec=12, nanosec=34),
        )


def _ready_initial_pose_cache(*, stopped=True, stationary=True):
    cache = StateCache(clock=lambda: 10.0)
    cache.update('mode', {
        'mode': 2, 'status': 0, 'ready': True,
        'active_autonomy_map': 'studio', 'runtime_epoch': 7,
    })
    cache.update('map_state', {
        'selected_map_applied': 'studio',
        'catalog': [{'name': 'studio', 'complete': True}],
    })
    cache.update('stop_state', {
        'stopped': stopped, 'locked': stopped,
        'healthy': True, 'applied': stopped,
    })
    cache.update('encoder_state', {'stationary': stationary})
    return cache


def _initial_pose_node(cache, subscribers=1):
    node = RosStateNode.__new__(RosStateNode)
    node._cache = cache
    node._initial_pose_pub = _InitialPosePublisher(subscribers)
    node._initial_pose_lock = threading.Lock()
    node._pending_initial_pose = None
    node._initial_pose_request_id = 0
    node._global_clear_client = _ClearClient()
    node._local_clear_client = _ClearClient()
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    return node


def _set_tf_pose(node, x, y, yaw, *, stamp_sec=11):
    transform = SimpleNamespace(
        header=SimpleNamespace(
            stamp=Time(sec=stamp_sec), frame_id='map'
        ),
        child_frame_id='base_link',
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0),
            ),
        ),
    )
    node._tf_buffer = SimpleNamespace(
        lookup_transform=lambda *_args: transform
    )


class _TuningClient:
    def __init__(self, owner, state, operation):
        self.owner = owner
        self.state = state
        self.operation = operation
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        if self.operation == 'get':
            response = SimpleNamespace(values=[ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=self.state[name],
            ) for name in request.names])
        else:
            for parameter in request.parameters:
                self.state[parameter.name] = parameter.value.double_value
            response = SimpleNamespace(result=SimpleNamespace(
                successful=True, reason='',
            ))
        return _ResponseFuture(response)


def _tuning_node(values):
    node = RosStateNode.__new__(RosStateNode)
    node._cache = StateCache()
    node._tuning_lock = threading.Lock()
    node._tuning_request_id = 0
    node._tuning_operation = None
    node._tuning_state = {
        'available': False, 'preset': 'custom', 'values': {},
        'status': 'unavailable', 'detail': 'waiting', 'request_id': 0,
    }
    states = {
        owner: {
            spec.parameter_name: values[field]
            for field, spec in TUNING_PARAMETERS.items()
            if spec.owner == owner
        }
        for owner in (CONTROLLER_OWNER, ADAPTER_OWNER)
    }
    node._tuning_get_clients = {
        owner: _TuningClient(owner, state, 'get')
        for owner, state in states.items()
    }
    node._tuning_set_clients = {
        owner: _TuningClient(owner, state, 'set')
        for owner, state in states.items()
    }
    return node


def _obstacle_node(*, current, set_success=True):
    node = RosStateNode.__new__(RosStateNode)
    node._cache = StateCache()
    node._obstacle_lock = threading.Lock()
    node._obstacle_request_id = 0
    node._obstacle_refresh_id = 0
    node._obstacle_operations = {}
    node._obstacle_states = {
        name: {
            'requested': None, 'applied': None, 'current': None,
            'status': 'unavailable', 'detail': 'waiting', 'request_id': 0,
            'confirmed_at': None,
        }
        for name in ('global', 'local')
    }
    set_response = SimpleNamespace(results=[SimpleNamespace(
        successful=set_success,
        reason='' if set_success else 'rejected by plugin',
    )])
    get_response = SimpleNamespace(values=[ParameterValue(
        type=ParameterType.PARAMETER_BOOL,
        bool_value=current,
    )])
    node._obstacle_set_clients = {
        name: _ParameterClient(set_response) for name in ('global', 'local')
    }
    node._obstacle_get_clients = {
        name: _ParameterClient(get_response) for name in ('global', 'local')
    }
    return node


def test_runtime_start_stop_leaves_no_executor_thread():
    runtime = RosRuntime(StateCache())
    runtime.start()
    assert runtime.thread_alive
    runtime.stop()
    assert not runtime.thread_alive


def test_authoritative_mode_stop_and_config_callbacks_reach_snapshot():
    cache = StateCache(clock=lambda: 10.0)
    node = SimpleNamespace(_cache=cache)
    config = ConfigState(
        stamp=Time(sec=1),
        request_id=4,
        revision=2,
        field='manual_max_speed_mps',
        requested_value=0.6,
        applied_value=0.6,
        accepted=True,
        reason='APPLIED',
    )
    mode = ModeState(
        stamp=Time(sec=2),
        mode=ModeState.MODE_MAPPING,
        status=ModeState.STATUS_STABLE,
        ready=True,
    )
    stop = StopState(
        stamp=Time(sec=3),
        boot_id='boot-1',
        generation=3,
        healthy=True,
        applied=True,
        reason='CLEAR',
    )

    # Config is transient-local and can be the first callback after startup.
    RosStateNode._on_config_state(node, config)
    RosStateNode._on_mode(node, mode)
    RosStateNode._on_stop_state(node, stop)

    snapshot = cache.state_snapshot()
    assert snapshot['config']['applied_value'] == 0.6
    assert snapshot['mode']['mode'] == ModeState.MODE_MAPPING
    assert snapshot['mode']['status'] == ModeState.STATUS_STABLE
    assert snapshot['mode']['ready']
    assert not snapshot['stop_state']['stopped']
    assert snapshot['stop_state']['healthy']


def test_existing_cpu_and_battery_topics_reach_snapshot_without_zero_fill():
    cache = StateCache(clock=lambda: 10.0)
    node = SimpleNamespace(_cache=cache)

    RosStateNode._on_system_telemetry(node, SystemTelemetry(
        stamp=Time(sec=1), cpu_valid=True,
        total_cpu_utilization_percent=37.5,
    ))
    battery = BatteryState(present=True, voltage=7.41)
    battery.header.stamp = Time(sec=2)
    RosStateNode._on_battery(node, battery)

    snapshot = cache.state_snapshot()
    assert snapshot['system_telemetry'][
        'total_cpu_utilization_percent'
    ] == 37.5
    assert snapshot['battery']['voltage'] == 7.41

    RosStateNode._on_system_telemetry(node, SystemTelemetry(cpu_valid=False))
    RosStateNode._on_battery(node, BatteryState(present=False))
    snapshot = cache.state_snapshot()
    assert snapshot['system_telemetry'][
        'total_cpu_utilization_percent'
    ] is None
    assert snapshot['battery']['voltage'] is None


def test_obstacle_set_is_verified_against_real_parameter_readback():
    node = _obstacle_node(current=False)

    result = node._request_obstacle_processing(
        ObstacleProcessingIntent('global', False), 'controller'
    )

    state = node._cache.state_snapshot()['obstacle_processing']['global']
    assert result.accepted
    assert state['requested'] is False
    assert state['applied'] is False
    assert state['current'] is False
    assert state['status'] == 'applied'
    assert node._obstacle_set_clients['global'].requests[0].parameters[
        0
    ].name == 'obstacle_layer.enabled'


def test_obstacle_silent_set_failure_is_exposed_by_readback():
    node = _obstacle_node(current=False)

    node._request_obstacle_processing(
        ObstacleProcessingIntent('local', True), 'controller'
    )

    state = node._cache.state_snapshot()['obstacle_processing']['local']
    assert state['requested'] is True
    assert state['applied'] is True
    assert state['current'] is False
    assert state['status'] == 'failed'
    assert 'reported success but read-back is OFF' in state['detail']


def test_stable_runtime_reacquires_map_with_epoch_bound_subscription():
    cache = StateCache(clock=lambda: 10.0)
    cache.update('map', {'frame_id': 'map', 'data': [100]})
    subscriptions = []
    destroyed = []

    def create_subscription(_message_type, _topic, callback, _qos):
        subscription = SimpleNamespace(callback=callback)
        subscriptions.append(subscription)
        return subscription

    node = SimpleNamespace(
        _cache=cache,
        _map_qos=object(),
        _map_subscription=SimpleNamespace(callback=None),
        _map_subscription_identity=(4, ModeState.MODE_MAPPING, 'old-session'),
        create_subscription=create_subscription,
        destroy_subscription=destroyed.append,
    )

    RosStateNode._on_mode(node, ModeState(
        mode=ModeState.MODE_MAPPING,
        status=ModeState.STATUS_TRANSITIONING,
        detail='Resetting MAPPING',
    ))

    assert cache.large_snapshot('map') == (2, None)
    assert node._map_subscription is None
    assert node._map_subscription_identity is None
    assert len(destroyed) == 1

    RosStateNode._on_mode(node, ModeState(
        mode=ModeState.MODE_MAPPING,
        status=ModeState.STATUS_STABLE,
        ready=True,
        runtime_epoch=5,
        mapping_session_id='current-session',
    ))
    assert node._map_subscription is subscriptions[-1]
    assert node._map_subscription_identity == (
        5, ModeState.MODE_MAPPING, 'current-session'
    )

    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.resolution = 0.05
    message.info.width = 1
    message.info.height = 1
    message.data = [0]
    subscriptions[-1].callback(message)
    revision, active_map = cache.large_snapshot('map')
    assert revision == 3
    assert active_map['runtime_epoch'] == 5


def test_grid_validation_preserves_cells_and_rejects_bad_input():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.resolution = 0.05
    message.info.width = 3
    message.info.height = 1
    message.data = [-1, 0, 100]

    assert _grid(message)['data'] == [-1, 0, 100]

    message.data = [-1, 0, 101]
    with pytest.raises(ValueError, match=r'outside \[-1, 100\]'):
        _grid(message)

    message.data = [0, 100]
    with pytest.raises(ValueError, match='dimensions do not match'):
        _grid(message)


def test_queued_previous_runtime_map_is_rejected_after_resubscribe():
    cache = StateCache(clock=lambda: 10.0)
    node = SimpleNamespace(
        _cache=cache,
        _map_subscription_identity=(8, ModeState.MODE_AUTONOMY, 'studio'),
    )
    message = OccupancyGrid()

    RosStateNode._on_map(
        node,
        message,
        (7, ModeState.MODE_AUTONOMY, 'previous-map'),
    )

    assert cache.large_snapshot('map') == (0, None)


def test_clear_costmaps_reports_both_nav2_service_responses():
    global_client = _ClearClient()
    local_client = _ClearClient()
    node = SimpleNamespace(
        _global_clear_client=global_client,
        _local_clear_client=local_client,
    )

    result = RosStateNode._clear_costmaps(
        node, GatewayResult(True, 'requested', role='controller')
    )

    assert result.accepted
    assert result.reason == 'Nav2 global and local costmaps cleared'
    assert global_client.calls == 1
    assert local_client.calls == 1


def test_clear_costmaps_rejects_when_a_nav2_service_is_unavailable():
    node = SimpleNamespace(
        _global_clear_client=_ClearClient(),
        _local_clear_client=_ClearClient(ready=False),
    )

    result = RosStateNode._clear_costmaps(
        node, GatewayResult(True, 'requested', role='controller')
    )

    assert not result.accepted
    assert result.reason == 'Nav2 local costmap clear service unavailable'


def test_backend_publishes_slam_toolbox_initialpose_with_map_semantics():
    node = _initial_pose_node(_ready_initial_pose_cache())
    intent = InitialPoseIntent(x=1.25, y=-0.5, yaw=0.75)

    result = RosStateNode._set_initial_pose(node, intent, 'controller')
    RosStateNode._advance_initial_pose_transaction(node)

    assert result.accepted
    assert len(node._initial_pose_pub.messages) == 1
    message = node._initial_pose_pub.messages[0]
    assert message.header.frame_id == 'map'
    assert message.pose.pose.position.x == 1.25
    assert message.pose.pose.position.y == -0.5
    assert math.isclose(
        2.0 * math.atan2(
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ),
        0.75,
    )
    assert message.pose.covariance[0] == 0.25
    assert message.pose.covariance[7] == 0.25
    assert message.pose.covariance[35] == 0.06853891909122467
    status = node._cache.state_snapshot()['initial_pose']
    assert status['state'] == 'localizing'
    assert status['map'] == 'studio'


def test_initial_pose_waits_for_stop_and_stationary_before_publish():
    node = _initial_pose_node(
        _ready_initial_pose_cache(stopped=False, stationary=False)
    )

    result = RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.0), 'controller'
    )

    assert result.accepted
    assert not node._initial_pose_pub.messages
    status = node._cache.state_snapshot()['initial_pose']
    assert status['state'] == 'stopping'

    node._cache.update('stop_state', {
        'stopped': True, 'locked': True, 'healthy': True, 'applied': True,
    })
    node._cache.update('encoder_state', {'stationary': True})
    RosStateNode._advance_initial_pose_transaction(node)

    assert len(node._initial_pose_pub.messages) == 1
    assert node._cache.state_snapshot()['initial_pose'][
        'state'
    ] == 'localizing'


def test_correct_tf_confirms_when_pose_scan_stamp_predates_request():
    cache = _ready_initial_pose_cache()
    node = _initial_pose_node(cache)
    RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.2), 'controller'
    )
    RosStateNode._advance_initial_pose_transaction(node)
    # E1's scan-derived /pose timestamp was before the 12 s request. The
    # existing post-request TF update is nevertheless the correct map-frame
    # truth and agrees with the requested seed.
    pre_request_pose_stamp = Time(sec=11, nanosec=939_210_000)
    assert pre_request_pose_stamp.sec * 1_000_000_000 \
        + pre_request_pose_stamp.nanosec < 12_000_000_034
    _set_tf_pose(node, 1.04, 1.98, 0.2, stamp_sec=11)

    RosStateNode._update_pose(node)

    status = cache.state_snapshot()['initial_pose']
    assert status['state'] == 'applied'
    assert status['observed']['pose']['position']['x'] == 1.04
    assert status['observed']['source'] == 'map_to_base_link_tf'
    assert node._global_clear_client.calls == 1
    assert node._local_clear_client.calls == 1
    assert cache.state_snapshot()['stop_state']['stopped']


def test_initial_pose_clear_failure_is_terminal_and_visible():
    cache = _ready_initial_pose_cache()
    node = _initial_pose_node(cache)
    node._local_clear_client.ready = False
    RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.2), 'controller'
    )
    RosStateNode._advance_initial_pose_transaction(node)
    _set_tf_pose(node, 1.0, 2.0, 0.2)

    RosStateNode._update_pose(node)

    status = cache.state_snapshot()['initial_pose']
    assert status['state'] == 'rejected'
    assert 'local costmap clear service unavailable' in status['detail']
    assert node._global_clear_client.calls == 0


def test_incorrect_tf_pose_does_not_confirm_and_uses_existing_timeout():
    cache = _ready_initial_pose_cache()
    node = _initial_pose_node(cache)
    RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.2), 'controller'
    )
    RosStateNode._advance_initial_pose_transaction(node)
    _set_tf_pose(node, 2.0, 2.0, 0.2)

    RosStateNode._update_pose(node)

    assert cache.state_snapshot()['initial_pose']['state'] == 'localizing'
    node._pending_initial_pose['deadline'] = 0.0
    RosStateNode._expire_initial_pose_confirmation(node)
    status = cache.state_snapshot()['initial_pose']
    assert status['state'] == 'rejected'
    assert 'did not match' in status['detail']
    assert node._global_clear_client.calls == 0
    assert node._local_clear_client.calls == 0


def test_timid_confident_and_custom_are_classified_from_live_readback():
    node = _tuning_node(TIMID)
    RosStateNode._start_tuning_read(node)
    timid = node._cache.state_snapshot()['autonomy_tuning']
    assert timid['available']
    assert timid['preset'] == 'timid'
    assert timid['values'] == TIMID

    result = RosStateNode._request_autonomy_tuning(
        node, AutonomyTuningIntent(preset='confident'), 'controller'
    )
    confident = node._cache.state_snapshot()['autonomy_tuning']
    assert result.accepted
    assert confident['status'] == 'applied'
    assert confident['preset'] == 'confident'
    assert confident['values'] == CONFIDENT

    custom_values = {**CONFIDENT, 'lookahead_time': 1.01}
    result = RosStateNode._request_autonomy_tuning(
        node, AutonomyTuningIntent(values=custom_values), 'controller'
    )
    custom = node._cache.state_snapshot()['autonomy_tuning']
    assert result.accepted
    assert custom['preset'] == 'custom'
    assert custom['values']['lookahead_time'] == 1.01


def test_tuning_owner_writes_are_atomic_and_read_back_after_each_write():
    node = _tuning_node(TIMID)

    RosStateNode._request_autonomy_tuning(
        node, AutonomyTuningIntent(preset='confident'), 'controller'
    )

    for client in node._tuning_set_clients.values():
        assert len(client.requests) == 1
        assert client.requests[0].__class__.__name__.endswith('Request')
        assert len(client.requests[0].parameters) > 1
    for client in node._tuning_get_clients.values():
        assert len(client.requests) == 1
