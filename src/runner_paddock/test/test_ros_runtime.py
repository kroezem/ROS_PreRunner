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
from geometry_msgs.msg import PoseWithCovarianceStamped
from runner_interfaces.msg import ConfigState
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import StopState
from runner_paddock.gateway import GatewayResult
from runner_paddock.gateway import InitialPoseIntent
from runner_paddock.ros_runtime import RosRuntime
from runner_paddock.ros_state_node import RosStateNode
from runner_paddock.state_cache import StateCache


class _Future:
    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return object()


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
    return SimpleNamespace(
        _cache=cache,
        _initial_pose_pub=_InitialPosePublisher(subscribers),
        _initial_pose_lock=threading.Lock(),
        _pending_initial_pose=None,
        _initial_pose_rejection=RosStateNode._initial_pose_rejection,
        _initial_pose_status=RosStateNode._initial_pose_status,
        get_clock=lambda: _Clock(),
    )


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
    assert status['state'] == 'accepted'
    assert status['map'] == 'studio'


def test_initial_pose_rejection_is_visible_and_publishes_nothing():
    node = _initial_pose_node(
        _ready_initial_pose_cache(stopped=False, stationary=False)
    )

    result = RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.0), 'controller'
    )

    assert not result.accepted
    assert result.reason == 'initial pose requires healthy applied STOP'
    assert not node._initial_pose_pub.messages
    status = node._cache.state_snapshot()['initial_pose']
    assert status['state'] == 'rejected'
    assert status['detail'] == result.reason


def test_post_request_slam_toolbox_pose_marks_seed_applied():
    cache = _ready_initial_pose_cache()
    node = _initial_pose_node(cache)
    RosStateNode._set_initial_pose(
        node, InitialPoseIntent(1.0, 2.0, 0.2), 'controller'
    )
    node.get_logger = lambda: SimpleNamespace(warning=lambda _message: None)
    observed = PoseWithCovarianceStamped()
    observed.header.frame_id = 'map'
    observed.header.stamp = Time(sec=13)
    observed.pose.pose.position.x = 1.04
    observed.pose.pose.position.y = 1.98
    observed.pose.pose.orientation.w = 1.0

    RosStateNode._on_localizer_pose(node, observed)

    status = cache.state_snapshot()['initial_pose']
    assert status['state'] == 'applied'
    assert status['observed']['pose']['position']['x'] == 1.04
