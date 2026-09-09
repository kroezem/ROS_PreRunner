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

from types import SimpleNamespace

from runner_paddock.gateway import GatewayResult
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


def test_runtime_start_stop_leaves_no_executor_thread():
    runtime = RosRuntime(StateCache())
    runtime.start()
    assert runtime.thread_alive
    runtime.stop()
    assert not runtime.thread_alive


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
