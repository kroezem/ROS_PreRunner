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

from runner_paddock.ros_runtime import RosRuntime
from runner_paddock.state_cache import StateCache


def test_runtime_start_stop_leaves_no_executor_thread():
    runtime = RosRuntime(StateCache())
    runtime.start()
    assert runtime.thread_alive
    runtime.stop()
    assert not runtime.thread_alive
