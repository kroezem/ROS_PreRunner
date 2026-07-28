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

"""Tests for adapter log throttling."""

from runner_drive_adapter.drive_adapter_node import WarningThrottle


def test_repeated_infeasible_warning_key_is_throttled():
    gate = WarningThrottle(interval=5.0)
    assert gate.allows('steering_infeasible', 0.0)
    assert not gate.allows('steering_infeasible', 0.1)
    assert not gate.allows('steering_infeasible', 4.999)
    assert gate.allows('steering_infeasible', 5.0)


def test_warning_keys_are_independent():
    gate = WarningThrottle(interval=5.0)
    assert gate.allows('steering_infeasible', 1.0)
    assert gate.allows('negative_speed', 1.0)
