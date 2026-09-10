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

"""Focused durable STOP persistence tests without starting ROS behavior."""

import json

import pytest

from runner_interfaces.msg import StopState
from runner_paddock.stop_enforcer import _state_key, persist, restore


def test_state_key_ignores_stamp_but_detects_stop_changes():
    message = StopState(stopped=True)
    before = _state_key(message)
    message.stamp.sec = 9
    assert _state_key(message) == before
    message.stopped = False
    assert _state_key(message) != before


def test_stop_record_round_trip_is_atomic_and_typed(tmp_path):
    target = tmp_path / 'stop.json'

    assert persist(target, 7, True) == (7, True)

    assert restore(target) == (7, True)
    assert not target.with_suffix('.pending').exists()


@pytest.mark.parametrize(
    'value',
    [
        {},
        {'version': 2, 'generation': 1, 'stopped': True},
        {'version': 1, 'generation': -1, 'stopped': True},
        {'version': 1, 'generation': 1, 'stopped': 1},
    ],
)
def test_invalid_stop_record_fails_closed(value, tmp_path):
    target = tmp_path / 'stop.json'
    target.write_text(json.dumps(value))

    with pytest.raises(ValueError):
        restore(target)
