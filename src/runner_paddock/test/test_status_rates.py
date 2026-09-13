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

"""Focused status-publication cadence contracts."""

from runner_interfaces.msg import MapCatalogEntry, MapState
from runner_paddock.map_session_node import _message_key
from runner_paddock.map_session_node import MAP_STATE_HEARTBEAT_PERIOD_SEC


def test_map_state_is_one_hz_and_stamp_does_not_defeat_change_detection():
    assert MAP_STATE_HEARTBEAT_PERIOD_SEC == 1.0
    message = MapState(selected_map_applied='studio')
    before = _message_key(message)

    message.stamp.sec = 7
    assert _message_key(message) == before

    message.catalog.append(MapCatalogEntry(name='studio', complete=True))
    assert _message_key(message) != before
