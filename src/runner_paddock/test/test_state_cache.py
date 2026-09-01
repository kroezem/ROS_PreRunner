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

"""Tests for latest-state revisions, age, and freshness."""

import pytest

from runner_paddock.state_cache import StateCache


def test_revisions_only_advance_when_payload_changes():
    now = [10.0]
    cache = StateCache(clock=lambda: now[0])
    value = {'frame_id': 'map', 'poses': []}

    assert cache.update('plan', value)
    assert cache.large_snapshot('plan') == (1, value)
    now[0] += 0.5
    assert not cache.update('plan', value)
    assert cache.large_snapshot('plan') == (1, value)

    changed = {'frame_id': 'map', 'poses': [{'x': 1.0}]}
    assert cache.update('plan', changed)
    assert cache.large_snapshot('plan') == (2, changed)


def test_health_reports_age_and_freshness_without_wall_clock_math():
    now = [20.0]
    cache = StateCache(clock=lambda: now[0])
    cache.update('pose', {'x': 1.0})
    cache.update('map', {'data': []})
    assert cache.state_snapshot()['health']['status'] == 'ok'

    now[0] += 0.6
    health = cache.state_snapshot()['health']
    assert health['status'] == 'degraded'
    assert health['sources']['pose']['age_sec'] == pytest.approx(0.6)
    assert not health['sources']['pose']['fresh']
    assert health['sources']['map']['fresh']


def test_small_snapshots_do_not_expose_mutable_cache_state():
    cache = StateCache(clock=lambda: 1.0)
    cache.update('pose', {'position': {'x': 2.0}})
    snapshot = cache.state_snapshot()
    snapshot['pose']['position']['x'] = 99.0
    assert cache.state_snapshot()['pose']['position']['x'] == 2.0
