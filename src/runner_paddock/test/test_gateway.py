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

"""Unit tests for the ROS-free operator-intent gateway."""

import itertools

from runner_paddock.gateway import (
    ControlEvent,
    ControlEventIntent,
    MapRequestIntent,
    ModeRequestIntent,
    OperatorGateway,
)


def _gateway():
    counter = itertools.count(1)
    return OperatorGateway(_uuid_factory=lambda: f'id{next(counter)}')


def test_single_lease_second_connection_is_observer():
    gw = _gateway()

    first = gw.handle('c1', {'action': 'acquire'})
    assert first.accepted and first.role == 'controller'
    assert first.intents[0].event == ControlEvent.LEASE_ACQUIRED

    second = gw.handle('c2', {'action': 'acquire'})
    assert not second.accepted and second.role == 'observer'

    # An observer cannot move the runtime or the car.
    assert not gw.handle('c2', {'action': 'stop'}).accepted
    assert not gw.handle('c2', {'action': 'select_mode',
                                'mode': 'autonomy'}).accepted


def test_control_event_sequence_is_monotonic_and_lease_scoped():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    seqs = []
    for _ in range(3):
        result = gw.handle('c1', {'action': 'heartbeat'})
        seqs.append(result.intents[0].sequence)
    assert seqs == sorted(seqs) and len(set(seqs)) == 3
    # acquire minted client_id=id1, lease_id=id2; every event carries them.
    intent = gw.handle('c1', {'action': 'heartbeat'}).intents[0]
    assert intent.client_id == 'id1' and intent.lease_id == 'id2'


def test_hold_to_run_edges():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})

    press = gw.handle('c1', {'action': 'run', 'held': True})
    assert press.intents[0].event == ControlEvent.RUN_PRESSED

    hold = gw.handle('c1', {'action': 'run', 'held': True})
    assert hold.intents[0].event == ControlEvent.HEARTBEAT

    release = gw.handle('c1', {'action': 'run', 'held': False})
    assert release.intents[0].event == ControlEvent.RUN_RELEASED

    # A second release is a no-op: nothing to revoke.
    assert gw.handle('c1', {'action': 'run', 'held': False}).intents == ()


def test_stop_clears_local_run_latch():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    gw.handle('c1', {'action': 'run', 'held': True})
    gw.handle('c1', {'action': 'stop'})
    # After STOP the local latch is cleared: holding again is a fresh press.
    assert gw.handle('c1', {'action': 'run', 'held': True}).intents[0].event == (
        ControlEvent.RUN_PRESSED
    )


def test_disconnect_releases_the_lease_and_reconnect_starts_clean():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    gw.handle('c1', {'action': 'run', 'held': True})

    released = gw.on_disconnect('c1')
    assert released.intents[0].event == ControlEvent.LEASE_RELEASED
    assert not gw.lease_held

    again = gw.handle('c2', {'action': 'acquire'})
    assert again.accepted
    # No RUN carried over: the run latch is reset.
    assert gw.public_state()['run_pressed'] is False


def test_goal_and_map_intents():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})

    goal = gw.handle('c1', {'action': 'select_goal', 'x': 1.5, 'y': -2.0,
                            'yaw': 0.3})
    intent = goal.intents[0]
    assert isinstance(intent, ControlEventIntent)
    assert intent.event == ControlEvent.GOAL_SELECTED
    assert (intent.goal_x, intent.goal_y, intent.goal_yaw) == (1.5, -2.0, 0.3)

    assert not gw.handle('c1', {'action': 'select_goal', 'x': 'x',
                                'y': 0.0}).accepted
    assert not gw.handle('c1', {'action': 'select_goal', 'x': float('nan'),
                                'y': 0.0}).accepted

    save = gw.handle('c1', {'action': 'save_map', 'name': 'studio2'})
    assert isinstance(save.intents[0], MapRequestIntent)
    assert save.intents[0].name == 'studio2'
    assert not gw.handle('c1', {'action': 'save_map', 'name': '  '}).accepted

    mode = gw.handle('c1', {'action': 'select_mode', 'mode': 'mapping'})
    assert isinstance(mode.intents[0], ModeRequestIntent)


def test_unknown_action_is_rejected():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    assert not gw.handle('c1', {'action': 'launch_missiles'}).accepted
    assert not gw.handle('c1', 'nope').accepted
