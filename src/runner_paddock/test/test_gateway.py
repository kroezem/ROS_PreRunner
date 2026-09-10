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
    ClearCostmapsIntent,
    ConfigRequestIntent,
    ControlEvent,
    ControlEventIntent,
    InitialPoseIntent,
    MapRequestIntent,
    ModeRequestIntent,
    OperatorGateway,
    RecordingRequestIntent,
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


def test_clear_obstacles_requires_lease_and_requests_nav2_services():
    gw = _gateway()
    assert not gw.handle('observer', {'action': 'clear_obstacles'}).accepted

    gw.handle('c1', {'action': 'acquire'})
    result = gw.handle('c1', {'action': 'clear_obstacles'})

    assert result.accepted
    assert result.role == 'controller'
    assert result.intents == (ClearCostmapsIntent(),)


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
    non_map = gw.handle('c1', {'action': 'select_goal', 'frame': 'odom',
                               'x': 1.0, 'y': 2.0, 'yaw': 0.0})
    assert not non_map.accepted
    assert non_map.reason == 'goal frame must be map'

    save = gw.handle('c1', {'action': 'save_map', 'name': 'studio2'})
    assert isinstance(save.intents[0], MapRequestIntent)
    assert save.intents[0].name == 'studio2'
    assert not gw.handle('c1', {'action': 'save_map', 'name': '  '}).accepted

    select = gw.handle('c1', {'action': 'select_map', 'name': 'studio2'})
    assert select.accepted
    assert len(select.intents) == 1
    assert isinstance(select.intents[0], MapRequestIntent)
    assert select.intents[0].name == 'studio2'
    assert not gw.handle(
        'c1', {'action': 'select_map', 'name': '  '}
    ).accepted

    delete = gw.handle('c1', {'action': 'delete_map', 'name': 'studio2'})
    assert delete.accepted
    assert isinstance(delete.intents[0], MapRequestIntent)
    assert delete.intents[0].name == 'studio2'
    assert not gw.handle(
        'c1', {'action': 'delete_map', 'name': '  '}
    ).accepted

    mode = gw.handle('c1', {'action': 'select_mode', 'mode': 'mapping'})
    assert isinstance(mode.intents[0], ModeRequestIntent)


def test_initial_pose_is_lease_scoped_finite_and_map_frame_only():
    gw = _gateway()
    action = {
        'action': 'set_initial_pose', 'frame': 'map',
        'x': 1.25, 'y': -0.5, 'yaw': 0.75,
    }
    assert not gw.handle('observer', action).accepted
    gw.handle('c1', {'action': 'acquire'})

    result = gw.handle('c1', action)

    assert result.accepted
    assert result.intents == (InitialPoseIntent(
        x=1.25, y=-0.5, yaw=0.75, frame='map'
    ),)
    assert not gw.handle('c1', {**action, 'frame': 'odom'}).accepted
    assert not gw.handle('c1', {
        **action, 'x': float('nan')
    }).accepted


def test_unknown_action_is_rejected():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    assert not gw.handle('c1', {'action': 'launch_missiles'}).accepted
    assert not gw.handle('c1', 'nope').accepted


def test_manual_samples_keep_si_units_and_release_explicitly():
    gw = _gateway()
    gw.handle('c1', {'action': 'acquire'})
    active = gw.handle('c1', {
        'action': 'manual', 'active': True,
        'speed_mps': 0.30, 'steering': -0.5,
    })
    intent = active.intents[0]
    assert intent.event == ControlEvent.MANUAL_ACTIVE
    assert intent.manual_speed_mps == 0.30
    assert intent.manual_steering == -0.5

    released = gw.handle('c1', {'action': 'manual', 'active': False})
    assert released.intents[0].event == ControlEvent.MANUAL_INACTIVE


def test_manual_speed_configuration_is_lease_scoped_and_revisioned():
    gw = _gateway()
    action = {
        'action': 'set_config',
        'field': 'manual_max_speed_mps',
        'value': 0.40,
        'expected_revision': 2,
    }
    assert not gw.handle('observer', action).accepted
    gw.handle('c1', {'action': 'acquire'})

    result = gw.handle('c1', action)

    assert result.accepted
    assert result.intents == (ConfigRequestIntent(
        lease_id='id2', expected_revision=2,
        field='manual_max_speed_mps', value=0.40,
    ),)


def test_recording_requests_are_lease_scoped_but_not_socket_owned():
    gw = _gateway()
    assert not gw.handle('observer', {'action': 'start_recording'}).accepted
    gw.handle('c1', {'action': 'acquire'})

    start = gw.handle('c1', {
        'action': 'start_recording', 'name': '', 'profile': 'runner_debug'
    })
    assert isinstance(start.intents[0], RecordingRequestIntent)
    assert start.intents[0].name == ''
    assert start.intents[0].profile == 'runner_debug'
    disconnect = gw.on_disconnect('c1')
    assert not any(
        isinstance(intent, RecordingRequestIntent)
        for intent in disconnect.intents
    )

    gw.handle('c2', {'action': 'acquire'})
    stop = gw.handle('c2', {'action': 'stop_recording'})
    assert isinstance(stop.intents[0], RecordingRequestIntent)
