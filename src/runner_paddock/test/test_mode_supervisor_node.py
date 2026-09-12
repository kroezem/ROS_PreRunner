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

"""Safety-barrier tests for NEW MAP mode supervision."""

from runner_interfaces.msg import CommandAuthorityState, EncoderState
from runner_paddock.map_session_node import MapSessionNode
from runner_paddock.mode_supervisor_node import ModeSupervisorNode


def test_new_map_mode_request_id_advances_past_web_request_ids():
    node = MapSessionNode.__new__(MapSessionNode)
    node._new_map_request_seq = 7
    node._mode_accepted_request_id = 9_000_000_000_000_000_000

    assert node._next_mode_request_id() == 9_000_000_000_000_000_001


def test_quiescence_requires_stationary_sample_after_authority_revoke():
    node = ModeSupervisorNode.__new__(ModeSupervisorNode)
    node._quiescence_started_at = None
    node._quiescence_runtime_epoch = 0
    node._authority_revoked_at = None
    node._encoder_at = None
    node._encoder_stationary = False
    node._runtime = type('Runtime', (), {
        'state': type('State', (), {'runtime_epoch': 7})()
    })()

    node._begin_quiescence()
    node._on_encoder_state(EncoderState(stationary=True))
    node._on_authority_state(CommandAuthorityState(
        authority=CommandAuthorityState.AUTHORITY_NONE,
        runtime_epoch=7,
        brake_intent=True,
        reason='RUNTIME_NOT_STABLE',
    ))

    ready, reason = node._quiescence_ready()
    assert not ready
    assert reason == 'waiting for post-revocation encoder evidence'

    node._on_encoder_state(EncoderState(stationary=True))
    assert node._quiescence_ready() == (True, '')


def test_quiescence_rejects_manual_authority_and_moving_encoder():
    node = ModeSupervisorNode.__new__(ModeSupervisorNode)
    node._quiescence_started_at = None
    node._quiescence_runtime_epoch = 0
    node._authority_revoked_at = None
    node._encoder_at = None
    node._encoder_stationary = False
    node._runtime = type('Runtime', (), {
        'state': type('State', (), {'runtime_epoch': 7})()
    })()

    node._begin_quiescence()
    node._on_authority_state(CommandAuthorityState(
        authority=CommandAuthorityState.AUTHORITY_PADDOCK_MANUAL,
        runtime_epoch=7,
        brake_intent=False,
        reason='MANUAL_ACTIVE',
    ))
    assert node._quiescence_ready() == (
        False, 'revoking browser/manual motion'
    )

    node._on_authority_state(CommandAuthorityState(
        authority=CommandAuthorityState.AUTHORITY_NONE,
        runtime_epoch=7,
        brake_intent=True,
        reason='RUNTIME_NOT_STABLE',
    ))
    node._on_encoder_state(EncoderState(stationary=False))
    assert node._quiescence_ready() == (
        False, 'braking; encoder still reports motion'
    )


def test_quiescence_accepts_existing_stop_without_changing_it():
    node = ModeSupervisorNode.__new__(ModeSupervisorNode)
    node._quiescence_started_at = None
    node._quiescence_runtime_epoch = 0
    node._authority_revoked_at = None
    node._encoder_at = None
    node._encoder_stationary = False
    node._runtime = type('Runtime', (), {
        'state': type('State', (), {'runtime_epoch': 7})()
    })()

    node._begin_quiescence()
    node._on_authority_state(CommandAuthorityState(
        authority=CommandAuthorityState.AUTHORITY_NONE,
        runtime_epoch=7,
        brake_intent=True,
        reason='STOP_ASSERTED',
    ))
    node._on_encoder_state(EncoderState(stationary=True))

    assert node._quiescence_ready() == (True, '')
