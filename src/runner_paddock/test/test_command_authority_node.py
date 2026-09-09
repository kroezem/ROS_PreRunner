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

"""ROS-plumbing contract tests that do not launch production behavior."""

from dataclasses import replace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from runner_interfaces.msg import PaddockControlEvent

from runner_paddock.command_authority_node import _authority_message
from runner_paddock.command_authority_node import _AutonomyOutputGate
from runner_paddock.command_authority_node import _from_twist
from runner_paddock.command_authority_node import _to_twist
from runner_paddock.command_authority_node import DEFAULT_SUPERVISION_PERIOD_SEC
from runner_paddock.command_authority_node import PADDOCK_OUTPUT_TOPIC
from runner_paddock.command_authority_node import RAW_AUTONOMY_TOPIC
from runner_paddock.command_authority_node import SUPERVISED_AUTONOMY_TOPIC
from runner_paddock.command_supervisor import CommandSupervisor
from runner_paddock.command_supervisor import ControlEvent
from runner_paddock.command_supervisor import SupervisorResult
from runner_paddock.command_supervisor import VelocityCommand
from runner_paddock.state_machine import GoalIntent


def test_topic_contract_never_names_live_cmd_vel_as_an_output():
    assert RAW_AUTONOMY_TOPIC == '/cmd_vel_auto_raw'
    # v1.3 cutover: this node is the sole supervised writer of the one mux
    # autonomy input. It still must never write the final /cmd_vel itself.
    assert SUPERVISED_AUTONOMY_TOPIC == '/cmd_vel_auto'
    assert PADDOCK_OUTPUT_TOPIC == '/cmd_vel_paddock'
    assert '/cmd_vel' not in {
        RAW_AUTONOMY_TOPIC,
        SUPERVISED_AUTONOMY_TOPIC,
        PADDOCK_OUTPUT_TOPIC,
    }
    assert DEFAULT_SUPERVISION_PERIOD_SEC == 0.010


def test_twist_conversion_preserves_every_component():
    message = Twist()
    message.linear.x = 1.0
    message.linear.y = 2.0
    message.linear.z = 3.0
    message.angular.x = 4.0
    message.angular.y = 5.0
    message.angular.z = 6.0

    restored = _to_twist(_from_twist(message))

    assert restored == message


def test_control_event_enum_matches_generated_interface():
    assert int(ControlEvent.RUN_PRESSED) == PaddockControlEvent.EVENT_RUN_PRESSED
    assert int(ControlEvent.RUN_RELEASED) == PaddockControlEvent.EVENT_RUN_RELEASED
    assert int(ControlEvent.STOP) == PaddockControlEvent.EVENT_STOP
    assert int(ControlEvent.GOAL_SELECTED) == PaddockControlEvent.EVENT_GOAL_SELECTED
    assert int(ControlEvent.LEASE_ACQUIRED) == PaddockControlEvent.EVENT_LEASE_ACQUIRED
    assert int(ControlEvent.LEASE_RELEASED) == PaddockControlEvent.EVENT_LEASE_RELEASED
    assert int(ControlEvent.HEARTBEAT) == PaddockControlEvent.EVENT_HEARTBEAT
    assert int(ControlEvent.CLEAR_STOP) == PaddockControlEvent.EVENT_CLEAR_STOP


def test_authority_message_exposes_monotonic_ages_and_brake_state():
    supervisor = CommandSupervisor(active_autonomy_map='map')
    snapshot = supervisor.tick(4.0).snapshot

    message = _authority_message(SupervisorResult(snapshot), Time(sec=9))

    assert message.stamp.sec == 9
    assert message.brake_intent
    assert not message.lease_fresh
    assert message.lease_age_sec == -1.0
    assert not message.raw_autonomy_fresh
    assert message.raw_autonomy_age_sec == -1.0
    assert message.reason == 'IDLE_BRAKE'


def test_authority_message_exposes_selected_goal_as_backend_truth():
    supervisor = CommandSupervisor(active_autonomy_map='studio')
    snapshot = supervisor.tick(4.0).snapshot
    goal = GoalIntent('studio', 1.25, -0.75, 0.4)
    state = replace(snapshot.state, goal=goal)
    snapshot = replace(snapshot, state=state)

    message = _authority_message(SupervisorResult(snapshot), Time(sec=9))

    assert message.autonomy_goal_selected
    assert message.goal_frame == 'map'
    assert message.goal_map == 'studio'
    assert (message.goal_x, message.goal_y, message.goal_yaw) == (
        1.25, -0.75, 0.4,
    )


def test_steady_20_hz_autonomy_has_no_injected_zeros_between_samples():
    gate = _AutonomyOutputGate(brake_window_sec=0.30)
    command = VelocityCommand(linear_x=0.4, angular_z=0.2)
    outputs = []

    for sample_index in range(5):
        sample_at = sample_index * 0.050
        outputs.append(gate.update(
            now=sample_at, permitted=True, command=command,
        ))
        for tick_index in range(1, 5):
            outputs.append(gate.update(
                now=sample_at + tick_index * 0.010,
                permitted=True,
                command=None,
            ))

    published = [output for output in outputs if output is not None]
    assert published == [command] * 5
    assert all(output != VelocityCommand() for output in published)


def test_raw_timeout_transition_brakes_then_goes_silent():
    gate = _AutonomyOutputGate(brake_window_sec=0.30)
    command = VelocityCommand(linear_x=0.4)

    assert gate.update(now=0.0, permitted=True, command=command) == command
    assert gate.update(now=0.150, permitted=True, command=None) is None
    assert gate.update(
        now=0.150000001, permitted=False, command=None,
    ) == VelocityCommand()
    assert gate.update(
        now=0.300, permitted=False, command=None,
    ) == VelocityCommand()
    assert gate.update(now=0.450000002, permitted=False, command=None) is None
    assert gate.update(now=0.500, permitted=False, command=None) is None


def test_run_or_stop_revoke_brakes_without_rearming_the_window():
    gate = _AutonomyOutputGate(brake_window_sec=0.30)
    command = VelocityCommand(linear_x=0.4)

    assert gate.update(now=1.0, permitted=True, command=command) == command
    assert gate.update(
        now=1.01, permitted=False, command=None,
    ) == VelocityCommand()
    assert gate.update(
        now=1.30, permitted=False, command=None,
    ) == VelocityCommand()
    assert gate.update(now=1.310000001, permitted=False, command=None) is None

    # Remaining revoked cannot start another brake window.
    assert gate.update(now=2.0, permitted=False, command=None) is None
