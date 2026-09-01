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

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from runner_interfaces.msg import PaddockControlEvent

from runner_paddock.command_authority_node import _authority_message
from runner_paddock.command_authority_node import _from_twist
from runner_paddock.command_authority_node import _to_twist
from runner_paddock.command_authority_node import DEFAULT_SUPERVISION_PERIOD_SEC
from runner_paddock.command_authority_node import PADDOCK_OUTPUT_TOPIC
from runner_paddock.command_authority_node import RAW_AUTONOMY_TOPIC
from runner_paddock.command_authority_node import SUPERVISED_AUTONOMY_TOPIC
from runner_paddock.command_supervisor import CommandSupervisor
from runner_paddock.command_supervisor import ControlEvent
from runner_paddock.command_supervisor import SupervisorResult


def test_topic_contract_never_names_live_cmd_vel_as_an_output():
    assert RAW_AUTONOMY_TOPIC == '/cmd_vel_auto_raw'
    assert SUPERVISED_AUTONOMY_TOPIC == '/cmd_vel_auto'
    assert PADDOCK_OUTPUT_TOPIC == '/cmd_vel_paddock'
    assert '/cmd_vel' not in {
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
