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

"""ROS graph integration tests for Stage 1 topic isolation."""

import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from runner_drive_adapter.drive_adapter_node import DriveAdapterNode
from runner_interfaces.msg import AdapterState, EncoderState
from std_msgs.msg import String


def _spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def test_graph_ownership_staleness_and_diagnostics():
    previous_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 30)
    rclpy.init()
    adapter = DriveAdapterNode()
    probe = Node('drive_adapter_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)

    commands = []
    states = []
    typed_states = []
    probe.create_subscription(
        Twist,
        '/cmd_vel_auto',
        commands.append,
        10,
    )
    probe.create_subscription(
        String,
        '/drive_adapter/state',
        states.append,
        10,
    )
    probe.create_subscription(
        AdapterState,
        '/drive_adapter/state_typed',
        typed_states.append,
        10,
    )
    nav_pub = probe.create_publisher(Twist, '/cmd_vel_nav', 10)
    encoder_pub = probe.create_publisher(
        EncoderState,
        '/wheel/encoder_state',
        10,
    )
    motion_pub = probe.create_publisher(
        Odometry, '/odometry/filtered', 10
    )
    active_mode_pub = probe.create_publisher(
        String, '/teleop/active_mode', 10
    )
    mux_output_pub = probe.create_publisher(Twist, '/cmd_vel', 10)

    try:
        _spin_for(executor, 0.15)
        assert commands == []
        assert any('reason=no_command' in state.data for state in states)
        assert len(probe.get_publishers_info_by_topic('/cmd_vel_auto')) == 1
        cmd_vel_publishers = probe.get_publishers_info_by_topic('/cmd_vel')
        assert not any(
            endpoint.node_name == 'drive_adapter'
            for endpoint in cmd_vel_publishers
        )
        nav_subscriptions = probe.get_subscriptions_info_by_topic(
            '/cmd_vel_nav'
        )
        encoder_subscriptions = probe.get_subscriptions_info_by_topic(
            '/wheel/encoder_state'
        )
        active_mode_subscriptions = probe.get_subscriptions_info_by_topic(
            '/teleop/active_mode'
        )
        mux_output_subscriptions = probe.get_subscriptions_info_by_topic(
            '/cmd_vel'
        )
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in nav_subscriptions
        ) == 1
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in encoder_subscriptions
        ) == 1
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in active_mode_subscriptions
        ) == 1
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in mux_output_subscriptions
        ) == 1

        encoder_pub.publish(EncoderState(
            stationary=False,
            edge_rate=2.0,
            pending_direction=1,
        ))
        motion = Odometry()
        motion.twist.twist.linear.x = 0.20
        motion.twist.twist.angular.z = 0.15
        motion_pub.publish(motion)
        command = Twist()
        command.linear.x = 0.20
        command.angular.z = 0.10
        nav_pub.publish(command)
        mux_output_pub.publish(command)
        _spin_for(executor, 0.15)
        assert commands
        assert commands[-1].linear.x > 0.0
        assert commands[-1].angular.z > 0.0
        assert any('mode=forward' in state.data for state in states)
        assert 'measured_speed=' in states[-1].data
        assert 'integrator_state=' in states[-1].data
        assert 'wheelspin_guard=' in states[-1].data
        assert 'steering_saturated=false' in states[-1].data
        assert typed_states
        assert typed_states[-1].commanded_speed == 0.20
        assert typed_states[-1].effective_speed == 0.25
        assert typed_states[-1].speed_error == (
            typed_states[-1].effective_speed
            - typed_states[-1].measured_speed
        )
        assert not typed_states[-1].feedforward_floor_violation
        assert typed_states[-1].commanded_yaw_rate == 0.10
        assert typed_states[-1].measured_yaw_rate == 0.15
        assert typed_states[-1].steering_curvature_requested == 0.5
        assert typed_states[-1].steering_curvature_max > 0.0
        assert (
            typed_states[-1].integrator_freeze_reason
            == AdapterState.INTEGRATOR_GAIN_DISABLED
        )
        assert typed_states[-1].mode.startswith('forward;')
        assert 'active_mode_received=false' in typed_states[-1].mode
        assert probe.get_publishers_info_by_topic('/stall_assist/state') == []

        typed_state_start = len(typed_states)
        active_mode_pub.publish(String(data='manual'))
        nav_pub.publish(command)
        encoder_pub.publish(EncoderState(
            stationary=False,
            edge_rate=2.0,
            pending_direction=1,
        ))
        motion_pub.publish(motion)
        _spin_for(executor, 0.10)
        assert commands
        manual_states = [
            state for state in typed_states[typed_state_start:]
            if 'active_mode=manual' in state.mode
        ]
        assert manual_states
        manual_state = manual_states[-1]
        assert 'preempted=true' in manual_state.mode
        assert 'integral_decay_active=false' in manual_state.mode
        assert not manual_state.integrator_enabled
        assert manual_state.integrator_state == 0.0
        assert manual_state.pi_term == manual_state.proportional_term

        typed_state_start = len(typed_states)
        active_mode_pub.publish(String(data='teleop_suppress'))
        nav_pub.publish(command)
        encoder_pub.publish(EncoderState(
            stationary=False,
            edge_rate=2.0,
            pending_direction=1,
        ))
        motion_pub.publish(motion)
        _spin_for(executor, 0.10)
        suppress_states = [
            state for state in typed_states[typed_state_start:]
            if 'active_mode=teleop_suppress' in state.mode
        ]
        assert suppress_states
        assert 'preempted=false' in suppress_states[-1].mode

        _spin_for(executor, 0.30)
        commands.clear()
        _spin_for(executor, 0.15)
        assert commands == []
        assert 'reason=stale_command' in states[-1].data

        stop = Twist()
        nav_pub.publish(stop)
        _spin_for(executor, 0.20)
        assert commands
        assert commands[-1].linear.x == 0.0
        assert commands[-1].angular.z == 0.0
        assert 'reason=explicit_stop' in states[-1].data

        state_count = len(typed_states)
        _spin_for(executor, 0.50)
        published = len(typed_states) - state_count
        assert 8 <= published <= 12

        adapter.adapter._integrator = 0.004
        result = adapter.set_parameters([
            Parameter('integral_gain', Parameter.Type.DOUBLE, 0.01)
        ])[0]
        assert result.successful
        assert adapter.adapter.integrator_state == 0.0
        result = adapter.set_parameters([
            Parameter('integral_gain', Parameter.Type.DOUBLE, 0.0)
        ])[0]
        assert result.successful
    finally:
        executor.remove_node(probe)
        executor.remove_node(adapter)
        probe.destroy_node()
        adapter.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        if previous_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain
