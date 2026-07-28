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

import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from runner_drive_adapter.drive_adapter_node import DriveAdapterNode
from runner_interfaces.msg import EncoderState
from std_msgs.msg import String


def _spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def test_graph_ownership_staleness_and_diagnostics():
    rclpy.init()
    adapter = DriveAdapterNode()
    probe = Node('drive_adapter_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(adapter)
    executor.add_node(probe)

    commands = []
    states = []
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
    nav_pub = probe.create_publisher(Twist, '/cmd_vel_nav', 10)
    encoder_pub = probe.create_publisher(
        EncoderState,
        '/wheel/encoder_state',
        10,
    )
    motion_pub = probe.create_publisher(
        Odometry, '/odometry/filtered', 10
    )

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
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in nav_subscriptions
        ) == 1
        assert sum(
            endpoint.node_name == 'drive_adapter'
            for endpoint in encoder_subscriptions
        ) == 1

        encoder_pub.publish(EncoderState(
            stationary=False,
            edge_rate=2.0,
            pending_direction=1,
        ))
        motion = Odometry()
        motion.twist.twist.linear.x = 0.20
        motion_pub.publish(motion)
        command = Twist()
        command.linear.x = 0.20
        nav_pub.publish(command)
        _spin_for(executor, 0.15)
        assert commands
        assert commands[-1].linear.x > 0.0
        assert commands[-1].angular.z == 0.0
        assert any('mode=forward' in state.data for state in states)
        assert 'measured_speed=' in states[-1].data
        assert 'integrator_state=' in states[-1].data
        assert 'wheelspin_guard=' in states[-1].data
        assert probe.get_publishers_info_by_topic('/stall_assist/state') == []

        _spin_for(executor, 0.30)
        commands.clear()
        _spin_for(executor, 0.15)
        assert commands == []
        assert 'reason=stale_command' in states[-1].data

        stop = Twist()
        nav_pub.publish(stop)
        _spin_for(executor, 0.10)
        assert commands[-1].linear.x == -1.0
        assert commands[-1].angular.z == 0.0
        assert 'reason=explicit_stop' in states[-1].data
    finally:
        executor.remove_node(probe)
        executor.remove_node(adapter)
        probe.destroy_node()
        adapter.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
