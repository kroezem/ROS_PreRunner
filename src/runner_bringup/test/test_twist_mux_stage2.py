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

"""Configuration and runtime arbitration tests for the Stage 2 mux."""

import os
from pathlib import Path
import random
import subprocess
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MUX_CONFIG = PACKAGE_ROOT / 'config' / 'twist_mux.yaml'
BENCH_LAUNCH = PACKAGE_ROOT / 'launch' / 'autonomy_bench.launch.py'
TELEOP_LAUNCH = PACKAGE_ROOT / 'launch' / 'teleop.launch.py'


def _parameters():
    document = yaml.safe_load(MUX_CONFIG.read_text())
    return document['twist_mux']['ros__parameters']


def test_mux_configuration_has_autonomy_local_and_stop_inputs():
    parameters = _parameters()

    assert parameters['use_stamped'] is False
    # v1.3 autonomy cutover adds exactly one autonomy input, below local
    # DualSense teleop and below the global STOP zero.
    assert set(parameters['topics']) == {
        'autonomy', 'paddock_manual', 'teleop', 'global_stop'
    }
    assert parameters['topics']['autonomy'] == {
        'topic': '/cmd_vel_auto',
        'timeout': 0.30,
        'priority': 50,
    }
    assert parameters['topics']['teleop'] == {
        'topic': '/cmd_vel_teleop',
        'timeout': 0.15,
        'priority': 100,
    }
    assert parameters['topics']['paddock_manual'] == {
        'topic': '/cmd_vel_paddock',
        'timeout': 0.30,
        'priority': 75,
    }
    assert parameters['topics']['global_stop'] == {
        'topic': '/cmd_vel_stop',
        'timeout': 0.10,
        'priority': 255,
    }
    assert (
        parameters['topics']['autonomy']['priority']
        < parameters['topics']['paddock_manual']['priority']
        < parameters['topics']['teleop']['priority']
        < parameters['locks']['global_stop']['priority']
        < parameters['topics']['global_stop']['priority']
    )
    assert parameters['locks']['global_stop'] == {
        'topic': '/paddock/stop_lock',
        'timeout': 0.15,
        'priority': 200,
    }


def test_launches_remap_only_mux_output_to_normalized_motor_input():
    bench = BENCH_LAUNCH.read_text()
    manual = TELEOP_LAUNCH.read_text()

    assert bench.count("package='twist_mux'") == 1
    assert bench.count("package='runner_teleop'") == 1
    assert "executable='keyboard_bridge'" not in bench
    assert bench.count("package='runner_drive_adapter'") == 1
    assert "package='runner_motor'" not in bench
    assert "package='runner_encoder'" not in bench
    assert "('/cmd_vel_out', '/cmd_vel')" in bench
    assert "('/cmd_vel_nav', '/cmd_vel')" not in bench
    assert "package='nav2_controller'" not in bench
    assert 'costmap' not in bench
    assert 'bt_navigator' not in bench

    assert manual.count("package='twist_mux'") == 1
    assert manual.count("package='runner_teleop'") == 1
    assert "executable='keyboard_bridge'" not in manual
    assert "package='runner_motor'" not in manual
    assert "('/cmd_vel_out', '/cmd_vel')" in manual
    assert "('/cmd_vel_nav', '/cmd_vel')" not in manual
    assert "'manual_trigger_expo': 0.50" in bench
    assert "'manual_trigger_expo': 0.50" in manual


def _spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.005)


def _spin_until(executor, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.002)
        if predicate():
            return True
    return False


def _publish_for(executor, publisher, message, duration, period=0.02):
    deadline = time.monotonic() + duration
    next_publication = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publication:
            publisher.publish(message)
            next_publication += period
        executor.spin_once(timeout_sec=0.002)


def test_runtime_autonomy_input_is_below_local_teleop_and_stop(monkeypatch):
    """The one autonomy input never overrides DualSense teleop or STOP."""
    domain_id = str(random.randint(120, 220))
    monkeypatch.setenv('ROS_DOMAIN_ID', domain_id)
    environment = os.environ.copy()
    process = subprocess.Popen(
        [
            '/opt/ros/jazzy/lib/twist_mux/twist_mux',
            '--ros-args',
            '--params-file',
            str(MUX_CONFIG),
            '-r',
            '/cmd_vel_out:=/cmd_vel',
        ],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    probe = Node('stage7_mux_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    outputs = []
    probe.create_subscription(Twist, '/cmd_vel', outputs.append, 10)
    teleop_pub = probe.create_publisher(Twist, '/cmd_vel_teleop', 10)
    auto_pub = probe.create_publisher(Twist, '/cmd_vel_auto', 10)
    stop_pub = probe.create_publisher(Twist, '/cmd_vel_stop', 10)
    lock_pub = probe.create_publisher(Bool, '/paddock/stop_lock', 10)
    teleop = Twist()
    teleop.linear.x = 0.63
    auto = Twist()
    auto.linear.x = 0.21
    brake = Twist()

    try:
        assert _spin_until(
            executor,
            lambda: (
                len(probe.get_subscriptions_info_by_topic(
                    '/cmd_vel_teleop')) == 1
                and len(probe.get_subscriptions_info_by_topic(
                    '/cmd_vel_auto')) == 1
                and len(probe.get_publishers_info_by_topic('/cmd_vel')) == 1
            ),
            3.0,
        )

        # Local DualSense teleop (priority 100) overrides autonomy (50).
        outputs.clear()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=False))
            teleop_pub.publish(teleop)
            auto_pub.publish(auto)
            executor.spin_once(timeout_sec=0.01)
        assert outputs and outputs[-1].linear.x == pytest.approx(0.63)

        # Local release: autonomy IS now selected (the intended cutover), but
        # only because the command authority is its sole, supervised writer.
        outputs.clear()
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=False))
            auto_pub.publish(auto)
            executor.spin_once(timeout_sec=0.01)
        assert outputs and outputs[-1].linear.x == pytest.approx(0.21)

        # STOP lock (priority 200) masks the autonomy input entirely.
        outputs.clear()
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=True))
            auto_pub.publish(auto)
            executor.spin_once(timeout_sec=0.01)
        assert all(message.linear.x == 0.0 for message in outputs)

        # Global STOP zero (priority 255) overrides autonomy directly.
        outputs.clear()
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=False))
            stop_pub.publish(brake)
            auto_pub.publish(auto)
            executor.spin_once(timeout_sec=0.01)
        assert outputs and all(
            message.linear.x == 0.0 for message in outputs
        )
    finally:
        executor.remove_node(probe)
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
