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


def test_mux_configuration_has_only_persistent_local_and_stop_inputs():
    parameters = _parameters()

    assert parameters['use_stamped'] is False
    assert set(parameters['topics']) == {'teleop', 'global_stop'}
    assert parameters['topics']['teleop'] == {
        'topic': '/cmd_vel_teleop',
        'timeout': 0.15,
        'priority': 100,
    }
    assert parameters['topics']['global_stop'] == {
        'topic': '/cmd_vel_stop',
        'timeout': 0.10,
        'priority': 255,
    }
    assert (
        parameters['topics']['teleop']['priority']
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
    assert bench.count("package='runner_teleop'") == 2
    assert bench.count("executable='keyboard_bridge'") == 1
    assert bench.count("package='runner_drive_adapter'") == 1
    assert "package='runner_motor'" not in bench
    assert "package='runner_encoder'" not in bench
    assert "('/cmd_vel_out', '/cmd_vel')" in bench
    assert "('/cmd_vel_nav', '/cmd_vel')" not in bench
    assert "package='nav2_controller'" not in bench
    assert 'costmap' not in bench
    assert 'bt_navigator' not in bench

    assert manual.count("package='twist_mux'") == 1
    assert manual.count("package='runner_teleop'") == 2
    assert manual.count("executable='keyboard_bridge'") == 1
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


def test_runtime_local_release_cannot_expose_legacy_autonomy(monkeypatch):
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
    probe = Node('stage3_mux_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    outputs = []
    probe.create_subscription(Twist, '/cmd_vel', outputs.append, 10)
    teleop_pub = probe.create_publisher(Twist, '/cmd_vel_teleop', 10)
    legacy_auto_pub = probe.create_publisher(Twist, '/cmd_vel_auto', 10)
    lock_pub = probe.create_publisher(Bool, '/paddock/stop_lock', 10)
    teleop = Twist()
    teleop.linear.x = 0.63
    legacy = Twist()
    legacy.linear.x = 0.21
    brake = Twist()

    def publish(publisher, message, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=False))
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.01)

    try:
        assert _spin_until(
            executor,
            lambda: (
                len(probe.get_subscriptions_info_by_topic(
                    '/cmd_vel_teleop')) == 1
                and len(probe.get_publishers_info_by_topic('/cmd_vel')) == 1
            ),
            3.0,
        )
        assert probe.get_subscriptions_info_by_topic('/cmd_vel_auto') == []
        assert len(probe.get_publishers_info_by_topic('/cmd_vel')) == 1

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not outputs:
            lock_pub.publish(Bool(data=False))
            teleop_pub.publish(teleop)
            executor.spin_once(timeout_sec=0.01)
        assert outputs and outputs[-1].linear.x == pytest.approx(0.63)

        outputs.clear()
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            lock_pub.publish(Bool(data=False))
            teleop_pub.publish(brake)
            legacy_auto_pub.publish(legacy)
            executor.spin_once(timeout_sec=0.01)
        assert outputs
        assert all(message.linear.x == 0.0 for message in outputs)

        outputs.clear()
        publish(legacy_auto_pub, legacy, 0.40)
        assert outputs == []
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
