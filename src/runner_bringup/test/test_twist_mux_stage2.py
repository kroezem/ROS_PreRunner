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
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MUX_CONFIG = PACKAGE_ROOT / 'config' / 'twist_mux.yaml'
BENCH_LAUNCH = PACKAGE_ROOT / 'launch' / 'autonomy_bench.launch.py'
TELEOP_LAUNCH = PACKAGE_ROOT / 'launch' / 'teleop.launch.py'


def _parameters():
    document = yaml.safe_load(MUX_CONFIG.read_text())
    return document['twist_mux']['ros__parameters']


def test_mux_configuration_is_explicit_and_uses_only_stage2_inputs():
    parameters = _parameters()

    assert parameters['use_stamped'] is False
    assert set(parameters['topics']) == {'teleop', 'autonomy'}
    assert parameters['topics']['teleop'] == {
        'topic': '/cmd_vel_teleop',
        'timeout': 0.20,
        'priority': 100,
    }
    assert parameters['topics']['autonomy'] == {
        'topic': '/cmd_vel_auto',
        'timeout': 0.30,
        'priority': 50,
    }
    assert (
        parameters['topics']['teleop']['priority']
        > parameters['topics']['autonomy']['priority']
    )
    assert 'locks' not in parameters


def test_launches_remap_only_mux_output_to_normalized_motor_input():
    bench = BENCH_LAUNCH.read_text()
    manual = TELEOP_LAUNCH.read_text()

    assert bench.count("package='twist_mux'") == 1
    assert bench.count("package='runner_teleop'") == 2
    assert bench.count("executable='keyboard_bridge'") == 1
    assert bench.count("package='runner_drive_adapter'") == 1
    assert bench.count("package='runner_motor'") == 1
    assert bench.count("package='runner_encoder'") == 1
    assert "('/cmd_vel_out', '/cmd_vel')" in bench
    assert "('/cmd_vel_nav', '/cmd_vel')" not in bench
    assert "package='nav2_controller'" not in bench
    assert 'costmap' not in bench
    assert 'bt_navigator' not in bench

    assert manual.count("package='twist_mux'") == 1
    assert manual.count("package='runner_teleop'") == 2
    assert manual.count("executable='keyboard_bridge'") == 1
    assert "('/cmd_vel_out', '/cmd_vel')" in manual
    assert "('/cmd_vel_nav', '/cmd_vel')" not in manual


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


def test_runtime_arbitration_preemption_fallthrough_and_stale_silence(
    monkeypatch,
):
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
    probe = Node('stage2_mux_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    outputs = []
    output_times = []

    def on_output(message):
        outputs.append(message)
        output_times.append(time.monotonic())

    probe.create_subscription(Twist, '/cmd_vel', on_output, 10)
    teleop_pub = probe.create_publisher(
        Twist, '/cmd_vel_teleop', 10
    )
    auto_pub = probe.create_publisher(Twist, '/cmd_vel_auto', 10)
    teleop = Twist()
    teleop.linear.x = 0.63
    auto = Twist()
    auto.linear.x = 0.21
    brake = Twist()
    brake.linear.x = -1.0

    try:
        assert _spin_until(
            executor,
            lambda: len(
                probe.get_subscriptions_info_by_topic('/cmd_vel_teleop')
            ) == 1,
            3.0,
        )
        assert len(
            probe.get_subscriptions_info_by_topic('/cmd_vel_auto')
        ) == 1
        assert len(probe.get_publishers_info_by_topic('/cmd_vel')) == 1
        assert probe.get_publishers_info_by_topic('/cmd_vel_nav') == []
        assert all(
            endpoint.topic_type == 'geometry_msgs/msg/Twist'
            for endpoint in (
                probe.get_subscriptions_info_by_topic('/cmd_vel_teleop')
                + probe.get_subscriptions_info_by_topic('/cmd_vel_auto')
                + probe.get_publishers_info_by_topic('/cmd_vel')
            )
        )
        _spin_for(executor, 0.25)

        outputs.clear()
        _publish_for(executor, auto_pub, auto, 0.15)
        assert outputs
        assert outputs[-1].linear.x == pytest.approx(0.21)

        last_teleop = time.monotonic()
        teleop_pub.publish(teleop)
        assert _spin_until(
            executor,
            lambda: outputs and outputs[-1].linear.x == 0.63,
            0.10,
        )
        output_count = len(outputs)
        _publish_for(executor, auto_pub, auto, 0.10)
        assert len(outputs) == output_count

        first_fallthrough = None
        deadline = last_teleop + 0.35
        while time.monotonic() < deadline and first_fallthrough is None:
            auto_pub.publish(auto)
            before = len(outputs)
            _spin_for(executor, 0.01)
            for index in range(before, len(outputs)):
                if outputs[index].linear.x == pytest.approx(0.21):
                    first_fallthrough = output_times[index]
                    break
        assert first_fallthrough is not None
        assert 0.20 <= first_fallthrough - last_teleop <= 0.27

        published_at = time.monotonic()
        teleop_pub.publish(teleop)
        previous_count = len(outputs)
        assert _spin_until(
            executor,
            lambda: len(outputs) > previous_count,
            0.10,
        )
        assert outputs[-1].linear.x == pytest.approx(0.63)
        assert output_times[-1] - published_at < 0.05

        published_at = time.monotonic()
        teleop_pub.publish(brake)
        previous_count = len(outputs)
        assert _spin_until(
            executor,
            lambda: len(outputs) > previous_count,
            0.10,
        )
        assert outputs[-1].linear.x == -1.0
        assert output_times[-1] - published_at < 0.05

        outputs.clear()
        output_times.clear()
        _spin_for(executor, 0.45)
        assert outputs == []

        auto_pub.publish(auto)
        assert _spin_until(executor, lambda: bool(outputs), 0.10)
        outputs.clear()
        _spin_for(executor, 0.40)
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
