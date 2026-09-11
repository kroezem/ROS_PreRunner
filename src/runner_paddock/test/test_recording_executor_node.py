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

"""ROS QoS regression coverage for the recording executor."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from runner_interfaces.msg import PaddockControlLease

from runner_paddock.recording import RecordingExecutor
import runner_paddock.recording_executor_node as recording_executor_node


class _ObservedRecordingExecutorNode(
    recording_executor_node.RecordingExecutorNode
):
    def __init__(self):
        self.subscription_qos = {}
        super().__init__()

    def create_subscription(
        self, message_type, topic, callback, qos_profile, **kwargs
    ):
        self.subscription_qos[topic] = qos_profile
        return super().create_subscription(
            message_type, topic, callback, qos_profile, **kwargs
        )


def test_volatile_control_lease_reaches_recording_executor(
    tmp_path, monkeypatch,
):
    """Match the authority lease publisher and receive its current lease."""
    runtime_path = tmp_path / 'runtime.json'
    monkeypatch.setattr(
        recording_executor_node,
        'RecordingExecutor',
        lambda root: RecordingExecutor(root, runtime_path=runtime_path),
    )
    rclpy.init(args=[
        '--ros-args',
        '-p', f'recording_directory:={tmp_path / "bags"}',
    ])
    executor = None
    authority = None
    try:
        executor = _ObservedRecordingExecutorNode()
        authority = Node('recording_executor_qos_test_authority')
        lease_publisher = authority.create_publisher(
            PaddockControlLease, '/paddock/control_lease', 10
        )

        qos = executor.subscription_qos['/paddock/control_lease']
        assert qos.depth == 10
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.VOLATILE

        lease = PaddockControlLease(active=True, lease_id='current-lease')
        deadline = time.monotonic() + 2.0
        while executor._lease_id != lease.lease_id:
            lease_publisher.publish(lease)
            rclpy.spin_once(executor, timeout_sec=0.05)
            assert time.monotonic() < deadline
    finally:
        if authority is not None:
            authority.destroy_node()
        if executor is not None:
            executor.destroy_node()
        rclpy.shutdown()
