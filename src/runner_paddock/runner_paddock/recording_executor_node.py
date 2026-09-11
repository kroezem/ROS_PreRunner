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

"""ROS owner for the persistent Paddock rosbag2 MCAP executor."""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import PaddockControlLease
from runner_interfaces.msg import RecordingEntry, RecordingRequest, RecordingState
from runner_paddock.recording import DEFAULT_RECORDING_DIRECTORY
from runner_paddock.recording import RecordingExecutor


class RecordingExecutorNode(Node):
    """Validate lease-scoped operations and publish truthful recorder state."""

    def __init__(self) -> None:
        super().__init__('runner_recording_executor')
        root = Path(str(self.declare_parameter(
            'recording_directory', str(DEFAULT_RECORDING_DIRECTORY)
        ).value))
        self._executor = RecordingExecutor(root)
        self._lease_id = ''
        recording_state_qos = QoSProfile(depth=1)
        recording_state_qos.reliability = ReliabilityPolicy.RELIABLE
        recording_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            RecordingState, '/paddock/recording_state', recording_state_qos
        )
        self.create_subscription(
            RecordingRequest,
            '/paddock/recording_request',
            self._on_request,
            10,
        )
        self.create_subscription(
            PaddockControlLease,
            '/paddock/control_lease',
            self._on_lease,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_timer(0.5, self._tick)
        self._publish()

    def _on_lease(self, message: PaddockControlLease) -> None:
        self._lease_id = message.lease_id if message.active else ''

    def _on_request(self, message: RecordingRequest) -> None:
        if not self._lease_id or message.lease_id != self._lease_id:
            self._executor.detail = 'recording request rejected: lease mismatch'
            self._publish()
            return
        try:
            if message.operation == RecordingRequest.OP_START:
                self._executor.start(
                    int(message.request_id), message.name, message.profile
                )
            elif message.operation == RecordingRequest.OP_STOP:
                self._executor.stop(int(message.request_id))
            elif message.operation == RecordingRequest.OP_DELETE:
                self._executor.delete(int(message.request_id), message.name)
            else:
                raise ValueError('unknown recording operation')
        except ValueError as error:
            self._executor.detail = f'recording request rejected: {error}'
        self._publish()

    def _tick(self) -> None:
        self._executor.poll()
        self._publish()

    def _publish(self) -> None:
        owner = self._executor
        message = RecordingState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = owner.state
        message.accepted_request_id = owner.accepted_request_id
        message.name = owner.name
        message.profile = owner.profile
        message.start_time.sec = owner.start_time_ns // 1_000_000_000
        message.start_time.nanosec = owner.start_time_ns % 1_000_000_000
        message.elapsed_sec = owner.elapsed_sec()
        message.size_bytes = owner.size_bytes()
        message.output_path = str(owner.output_path or '')
        message.detail = owner.detail
        message.recorder_pid = owner.pid
        message.process_healthy = owner.process_healthy()
        for info in owner.catalog():
            entry = RecordingEntry()
            entry.name = info.name
            entry.profile = info.profile
            entry.start_time.sec = info.start_time_ns // 1_000_000_000
            entry.start_time.nanosec = info.start_time_ns % 1_000_000_000
            entry.duration_sec = info.duration_sec
            entry.size_bytes = info.size_bytes
            entry.output_path = info.output_path
            message.recordings.append(entry)
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the recording executor until ROS shutdown."""
    rclpy.init(args=args)
    node = RecordingExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
