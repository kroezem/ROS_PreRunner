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

"""Persistent ROS adapter for the systemd-backed Paddock mode runtime."""

from collections import Counter
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import ModeRequest, ModeState, PaddockControlLease

from runner_paddock.mode_runtime import Lifecycle, ModeRuntime, SystemdManager
from runner_paddock.state_machine import Mode


MODE_REQUEST_TOPIC = '/paddock/mode_request'
MODE_STATE_TOPIC = '/paddock/mode_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'


class ModeSupervisorNode(Node):
    """Serialize typed mode requests and publish actual systemd state."""

    def __init__(self) -> None:
        super().__init__('runner_mode_supervisor')
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            ModeState, MODE_STATE_TOPIC, state_qos
        )
        self._lease_id = ''
        self._lease_active = False
        self._lock = threading.Lock()
        self._runtime = ModeRuntime(
            SystemdManager(),
            self._graph_nodes,
            self._publish,
            ownership_ready=self._ownership_ready,
        )
        self.create_subscription(
            PaddockControlLease,
            LEASE_STATE_TOPIC,
            self._on_lease,
            10,
        )
        self.create_subscription(
            ModeRequest, MODE_REQUEST_TOPIC, self._on_request, 10
        )
        state = self._runtime.reconcile()
        self.get_logger().info(
            f'reconciled Paddock mode as {state.mode.name}/{state.lifecycle.name}'
        )

    def _graph_nodes(self) -> Counter:
        names = (
            f'{namespace.rstrip("/")}/{name}'
            if namespace != '/' else f'/{name}'
            for name, namespace in self.get_node_names_and_namespaces()
        )
        return Counter(names)

    def _on_lease(self, message: PaddockControlLease) -> None:
        self._lease_active = bool(message.active)
        self._lease_id = message.lease_id if message.active else ''

    def _publisher_owners(self, topic: str) -> set[str]:
        owners = set()
        for endpoint in self.get_publishers_info_by_topic(topic):
            namespace = endpoint.node_namespace.rstrip('/')
            owners.add(
                f'{namespace}/{endpoint.node_name}'
                if namespace else f'/{endpoint.node_name}'
            )
        return owners

    def _ownership_ready(self, mode: Mode) -> bool:
        expected_map_owner = (
            '/slam_toolbox' if mode == Mode.MAPPING else '/map_server'
        )
        expected = {
            '/map': {expected_map_owner},
            '/scan': {'/LD19'},
            '/odometry/filtered': {'/ekf_node'},
            '/cmd_vel': {'/twist_mux'},
            '/tf': {'/ekf_node', '/slam_toolbox'},
            '/tf_static': {
                '/base_link_to_base_laser',
                '/base_link_to_imu_link',
            },
        }
        if mode == Mode.AUTONOMY:
            expected['/slam_map'] = {'/slam_toolbox'}
        return all(
            self._publisher_owners(topic) == owners
            for topic, owners in expected.items()
        )

    def _on_request(self, message: ModeRequest) -> None:
        if not self._lease_active or message.lease_id != self._lease_id:
            self.get_logger().warning('rejected mode request from non-owner lease')
            return
        try:
            requested = Mode(message.requested_mode)
        except ValueError:
            self.get_logger().warning(
                f'rejected unknown Paddock mode {message.requested_mode}'
            )
            return
        with self._lock:
            result = self._runtime.transition(
                requested,
                message.request_id,
                autonomy_map=message.autonomy_map,
            )
        if result.lifecycle == Lifecycle.FAULT:
            self.get_logger().error(result.detail)

    def _publish(self, state) -> None:
        message = ModeState()
        message.stamp = self.get_clock().now().to_msg()
        message.mode = int(state.mode)
        message.status = int(state.lifecycle)
        message.accepted_request_id = state.accepted_request_id
        message.active_autonomy_map = state.active_autonomy_map
        message.detail = state.detail
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the persistent mode supervisor."""
    rclpy.init(args=args)
    node = None
    try:
        node = ModeSupervisorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
