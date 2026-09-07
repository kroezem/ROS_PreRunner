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
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import ModeRequest, ModeState, PaddockControlLease
from runner_paddock.mode_runtime import (
    Lifecycle,
    ModeRuntime,
    OP_NEW_MAP,
    SystemdManager,
)
from runner_paddock.state_machine import Mode
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


MODE_REQUEST_TOPIC = '/paddock/mode_request'
MODE_STATE_TOPIC = '/paddock/mode_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'
SCAN_SLAM_TOPIC = '/scan_slam'
SCAN_TOPIC = '/scan'
MAP_TOPIC = '/map'

# Freshness bounds tied to observed producer rates, not the control loop.
SCAN_SLAM_MAX_AGE_SEC = 1.5
SCAN_MAX_AGE_SEC = 1.5
# slam_toolbox map_update_interval is 5 s; allow generous margin for rastering.
MAP_MAX_AGE_SEC = 15.0


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
        self._scan_slam_at: float | None = None
        self._scan_at: float | None = None
        self._map_at: float | None = None
        self._session_started_at: float | None = None
        self._runtime = ModeRuntime(
            SystemdManager(),
            self._graph_nodes,
            self._publish,
            ownership_ready=self._ownership_ready,
            capability_ready=self._capability_ready,
        )
        # Evidence callbacks run in their own thread so a blocking transition
        # wait cannot starve the readiness inputs it is waiting on.
        evidence_group = MutuallyExclusiveCallbackGroup()
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            LaserScan, SCAN_SLAM_TOPIC, self._on_scan_slam, sensor_qos,
            callback_group=evidence_group,
        )
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self._on_scan, sensor_qos,
            callback_group=evidence_group,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map, map_qos,
            callback_group=evidence_group,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True: TF runs its own executor, independent of transitions.
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=True
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
        if state.mode == Mode.MAPPING:
            self._session_started_at = time.monotonic()
        self.get_logger().info(
            f'reconciled Paddock mode as {state.mode.name}/{state.lifecycle.name}'
        )
        self.create_timer(0.5, self._on_refresh_timer)

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

    def _on_scan_slam(self, _message: LaserScan) -> None:
        self._scan_slam_at = time.monotonic()

    def _on_scan(self, _message: LaserScan) -> None:
        self._scan_at = time.monotonic()

    def _on_map(self, _message: OccupancyGrid) -> None:
        self._map_at = time.monotonic()

    def _publisher_owners(self, topic: str) -> set[str]:
        owners = set()
        for endpoint in self.get_publishers_info_by_topic(topic):
            name = endpoint.node_name
            # Ignore endpoints whose node identity has not yet propagated:
            # transient DDS discovery residue, not a real owner (v1.3 s12).
            if not name or name.startswith('_NODE_NAME_UNKNOWN_'):
                continue
            namespace = endpoint.node_namespace.rstrip('/')
            owners.add(
                f'{namespace}/{name}' if namespace else f'/{name}'
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

    def _tf_ok(self, target: str, source: str) -> bool:
        try:
            self._tf_buffer.lookup_transform(
                target, source, rclpy.time.Time()
            )
            return True
        except TransformException:
            return False

    def _capability_ready(self, mode: Mode) -> tuple[bool, str]:
        """Capability-specific minimum runtime evidence with inhibit reasons."""
        now = time.monotonic()

        def fresh(at: float | None, bound: float) -> bool:
            return at is not None and now - at <= bound

        if not self._tf_ok('odom', 'base_link'):
            return False, 'odom->base_link TF unavailable'
        if not self._tf_ok('map', 'odom'):
            return False, 'map->odom TF unavailable (SLAM not localized)'

        if mode == Mode.MAPPING:
            if not fresh(self._scan_slam_at, SCAN_SLAM_MAX_AGE_SEC):
                return False, f'{SCAN_SLAM_TOPIC} is stale or missing'
            if not fresh(self._map_at, MAP_MAX_AGE_SEC):
                return False, f'{MAP_TOPIC} has no recent update'
            if (
                self._session_started_at is not None
                and (
                    self._map_at is None
                    or self._map_at < self._session_started_at
                )
            ):
                return False, 'no current-session map evidence yet'
            return True, ''

        # AUTONOMY
        if not fresh(self._scan_at, SCAN_MAX_AGE_SEC):
            return False, f'{SCAN_TOPIC} is stale or missing'
        if self._map_at is None:
            return False, f'{MAP_TOPIC} not published by map_server'
        return True, ''

    def _on_refresh_timer(self) -> None:
        if not self._lock.acquire(blocking=False):
            return
        try:
            state = self._runtime.refresh()
            # Heartbeat: re-publish the current runtime state every tick so
            # /paddock/mode_state carries the same liveness guarantee as the
            # other Paddock status topics (e.g. /paddock/map_state). The
            # runtime itself only publishes on a state change, so a steady
            # IDLE runtime would otherwise let mode_state age without bound.
            self._publish(state)
        finally:
            self._lock.release()

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
        operation = int(message.operation)
        if operation == OP_NEW_MAP and requested != Mode.MAPPING:
            self.get_logger().warning('NEW MAP is only valid with MAPPING')
            return
        with self._lock:
            before = self._runtime.state.mapping_session_id
            result = self._runtime.transition(
                requested,
                message.request_id,
                autonomy_map=message.autonomy_map,
                operation=operation,
            )
            if (
                result.mode == Mode.MAPPING
                and result.mapping_session_id != before
            ):
                self._session_started_at = time.monotonic()
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
        message.runtime_epoch = state.runtime_epoch
        message.mapping_session_id = state.mapping_session_id
        message.ready = state.ready
        message.readiness_reason = state.readiness_reason
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the persistent mode supervisor."""
    rclpy.init(args=args)
    node = None
    try:
        node = ModeSupervisorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
