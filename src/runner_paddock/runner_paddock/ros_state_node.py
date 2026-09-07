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

"""Paddock gateway ROS node: reads state topics, writes operator intent."""

import math
import threading
import time
from typing import Any

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from runner_interfaces.msg import CommandAuthorityState
from runner_interfaces.msg import LocalControlState
from runner_interfaces.msg import MapRequest
from runner_interfaces.msg import MapState
from runner_interfaces.msg import ModeRequest
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import NavigationState
from runner_interfaces.msg import PaddockControlEvent
from runner_interfaces.msg import PaddockControlLease
from runner_interfaces.msg import StopState
from runner_paddock.gateway import (
    ControlEventIntent,
    MapRequestIntent,
    ModeRequestIntent,
    OperatorGateway,
)
from runner_paddock.state_cache import StateCache
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


MAP_TOPIC = '/map'
PLAN_TOPIC = '/plan'
MODE_STATE_TOPIC = '/paddock/mode_state'
MAP_STATE_TOPIC = '/paddock/map_state'
NAVIGATION_STATE_TOPIC = '/paddock/navigation_state'
AUTHORITY_STATE_TOPIC = '/paddock/command_authority_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'
STOP_STATE_TOPIC = '/paddock/stop_state'
LOCAL_CONTROL_TOPIC = '/teleop/control_state'
CONTROL_EVENT_TOPIC = '/paddock/control_event'
MODE_REQUEST_TOPIC = '/paddock/mode_request'
MAP_REQUEST_TOPIC = '/paddock/map_request'
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'


def _stamp(stamp: Any) -> dict[str, int]:
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    if nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError('ROS timestamp nanosec is outside [0, 1e9)')
    return {'sec': sec, 'nanosec': nanosec}


def _finite(*values: float) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError('ROS message contains a non-finite number')


def _pose(pose: Any) -> dict[str, Any]:
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    _finite(*values)
    return {
        'position': {
            'x': values[0], 'y': values[1], 'z': values[2],
        },
        'orientation': {
            'x': values[3], 'y': values[4],
            'z': values[5], 'w': values[6],
        },
    }


class RosStateNode(Node):
    """Read established state topics and TF; write validated operator intent."""

    def __init__(self, cache: StateCache, *, context=None) -> None:
        super().__init__('runner_paddock_web_state', context=context)
        self._cache = cache
        self._gateway = OperatorGateway()
        self._gateway_lock = threading.Lock()
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        latest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map, map_qos
        )
        self.create_subscription(Path, PLAN_TOPIC, self._on_plan, latest_qos)
        self.create_subscription(
            ModeState, MODE_STATE_TOPIC, self._on_mode, map_qos
        )
        self.create_subscription(
            MapState, MAP_STATE_TOPIC, self._on_map_state, map_qos
        )
        self.create_subscription(
            NavigationState,
            NAVIGATION_STATE_TOPIC,
            self._on_navigation_state,
            map_qos,
        )
        self.create_subscription(
            CommandAuthorityState,
            AUTHORITY_STATE_TOPIC,
            self._on_authority,
            latest_qos,
        )
        self.create_subscription(
            PaddockControlLease, LEASE_STATE_TOPIC, self._on_lease, latest_qos
        )
        self.create_subscription(
            StopState,
            STOP_STATE_TOPIC,
            self._on_stop_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            LocalControlState,
            LOCAL_CONTROL_TOPIC,
            self._on_local_control,
            latest_qos,
        )

        # Operator-intent writers. This node is the sole browser-side writer of
        # each of these topics.
        self._control_event_pub = self.create_publisher(
            PaddockControlEvent, CONTROL_EVENT_TOPIC, 10
        )
        self._mode_request_pub = self.create_publisher(
            ModeRequest, MODE_REQUEST_TOPIC, 10
        )
        self._map_request_pub = self.create_publisher(
            MapRequest, MAP_REQUEST_TOPIC, 10
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self.create_timer(0.1, self._update_pose)
        self._publish_gateway_state()

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            info = message.info
            _finite(info.resolution)
            if info.resolution <= 0.0:
                raise ValueError('OccupancyGrid resolution must be positive')
            width = int(info.width)
            height = int(info.height)
            data = [int(value) for value in message.data]
            if width * height != len(data):
                raise ValueError('OccupancyGrid dimensions do not match data')
            if any(value < -1 or value > 100 for value in data):
                raise ValueError('OccupancyGrid data is outside [-1, 100]')
            self._cache.update('map', {
                'stamp': _stamp(message.header.stamp),
                'frame_id': message.header.frame_id,
                'map_load_time': _stamp(info.map_load_time),
                'resolution': info.resolution,
                'width': width,
                'height': height,
                'origin': _pose(info.origin),
                'data': data,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'Rejected invalid {MAP_TOPIC}: {error}')

    def _on_plan(self, message: Path) -> None:
        try:
            poses = []
            for stamped_pose in message.poses:
                poses.append({
                    'stamp': _stamp(stamped_pose.header.stamp),
                    'frame_id': stamped_pose.header.frame_id,
                    'pose': _pose(stamped_pose.pose),
                })
            self._cache.update('plan', {
                'stamp': _stamp(message.header.stamp),
                'frame_id': message.header.frame_id,
                'poses': poses,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'Rejected invalid {PLAN_TOPIC}: {error}')

    def _on_mode(self, message: ModeState) -> None:
        try:
            self._cache.update('mode', {
                'stamp': _stamp(message.stamp),
                'mode': int(message.mode),
                'status': int(message.status),
                'accepted_request_id': int(message.accepted_request_id),
                'active_autonomy_map': message.active_autonomy_map,
                'detail': message.detail,
                'runtime_epoch': int(message.runtime_epoch),
                'mapping_session_id': message.mapping_session_id,
                'ready': bool(message.ready),
                'readiness_reason': message.readiness_reason,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {MODE_STATE_TOPIC}: {error}'
            )

    def _on_map_state(self, message: MapState) -> None:
        try:
            catalog = [
                {
                    'name': entry.name,
                    'revision': entry.revision,
                    'complete': bool(entry.complete),
                    'selected': bool(entry.selected),
                    'session_id': entry.session_id,
                    'reason': entry.reason,
                    'resolution': float(entry.resolution),
                    'width': int(entry.width),
                    'height': int(entry.height),
                }
                for entry in message.catalog
            ]
            _finite(*[row['resolution'] for row in catalog])
            self._cache.update('map_state', {
                'stamp': _stamp(message.stamp),
                'session_id': message.session_id,
                'runtime_epoch': int(message.runtime_epoch),
                'mapping_active': bool(message.mapping_active),
                'session_phase': int(message.session_phase),
                'session_ready': bool(message.session_ready),
                'unsaved': bool(message.unsaved),
                'saved_name': message.saved_name,
                'saved_revision': message.saved_revision,
                'save_state': int(message.save_state),
                'save_request_id': int(message.save_request_id),
                'save_detail': message.save_detail,
                'selected_map_requested': message.selected_map_requested,
                'selected_map_applied': message.selected_map_applied,
                'selected_map_reason': message.selected_map_reason,
                'catalog': catalog,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {MAP_STATE_TOPIC}: {error}'
            )

    def _on_navigation_state(self, message: NavigationState) -> None:
        try:
            self._cache.update('navigation_state', {
                'stamp': _stamp(message.stamp),
                'boot_id': message.boot_id,
                'state': int(message.state),
                'mission_id': message.mission_id,
                'mission_revision': int(message.mission_revision),
                'runtime_epoch': int(message.runtime_epoch),
                'map_id': message.map_id,
                'mission_type': int(message.mission_type),
                'mission_valid': bool(message.mission_valid),
                'action_generation': int(message.action_generation),
                'goal_uuid': message.goal_uuid,
                'nav2_status': int(message.nav2_status),
                'error_code': int(message.error_code),
                'error_meaning': message.error_meaning,
                'detail': message.detail,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {NAVIGATION_STATE_TOPIC}: {error}'
            )

    def _on_authority(self, message: CommandAuthorityState) -> None:
        try:
            _finite(message.lease_age_sec, message.raw_autonomy_age_sec)
            self._cache.update('command_authority', {
                'stamp': _stamp(message.stamp),
                'authority': int(message.authority),
                'client_id': message.client_id,
                'lease_id': message.lease_id,
                'dualsense_active': message.dualsense_active,
                'run_held': message.run_held,
                'autonomy_permitted': message.autonomy_permitted,
                'autonomy_goal_selected': message.autonomy_goal_selected,
                'autonomy_action_active': message.autonomy_action_active,
                'brake_intent': message.brake_intent,
                'lease_fresh': message.lease_fresh,
                'lease_age_sec': message.lease_age_sec,
                'raw_autonomy_fresh': message.raw_autonomy_fresh,
                'raw_autonomy_age_sec': message.raw_autonomy_age_sec,
                'last_control_sequence': int(message.last_control_sequence),
                'reason': message.reason,
                'stop_state_fresh': bool(message.stop_state_fresh),
                'stop_healthy': bool(message.stop_healthy),
                'stop_applied': bool(message.stop_applied),
                'stop_clear': bool(message.stop_clear),
                'stop_reason': message.stop_reason,
            })
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected invalid {AUTHORITY_STATE_TOPIC}: {error}'
            )

    def _on_lease(self, message: PaddockControlLease) -> None:
        self._cache.update('control_lease', {
            'stamp': _stamp(message.stamp),
            'active': bool(message.active),
            'client_id': message.client_id,
            'lease_id': message.lease_id,
            'generation': int(message.generation),
        })

    def _on_stop_state(self, message: StopState) -> None:
        self._cache.update('stop_state', {
            'stamp': _stamp(message.stamp),
            'boot_id': message.boot_id,
            'generation': int(message.generation),
            'stopped': bool(message.stopped),
            'locked': bool(message.locked),
            'healthy': bool(message.healthy),
            'applied': bool(message.applied),
            'clear_pending': bool(message.clear_pending),
            'reason': message.reason,
            'last_request_accepted': bool(message.last_request_accepted),
            'last_request_reason': message.last_request_reason,
        })

    def _on_local_control(self, message: LocalControlState) -> None:
        try:
            _finite(message.sample_age_sec)
        except ValueError:
            return
        self._cache.update('local_control', {
            'stamp': _stamp(message.stamp),
            'process_epoch': message.process_epoch,
            'takeover_epoch': int(message.takeover_epoch),
            'connected': bool(message.connected),
            'active': bool(message.active),
            'neutral': bool(message.neutral),
            'released': bool(message.released),
            'sample_age_sec': float(message.sample_age_sec),
            'mode': message.mode,
        })

    # -- operator intent ---------------------------------------------------

    def submit(self, conn_id: str, action: dict) -> dict:
        """Validate one browser action, publish its intents, report outcome."""
        with self._gateway_lock:
            result = self._gateway.handle(conn_id, action)
            for intent in result.intents:
                self._publish_intent(intent)
            self._publish_gateway_state()
        if not result.accepted:
            self.get_logger().warning(
                f'rejected browser action {action.get("action")!r}: '
                f'{result.reason}'
            )
        return {
            'accepted': result.accepted,
            'reason': result.reason,
            'role': result.role,
        }

    def disconnect(self, conn_id: str) -> None:
        """Release the lease if this browser connection held it."""
        with self._gateway_lock:
            result = self._gateway.on_disconnect(conn_id)
            for intent in result.intents:
                self._publish_intent(intent)
            self._publish_gateway_state()

    def _publish_intent(self, intent) -> None:
        stamp = self.get_clock().now().to_msg()
        if isinstance(intent, ControlEventIntent):
            message = PaddockControlEvent()
            message.stamp = stamp
            message.sequence = int(intent.sequence)
            message.client_id = intent.client_id
            message.lease_id = intent.lease_id
            message.event = int(intent.event)
            message.goal_frame = intent.goal_frame
            message.goal_x = float(intent.goal_x)
            message.goal_y = float(intent.goal_y)
            message.goal_yaw = float(intent.goal_yaw)
            self._control_event_pub.publish(message)
        elif isinstance(intent, ModeRequestIntent):
            message = ModeRequest()
            message.stamp = stamp
            message.request_id = time.time_ns()
            message.lease_id = intent.lease_id
            message.requested_mode = int(intent.requested_mode)
            message.operation = int(intent.operation)
            message.autonomy_map = intent.autonomy_map or self._selected_map()
            self._mode_request_pub.publish(message)
        elif isinstance(intent, MapRequestIntent):
            message = MapRequest()
            message.stamp = stamp
            message.request_id = time.time_ns()
            message.lease_id = intent.lease_id
            message.operation = int(intent.operation)
            message.name = intent.name
            message.session_id = intent.session_id or self._mapping_session_id()
            self._map_request_pub.publish(message)

    def _selected_map(self) -> str:
        snapshot = self._cache.state_snapshot()
        map_state = snapshot.get('map_state') or {}
        return (
            map_state.get('selected_map_applied')
            or map_state.get('selected_map_requested')
            or ''
        )

    def _mapping_session_id(self) -> str:
        snapshot = self._cache.state_snapshot()
        mode = snapshot.get('mode') or {}
        return mode.get('mapping_session_id', '') or ''

    def _publish_gateway_state(self) -> None:
        self._cache.update('gateway', self._gateway.public_state())

    def _update_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time()
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            _finite(
                translation.x, translation.y, translation.z,
                rotation.x, rotation.y, rotation.z, rotation.w,
            )
            self._cache.update('pose', {
                'stamp': _stamp(transform.header.stamp),
                'frame_id': transform.header.frame_id,
                'child_frame_id': transform.child_frame_id,
                'position': {
                    'x': translation.x,
                    'y': translation.y,
                    'z': translation.z,
                },
                'orientation': {
                    'x': rotation.x,
                    'y': rotation.y,
                    'z': rotation.z,
                    'w': rotation.w,
                },
            })
        except (TransformException, ValueError):
            pass
