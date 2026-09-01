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

"""Subscriber-only ROS node which writes serializable values to a cache."""

import math
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
from runner_interfaces.msg import ModeState
from runner_paddock.state_cache import StateCache
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


MAP_TOPIC = '/map'
PLAN_TOPIC = '/plan'
MODE_STATE_TOPIC = '/paddock/mode_state'
AUTHORITY_STATE_TOPIC = '/paddock/command_authority_state'
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
    """Read established state topics and TF without publishing anything."""

    def __init__(self, cache: StateCache, *, context=None) -> None:
        super().__init__('runner_paddock_web_state', context=context)
        self._cache = cache
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
            ModeState, MODE_STATE_TOPIC, self._on_mode, latest_qos
        )
        self.create_subscription(
            CommandAuthorityState,
            AUTHORITY_STATE_TOPIC,
            self._on_authority,
            latest_qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self.create_timer(0.1, self._update_pose)

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
                'accepted_request_id': int(message.accepted_request_id),
                'active_autonomy_map': message.active_autonomy_map,
            })
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {MODE_STATE_TOPIC}: {error}'
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
            })
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected invalid {AUTHORITY_STATE_TOPIC}: {error}'
            )

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
