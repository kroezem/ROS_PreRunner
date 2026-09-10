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

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from runner_interfaces.msg import AdapterState
from runner_interfaces.msg import CommandAuthorityState
from runner_interfaces.msg import ConfigRequest
from runner_interfaces.msg import ConfigState
from runner_interfaces.msg import EncoderState
from runner_interfaces.msg import LocalControlState
from runner_interfaces.msg import MapRequest
from runner_interfaces.msg import MapState
from runner_interfaces.msg import ModeRequest
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import NavigationState
from runner_interfaces.msg import PaddockControlEvent
from runner_interfaces.msg import PaddockControlLease
from runner_interfaces.msg import RecordingRequest, RecordingState
from runner_interfaces.msg import StopState
from runner_paddock.gateway import (
    ClearCostmapsIntent,
    ConfigRequestIntent,
    ControlEventIntent,
    GatewayResult,
    InitialPoseIntent,
    MapRequestIntent,
    ModeRequestIntent,
    OperatorGateway,
    RecordingRequestIntent,
)
from runner_paddock.grid_geometry import compose, PlanarPose
from runner_paddock.state_cache import StateCache
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


MAP_TOPIC = '/map'
GLOBAL_COSTMAP_TOPIC = '/global_costmap/costmap'
LOCAL_COSTMAP_TOPIC = '/local_costmap/costmap'
GLOBAL_CLEAR_SERVICE = '/global_costmap/clear_entirely_global_costmap'
LOCAL_CLEAR_SERVICE = '/local_costmap/clear_entirely_local_costmap'
PLAN_TOPIC = '/plan'
MODE_STATE_TOPIC = '/paddock/mode_state'
MAP_STATE_TOPIC = '/paddock/map_state'
NAVIGATION_STATE_TOPIC = '/paddock/navigation_state'
AUTHORITY_STATE_TOPIC = '/paddock/command_authority_state'
CONFIG_REQUEST_TOPIC = '/paddock/config_request'
CONFIG_STATE_TOPIC = '/paddock/config_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'
STOP_STATE_TOPIC = '/paddock/stop_state'
LOCAL_CONTROL_TOPIC = '/teleop/control_state'
ADAPTER_STATE_TOPIC = '/drive_adapter/state_typed'
RECORDING_STATE_TOPIC = '/paddock/recording_state'
RECORDING_REQUEST_TOPIC = '/paddock/recording_request'
CONTROL_EVENT_TOPIC = '/paddock/control_event'
MODE_REQUEST_TOPIC = '/paddock/mode_request'
MAP_REQUEST_TOPIC = '/paddock/map_request'
INITIAL_POSE_TOPIC = '/initialpose'
LOCALIZER_POSE_TOPIC = '/pose'
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'
INITIAL_POSE_CONFIRM_TIMEOUT_SEC = 5.0
# Match slam_toolbox's RViz SetInitialPose defaults. slam_toolbox 2.8.5 uses
# x/y/yaw as a scan-matching seed and ignores this input covariance, but a
# complete, realistic planar covariance remains part of Runner's interface.
INITIAL_POSE_X_VARIANCE = 0.25
INITIAL_POSE_Y_VARIANCE = 0.25
INITIAL_POSE_YAW_VARIANCE = 0.06853891909122467


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


def _yaw(orientation: Any) -> float:
    """Return planar yaw from a finite ROS quaternion."""
    _finite(orientation.x, orientation.y, orientation.z, orientation.w)
    return math.atan2(
        2.0 * (orientation.w * orientation.z
               + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y
                     + orientation.z * orientation.z),
    )


def _planar_pose(pose: Any) -> PlanarPose:
    return PlanarPose(
        float(pose.position.x), float(pose.position.y), _yaw(pose.orientation)
    )


def _grid(message: OccupancyGrid, *, origin: dict | None = None) -> dict:
    """Validate and serialize ROS OccupancyGrid map semantics."""
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
    return {
        'stamp': _stamp(message.header.stamp),
        'frame_id': message.header.frame_id,
        'map_load_time': _stamp(info.map_load_time),
        'resolution': info.resolution,
        'width': width,
        'height': height,
        'origin': origin if origin is not None else _pose(info.origin),
        'data': data,
    }


class RosStateNode(Node):
    """Read established state topics and TF; write validated operator intent."""

    def __init__(self, cache: StateCache, *, context=None) -> None:
        super().__init__('runner_paddock_web_state', context=context)
        self._cache = cache
        self._gateway = OperatorGateway()
        self._gateway_lock = threading.Lock()
        self._initial_pose_lock = threading.Lock()
        self._pending_initial_pose = None
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
        self.create_subscription(
            OccupancyGrid,
            GLOBAL_COSTMAP_TOPIC,
            self._on_global_costmap,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            LOCAL_COSTMAP_TOPIC,
            self._on_local_costmap,
            map_qos,
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
            ConfigState, CONFIG_STATE_TOPIC, self._on_config_state, map_qos
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
        self.create_subscription(
            AdapterState, ADAPTER_STATE_TOPIC, self._on_adapter_state,
            latest_qos,
        )
        self.create_subscription(
            RecordingState,
            RECORDING_STATE_TOPIC,
            self._on_recording_state,
            map_qos,
        )
        self.create_subscription(
            EncoderState, '/wheel/encoder_state', self._on_encoder_state,
            latest_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            LOCALIZER_POSE_TOPIC,
            self._on_localizer_pose,
            10,
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
        self._recording_request_pub = self.create_publisher(
            RecordingRequest, RECORDING_REQUEST_TOPIC, 10
        )
        self._config_request_pub = self.create_publisher(
            ConfigRequest, CONFIG_REQUEST_TOPIC, 10
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 1
        )
        self._global_clear_client = self.create_client(
            ClearEntireCostmap, GLOBAL_CLEAR_SERVICE
        )
        self._local_clear_client = self.create_client(
            ClearEntireCostmap, LOCAL_CLEAR_SERVICE
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self.create_timer(0.1, self._update_pose)
        self._publish_gateway_state()

    def _on_map(self, message: OccupancyGrid) -> None:
        try:
            self._cache.update('map', _grid(message))
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'Rejected invalid {MAP_TOPIC}: {error}')

    def _on_global_costmap(self, message: OccupancyGrid) -> None:
        """Publish only an authoritative map-frame global costmap."""
        try:
            if message.header.frame_id != MAP_FRAME:
                raise ValueError(
                    f'expected {MAP_FRAME!r} frame, got '
                    f'{message.header.frame_id!r}'
                )
            self._cache.update('global_costmap', _grid(message))
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {GLOBAL_COSTMAP_TOPIC}: {error}'
            )

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        """Place the odom-frame rolling costmap in the authoritative map frame."""
        try:
            source_frame = message.header.frame_id
            if not source_frame:
                raise ValueError('local costmap frame is empty')
            transform = self._tf_buffer.lookup_transform(
                MAP_FRAME, source_frame, rclpy.time.Time()
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            map_from_source = PlanarPose(
                float(translation.x), float(translation.y), _yaw(rotation)
            )
            map_origin = compose(map_from_source, _planar_pose(message.info.origin))
            half_yaw = 0.5 * map_origin.yaw
            origin = {
                'position': {'x': map_origin.x, 'y': map_origin.y, 'z': 0.0},
                'orientation': {
                    'x': 0.0, 'y': 0.0,
                    'z': math.sin(half_yaw), 'w': math.cos(half_yaw),
                },
            }
            value = _grid(message, origin=origin)
            value['source_frame_id'] = source_frame
            value['frame_id'] = MAP_FRAME
            value['transform_stamp'] = _stamp(transform.header.stamp)
            self._cache.update('local_costmap', value)
        except (TransformException, TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected unaligned {LOCAL_COSTMAP_TOPIC}: {error}',
                throttle_duration_sec=5.0,
            )

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
                'delete_state': int(message.delete_state),
                'delete_request_id': int(message.delete_request_id),
                'delete_name': message.delete_name,
                'delete_detail': message.delete_detail,
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
            if message.autonomy_goal_selected:
                _finite(message.goal_x, message.goal_y, message.goal_yaw)
            self._cache.update('command_authority', {
                'stamp': _stamp(message.stamp),
                'authority': int(message.authority),
                'client_id': message.client_id,
                'lease_id': message.lease_id,
                'dualsense_active': message.dualsense_active,
                'run_held': message.run_held,
                'autonomy_permitted': message.autonomy_permitted,
                'autonomy_goal_selected': message.autonomy_goal_selected,
                'goal_frame': message.goal_frame,
                'goal_map': message.goal_map,
                'goal_x': message.goal_x,
                'goal_y': message.goal_y,
                'goal_yaw': message.goal_yaw,
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

    def _on_config_state(self, message: ConfigState) -> None:
        try:
            _finite(message.requested_value, message.applied_value)
            self._cache.update('config', {
                'stamp': _stamp(message.stamp),
                'request_id': int(message.request_id),
                'revision': int(message.revision),
                'field': message.field,
                'requested_value': float(message.requested_value),
                'applied_value': float(message.applied_value),
                'accepted': bool(message.accepted),
                'reason': message.reason,
            })
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected invalid {CONFIG_STATE_TOPIC}: {error}'
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

    def _on_adapter_state(self, message: AdapterState) -> None:
        try:
            _finite(
                message.commanded_speed, message.effective_speed,
                message.measured_speed, message.commanded_yaw_rate,
                message.measured_yaw_rate,
            )
        except ValueError:
            return
        self._cache.update('adapter_state', {
            'stamp': _stamp(message.stamp),
            'commanded_speed': float(message.commanded_speed),
            'effective_speed': float(message.effective_speed),
            'measured_speed': float(message.measured_speed),
            'commanded_yaw_rate': float(message.commanded_yaw_rate),
            'measured_yaw_rate': float(message.measured_yaw_rate),
            'mode': message.mode,
        })

    def _on_encoder_state(self, message: EncoderState) -> None:
        """Cache the encoder owner's explicit stationary determination."""
        self._cache.update('encoder_state', {
            'stamp': _stamp(message.stamp),
            'stationary': bool(message.stationary),
        })

    def _on_localizer_pose(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        """Confirm a seed only from subsequent slam_toolbox pose truth."""
        try:
            if message.header.frame_id != MAP_FRAME:
                raise ValueError(
                    f'expected {MAP_FRAME!r} frame, got '
                    f'{message.header.frame_id!r}'
                )
            observed = _pose(message.pose.pose)
            covariance = [float(value) for value in message.pose.covariance]
            _finite(*covariance)
            stamp = _stamp(message.header.stamp)
            stamp_ns = stamp['sec'] * 1_000_000_000 + stamp['nanosec']
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected invalid {LOCALIZER_POSE_TOPIC}: {error}'
            )
            return

        with self._initial_pose_lock:
            pending = self._pending_initial_pose
            if pending is None or stamp_ns < pending['request_stamp_ns']:
                return
            self._pending_initial_pose = None
        snapshot = self._cache.state_snapshot()
        mode = snapshot.get('mode') or {}
        map_state = snapshot.get('map_state') or {}
        if (
            int(mode.get('runtime_epoch') or 0)
            != pending['status']['runtime_epoch']
            or (map_state.get('selected_map_applied') or '')
            != pending['status']['map']
            or (mode.get('active_autonomy_map') or '')
            != pending['status']['map']
        ):
            self._cache.update('initial_pose', {
                **pending['status'],
                'state': 'rejected',
                'detail': 'runtime or active map changed before application',
                'observed': None,
            })
            return
        self._cache.update('initial_pose', {
            **pending['status'],
            'state': 'applied',
            'detail': 'slam_toolbox published a subsequent localized pose',
            'observed': {
                'stamp': stamp,
                'frame_id': message.header.frame_id,
                'pose': observed,
                'covariance': covariance,
            },
        })

    def _on_recording_state(self, message: RecordingState) -> None:
        try:
            _finite(message.elapsed_sec)
            recordings = []
            for entry in message.recordings:
                _finite(entry.duration_sec)
                recordings.append({
                    'name': entry.name,
                    'profile': entry.profile,
                    'start_time': _stamp(entry.start_time),
                    'duration_sec': float(entry.duration_sec),
                    'size_bytes': int(entry.size_bytes),
                    'output_path': entry.output_path,
                })
            self._cache.update('recording_state', {
                'stamp': _stamp(message.stamp),
                'state': int(message.state),
                'accepted_request_id': int(message.accepted_request_id),
                'name': message.name,
                'profile': message.profile,
                'start_time': _stamp(message.start_time),
                'elapsed_sec': float(message.elapsed_sec),
                'size_bytes': int(message.size_bytes),
                'output_path': message.output_path,
                'detail': message.detail,
                'recorder_pid': int(message.recorder_pid),
                'process_healthy': bool(message.process_healthy),
                'recordings': recordings,
            })
        except ValueError as error:
            self.get_logger().warning(
                f'Rejected invalid {RECORDING_STATE_TOPIC}: {error}'
            )

    # -- operator intent ---------------------------------------------------

    def submit(self, conn_id: str, action: dict) -> dict:
        """Validate one browser action, publish its intents, report outcome."""
        with self._gateway_lock:
            result = self._gateway.handle(conn_id, action)
            if result.accepted and self._autonomy_without_map(action):
                # Operator precondition: never send an AUTONOMY runtime request
                # with no map selected -- that path stops the runtime and
                # faults on an obscure basename error. Reject it here with a
                # clear reason and publish nothing.
                result = GatewayResult(
                    False,
                    'select a completed map before AUTONOMY',
                    (),
                    result.role,
                )
            for intent in result.intents:
                if isinstance(intent, ClearCostmapsIntent):
                    result = self._clear_costmaps(result)
                    break
                if isinstance(intent, InitialPoseIntent):
                    result = self._set_initial_pose(intent, result.role)
                    break
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

    def _set_initial_pose(
        self, intent: InitialPoseIntent, role: str
    ) -> GatewayResult:
        """Gate and publish a localization seed to slam_toolbox alone."""
        snapshot = self._cache.state_snapshot()
        reason = self._initial_pose_rejection(snapshot)
        if not reason and self._initial_pose_pub.get_subscription_count() < 1:
            reason = 'slam_toolbox /initialpose subscriber unavailable'
        status = self._initial_pose_status(intent, snapshot)
        if reason:
            with self._initial_pose_lock:
                self._pending_initial_pose = None
            self._cache.update('initial_pose', {
                **status, 'state': 'rejected', 'detail': reason,
                'observed': None,
            })
            return GatewayResult(False, reason, (), role)

        now = self.get_clock().now()
        message = PoseWithCovarianceStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = MAP_FRAME
        message.pose.pose.position.x = intent.x
        message.pose.pose.position.y = intent.y
        message.pose.pose.orientation.z = math.sin(intent.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(intent.yaw / 2.0)
        message.pose.covariance[0] = INITIAL_POSE_X_VARIANCE
        message.pose.covariance[7] = INITIAL_POSE_Y_VARIANCE
        message.pose.covariance[35] = INITIAL_POSE_YAW_VARIANCE
        accepted = {
            **status,
            'state': 'accepted',
            'detail': 'published to slam_toolbox; awaiting localized pose',
            'observed': None,
        }
        with self._initial_pose_lock:
            self._pending_initial_pose = {
                'request_stamp_ns': now.nanoseconds,
                'deadline': time.monotonic()
                + INITIAL_POSE_CONFIRM_TIMEOUT_SEC,
                'status': status,
            }
            # Keep a very fast localizer response from being overwritten by
            # the preceding accepted/awaiting state.
            self._initial_pose_pub.publish(message)
            self._cache.update('initial_pose', accepted)
        return GatewayResult(
            True, 'initial pose accepted; awaiting slam_toolbox pose', (), role
        )

    @staticmethod
    def _initial_pose_rejection(snapshot: dict) -> str:
        """Return the first stopped-localization safety precondition failure."""
        health = snapshot.get('health') or {}
        sources = health.get('sources') or {}
        mode = snapshot.get('mode') or {}
        map_state = snapshot.get('map_state') or {}
        stop = snapshot.get('stop_state') or {}
        encoder = snapshot.get('encoder_state') or {}

        if not (sources.get('mode') or {}).get('fresh'):
            return 'runtime state unavailable or stale'
        if mode.get('mode') != ModeState.MODE_AUTONOMY \
                or mode.get('status') != ModeState.STATUS_STABLE:
            return 'initial pose requires stable AUTONOMY localization'
        if not mode.get('ready'):
            return 'localization is not ready'
        selected = map_state.get('selected_map_applied') or ''
        active = mode.get('active_autonomy_map') or ''
        complete = any(
            entry.get('name') == selected and entry.get('complete')
            for entry in map_state.get('catalog', [])
        )
        if not (sources.get('map_state') or {}).get('fresh') \
                or not selected or not complete:
            return 'a valid selected map is required'
        if active != selected:
            return 'selected map is not the active localization map'
        if not (sources.get('stop_state') or {}).get('fresh'):
            return 'STOP state unavailable or stale'
        if not (stop.get('stopped') and stop.get('locked')
                and stop.get('healthy') and stop.get('applied')):
            return 'initial pose requires healthy applied STOP'
        if not (sources.get('encoder_state') or {}).get('fresh'):
            return 'encoder stationary state unavailable or stale'
        if not encoder.get('stationary'):
            return 'robot is not stationary'
        return ''

    @staticmethod
    def _initial_pose_status(
        intent: InitialPoseIntent, snapshot: dict
    ) -> dict:
        mode = snapshot.get('mode') or {}
        map_state = snapshot.get('map_state') or {}
        return {
            'map': map_state.get('selected_map_applied') or '',
            'runtime_epoch': int(mode.get('runtime_epoch') or 0),
            'requested': {
                'frame_id': intent.frame,
                'x': intent.x,
                'y': intent.y,
                'yaw': intent.yaw,
                'covariance': {
                    'x': INITIAL_POSE_X_VARIANCE,
                    'y': INITIAL_POSE_Y_VARIANCE,
                    'yaw': INITIAL_POSE_YAW_VARIANCE,
                },
            },
        }

    def _clear_costmaps(self, result: GatewayResult) -> GatewayResult:
        """Call both authoritative Nav2 clear services and await their replies."""
        clients = (
            ('global', self._global_clear_client),
            ('local', self._local_clear_client),
        )
        unavailable = [
            name for name, client in clients if not client.service_is_ready()
        ]
        if unavailable:
            return GatewayResult(
                False,
                f"Nav2 {' and '.join(unavailable)} costmap clear service "
                'unavailable',
                (),
                result.role,
            )

        futures = [
            (name, client.call_async(ClearEntireCostmap.Request()))
            for name, client in clients
        ]
        deadline = time.monotonic() + 2.0
        failed = []
        for name, future in futures:
            remaining = max(0.0, deadline - time.monotonic())
            done = threading.Event()
            future.add_done_callback(lambda _future, event=done: event.set())
            if not done.wait(remaining):
                failed.append(f'{name} timed out')
                continue
            try:
                future.result()
            except Exception as error:  # rclpy service transport/backend error
                failed.append(f'{name} failed: {error}')
        if failed:
            return GatewayResult(
                False,
                'Nav2 costmap clear failed (' + '; '.join(failed) + ')',
                (),
                result.role,
            )
        return GatewayResult(
            True,
            'Nav2 global and local costmaps cleared',
            (),
            result.role,
        )

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
            message.manual_speed_mps = float(intent.manual_speed_mps)
            message.manual_steering = float(intent.manual_steering)
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
        elif isinstance(intent, RecordingRequestIntent):
            message = RecordingRequest()
            message.stamp = stamp
            message.request_id = time.time_ns()
            message.lease_id = intent.lease_id
            message.operation = int(intent.operation)
            message.name = intent.name
            message.profile = intent.profile
            self._recording_request_pub.publish(message)
        elif isinstance(intent, ConfigRequestIntent):
            message = ConfigRequest()
            message.stamp = stamp
            message.request_id = time.time_ns()
            message.lease_id = intent.lease_id
            message.expected_revision = int(intent.expected_revision)
            message.field = intent.field
            message.value = float(intent.value)
            self._config_request_pub.publish(message)

    def _autonomy_without_map(self, action: dict) -> bool:
        """Return True for a 'select AUTONOMY runtime' action with no map set."""
        if not isinstance(action, dict) or action.get('action') != 'select_mode':
            return False
        if str(action.get('mode', '')).strip().lower() != 'autonomy':
            return False
        return not self._selected_map()

    def _selected_map(self) -> str:
        snapshot = self._cache.state_snapshot()
        map_state = snapshot.get('map_state') or {}
        # Only executor-applied state is authoritative. A rejected request is
        # retained separately for diagnosis and must never become a runtime map.
        return map_state.get('selected_map_applied') or ''

    def _mapping_session_id(self) -> str:
        snapshot = self._cache.state_snapshot()
        mode = snapshot.get('mode') or {}
        return mode.get('mapping_session_id', '') or ''

    def _publish_gateway_state(self) -> None:
        self._cache.update('gateway', self._gateway.public_state())

    def _update_pose(self) -> None:
        self._expire_initial_pose_confirmation()
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

    def _expire_initial_pose_confirmation(self) -> None:
        """Make a missing post-seed localizer result explicit to operators."""
        with self._initial_pose_lock:
            pending = self._pending_initial_pose
            if pending is None or time.monotonic() < pending['deadline']:
                return
            self._pending_initial_pose = None
        self._cache.update('initial_pose', {
            **pending['status'],
            'state': 'rejected',
            'detail': 'application unconfirmed: no fresh slam_toolbox pose',
            'observed': None,
        })
