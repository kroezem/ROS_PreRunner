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
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import SetParametersAtomically
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
from runner_interfaces.msg import SystemTelemetry
from runner_paddock.autonomy_tuning import (
    ADAPTER_OWNER,
    CONTROLLER_OWNER,
    matching_preset,
    PARAMETERS as TUNING_PARAMETERS,
    PRESETS as TUNING_PRESETS,
    validate_values as validate_tuning_values,
    values_for_owner,
)
from runner_paddock.gateway import (
    AutonomyTuningIntent,
    ClearCostmapsIntent,
    ConfigRequestIntent,
    ControlEventIntent,
    GatewayResult,
    InitialPoseIntent,
    MapRequestIntent,
    ModeRequestIntent,
    ObstacleProcessingIntent,
    OperatorGateway,
    RecordingRequestIntent,
)
from runner_paddock.grid_geometry import compose, PlanarPose
from runner_paddock.protocol import ValidatedGridData
from runner_paddock.state_cache import StateCache
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


MAP_TOPIC = '/map'
GLOBAL_COSTMAP_TOPIC = '/global_costmap/costmap'
LOCAL_COSTMAP_TOPIC = '/local_costmap/costmap'
GLOBAL_CLEAR_SERVICE = '/global_costmap/clear_entirely_global_costmap'
LOCAL_CLEAR_SERVICE = '/local_costmap/clear_entirely_local_costmap'
OBSTACLE_PARAMETER = 'obstacle_layer.enabled'
OBSTACLE_REFRESH_SEC = 1.0
OBSTACLE_TARGETS = {
    'global': '/global_costmap/global_costmap',
    'local': '/local_costmap/local_costmap',
}
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
SYSTEM_TELEMETRY_TOPIC = '/system/telemetry'
BATTERY_TOPIC = '/battery'
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'
INITIAL_POSE_CONFIRM_TIMEOUT_SEC = 5.0
INITIAL_POSE_STOP_TIMEOUT_SEC = 5.0
INITIAL_POSE_CLEAR_TIMEOUT_SEC = 2.0
TUNING_REFRESH_SEC = 1.0
TUNING_REQUEST_TIMEOUT_SEC = 2.0
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
    data = ValidatedGridData(message.data)
    if width * height != len(data):
        raise ValueError('OccupancyGrid dimensions do not match data')
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
        self._initial_pose_request_id = 0
        self._map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = None
        self._map_subscription_identity = None
        self._obstacle_lock = threading.Lock()
        self._obstacle_request_id = 0
        self._obstacle_refresh_id = 0
        self._obstacle_operations = {}
        self._obstacle_states = {
            costmap: {
                'requested': None,
                'applied': None,
                'current': None,
                'status': 'unavailable',
                'detail': 'waiting for Nav2 parameter service',
                'request_id': 0,
                'confirmed_at': None,
            }
            for costmap in OBSTACLE_TARGETS
        }
        map_qos = self._map_qos
        latest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
        self.create_subscription(
            SystemTelemetry,
            SYSTEM_TELEMETRY_TOPIC,
            self._on_system_telemetry,
            latest_qos,
        )
        self.create_subscription(
            BatteryState, BATTERY_TOPIC, self._on_battery, latest_qos
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
        self._obstacle_get_clients = {}
        self._obstacle_set_clients = {}
        for costmap, node_name in OBSTACLE_TARGETS.items():
            self._obstacle_get_clients[costmap] = self.create_client(
                GetParameters, f'{node_name}/get_parameters'
            )
            self._obstacle_set_clients[costmap] = self.create_client(
                SetParameters, f'{node_name}/set_parameters'
            )
        self._publish_obstacle_state()

        self._tuning_lock = threading.Lock()
        self._tuning_request_id = 0
        self._tuning_operation = None
        self._tuning_state = {
            'available': False,
            'preset': 'custom',
            'values': {},
            'status': 'unavailable',
            'detail': 'waiting for live ROS parameter read-back',
            'request_id': 0,
        }
        tuning_nodes = {
            owner: next(
                spec.node_name for spec in TUNING_PARAMETERS.values()
                if spec.owner == owner
            )
            for owner in (CONTROLLER_OWNER, ADAPTER_OWNER)
        }
        self._tuning_get_clients = {
            owner: self.create_client(
                GetParameters, f'{node_name}/get_parameters'
            )
            for owner, node_name in tuning_nodes.items()
        }
        self._tuning_set_clients = {
            owner: self.create_client(
                SetParametersAtomically,
                f'{node_name}/set_parameters_atomically',
            )
            for owner, node_name in tuning_nodes.items()
        }
        self._publish_tuning_state()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self.create_timer(0.1, self._update_pose)
        self.create_timer(OBSTACLE_REFRESH_SEC, self._refresh_obstacle_states)
        self.create_timer(TUNING_REFRESH_SEC, self._refresh_tuning_state)
        self._publish_gateway_state()

    def _on_map(self, message: OccupancyGrid, identity: tuple) -> None:
        """Accept a map only for the stable runtime that subscribed to it."""
        if identity != self._map_subscription_identity:
            return
        try:
            value = _grid(message)
            value['runtime_epoch'] = identity[0]
            self._cache.update('map', value)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f'Rejected invalid {MAP_TOPIC}: {error}')

    @staticmethod
    def _map_identity(message: ModeState) -> tuple | None:
        """Return a positive identity only for an applied, stable runtime map."""
        if (
            int(message.status) != ModeState.STATUS_STABLE
            or not bool(message.ready)
            or int(message.runtime_epoch) <= 0
        ):
            return None
        mode = int(message.mode)
        if mode == ModeState.MODE_MAPPING and message.mapping_session_id:
            applied_map = message.mapping_session_id
        elif mode == ModeState.MODE_AUTONOMY and message.active_autonomy_map:
            applied_map = message.active_autonomy_map
        else:
            return None
        return int(message.runtime_epoch), mode, applied_map

    def _select_map_subscription(self, identity: tuple | None) -> None:
        """Replace the map reader at runtime boundaries to reacquire durability."""
        if identity == getattr(self, '_map_subscription_identity', None):
            return
        previous = self._map_subscription
        self._map_subscription = None
        self._map_subscription_identity = None
        self._cache.invalidate('map')
        if previous is not None:
            self.destroy_subscription(previous)
        if identity is None:
            return
        self._map_subscription_identity = identity
        try:
            self._map_subscription = self.create_subscription(
                OccupancyGrid,
                MAP_TOPIC,
                lambda message, expected=identity: RosStateNode._on_map(
                    self, message, expected
                ),
                self._map_qos,
            )
        except Exception:
            self._map_subscription_identity = None
            raise

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
            identity = RosStateNode._map_identity(message)
            RosStateNode._select_map_subscription(self, identity)
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
                'reset_state': int(message.reset_state),
                'reset_request_id': int(message.reset_request_id),
                'reset_detail': message.reset_detail,
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
                'runtime_epoch': int(message.runtime_epoch),
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

    def _on_system_telemetry(self, message: SystemTelemetry) -> None:
        """Cache the existing one-hertz Pi CPU telemetry without inventing 0."""
        cpu_valid = bool(message.cpu_valid)
        cpu_percent = float(message.total_cpu_utilization_percent)
        if cpu_valid and not math.isfinite(cpu_percent):
            cpu_valid = False
        self._cache.update('system_telemetry', {
            'stamp': _stamp(message.stamp),
            'cpu_valid': cpu_valid,
            'total_cpu_utilization_percent': (
                cpu_percent if cpu_valid else None
            ),
        })

    def _on_battery(self, message: BatteryState) -> None:
        """Cache voltage from the existing Runner fuel-gauge publisher."""
        voltage = float(message.voltage)
        valid = bool(message.present) and math.isfinite(voltage)
        self._cache.update('battery', {
            'stamp': _stamp(message.header.stamp),
            'present': bool(message.present),
            'voltage_valid': valid,
            'voltage': voltage if valid else None,
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
            if pending is None or pending['phase'] != 'awaiting_pose' \
                    or stamp_ns < pending['request_stamp_ns']:
                return
        snapshot = self._cache.state_snapshot()
        reason = self._initial_pose_context_changed(pending, snapshot)
        if not reason:
            reason = self._initial_pose_safety_rejection(snapshot)
        if reason:
            self._fail_initial_pose(pending, reason)
            return
        observed_state = {
            'stamp': stamp,
            'frame_id': message.header.frame_id,
            'pose': observed,
            'covariance': covariance,
        }
        self._start_initial_pose_clear(pending, observed_state)

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

    # -- live Nav2 obstacle processing ------------------------------------

    @staticmethod
    def _boolean_parameter(response) -> tuple[bool | None, str]:
        values = getattr(response, 'values', ())
        if len(values) != 1:
            return None, 'get_parameters returned no single value'
        value = values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None, (
                f'{OBSTACLE_PARAMETER} is missing or is not boolean '
                f'(type={value.type})'
            )
        return bool(value.bool_value), ''

    def _publish_obstacle_state(self) -> None:
        now = time.monotonic()
        with self._obstacle_lock:
            value = {}
            for costmap, state in self._obstacle_states.items():
                confirmed_at = state['confirmed_at']
                age = (
                    None if confirmed_at is None
                    else max(0.0, now - confirmed_at)
                )
                value[costmap] = {
                    key: item for key, item in state.items()
                    if key != 'confirmed_at'
                }
                value[costmap]['available'] = confirmed_at is not None
                value[costmap]['age_sec'] = age
                value[costmap]['stale'] = age is None or age > 3.0
        self._cache.update('obstacle_processing', value)

    def _refresh_obstacle_states(self) -> None:
        """Read both real plugin parameters at a low operator cadence."""
        now = time.monotonic()
        timed_out = []
        with self._obstacle_lock:
            for costmap, operation in tuple(self._obstacle_operations.items()):
                if now < operation['deadline']:
                    continue
                timed_out.append((costmap, operation))
                del self._obstacle_operations[costmap]
                state = self._obstacle_states[costmap]
                state['status'] = 'failed'
                state['detail'] = (
                    f'{operation["phase"]} timed out waiting for Nav2'
                )
        for costmap, operation in timed_out:
            self.get_logger().warning(
                f'{costmap} obstacle processing {operation["phase"]} timed out'
            )

        for costmap, client in self._obstacle_get_clients.items():
            with self._obstacle_lock:
                if costmap in self._obstacle_operations:
                    continue
                self._obstacle_refresh_id -= 1
                refresh_id = self._obstacle_refresh_id
            if not client.service_is_ready():
                with self._obstacle_lock:
                    state = self._obstacle_states[costmap]
                    if state['confirmed_at'] is None:
                        state['status'] = 'unavailable'
                        state['detail'] = 'Nav2 get_parameters unavailable'
                continue
            self._start_obstacle_get(
                costmap, 'refresh', refresh_id, None
            )
        self._publish_obstacle_state()

    def _start_obstacle_get(
        self, costmap: str, phase: str, request_id: int,
        requested: bool | None,
    ) -> bool:
        client = self._obstacle_get_clients[costmap]
        if not client.service_is_ready():
            return False
        request = GetParameters.Request()
        request.names = [OBSTACLE_PARAMETER]
        with self._obstacle_lock:
            self._obstacle_operations[costmap] = {
                'phase': phase,
                'request_id': request_id,
                'requested': requested,
                'deadline': time.monotonic() + 2.0,
            }
        try:
            future = client.call_async(request)
        except Exception as error:  # noqa: B902
            with self._obstacle_lock:
                self._obstacle_operations.pop(costmap, None)
                state = self._obstacle_states[costmap]
                state['status'] = 'failed'
                state['detail'] = f'get_parameters failed to start: {error}'
            return False
        future.add_done_callback(
            lambda done, name=costmap, rid=request_id, kind=phase:
            self._finish_obstacle_get(name, kind, rid, done)
        )
        return True

    def _finish_obstacle_get(
        self, costmap: str, phase: str, request_id: int, future
    ) -> None:
        with self._obstacle_lock:
            operation = self._obstacle_operations.get(costmap)
            if operation is None or operation['phase'] != phase \
                    or operation['request_id'] != request_id:
                return
            requested = operation['requested']
            del self._obstacle_operations[costmap]
        try:
            response = future.result()
            current, error = self._boolean_parameter(response)
        except Exception as exception:  # noqa: B902
            current, error = None, f'get_parameters failed: {exception}'
        with self._obstacle_lock:
            state = self._obstacle_states[costmap]
            if error:
                state['status'] = 'failed'
                state['detail'] = error
            else:
                state['current'] = current
                state['confirmed_at'] = time.monotonic()
                if phase == 'verify':
                    if current == requested:
                        state['status'] = 'applied'
                        state['detail'] = (
                            f'Nav2 read-back confirmed '
                            f'{"ON" if current else "OFF"}'
                        )
                    else:
                        state['status'] = 'failed'
                        state['detail'] = (
                            'set_parameters reported success but read-back '
                            f'is {"ON" if current else "OFF"}'
                        )
                elif state['applied'] is not None \
                        and current != state['applied']:
                    state['status'] = 'drifted'
                    state['detail'] = (
                        'authoritative Nav2 state changed outside Paddock'
                    )
                elif state['status'] in ('unavailable', 'current'):
                    state['status'] = 'current'
                    state['detail'] = 'authoritative Nav2 parameter read'
        self._publish_obstacle_state()

    def _request_obstacle_processing(
        self, intent: ObstacleProcessingIntent, role: str
    ) -> GatewayResult:
        costmap = intent.costmap
        client = self._obstacle_set_clients[costmap]
        with self._obstacle_lock:
            self._obstacle_request_id += 1
            request_id = self._obstacle_request_id
            # Supersede an in-flight refresh; its callback is token-checked.
            self._obstacle_operations.pop(costmap, None)
            state = self._obstacle_states[costmap]
            state['request_id'] = request_id
            state['requested'] = intent.enabled
            state['applied'] = None
            state['status'] = 'requested'
            state['detail'] = 'waiting for Nav2 set_parameters'
        if not client.service_is_ready():
            with self._obstacle_lock:
                state['status'] = 'rejected'
                state['detail'] = 'Nav2 set_parameters unavailable'
            self._publish_obstacle_state()
            return GatewayResult(False, state['detail'], (), role)

        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=OBSTACLE_PARAMETER,
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=intent.enabled,
            ),
        )]
        with self._obstacle_lock:
            self._obstacle_operations[costmap] = {
                'phase': 'set',
                'request_id': request_id,
                'requested': intent.enabled,
                'deadline': time.monotonic() + 2.0,
            }
        try:
            future = client.call_async(request)
        except Exception as error:  # noqa: B902
            with self._obstacle_lock:
                self._obstacle_operations.pop(costmap, None)
                state['status'] = 'failed'
                state['detail'] = f'set_parameters failed to start: {error}'
            self._publish_obstacle_state()
            return GatewayResult(False, state['detail'], (), role)
        future.add_done_callback(
            lambda done, name=costmap, rid=request_id:
            self._finish_obstacle_set(name, rid, done)
        )
        self._publish_obstacle_state()
        return GatewayResult(
            True,
            f'{costmap} obstacle processing requested; awaiting read-back',
            (),
            role,
        )

    def _finish_obstacle_set(
        self, costmap: str, request_id: int, future
    ) -> None:
        with self._obstacle_lock:
            operation = self._obstacle_operations.get(costmap)
            if operation is None or operation['phase'] != 'set' \
                    or operation['request_id'] != request_id:
                return
            requested = operation['requested']
            del self._obstacle_operations[costmap]
        try:
            response = future.result()
            results = getattr(response, 'results', ())
            accepted = len(results) == 1 and results[0].successful
            reason = (
                results[0].reason if len(results) == 1 else
                'set_parameters returned no single result'
            )
        except Exception as error:  # noqa: B902
            accepted = False
            reason = f'set_parameters failed: {error}'
        with self._obstacle_lock:
            state = self._obstacle_states[costmap]
            if not accepted:
                state['status'] = 'rejected'
                state['detail'] = reason or 'Nav2 rejected the parameter'
            else:
                state['applied'] = requested
                state['status'] = 'verifying'
                state['detail'] = 'set accepted; verifying with get_parameters'
        if not accepted:
            self._publish_obstacle_state()
            return
        if not self._start_obstacle_get(
            costmap, 'verify', request_id, requested
        ):
            with self._obstacle_lock:
                state['status'] = 'failed'
                state['detail'] = 'set accepted but read-back unavailable'
            self._publish_obstacle_state()

    # -- live autonomy tuning --------------------------------------------

    @staticmethod
    def _double_parameters(response, owner: str) -> dict[str, float]:
        fields = [
            (field, spec) for field, spec in TUNING_PARAMETERS.items()
            if spec.owner == owner
        ]
        values = getattr(response, 'values', ())
        if len(values) != len(fields):
            raise ValueError('get_parameters returned an incomplete result')
        result = {}
        for (field, _spec), value in zip(fields, values):
            if value.type != ParameterType.PARAMETER_DOUBLE:
                raise ValueError(f'{field} is missing or is not double')
            result[field] = float(value.double_value)
        return result

    def _publish_tuning_state(self) -> None:
        with self._tuning_lock:
            state = dict(self._tuning_state)
            state['values'] = dict(state['values'])
        self._cache.update('autonomy_tuning', state)

    def _refresh_tuning_state(self) -> None:
        """Refresh every displayed value from its live ROS parameter owner."""
        with self._tuning_lock:
            operation = self._tuning_operation
            if operation is not None \
                    and time.monotonic() >= operation['deadline']:
                self._tuning_operation = None
                self._tuning_state.update({
                    'status': 'failed',
                    'detail': f'{operation["kind"]} timed out',
                    'available': False,
                    'preset': 'custom',
                    'values': {},
                })
                operation = None
        if operation is None:
            self._publish_tuning_state()
        self._start_tuning_read()

    def _start_tuning_read(self, verification: dict | None = None) -> None:
        with self._tuning_lock:
            if self._tuning_operation is not None:
                return
            self._tuning_request_id += 1
            request_id = self._tuning_request_id
            self._tuning_operation = {
                'kind': 'read',
                'request_id': request_id,
                'pending': set(self._tuning_get_clients),
                'values': {},
                'errors': [],
                'verification': verification,
                'deadline': time.monotonic() + TUNING_REQUEST_TIMEOUT_SEC,
            }
        for owner, client in self._tuning_get_clients.items():
            fields = [
                (field, spec) for field, spec in TUNING_PARAMETERS.items()
                if spec.owner == owner
            ]
            if not client.service_is_ready():
                self._finish_tuning_read(
                    owner, request_id, None,
                    error=f'{owner} parameter read service unavailable',
                )
                continue
            request = GetParameters.Request()
            request.names = [spec.parameter_name for _field, spec in fields]
            try:
                future = client.call_async(request)
            except Exception as error:  # noqa: B902
                self._finish_tuning_read(
                    owner, request_id, None,
                    error=f'{owner} parameter read failed to start: {error}',
                )
                continue
            future.add_done_callback(
                lambda done, item=owner, rid=request_id:
                self._finish_tuning_read(item, rid, done)
            )

    def _finish_tuning_read(
        self, owner: str, request_id: int, future, error: str = ''
    ) -> None:
        observed = {}
        if not error:
            try:
                observed = self._double_parameters(future.result(), owner)
            except Exception as exception:  # noqa: B902
                error = f'{owner} parameter read failed: {exception}'
        with self._tuning_lock:
            operation = self._tuning_operation
            if operation is None or operation['kind'] != 'read' \
                    or operation['request_id'] != request_id \
                    or owner not in operation['pending']:
                return
            operation['pending'].remove(owner)
            operation['values'].update(observed)
            if error:
                operation['errors'].append(error)
            if operation['pending']:
                return
            self._tuning_operation = None
            errors = operation['errors']
            values = operation['values']
            verification = operation['verification']
            if errors:
                self._tuning_state.update({
                    'available': False,
                    'preset': 'custom',
                    'values': {},
                    'status': 'failed' if verification else 'unavailable',
                    'detail': '; '.join(errors),
                })
            else:
                matches_request = (
                    verification is None
                    or values == verification['requested']
                )
                set_errors = [] if verification is None else verification[
                    'set_errors'
                ]
                applied = not set_errors and matches_request
                if verification is None:
                    status = 'current'
                    detail = 'live ROS parameter read-back'
                elif applied:
                    status = 'applied'
                    detail = 'atomic owner writes confirmed by live read-back'
                else:
                    status = 'failed'
                    problems = list(set_errors)
                    if not matches_request:
                        problems.append('live read-back differs from request')
                    detail = '; '.join(problems)
                self._tuning_state.update({
                    'available': True,
                    'preset': matching_preset(values),
                    'values': values,
                    'status': status,
                    'detail': detail,
                })
        self._publish_tuning_state()

    def _request_autonomy_tuning(
        self, intent: AutonomyTuningIntent, role: str
    ) -> GatewayResult:
        try:
            requested = validate_tuning_values(
                dict(TUNING_PRESETS[intent.preset])
                if intent.preset else intent.values
            )
        except (KeyError, TypeError, ValueError) as error:
            return GatewayResult(False, str(error), (), role)
        unavailable = [
            owner for owner, client in self._tuning_set_clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            return GatewayResult(
                False,
                'atomic parameter service unavailable: '
                + ', '.join(unavailable),
                (), role,
            )
        with self._tuning_lock:
            if self._tuning_operation is not None:
                return GatewayResult(
                    False, 'autonomy tuning operation already in progress',
                    (), role,
                )
            self._tuning_request_id += 1
            request_id = self._tuning_request_id
            owners = set(self._tuning_set_clients)
            self._tuning_operation = {
                'kind': 'set',
                'request_id': request_id,
                'pending': owners,
                'requested': requested,
                'errors': [],
                'deadline': time.monotonic() + TUNING_REQUEST_TIMEOUT_SEC,
            }
            self._tuning_state.update({
                'status': 'applying',
                'detail': f'applying {intent.preset or "custom"} atomically',
                'request_id': request_id,
            })
        self._publish_tuning_state()
        for owner, client in self._tuning_set_clients.items():
            request = SetParametersAtomically.Request()
            request.parameters = [
                Parameter(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=value,
                    ),
                )
                for name, value in values_for_owner(requested, owner).items()
            ]
            try:
                future = client.call_async(request)
            except Exception as error:  # noqa: B902
                self._finish_tuning_set(
                    owner, request_id, None,
                    error=f'{owner} atomic write failed to start: {error}',
                )
                continue
            future.add_done_callback(
                lambda done, item=owner, rid=request_id:
                self._finish_tuning_set(item, rid, done)
            )
        return GatewayResult(
            True,
            f'{intent.preset or "custom"} tuning accepted; awaiting read-back',
            (), role,
        )

    def _finish_tuning_set(
        self, owner: str, request_id: int, future, error: str = ''
    ) -> None:
        if not error:
            try:
                result = future.result().result
                if not result.successful:
                    error = result.reason or f'{owner} rejected atomic write'
            except Exception as exception:  # noqa: B902
                error = f'{owner} atomic write failed: {exception}'
        with self._tuning_lock:
            operation = self._tuning_operation
            if operation is None or operation['kind'] != 'set' \
                    or operation['request_id'] != request_id \
                    or owner not in operation['pending']:
                return
            operation['pending'].remove(owner)
            if error:
                operation['errors'].append(error)
            if operation['pending']:
                return
            requested = operation['requested']
            errors = operation['errors']
            self._tuning_operation = None
        self._start_tuning_read({
            'requested': requested,
            'set_errors': errors,
        })

    # -- operator intent ---------------------------------------------------

    def submit(self, conn_id: str, action: dict) -> dict:
        """Validate one browser action, publish its intents, report outcome."""
        with self._gateway_lock:
            result = self._gateway.handle(conn_id, action)
            with self._initial_pose_lock:
                initial_pose_active = self._pending_initial_pose is not None
            if result.accepted and action.get('action') == 'clear_stop' \
                    and initial_pose_active:
                result = GatewayResult(
                    False,
                    'CLEAR STOP blocked during initial pose transaction',
                    (),
                    result.role,
                )
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
                if isinstance(intent, ObstacleProcessingIntent):
                    result = self._request_obstacle_processing(
                        intent, result.role
                    )
                    break
                if isinstance(intent, AutonomyTuningIntent):
                    result = self._request_autonomy_tuning(
                        intent, result.role
                    )
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
        """Begin STOP -> stationary -> pose -> confirmation -> clear."""
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

        with self._initial_pose_lock:
            if self._pending_initial_pose is not None:
                reason = 'initial pose transaction already in progress'
                return GatewayResult(False, reason, (), role)
            self._initial_pose_request_id += 1
            self._pending_initial_pose = {
                'request_id': self._initial_pose_request_id,
                'phase': 'waiting_stop',
                'deadline': time.monotonic() + INITIAL_POSE_STOP_TIMEOUT_SEC,
                'status': status,
                'intent': intent,
            }
        self._cache.update('initial_pose', {
            **status,
            'state': 'stopping',
            'detail': 'STOP requested; awaiting applied STOP and stationary',
            'observed': None,
        })
        return GatewayResult(
            True, 'initial pose accepted; awaiting safe stopped state', (), role
        )

    def _publish_initial_pose(self, pending: dict) -> None:
        """Publish the seed after stopped/stationary evidence is current."""
        intent = pending['intent']
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
        with self._initial_pose_lock:
            current = self._pending_initial_pose
            if current is not pending or current['phase'] != 'waiting_stop':
                return
            current.update({
                'phase': 'awaiting_pose',
                'request_stamp_ns': now.nanoseconds,
                'deadline': time.monotonic()
                + INITIAL_POSE_CONFIRM_TIMEOUT_SEC,
            })
            self._initial_pose_pub.publish(message)
            self._cache.update('initial_pose', {
                **pending['status'],
                'state': 'localizing',
                'detail': 'pose published; awaiting slam_toolbox confirmation',
                'observed': None,
            })

    @staticmethod
    def _initial_pose_rejection(snapshot: dict) -> str:
        """Return the first localization-context precondition failure."""
        health = snapshot.get('health') or {}
        sources = health.get('sources') or {}
        mode = snapshot.get('mode') or {}
        map_state = snapshot.get('map_state') or {}
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
        return ''

    @staticmethod
    def _initial_pose_safety_rejection(snapshot: dict) -> str:
        """Return why stopped and stationary evidence is not yet sufficient."""
        sources = (snapshot.get('health') or {}).get('sources') or {}
        stop = snapshot.get('stop_state') or {}
        encoder = snapshot.get('encoder_state') or {}
        if not (sources.get('stop_state') or {}).get('fresh'):
            return 'STOP state unavailable or stale'
        if not (stop.get('stopped') and stop.get('locked')
                and stop.get('healthy') and stop.get('applied')):
            return 'awaiting healthy applied STOP'
        if not (sources.get('encoder_state') or {}).get('fresh'):
            return 'encoder stationary state unavailable or stale'
        if not encoder.get('stationary'):
            return 'robot is not stationary'
        return ''

    @staticmethod
    def _initial_pose_context_changed(pending: dict, snapshot: dict) -> str:
        reason = RosStateNode._initial_pose_rejection(snapshot)
        if reason:
            return f'initial pose context lost: {reason}'
        mode = snapshot.get('mode') or {}
        map_state = snapshot.get('map_state') or {}
        status = pending['status']
        if (
            int(mode.get('runtime_epoch') or 0) != status['runtime_epoch']
            or (map_state.get('selected_map_applied') or '') != status['map']
            or (mode.get('active_autonomy_map') or '') != status['map']
        ):
            return 'runtime or active map changed during initial pose'
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

    def _advance_initial_pose_transaction(self) -> None:
        """Advance a pending seed only after fresh stopped-state evidence."""
        with self._initial_pose_lock:
            pending = self._pending_initial_pose
        if pending is None or pending['phase'] != 'waiting_stop':
            return
        snapshot = self._cache.state_snapshot()
        reason = self._initial_pose_context_changed(pending, snapshot)
        if reason:
            self._fail_initial_pose(pending, reason)
            return
        safety = self._initial_pose_safety_rejection(snapshot)
        if safety:
            self._cache.update('initial_pose', {
                **pending['status'],
                'state': 'stopping',
                'detail': safety,
                'observed': None,
            })
            return
        if self._initial_pose_pub.get_subscription_count() < 1:
            self._fail_initial_pose(
                pending, 'slam_toolbox /initialpose subscriber unavailable'
            )
            return
        self._publish_initial_pose(pending)

    def _start_initial_pose_clear(
        self, pending: dict, observed: dict
    ) -> None:
        """Clear both dynamic costmaps after localization confirmation."""
        clients = {
            'global': self._global_clear_client,
            'local': self._local_clear_client,
        }
        unavailable = [
            name for name, client in clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            self._fail_initial_pose(
                pending,
                f"Nav2 {' and '.join(unavailable)} costmap clear service "
                'unavailable after localization confirmation',
                observed=observed,
            )
            return
        with self._initial_pose_lock:
            current = self._pending_initial_pose
            if current is not pending or current['phase'] != 'awaiting_pose':
                return
            current.update({
                'phase': 'clearing',
                'deadline': time.monotonic() + INITIAL_POSE_CLEAR_TIMEOUT_SEC,
                'clear_pending': set(clients),
                'clear_errors': [],
                'observed': observed,
            })
        self._cache.update('initial_pose', {
            **pending['status'],
            'state': 'clearing',
            'detail': 'pose confirmed; clearing global and local costmaps',
            'observed': observed,
        })
        for name, client in clients.items():
            try:
                future = client.call_async(ClearEntireCostmap.Request())
            except Exception as error:  # noqa: B902
                self._finish_initial_pose_clear(
                    pending['request_id'], name, None,
                    error=f'{name} clear failed to start: {error}',
                )
                continue
            future.add_done_callback(
                lambda done, costmap=name, rid=pending['request_id']:
                self._finish_initial_pose_clear(rid, costmap, done)
            )

    def _finish_initial_pose_clear(
        self, request_id: int, costmap: str, future, error: str = ''
    ) -> None:
        if not error:
            try:
                future.result()
            except Exception as exception:  # noqa: B902
                error = f'{costmap} clear failed: {exception}'
        with self._initial_pose_lock:
            pending = self._pending_initial_pose
            if pending is None or pending['request_id'] != request_id \
                    or pending['phase'] != 'clearing' \
                    or costmap not in pending['clear_pending']:
                return
            pending['clear_pending'].remove(costmap)
            if error:
                pending['clear_errors'].append(error)
            if pending['clear_pending']:
                return
            errors = list(pending['clear_errors'])
            observed = pending['observed']
        if errors:
            self._fail_initial_pose(
                pending, 'costmap clear failed: ' + '; '.join(errors),
                observed=observed,
            )
            return
        snapshot = self._cache.state_snapshot()
        safety = self._initial_pose_context_changed(pending, snapshot)
        if not safety:
            safety = self._initial_pose_safety_rejection(snapshot)
        if safety:
            self._fail_initial_pose(
                pending, f'safety invariant lost before completion: {safety}',
                observed=observed,
            )
            return
        with self._initial_pose_lock:
            if self._pending_initial_pose is not pending:
                return
            self._pending_initial_pose = None
        self._cache.update('initial_pose', {
            **pending['status'],
            'state': 'applied',
            'detail': 'pose confirmed; global and local costmaps cleared; '
            'STOP remains asserted',
            'observed': observed,
        })

    def _fail_initial_pose(
        self, pending: dict, detail: str, observed: dict | None = None
    ) -> None:
        with self._initial_pose_lock:
            if self._pending_initial_pose is not pending:
                return
            self._pending_initial_pose = None
        self._cache.update('initial_pose', {
            **pending['status'],
            'state': 'rejected',
            'detail': detail,
            'observed': observed,
        })

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
        self._advance_initial_pose_transaction()
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
        """Make a stalled initial-pose transaction explicit to operators."""
        with self._initial_pose_lock:
            pending = self._pending_initial_pose
            if pending is None or time.monotonic() < pending['deadline']:
                return
        details = {
            'waiting_stop': 'timed out waiting for applied STOP and stationary',
            'awaiting_pose': (
                'application unconfirmed: no fresh slam_toolbox pose'
            ),
            'clearing': 'timed out clearing dynamic costmaps',
        }
        self._fail_initial_pose(
            pending,
            details.get(pending['phase'], 'initial pose transaction timed out'),
            observed=pending.get('observed'),
        )
