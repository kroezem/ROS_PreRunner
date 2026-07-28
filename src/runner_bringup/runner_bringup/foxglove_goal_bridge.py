"""Bridge Foxglove simple goals and persistent routes to Nav2 actions."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}

NAV2_ERROR_NAMES = {
    0: 'NONE',
    100: 'UNKNOWN',
    101: 'INVALID_CONTROLLER',
    102: 'TF_ERROR',
    103: 'INVALID_PATH',
    104: 'PATIENCE_EXCEEDED',
    105: 'FAILED_TO_MAKE_PROGRESS',
    106: 'NO_VALID_CONTROL',
    107: 'CONTROLLER_TIMED_OUT',
    200: 'UNKNOWN',
    201: 'INVALID_PLANNER',
    202: 'TF_ERROR',
    203: 'START_OUTSIDE_MAP',
    204: 'GOAL_OUTSIDE_MAP',
    205: 'START_OCCUPIED',
    206: 'GOAL_OCCUPIED',
    207: 'TIMEOUT',
    208: 'NO_VALID_PATH',
    300: 'UNKNOWN',
    301: 'INVALID_PLANNER',
    302: 'TF_ERROR',
    303: 'START_OUTSIDE_MAP',
    304: 'GOAL_OUTSIDE_MAP',
    305: 'START_OCCUPIED',
    306: 'GOAL_OCCUPIED',
    307: 'TIMEOUT',
    308: 'NO_VALID_PATH',
    309: 'NO_VIAPOINTS_GIVEN',
    500: 'UNKNOWN',
    501: 'INVALID_SMOOTHER',
    502: 'TIMEOUT',
    503: 'SMOOTHED_PATH_IN_COLLISION',
    504: 'FAILED_TO_SMOOTH_PATH',
    505: 'INVALID_PATH',
    700: 'UNKNOWN',
    701: 'TIMEOUT',
    702: 'TF_ERROR',
    703: 'COLLISION_AHEAD',
    710: 'UNKNOWN',
    711: 'TIMEOUT',
    712: 'TF_ERROR',
    713: 'INVALID_INPUT',
    714: 'COLLISION_AHEAD',
    720: 'UNKNOWN',
    721: 'TIMEOUT',
    722: 'TF_ERROR',
    723: 'COLLISION_AHEAD',
    724: 'INVALID_INPUT',
    730: 'UNKNOWN',
    731: 'TIMEOUT',
    732: 'TF_ERROR',
}


@dataclass(frozen=True)
class _Request:
    kind: str
    poses: tuple[PoseStamped, ...]


class FoxgloveGoalBridge(Node):
    """Forward only the latest simple goal or route, serially, to Nav2."""

    CANCEL_RETRY_INTERVAL = 0.5
    MAX_CANCEL_ATTEMPTS = 3
    RESULT_RETRY_INTERVAL = 0.5
    SHUTDOWN_TIMEOUT = 1.0
    DEFAULT_ROUTE_FILE = '~/.ros/runner_route.json'
    ROUTE_FRAME = 'map'
    MARKER_NAMESPACE = 'runner_route'
    WAYPOINT_DIAMETER = 0.06
    LABEL_HEIGHT = 0.055
    LABEL_Z_OFFSET = 0.08

    def __init__(
        self,
        action_client=None,
        route_action_client=None,
        route_file=None,
    ):
        super().__init__('foxglove_goal_bridge')
        self._action_client = action_client or ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._route_action_client = (
            route_action_client
            or ActionClient(
                self,
                NavigateThroughPoses,
                '/navigate_through_poses',
            )
        )
        configured_route_file = self.declare_parameter(
            'route_file',
            self.DEFAULT_ROUTE_FILE,
        ).value
        self._route_file = Path(
            route_file or configured_route_file
        ).expanduser()
        self._route = []
        self._loop_enabled = False
        self._route_run_enabled = False
        self._pending_request = None
        self._next_generation = 1
        self._send_generation = None
        self._send_request = None
        self._send_future = None
        self._active_generation = None
        self._active_kind = None
        self._goal_handle = None
        self._result_future = None
        self._next_result_attempt = 0.0
        self._cancel_future = None
        self._cancel_attempts = 0
        self._cancel_accepted = False
        self._cancel_requested = False
        self._next_cancel_attempt = 0.0
        self._cancel_exhausted_logged = False
        self._shutting_down = False
        self._goal_state = 'none'
        self._goal_state_since = time.monotonic()
        self._last_error_code = 0
        self._last_error_meaning = NAV2_ERROR_NAMES[0]
        self._current_waypoint_index = -1
        route_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._route_pub = self.create_publisher(
            PathMessage,
            '/runner/route',
            route_qos,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            '/runner/route_markers',
            route_qos,
        )
        self._autonomy_state_pub = self.create_publisher(
            String,
            '/runner/autonomy_state',
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self._goal_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/runner/waypoint',
            self._waypoint_callback,
            10,
        )
        self.create_subscription(
            String,
            '/runner/route_control',
            self._route_control_callback,
            10,
        )
        self._load_route()
        self._publish_route()
        self.create_timer(0.5, self._timer_callback)

    @property
    def route(self):
        """Return an immutable snapshot of the current route."""
        return tuple(self._route)

    @property
    def loop_enabled(self):
        """Return whether successful route completion repeats."""
        return self._loop_enabled

    def _goal_callback(self, pose):
        if self._shutting_down or not self._valid_pose(pose):
            if not self._shutting_down:
                self.get_logger().warning(
                    'Rejecting malformed /move_base_simple/goal pose'
                )
            return
        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(
            'Foxglove goal received: '
            f'frame={pose.header.frame_id!r} '
            f'position=({position.x:.6f}, {position.y:.6f}, '
            f'{position.z:.6f}) '
            f'orientation=({orientation.x:.6f}, '
            f'{orientation.y:.6f}, {orientation.z:.6f}, '
            f'{orientation.w:.6f})'
        )
        self._route_run_enabled = False
        self._queue_request(_Request('NavigateToPose', (pose,)))

    def _waypoint_callback(self, pose):
        if self._shutting_down:
            return
        if not self._valid_pose(pose):
            self.get_logger().warning(
                'Rejecting malformed /runner/waypoint pose'
            )
            return
        if pose.header.frame_id != self.ROUTE_FRAME:
            self.get_logger().warning(
                'Rejecting /runner/waypoint outside the map frame'
            )
            return
        self._route.append(pose)
        self._persist_route()
        self._publish_route()
        self.get_logger().info(
            f'Route waypoint appended; count={len(self._route)}'
        )

    def _route_control_callback(self, message):
        if self._shutting_down:
            return
        command = message.data
        if command == 'start':
            if not self._route:
                self.get_logger().warning(
                    'Cannot start route: no waypoints are stored'
                )
                return
            self._route_run_enabled = True
            self._queue_request(
                _Request('NavigateThroughPoses', tuple(self._route))
            )
        elif command == 'stop':
            self._route_run_enabled = False
            self._pending_request = None
            self._cancel_requested = True
            self._request_cancel()
        elif command == 'clear':
            self._route_run_enabled = False
            self._pending_request = None
            self._route = []
            self._persist_route()
            self._publish_route()
            self._cancel_requested = True
            self._request_cancel()
        elif command == 'loop_on':
            self._loop_enabled = True
            self._persist_route()
        elif command == 'loop_off':
            self._loop_enabled = False
            self._persist_route()
        elif command == 'loop_toggle':
            self._loop_enabled = not self._loop_enabled
            self._persist_route()
        elif command == 'remove_last':
            if not self._route:
                self.get_logger().warning(
                    'Cannot remove last waypoint: route is empty'
                )
                return
            self._route.pop()
            self._persist_route()
            self._publish_route()
            self.get_logger().info(
                f'Last route waypoint removed; count={len(self._route)}'
            )
        else:
            self.get_logger().warning(
                f'Rejecting unknown route command {command!r}'
            )

    def _queue_request(self, request):
        self._cancel_requested = False
        self._pending_request = request
        if self._goal_handle is not None:
            self._request_cancel()
        self._pump()

    def _timer_callback(self):
        self._pump()
        self._publish_autonomy_state()

    def _set_goal_state(self, state):
        if state == self._goal_state:
            return
        self._goal_state = state
        self._goal_state_since = time.monotonic()

    def _publish_autonomy_state(self):
        elapsed = max(0.0, time.monotonic() - self._goal_state_since)
        document = {
            'goal_state': self._goal_state,
            'last_error_code': self._last_error_code,
            'last_error_meaning': self._last_error_meaning,
            'route_length': len(self._route),
            'current_waypoint_index': self._current_waypoint_index,
            'loop_mode': self._loop_enabled,
            'time_in_state_seconds': round(elapsed, 3),
        }
        self._autonomy_state_pub.publish(
            String(data=json.dumps(document, sort_keys=True))
        )

    def _client_for(self, kind):
        if kind == 'NavigateThroughPoses':
            return self._route_action_client
        return self._action_client

    def _pump(self):
        if self._shutting_down:
            if self._goal_handle is not None:
                self._ensure_result_request()
                self._request_cancel(force=True)
            return
        if self._goal_handle is not None:
            self._ensure_result_request()
            if self._pending_request is not None or self._cancel_requested:
                self._request_cancel()
            return
        if (
            self._pending_request is None
            or self._send_generation is not None
        ):
            return

        request = self._pending_request
        client = self._client_for(request.kind)
        if not client.server_is_ready():
            self.get_logger().warning(
                f'{request.kind} action server is not ready; '
                'latest request pending'
            )
            return

        self._pending_request = None
        generation = self._next_generation
        self._next_generation += 1
        self._send_generation = generation
        self._send_request = request
        if request.kind == 'NavigateThroughPoses':
            goal = NavigateThroughPoses.Goal()
            goal.poses = list(request.poses)
        else:
            goal = NavigateToPose.Goal()
            goal.pose = request.poses[0]
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=lambda message,
                sent_generation=generation:
                self._feedback_callback(message, sent_generation),
            )
        except Exception as error:
            if self._send_generation == generation:
                self._clear_send_state()
                self._restore_request_if_latest(request)
            self.get_logger().error(
                f'{request.kind} goal request raised synchronously: {error}'
            )
            return

        self._send_future = future
        future.add_done_callback(
            lambda completed, sent_generation=generation,
            sent_future=future, sent_request=request:
            self._goal_response_callback(
                completed,
                sent_generation,
                sent_future,
                sent_request,
            )
        )

    def _goal_response_callback(
        self,
        future,
        generation,
        expected_future,
        request,
    ):
        if (
            self._send_generation != generation
            or self._send_future is not expected_future
            or future is not expected_future
        ):
            self.get_logger().warning(
                f'Ignoring stale {request.kind} send callback '
                f'for generation {generation}'
            )
            return

        self._clear_send_state()
        try:
            goal_handle = future.result()
        except Exception as error:
            self._restore_request_if_latest(request)
            self.get_logger().error(
                f'{request.kind} goal request failed: {error}'
            )
            self._pump()
            return

        if not goal_handle.accepted:
            self._restore_request_if_latest(request)
            self.get_logger().warning(f'{request.kind} goal rejected')
            self._set_goal_state('aborted')
            self._pump()
            return

        self._active_generation = generation
        self._active_kind = request.kind
        self._goal_handle = goal_handle
        self._current_waypoint_index = (
            0 if request.kind == 'NavigateThroughPoses' else -1
        )
        self._set_goal_state('accepted')
        self._reset_cancel_state()
        self.get_logger().info(f'{request.kind} goal accepted')
        self._ensure_result_request()
        if (
            self._shutting_down
            or self._pending_request is not None
            or self._cancel_requested
        ):
            self._request_cancel(force=self._shutting_down)

    def _feedback_callback(self, message, generation):
        if self._active_generation != generation:
            return
        self._set_goal_state('executing')
        if self._active_kind != 'NavigateThroughPoses':
            return
        remaining = max(0, int(message.feedback.number_of_poses_remaining))
        route_length = len(self._route)
        if route_length:
            self._current_waypoint_index = min(
                route_length - 1,
                max(0, route_length - remaining),
            )

    def _ensure_result_request(self):
        if (
            self._goal_handle is None
            or self._result_future is not None
            or time.monotonic() < self._next_result_attempt
        ):
            return
        generation = self._active_generation
        goal_handle = self._goal_handle
        kind = self._active_kind
        try:
            future = goal_handle.get_result_async()
        except Exception as error:
            if self._active_matches(generation, goal_handle):
                self._next_result_attempt = (
                    time.monotonic() + self.RESULT_RETRY_INTERVAL
                )
            self.get_logger().error(
                f'{kind} result request raised synchronously; '
                f'the accepted goal remains active and will be retried: '
                f'{error}'
            )
            return

        self._result_future = future
        future.add_done_callback(
            lambda completed, active_generation=generation,
            active_handle=goal_handle, expected_future=future:
            self._result_callback(
                completed,
                active_generation,
                active_handle,
                expected_future,
            )
        )

    def _request_cancel(self, force=False):
        if self._goal_handle is None or self._cancel_future is not None:
            return
        if self._cancel_accepted:
            return
        kind = self._active_kind
        if not force:
            if self._cancel_attempts >= self.MAX_CANCEL_ATTEMPTS:
                if not self._cancel_exhausted_logged:
                    self.get_logger().error(
                        f'{kind} cancellation attempts exhausted; '
                        'replacement remains blocked until the active goal '
                        'reaches a terminal result'
                    )
                    self._cancel_exhausted_logged = True
                return
            if time.monotonic() < self._next_cancel_attempt:
                return

        generation = self._active_generation
        goal_handle = self._goal_handle
        self._cancel_attempts += 1
        self.get_logger().info(
            f'Canceling active {kind} goal '
            f'(attempt {self._cancel_attempts})'
        )
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().error(
                f'{kind} cancel request raised synchronously: {error}'
            )
            self._schedule_cancel_retry(generation, goal_handle)
            return

        self._cancel_future = future
        future.add_done_callback(
            lambda completed, active_generation=generation,
            active_handle=goal_handle, cancel_future=future:
            self._cancel_callback(
                completed,
                active_generation,
                active_handle,
                cancel_future,
            )
        )

    def _cancel_callback(
        self,
        future,
        generation,
        goal_handle,
        expected_future,
    ):
        if (
            not self._active_matches(generation, goal_handle)
            or self._cancel_future is not expected_future
            or future is not expected_future
        ):
            self.get_logger().warning(
                f'Ignoring stale cancel callback for generation {generation}'
            )
            return

        self._cancel_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'{self._active_kind} cancel request failed: {error}'
            )
            self._schedule_cancel_retry(generation, goal_handle)
            return

        if response.goals_canceling:
            self._cancel_accepted = True
            self.get_logger().info(
                f'{self._active_kind} cancellation accepted'
            )
        else:
            self.get_logger().warning(
                f'{self._active_kind} cancellation was not accepted'
            )
            self._schedule_cancel_retry(generation, goal_handle)

    def _schedule_cancel_retry(self, generation, goal_handle):
        if not self._active_matches(generation, goal_handle):
            return
        self._next_cancel_attempt = (
            time.monotonic() + self.CANCEL_RETRY_INTERVAL
        )
        if self._cancel_attempts >= self.MAX_CANCEL_ATTEMPTS:
            self._request_cancel()

    def _result_callback(
        self,
        future,
        generation,
        goal_handle,
        expected_future,
    ):
        if (
            not self._active_matches(generation, goal_handle)
            or self._result_future is not expected_future
            or future is not expected_future
        ):
            self.get_logger().warning(
                f'Ignoring stale result callback for generation {generation}'
            )
            return
        kind = self._active_kind
        try:
            response = future.result()
            result = response.result
            status_name = STATUS_NAMES.get(
                response.status,
                f'UNKNOWN({response.status})',
            )
            self.get_logger().info(
                f'{kind} result: status={status_name} '
                f'error_code={result.error_code} '
                f'error_msg={result.error_msg!r}'
            )
        except Exception as error:
            self.get_logger().error(
                f'{kind} result request failed; the accepted goal '
                f'remains active and will be retried: {error}'
            )
            if self._active_matches(generation, goal_handle):
                self._result_future = None
                self._next_result_attempt = (
                    time.monotonic() + self.RESULT_RETRY_INTERVAL
                )
                self._pump()
            return

        if not self._active_matches(generation, goal_handle):
            return
        self._last_error_code = int(result.error_code)
        self._last_error_meaning = NAV2_ERROR_NAMES.get(
            self._last_error_code,
            result.error_msg or 'UNKNOWN_ERROR_CODE',
        )
        terminal_state = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(response.status, 'aborted')
        self._set_goal_state(terminal_state)
        succeeded = response.status == GoalStatus.STATUS_SUCCEEDED
        should_loop = (
            kind == 'NavigateThroughPoses'
            and succeeded
            and self._route_run_enabled
            and self._loop_enabled
            and bool(self._route)
            and self._pending_request is None
        )
        if kind == 'NavigateThroughPoses' and not should_loop:
            self._route_run_enabled = False
        self._clear_active_state()
        if should_loop:
            self._pending_request = _Request(
                'NavigateThroughPoses',
                tuple(self._route),
            )
            self._current_waypoint_index = 0
            self.get_logger().info(
                'Route completed successfully; dispatching loop'
            )
        self._pump()

    def _active_matches(self, generation, goal_handle):
        return (
            self._active_generation == generation
            and self._goal_handle is goal_handle
        )

    def _restore_request_if_latest(self, request):
        if not self._shutting_down and self._pending_request is None:
            self._pending_request = request

    def _clear_send_state(self):
        self._send_generation = None
        self._send_request = None
        self._send_future = None

    def _reset_cancel_state(self):
        self._cancel_future = None
        self._cancel_attempts = 0
        self._cancel_accepted = False
        self._next_cancel_attempt = 0.0
        self._cancel_exhausted_logged = False

    def _clear_active_state(self):
        self._active_generation = None
        self._active_kind = None
        self._goal_handle = None
        self._result_future = None
        self._next_result_attempt = 0.0
        self._cancel_requested = False
        self._reset_cancel_state()

    def _valid_pose(self, pose):
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        orientation_norm = math.sqrt(sum(value * value for value in values[3:]))
        return (
            bool(pose.header.frame_id)
            and all(math.isfinite(value) for value in values)
            and orientation_norm > 1e-6
        )

    def _publish_route(self):
        stamp = self.get_clock().now().to_msg()
        message = PathMessage()
        message.header.stamp = stamp
        message.header.frame_id = self.ROUTE_FRAME
        message.poses = list(self._route)
        self._route_pub.publish(message)

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.ROUTE_FRAME
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        for index, pose in enumerate(self._route):
            sphere = Marker()
            sphere.header.frame_id = self.ROUTE_FRAME
            sphere.header.stamp = stamp
            sphere.ns = self.MARKER_NAMESPACE
            sphere.id = index * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale.x = self.WAYPOINT_DIAMETER
            sphere.scale.y = self.WAYPOINT_DIAMETER
            sphere.scale.z = self.WAYPOINT_DIAMETER
            sphere.color.r = 0.1
            sphere.color.g = 0.8
            sphere.color.b = 1.0
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.ROUTE_FRAME
            label.header.stamp = stamp
            label.ns = self.MARKER_NAMESPACE
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose.pose.position.x
            label.pose.position.y = pose.pose.position.y
            label.pose.position.z = (
                pose.pose.position.z + self.LABEL_Z_OFFSET
            )
            label.pose.orientation.w = 1.0
            label.scale.z = self.LABEL_HEIGHT
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(index + 1)
            markers.markers.append(label)
        self._marker_pub.publish(markers)

    def _persist_route(self):
        document = {
            'loop_enabled': self._loop_enabled,
            'poses': [self._pose_to_document(pose) for pose in self._route],
        }
        try:
            self._route_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._route_file.with_suffix(
                self._route_file.suffix + '.tmp'
            )
            temporary.write_text(
                json.dumps(document, indent=2, sort_keys=True) + '\n'
            )
            temporary.replace(self._route_file)
        except OSError as error:
            self.get_logger().error(
                f'Failed to persist route to {self._route_file}: {error}'
            )

    def _load_route(self):
        if not self._route_file.exists():
            return
        try:
            document = json.loads(self._route_file.read_text())
            poses = [
                self._pose_from_document(item)
                for item in document['poses']
            ]
            if not all(self._valid_pose(pose) for pose in poses):
                raise ValueError('route contains an invalid pose')
            if any(
                pose.header.frame_id != self.ROUTE_FRAME
                for pose in poses
            ):
                raise ValueError('route contains a pose outside the map frame')
            loop_enabled = document.get('loop_enabled', False)
            if not isinstance(loop_enabled, bool):
                raise ValueError('loop_enabled is not a boolean')
        except (
            json.JSONDecodeError,
            KeyError,
            TypeError,
            ValueError,
            OSError,
        ) as error:
            self.get_logger().warning(
                f'Ignoring malformed persisted route '
                f'{self._route_file}: {error}'
            )
            return
        self._route = poses
        self._loop_enabled = loop_enabled

    @staticmethod
    def _pose_to_document(pose):
        return {
            'frame_id': pose.header.frame_id,
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
            },
            'orientation': {
                'x': pose.pose.orientation.x,
                'y': pose.pose.orientation.y,
                'z': pose.pose.orientation.z,
                'w': pose.pose.orientation.w,
            },
        }

    @staticmethod
    def _pose_from_document(document):
        pose = PoseStamped()
        pose.header.frame_id = document['frame_id']
        pose.pose.position.x = float(document['position']['x'])
        pose.pose.position.y = float(document['position']['y'])
        pose.pose.position.z = float(document['position']['z'])
        pose.pose.orientation.x = float(document['orientation']['x'])
        pose.pose.orientation.y = float(document['orientation']['y'])
        pose.pose.orientation.z = float(document['orientation']['z'])
        pose.pose.orientation.w = float(document['orientation']['w'])
        return pose

    def shutdown(self, timeout_sec=None):
        """Cancel an active goal and destroy the node within a fixed bound."""
        self._shutting_down = True
        self._route_run_enabled = False
        self._pending_request = None
        self._cancel_requested = True
        timeout = (
            self.SHUTDOWN_TIMEOUT if timeout_sec is None else timeout_sec
        )
        deadline = time.monotonic() + max(0.0, timeout)

        if not rclpy.ok(context=self.context):
            self.destroy_node()
            return

        if self._goal_handle is not None:
            self._request_cancel(force=True)
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        try:
            while (
                self._send_generation is not None
                or self._goal_handle is not None
            ) and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                executor.spin_once(
                    timeout_sec=min(0.02, max(0.0, remaining))
                )
                self._pump()
        finally:
            executor.remove_node(self)
            executor.shutdown(timeout_sec=0)
        self.destroy_node()


def main(args=None):
    """Run the Foxglove goal bridge."""
    rclpy.init(args=args)
    node = FoxgloveGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
