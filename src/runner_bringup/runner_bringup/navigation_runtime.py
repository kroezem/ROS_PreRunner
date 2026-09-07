"""
The one production Nav2 mission/action owner for Runner v1.3.

This module refactors the useful generation-tracking, cancellation-retry and
delayed-callback protection of the retired ``foxglove_goal_bridge`` into a
single navigation runtime that:

* accepts authorized mission intent with real map-frame poses on
  ``/paddock/navigation_request`` (sole writer: the command authority),
* owns every Nav2 mission action client (``NavigateToPose`` and
  ``NavigateThroughPoses`` are one execution component, not two owners),
* publishes a truthful mission/action lifecycle on
  ``/paddock/navigation_state`` and never reports optimistic "active" state
  before Nav2 has actually accepted a goal,
* tracks the current mission revision and action generation so a stale
  callback or result from an older goal can never become current,
* cancels the in-flight action on explicit request, and revokes the logical
  mission when the runtime leaves AUTONOMY or a new runtime epoch appears.

It does not authorize motion (hold-to-RUN) and does not touch the mux; those
belong to later staged work.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import math
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import ModeState, NavigationRequest, NavigationState


NAVIGATION_REQUEST_TOPIC = '/paddock/navigation_request'
NAVIGATION_STATE_TOPIC = '/paddock/navigation_state'
MODE_STATE_TOPIC = '/paddock/mode_state'
NAVIGATE_TO_POSE_ACTION = '/navigate_to_pose'
NAVIGATE_THROUGH_POSES_ACTION = '/navigate_through_poses'

MISSION_SINGLE_GOAL = 0
MISSION_ORDERED_POSES = 1
MAP_FRAME = 'map'

CANCEL_RETRY_INTERVAL_SEC = 0.5
MAX_CANCEL_ATTEMPTS = 5
RESULT_RETRY_INTERVAL_SEC = 0.5

NAV2_ERROR_NAMES = {
    0: 'NONE',
    1: 'UNKNOWN',
}

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class MissionState(IntEnum):
    """Mirror of runner_interfaces/NavigationState STATE_* constants."""

    IDLE = 0
    DISPATCHING = 1
    ACTIVE = 2
    CANCELING = 3
    SUCCEEDED = 4
    FAILED = 5
    CANCELED = 6


@dataclass(frozen=True)
class MissionPose:
    """One validated map-frame pose, independent of any ROS message type."""

    frame_id: str
    position: tuple
    orientation: tuple

    def is_valid(self, expected_frame: str = MAP_FRAME) -> bool:
        """Return whether every component is finite with a usable rotation."""
        values = tuple(self.position) + tuple(self.orientation)
        if len(self.position) != 3 or len(self.orientation) != 4:
            return False
        if not all(math.isfinite(value) for value in values):
            return False
        norm = math.sqrt(sum(value * value for value in self.orientation))
        if norm < 1e-6:
            return False
        return self.frame_id == expected_frame


@dataclass(frozen=True)
class Mission:
    """Immutable logical mission bound to one runtime epoch and map."""

    mission_id: str
    revision: int
    runtime_epoch: int
    map_id: str
    mission_type: int
    poses: tuple


@dataclass(frozen=True)
class SendGoal:
    """Command: send ``mission`` to Nav2 under ``generation``."""

    generation: int
    mission: Mission


@dataclass(frozen=True)
class CancelActiveGoal:
    """Command: cancel the live goal handle for ``generation``."""

    generation: int


@dataclass(frozen=True)
class CancelResidualGoals:
    """Command: best-effort cancel of any goal left on the servers at boot."""


@dataclass
class MissionRuntime:
    """ROS-free mission lifecycle with strict generation invalidation."""

    boot_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    runtime_epoch: int = 0
    map_id: str = ''
    state: MissionState = MissionState.IDLE
    mission: Mission | None = None
    mission_valid: bool = False
    action_generation: int = 0
    goal_uuid: str = ''
    nav2_status: int = -1
    error_code: int = 0
    error_meaning: str = ''
    detail: str = 'runtime started'
    _inflight: bool = False
    _cancel_requested: bool = False
    _autonomy_ok: bool | None = None
    _autonomy_key: tuple | None = None
    _max_revision: int = 0
    _residual_cleared: bool = False

    # -- observation ----------------------------------------------------------

    def observe_runtime(
        self,
        *,
        runtime_epoch: int,
        map_id: str,
        autonomy_ok: bool,
        now: float,
    ) -> tuple:
        """Follow ModeState; invalidate on epoch/map change or leaving AUTONOMY."""
        commands: list = []
        was_ok = self._autonomy_ok
        self._autonomy_ok = autonomy_ok
        if autonomy_ok:
            key = (int(runtime_epoch), map_id)
            if self._autonomy_key is not None and key != self._autonomy_key:
                commands += self._invalidate(
                    'runtime epoch or map changed', now
                )
            self._autonomy_key = key
            self.runtime_epoch = int(runtime_epoch)
            self.map_id = map_id
        elif was_ok:
            commands += self._invalidate('runtime left AUTONOMY', now)
        return tuple(commands)

    # -- authorized operations ---------------------------------------------

    def select(
        self,
        *,
        mission_id: str,
        mission_revision: int,
        runtime_epoch: int,
        map_id: str,
        mission_type: int,
        poses: list,
        now: float,
    ) -> tuple:
        """Bind a new validated logical mission; never dispatches."""
        if not mission_id:
            return self._reject('select rejected: empty mission_id')
        if mission_type not in (MISSION_SINGLE_GOAL, MISSION_ORDERED_POSES):
            return self._reject('select rejected: unknown mission_type')
        if not poses or not all(pose.is_valid() for pose in poses):
            return self._reject('select rejected: invalid map-frame pose(s)')
        if mission_type == MISSION_SINGLE_GOAL and len(poses) != 1:
            return self._reject(
                'select rejected: a single goal needs exactly one pose'
            )
        if self._autonomy_ok is not True:
            return self._reject('select rejected: runtime is not in AUTONOMY')
        if int(runtime_epoch) != self.runtime_epoch:
            return self._reject(
                'select rejected: mission epoch '
                f'{int(runtime_epoch)} != runtime epoch {self.runtime_epoch}'
            )
        if self.map_id and map_id != self.map_id:
            return self._reject('select rejected: mission map does not match')
        if int(mission_revision) <= self._max_revision:
            return self._reject('select rejected: stale mission_revision')

        self._max_revision = int(mission_revision)
        commands: list = []
        if self._inflight:
            commands += self._begin_cancel('superseded by a new mission', now)
        self.mission = Mission(
            mission_id=mission_id,
            revision=int(mission_revision),
            runtime_epoch=int(runtime_epoch),
            map_id=map_id,
            mission_type=int(mission_type),
            poses=tuple(poses),
        )
        self.mission_valid = True
        if not self._inflight:
            self.state = MissionState.IDLE
        self.detail = 'mission selected and validated'
        return tuple(commands)

    def dispatch(self, now: float) -> tuple:
        """Dispatch the currently selected mission to Nav2."""
        if not self.mission_valid or self.mission is None:
            return self._reject('dispatch rejected: no validated mission')
        if self._inflight or self.state == MissionState.CANCELING:
            return self._reject('dispatch rejected: an action is still in flight')
        if self._autonomy_ok is not True:
            return self._reject('dispatch rejected: runtime is not in AUTONOMY')
        if self.mission.runtime_epoch != self.runtime_epoch:
            self.mission = None
            self.mission_valid = False
            return self._reject('dispatch rejected: mission epoch superseded')

        self.action_generation += 1
        self._inflight = True
        self._cancel_requested = False
        self.goal_uuid = ''
        self.nav2_status = -1
        self.error_code = 0
        self.error_meaning = ''
        self.state = MissionState.DISPATCHING
        self.detail = 'dispatch requested; awaiting Nav2 acceptance'
        commands: list = []
        if not self._residual_cleared:
            self._residual_cleared = True
            commands.append(CancelResidualGoals())
        commands.append(SendGoal(self.action_generation, self.mission))
        return tuple(commands)

    def cancel(self, reason: str, now: float) -> tuple:
        """Explicit cancel; keeps the logical mission as a continuation candidate."""
        if self._inflight:
            return self._begin_cancel(reason or 'operator cancel', now)
        if self.state in (
            MissionState.DISPATCHING,
            MissionState.ACTIVE,
            MissionState.CANCELING,
        ):
            self.state = MissionState.IDLE
        self.detail = 'cancel requested; nothing in flight'
        return ()

    # -- asynchronous Nav2 callbacks -------------------------------------

    def on_goal_response(
        self, generation: int, accepted: bool, goal_uuid: str
    ) -> tuple:
        """Reduce a goal-acceptance callback, dropping stale generations."""
        if generation != self.action_generation or not self._inflight:
            return ()
        if not accepted:
            self._inflight = False
            self.state = MissionState.FAILED
            self.nav2_status = GoalStatus.STATUS_ABORTED
            self.error_meaning = 'Nav2 rejected the goal'
            self.detail = self.error_meaning
            return ()
        self.goal_uuid = goal_uuid
        if self._cancel_requested:
            self.state = MissionState.CANCELING
            return (CancelActiveGoal(generation),)
        self.state = MissionState.ACTIVE
        self.nav2_status = GoalStatus.STATUS_ACCEPTED
        self.detail = 'Nav2 accepted the goal; executing'
        return ()

    def on_feedback(self, generation: int) -> None:
        """Note real execution feedback for the current generation only."""
        if generation != self.action_generation or not self._inflight:
            return
        if self.state == MissionState.DISPATCHING:
            self.state = MissionState.ACTIVE
        if self.state == MissionState.ACTIVE:
            self.nav2_status = GoalStatus.STATUS_EXECUTING

    def on_result(
        self,
        generation: int,
        status: int,
        error_code: int,
        error_meaning: str,
    ) -> None:
        """Apply a terminal action result, dropping stale generations."""
        if generation != self.action_generation or not self._inflight:
            return
        self._inflight = False
        self._cancel_requested = False
        self.goal_uuid = ''
        self.nav2_status = int(status)
        self.error_code = int(error_code)
        self.error_meaning = error_meaning or NAV2_ERROR_NAMES.get(
            int(error_code), ''
        )
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = MissionState.SUCCEEDED
        elif status == GoalStatus.STATUS_CANCELED:
            self.state = MissionState.CANCELED
        else:
            self.state = MissionState.FAILED
        self.detail = (
            f'action terminal: {self.state.name} '
            f'({STATUS_NAMES.get(int(status), status)})'
        )

    def on_cancel_response(
        self, generation: int, goals_canceling: bool
    ) -> bool:
        """Return whether a cancel retry is still required for this generation."""
        if generation != self.action_generation or not self._inflight:
            return False
        if goals_canceling:
            self.detail = 'cancellation accepted by Nav2'
            return False
        self.detail = 'cancellation not yet accepted; retrying'
        return True

    def is_current(self, generation: int) -> bool:
        """Return whether ``generation`` is the live action attempt."""
        return generation == self.action_generation and self._inflight

    @property
    def inflight(self) -> bool:
        """Return whether a Nav2 goal is sent and not yet terminal."""
        return self._inflight

    @property
    def last_action_rejected(self) -> bool:
        """Return whether the most recent operation was rejected."""
        return 'rejected' in self.detail

    # -- helpers ----------------------------------------------------------

    def _begin_cancel(self, reason: str, now: float) -> list:
        self._cancel_requested = True
        self.state = MissionState.CANCELING
        self.detail = f'canceling: {reason}'
        if self.goal_uuid:
            return [CancelActiveGoal(self.action_generation)]
        return []

    def _invalidate(self, reason: str, now: float) -> list:
        commands: list = []
        self.mission_valid = False
        self.mission = None
        if self._inflight:
            commands += self._begin_cancel(reason, now)
        elif self.state in (
            MissionState.DISPATCHING,
            MissionState.ACTIVE,
            MissionState.CANCELING,
        ):
            self.state = MissionState.IDLE
        self.detail = f'mission invalidated: {reason}'
        return commands

    def _reject(self, reason: str) -> tuple:
        self.detail = reason
        return ()


def _pose_from_message(pose: PoseStamped) -> MissionPose:
    position = pose.pose.position
    orientation = pose.pose.orientation
    return MissionPose(
        frame_id=pose.header.frame_id,
        position=(position.x, position.y, position.z),
        orientation=(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ),
    )


def _pose_to_message(pose: MissionPose) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = pose.frame_id
    message.pose.position.x = float(pose.position[0])
    message.pose.position.y = float(pose.position[1])
    message.pose.position.z = float(pose.position[2])
    message.pose.orientation.x = float(pose.orientation[0])
    message.pose.orientation.y = float(pose.orientation[1])
    message.pose.orientation.z = float(pose.orientation[2])
    message.pose.orientation.w = float(pose.orientation[3])
    return message


class NavigationRuntimeNode(Node):
    """Sole Nav2 mission client: authorized intent in, truthful state out."""

    def __init__(self) -> None:
        super().__init__('runner_navigation_runtime')
        self._runtime = MissionRuntime()
        self._to_pose_client = ActionClient(
            self, NavigateToPose, NAVIGATE_TO_POSE_ACTION
        )
        self._through_poses_client = ActionClient(
            self, NavigateThroughPoses, NAVIGATE_THROUGH_POSES_ACTION
        )
        self._cancel_service_clients = {
            NAVIGATE_TO_POSE_ACTION: self.create_client(
                CancelGoal, f'{NAVIGATE_TO_POSE_ACTION}/_action/cancel_goal'
            ),
            NAVIGATE_THROUGH_POSES_ACTION: self.create_client(
                CancelGoal,
                f'{NAVIGATE_THROUGH_POSES_ACTION}/_action/cancel_goal',
            ),
        }

        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._pending_send = None
        self._cancel_attempts = 0
        self._next_cancel_at = 0.0
        self._next_result_at = 0.0

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(
            NavigationState, NAVIGATION_STATE_TOPIC, state_qos
        )
        self.create_subscription(
            NavigationRequest,
            NAVIGATION_REQUEST_TOPIC,
            self._on_request,
            10,
        )
        self.create_subscription(
            ModeState, MODE_STATE_TOPIC, self._on_mode_state, state_qos
        )
        self.create_timer(0.2, self._tick)
        self._publish_state()
        self.get_logger().info(
            'navigation runtime is the sole Nav2 mission owner; '
            f'boot_id={self._runtime.boot_id}'
        )

    # -- inputs ---------------------------------------------------------------

    def _on_mode_state(self, message: ModeState) -> None:
        autonomy_ok = (
            int(message.mode) == ModeState.MODE_AUTONOMY
            and int(message.status) == ModeState.STATUS_STABLE
        )
        commands = self._runtime.observe_runtime(
            runtime_epoch=int(message.runtime_epoch),
            map_id=message.active_autonomy_map,
            autonomy_ok=autonomy_ok,
            now=time.monotonic(),
        )
        self._run_commands(commands)
        self._publish_state()

    def _on_request(self, message: NavigationRequest) -> None:
        operation = int(message.operation)
        now = time.monotonic()
        if operation == NavigationRequest.OP_SELECT:
            commands = self._runtime.select(
                mission_id=message.mission_id,
                mission_revision=int(message.mission_revision),
                runtime_epoch=int(message.runtime_epoch),
                map_id=message.map_id,
                mission_type=int(message.mission_type),
                poses=[_pose_from_message(pose) for pose in message.poses],
                now=now,
            )
        elif operation == NavigationRequest.OP_DISPATCH:
            commands = self._runtime.dispatch(now)
        elif operation == NavigationRequest.OP_CANCEL:
            commands = self._runtime.cancel('authority cancel request', now)
        else:
            self.get_logger().warning(
                f'rejecting unknown navigation operation {operation}'
            )
            return
        if not commands and self._runtime.last_action_rejected:
            self.get_logger().warning(self._runtime.detail)
        self._run_commands(commands)
        self._publish_state()

    # -- command execution --------------------------------------------------

    def _run_commands(self, commands) -> None:
        for command in commands:
            if isinstance(command, SendGoal):
                self._pending_send = command
                self._pump_send()
            elif isinstance(command, CancelActiveGoal):
                self._cancel_attempts = 0
                self._next_cancel_at = 0.0
                self._pump_cancel()
            elif isinstance(command, CancelResidualGoals):
                self._cancel_residual_goals()
        if self._runtime.state == MissionState.CANCELING:
            # A dispatch that never reached the server is dropped here; its
            # generation is retired so a late acceptance cannot revive it.
            self._pending_send = None
            self._resolve_undelivered_cancel()

    def _resolve_undelivered_cancel(self) -> None:
        if (
            self._runtime.state == MissionState.CANCELING
            and self._runtime.inflight
            and self._pending_send is None
            and self._send_future is None
            and self._goal_handle is None
            and self._cancel_future is None
        ):
            self._runtime.on_result(
                self._runtime.action_generation,
                GoalStatus.STATUS_CANCELED,
                0,
                'canceled before Nav2 received the goal',
            )

    def _client_for(self, mission_type: int):
        if mission_type == MISSION_ORDERED_POSES:
            return self._through_poses_client, NAVIGATE_THROUGH_POSES_ACTION
        return self._to_pose_client, NAVIGATE_TO_POSE_ACTION

    def _pump_send(self) -> None:
        command = self._pending_send
        if command is None or not self._runtime.is_current(command.generation):
            self._pending_send = None
            return
        if self._runtime.state == MissionState.CANCELING:
            self._pending_send = None
            return
        if self._goal_handle is not None or self._send_future is not None:
            return
        client, _name = self._client_for(command.mission.mission_type)
        if not client.server_is_ready():
            self.get_logger().warning(
                'Nav2 action server not ready; holding latest dispatch'
            )
            return
        self._pending_send = None
        generation = command.generation
        if command.mission.mission_type == MISSION_ORDERED_POSES:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [
                _pose_to_message(pose) for pose in command.mission.poses
            ]
        else:
            goal = NavigateToPose.Goal()
            goal.pose = _pose_to_message(command.mission.poses[0])
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=lambda message, sent=generation:
                self._runtime.on_feedback(sent),
            )
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'send_goal_async raised: {error}')
            self._runtime.on_goal_response(generation, False, '')
            self._publish_state()
            return
        self._send_future = future
        future.add_done_callback(
            lambda done, sent=generation, expected=future:
            self._goal_response_callback(done, sent, expected)
        )

    def _goal_response_callback(self, future, generation, expected) -> None:
        if self._send_future is not expected:
            return
        self._send_future = None
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'goal response failed: {error}')
            self._runtime.on_goal_response(generation, False, '')
            self._publish_state()
            return
        accepted = bool(handle is not None and handle.accepted)
        goal_uuid = bytes(handle.goal_id.uuid).hex() if accepted else ''
        commands = self._runtime.on_goal_response(
            generation, accepted, goal_uuid
        )
        if accepted and self._runtime.is_current(generation):
            self._goal_handle = handle
        self._run_commands(commands)
        if (
            self._goal_handle is not None
            and self._result_future is None
            and self._runtime.is_current(generation)
        ):
            self._request_result(generation)
        self._publish_state()

    def _request_result(self, generation) -> None:
        handle = self._goal_handle
        if handle is None:
            return
        try:
            future = handle.get_result_async()
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'get_result_async raised: {error}')
            self._next_result_at = time.monotonic() + RESULT_RETRY_INTERVAL_SEC
            return
        self._result_future = future
        future.add_done_callback(
            lambda done, sent=generation, expected=future:
            self._result_callback(done, sent, expected)
        )

    def _result_callback(self, future, generation, expected) -> None:
        if self._result_future is not expected:
            return
        self._result_future = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'result request failed: {error}')
            if self._runtime.is_current(generation):
                self._next_result_at = (
                    time.monotonic() + RESULT_RETRY_INTERVAL_SEC
                )
            return
        result = response.result
        self._runtime.on_result(
            generation,
            int(response.status),
            int(getattr(result, 'error_code', 0)),
            str(getattr(result, 'error_msg', '')),
        )
        if not self._runtime.is_current(generation):
            self._goal_handle = None
        self._clear_action_futures()
        self._publish_state()

    def _pump_cancel(self) -> None:
        handle = self._goal_handle
        if handle is None or self._cancel_future is not None:
            return
        if self._cancel_attempts >= MAX_CANCEL_ATTEMPTS:
            self.get_logger().error(
                'cancellation attempts exhausted; motion stays revoked and '
                'replacement stays blocked until a terminal result'
            )
            return
        if time.monotonic() < self._next_cancel_at:
            return
        generation = self._runtime.action_generation
        self._cancel_attempts += 1
        try:
            future = handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'cancel_goal_async raised: {error}')
            self._next_cancel_at = time.monotonic() + CANCEL_RETRY_INTERVAL_SEC
            return
        self._cancel_future = future
        future.add_done_callback(
            lambda done, sent=generation, expected=future:
            self._cancel_callback(done, sent, expected)
        )

    def _cancel_callback(self, future, generation, expected) -> None:
        if self._cancel_future is not expected:
            return
        self._cancel_future = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - reported, not swallowed
            self.get_logger().error(f'cancel request failed: {error}')
            self._next_cancel_at = time.monotonic() + CANCEL_RETRY_INTERVAL_SEC
            return
        retry = self._runtime.on_cancel_response(
            generation, bool(response.goals_canceling)
        )
        if retry:
            self._next_cancel_at = time.monotonic() + CANCEL_RETRY_INTERVAL_SEC
        self._publish_state()

    def _cancel_residual_goals(self) -> None:
        info = CancelGoal.Request()
        for name, client in self._cancel_service_clients.items():
            if not client.service_is_ready():
                continue
            try:
                future = client.call_async(info)
            except Exception as error:  # noqa: BLE001 - best effort
                self.get_logger().warning(
                    f'residual cancel on {name} raised: {error}'
                )
                continue
            future.add_done_callback(
                lambda done, action=name:
                self.get_logger().info(
                    f'residual goal cancel dispatched on {action}'
                )
            )

    def _clear_action_futures(self) -> None:
        self._goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_attempts = 0
        self._next_cancel_at = 0.0
        self._next_result_at = 0.0

    # -- periodic ---------------------------------------------------------

    def _tick(self) -> None:
        if self._pending_send is not None:
            self._pump_send()
        if self._runtime.state == MissionState.CANCELING:
            self._pump_cancel()
            self._resolve_undelivered_cancel()
        if (
            self._goal_handle is not None
            and self._result_future is None
            and self._runtime.inflight
            and time.monotonic() >= self._next_result_at
        ):
            self._request_result(self._runtime.action_generation)
        self._publish_state()

    def _publish_state(self) -> None:
        runtime = self._runtime
        message = NavigationState()
        message.stamp = self.get_clock().now().to_msg()
        message.boot_id = runtime.boot_id
        message.state = int(runtime.state)
        message.mission_id = (
            runtime.mission.mission_id if runtime.mission else ''
        )
        message.mission_revision = (
            runtime.mission.revision if runtime.mission else 0
        )
        message.runtime_epoch = (
            runtime.mission.runtime_epoch
            if runtime.mission
            else runtime.runtime_epoch
        )
        message.map_id = runtime.mission.map_id if runtime.mission else ''
        message.mission_type = (
            runtime.mission.mission_type if runtime.mission else 0
        )
        message.mission_valid = bool(runtime.mission_valid)
        message.action_generation = runtime.action_generation
        message.goal_uuid = runtime.goal_uuid
        message.nav2_status = int(runtime.nav2_status)
        message.error_code = int(max(0, runtime.error_code))
        message.error_meaning = runtime.error_meaning
        message.detail = runtime.detail
        self._state_pub.publish(message)


def main(args=None) -> None:
    """Run the navigation runtime."""
    rclpy.init(args=args)
    node = None
    try:
        node = NavigationRuntimeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
