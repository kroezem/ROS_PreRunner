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

"""
Command authority: sole supervised writer of normal Paddock mux inputs.

v1.3 supervised command paths:

``Nav2 -> /cmd_vel_nav -> drive_adapter -> /cmd_vel_auto_raw ->
command_authority -> /cmd_vel_auto -> twist_mux -> /cmd_vel -> motor``

``browser -> control_event -> command_authority -> /paddock/manual_demand ->
drive_adapter -> /cmd_vel_paddock_manual_raw -> command_authority ->
/cmd_vel_paddock -> twist_mux -> /cmd_vel -> motor``

The drive adapter is the sole writer of ``/cmd_vel_auto_raw``. This node is
the sole writer of the supervised ``/cmd_vel_auto`` and forwards a raw sample
only while every current-state interlock holds (runtime AUTONOMY and
ready, matching epoch/map, fresh lease, STOP clear, no DualSense takeover /
unresolved rearm, RUN held, fresh raw input, mission lifecycle compatible).
On revoke it emits a bounded brake transition on ``/cmd_vel_auto`` and then
goes silent; the mux (autonomy priority 50) is always overridden by local
DualSense teleop (100) and the global STOP zero (255). It never writes
``/cmd_vel``. Manual uses the same controller and has mux priority 75, below
DualSense (100) and STOP (255), and above autonomy (50).
"""

import math
import time
import uuid

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import CommandAuthorityState
from runner_interfaces.msg import ConvertedCommand
from runner_interfaces.msg import LocalControlState
from runner_interfaces.msg import ManualDemand as ManualDemandMessage
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import NavigationRequest
from runner_interfaces.msg import NavigationState
from runner_interfaces.msg import PaddockControlEvent
from runner_interfaces.msg import PaddockControlLease
from runner_interfaces.msg import StopRequest
from runner_interfaces.msg import StopState

from runner_paddock.command_supervisor import CommandSupervisor
from runner_paddock.command_supervisor import ControlEvent
from runner_paddock.command_supervisor import SupervisorResult
from runner_paddock.command_supervisor import VelocityCommand
from runner_paddock.state_machine import Mode


RAW_AUTONOMY_TOPIC = '/cmd_vel_auto_raw'
SUPERVISED_AUTONOMY_TOPIC = '/cmd_vel_auto'
PADDOCK_OUTPUT_TOPIC = '/cmd_vel_paddock'
MANUAL_DEMAND_TOPIC = '/paddock/manual_demand'
RAW_MANUAL_TOPIC = '/cmd_vel_paddock_manual_raw'
# Bounded brake transition emitted on /cmd_vel_auto after a revoke, then the
# publisher goes silent. Covers one full mux autonomy input timeout (0.30 s)
# so the deliberate zero is arbitrated before the input ages out.
DEFAULT_AUTO_BRAKE_WINDOW_SEC = 0.30
CONTROL_EVENT_TOPIC = '/paddock/control_event'
AUTHORITY_STATE_TOPIC = '/paddock/command_authority_state'
MODE_STATE_TOPIC = '/paddock/mode_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'
LOCAL_CONTROL_TOPIC = '/teleop/control_state'
DEFAULT_SUPERVISION_PERIOD_SEC = 0.010
DEFAULT_LOCAL_CONTROL_TIMEOUT_SEC = 0.100
DEFAULT_STOP_STATE_TIMEOUT_SEC = 0.150
STOP_REQUEST_TOPIC = '/paddock/internal/stop_request'
STOP_STATE_TOPIC = '/paddock/stop_state'
NAVIGATION_REQUEST_TOPIC = '/paddock/navigation_request'
NAVIGATION_STATE_TOPIC = '/paddock/navigation_state'
DEFAULT_NAVIGATION_STATE_TIMEOUT_SEC = 1.0


class _AutonomyOutputGate:
    """Forward fresh samples once and brake only on an output revoke edge."""

    def __init__(self, brake_window_sec: float) -> None:
        self._brake_window_sec = brake_window_sec
        self._was_forwarding = False
        self._brake_deadline = None

    def update(
        self,
        *,
        now: float,
        permitted: bool,
        command: VelocityCommand | None,
    ) -> VelocityCommand | None:
        """Return one sample to publish, or None when output must be silent."""
        if permitted and command is not None:
            self._was_forwarding = True
            self._brake_deadline = None
            return command

        if permitted:
            # A supervision tick between fresh raw samples is not a revoke.
            # It must neither replay the last command nor inject a brake zero.
            self._brake_deadline = None
            return None

        if self._was_forwarding:
            self._was_forwarding = False
            self._brake_deadline = now + self._brake_window_sec

        if self._brake_deadline is None:
            return None
        if now <= self._brake_deadline:
            return VelocityCommand()

        self._brake_deadline = None
        return None


def _from_twist(message: Twist) -> VelocityCommand:
    return VelocityCommand(
        linear_x=message.linear.x,
        linear_y=message.linear.y,
        linear_z=message.linear.z,
        angular_x=message.angular.x,
        angular_y=message.angular.y,
        angular_z=message.angular.z,
    )


def _to_twist(command: VelocityCommand) -> Twist:
    message = Twist()
    message.linear.x = command.linear_x
    message.linear.y = command.linear_y
    message.linear.z = command.linear_z
    message.angular.x = command.angular_x
    message.angular.y = command.angular_y
    message.angular.z = command.angular_z
    return message


def _authority_message(
    result: SupervisorResult,
    stamp,
    stop_state: StopState | None = None,
    *,
    stop_fresh: bool = False,
    nav_action_active: bool | None = None,
) -> CommandAuthorityState:
    """Convert one pure snapshot to its typed authority interface."""
    snapshot = result.snapshot
    state = snapshot.state
    message = CommandAuthorityState()
    message.stamp = stamp
    message.authority = int(state.authority)
    message.client_id = state.lease_client_id
    message.lease_id = state.lease_id
    message.dualsense_active = state.dualsense_active
    message.run_held = state.run_held
    message.autonomy_permitted = state.autonomy_permitted
    message.autonomy_goal_selected = state.goal is not None
    message.goal_frame = 'map' if state.goal is not None else ''
    message.goal_map = state.goal.map_name if state.goal is not None else ''
    message.goal_x = state.goal.x if state.goal is not None else 0.0
    message.goal_y = state.goal.y if state.goal is not None else 0.0
    message.goal_yaw = state.goal.yaw if state.goal is not None else 0.0
    # Truthful action state comes from the navigation runtime's real Nav2
    # lifecycle, never the reducer's optimistic dispatch intent.
    message.autonomy_action_active = (
        state.navigation_active
        if nav_action_active is None
        else bool(nav_action_active)
    )
    message.brake_intent = snapshot.brake_intent
    message.lease_fresh = snapshot.lease_fresh
    message.lease_age_sec = (
        -1.0 if snapshot.lease_age_sec is None else snapshot.lease_age_sec
    )
    message.raw_autonomy_fresh = snapshot.raw_autonomy_fresh
    message.raw_autonomy_age_sec = (
        -1.0
        if snapshot.raw_autonomy_age_sec is None
        else snapshot.raw_autonomy_age_sec
    )
    message.manual_input_fresh = snapshot.manual_input_fresh
    message.manual_input_age_sec = (
        -1.0
        if snapshot.manual_input_age_sec is None
        else snapshot.manual_input_age_sec
    )
    message.manual_applied_speed_mps = snapshot.manual_applied_speed_mps
    message.manual_applied_steering = snapshot.manual_applied_steering
    message.last_control_sequence = (
        0
        if snapshot.last_control_sequence is None
        else snapshot.last_control_sequence
    )
    message.reason = snapshot.reason
    message.stop_state_fresh = stop_fresh
    message.stop_healthy = bool(stop_fresh and stop_state.healthy)
    message.stop_applied = bool(stop_fresh and stop_state.applied)
    message.stop_clear = bool(
        stop_fresh and stop_state.healthy
        and not stop_state.stopped and not stop_state.locked
    )
    message.stop_boot_id = stop_state.boot_id if stop_fresh else ''
    message.stop_generation = stop_state.generation if stop_fresh else 0
    message.stop_reason = (
        stop_state.reason if stop_fresh else 'STOP_STATE_STALE'
    )
    return message


class CommandAuthorityNode(Node):
    """Expose the pure supervisor without touching the live /cmd_vel topic."""

    def __init__(self) -> None:
        super().__init__('runner_command_authority')
        lease_timeout = float(
            self.declare_parameter('lease_timeout_sec', 0.150).value
        )
        # Forgiving backstop on bare lease ownership, kept well clear of the
        # RUN / autonomous-motion deadman above so ordinary browser/Wi-Fi
        # jitter no longer drops the operator's control lease mid-session.
        # The tight motion deadman is unchanged.
        control_liveness = float(
            self.declare_parameter('control_liveness_sec', 3.0).value
        )
        raw_timeout = float(
            self.declare_parameter(
                'raw_autonomy_timeout_sec', 0.150
            ).value
        )
        supervision_period = float(
            self.declare_parameter(
                'supervision_period_sec',
                DEFAULT_SUPERVISION_PERIOD_SEC,
            ).value
        )
        self._local_control_timeout = float(
            self.declare_parameter(
                'local_control_timeout_sec',
                DEFAULT_LOCAL_CONTROL_TIMEOUT_SEC,
            ).value
        )
        active_map = str(
            self.declare_parameter('active_autonomy_map', '').value
        )
        if not math.isfinite(supervision_period) or supervision_period <= 0:
            raise ValueError(
                'supervision_period_sec must be finite and positive'
            )
        if (
            not math.isfinite(self._local_control_timeout)
            or not 0 < self._local_control_timeout < 0.15
        ):
            raise ValueError(
                'local_control_timeout_sec must be finite and below mux timeout'
            )

        self._supervisor = CommandSupervisor(
            lease_timeout_sec=lease_timeout,
            raw_autonomy_timeout_sec=raw_timeout,
            control_liveness_sec=control_liveness,
            active_autonomy_map=active_map,
        )
        self._last_local_control_at = None
        self._local_control_identity = None
        self._dualsense_active = False
        self._requester_id = str(uuid.uuid4())
        self._stop_request_id = 0
        self._pending_stop_request = None
        self._last_stop_request_at = None
        self._stop_state = None
        self._stop_state_at = None
        self._stop_state_timeout = DEFAULT_STOP_STATE_TIMEOUT_SEC
        self._runtime_epoch = 0
        self._mission_id = ''
        self._mission_revision = 0
        self._nav_last_goal_key = None
        self._nav_last_dispatch_active = False
        self._nav_action_active = False
        self._nav_state_at = None
        self._nav_state_timeout = DEFAULT_NAVIGATION_STATE_TIMEOUT_SEC
        self._auto_brake_window = float(
            self.declare_parameter(
                'auto_brake_window_sec', DEFAULT_AUTO_BRAKE_WINDOW_SEC
            ).value
        )
        if (
            not math.isfinite(self._auto_brake_window)
            or self._auto_brake_window <= 0.0
        ):
            raise ValueError('auto_brake_window_sec must be finite and positive')
        self._auto_output_gate = _AutonomyOutputGate(
            self._auto_brake_window
        )
        self._manual_output_gate = _AutonomyOutputGate(
            self._auto_brake_window
        )
        self._manual_demand_asserted = False
        self._auto_pub = self.create_publisher(
            Twist, SUPERVISED_AUTONOMY_TOPIC, 10
        )
        self._paddock_pub = self.create_publisher(
            Twist, PADDOCK_OUTPUT_TOPIC, 10
        )
        self._manual_demand_pub = self.create_publisher(
            ManualDemandMessage, MANUAL_DEMAND_TOPIC, 10
        )
        self._authority_pub = self.create_publisher(
            CommandAuthorityState, AUTHORITY_STATE_TOPIC, 10
        )
        self._lease_pub = self.create_publisher(
            PaddockControlLease, LEASE_STATE_TOPIC, 10
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.lifespan = Duration(seconds=0.10)
        self._stop_request_pub = self.create_publisher(
            StopRequest, STOP_REQUEST_TOPIC, stop_qos
        )
        self._nav_request_pub = self.create_publisher(
            NavigationRequest, NAVIGATION_REQUEST_TOPIC, 10
        )
        self.create_subscription(
            Twist, RAW_AUTONOMY_TOPIC, self._on_raw_autonomy, 10
        )
        self.create_subscription(
            ConvertedCommand, RAW_MANUAL_TOPIC, self._on_raw_manual, 10
        )
        self.create_subscription(
            PaddockControlEvent,
            CONTROL_EVENT_TOPIC,
            self._on_control_event,
            10,
        )
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            ModeState, MODE_STATE_TOPIC, self._on_mode, mode_qos
        )
        self.create_subscription(
            LocalControlState,
            LOCAL_CONTROL_TOPIC,
            self._on_local_control,
            10,
        )
        self.create_subscription(
            StopState,
            STOP_STATE_TOPIC,
            self._on_stop_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        nav_state_qos = QoSProfile(depth=1)
        nav_state_qos.reliability = ReliabilityPolicy.RELIABLE
        nav_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            NavigationState,
            NAVIGATION_STATE_TOPIC,
            self._on_navigation_state,
            nav_state_qos,
        )
        self.create_timer(supervision_period, self._on_supervision_timer)

        self._apply(self._supervisor.restart(time.monotonic()))
        self.get_logger().info(
            'Command authority supervised autonomy path: '
            f'{RAW_AUTONOMY_TOPIC} -> supervise -> {SUPERVISED_AUTONOMY_TOPIC} '
            '-> twist_mux (autonomy priority 50); '
            f'bounded brake window {self._auto_brake_window:.3f} s then silent; '
            'never publishes /cmd_vel'
        )

    def _on_raw_autonomy(self, message: Twist) -> None:
        result = self._supervisor.receive_raw_autonomy(
            _from_twist(message), time.monotonic()
        )
        self._apply(result)

    def _on_control_event(self, message: PaddockControlEvent) -> None:
        try:
            event = ControlEvent(message.event)
        except ValueError:
            self.get_logger().warning(
                f'Rejected unknown Paddock control event {message.event}'
            )
            return
        result = self._supervisor.handle_control_event(
            event=event,
            client_id=message.client_id,
            lease_id=message.lease_id,
            sequence=message.sequence,
            now=time.monotonic(),
            goal_x=message.goal_x,
            goal_y=message.goal_y,
            goal_yaw=message.goal_yaw,
            manual_speed_mps=message.manual_speed_mps,
            manual_steering=message.manual_steering,
        )
        if not result.accepted:
            self.get_logger().warning(result.snapshot.reason)
        elif event == ControlEvent.STOP:
            self._request_stop(True)
        elif event == ControlEvent.CLEAR_STOP:
            if not self._stop_is_fresh(time.monotonic()):
                self.get_logger().warning('CLEAR_STOP_STATE_STALE')
            elif not self._stop_state.healthy:
                self.get_logger().warning('CLEAR_STOP_ENFORCER_UNHEALTHY')
            elif not self._stop_state.stopped or not self._stop_state.applied:
                self.get_logger().warning('CLEAR_STOP_NOT_APPLIED')
            else:
                self._request_stop(False)
        self._apply(result)

    def _on_raw_manual(self, message: ConvertedCommand) -> None:
        """Validate adapter provenance before supervising manual output."""
        if (
            message.source != ConvertedCommand.SOURCE_PADDOCK_MANUAL
            or int(message.runtime_epoch) != self._runtime_epoch
            or int(message.lease_generation)
            != self._supervisor.state.lease_generation
        ):
            return
        result = self._supervisor.receive_raw_manual(
            _from_twist(message.command),
            int(message.sequence),
            time.monotonic(),
        )
        self._apply(result)

    def _stop_is_fresh(self, now: float) -> bool:
        return (
            self._stop_state is not None
            and self._stop_state_at is not None
            and now - self._stop_state_at <= self._stop_state_timeout
        )

    def _stop_clear(self, now: float) -> bool:
        return (
            self._stop_is_fresh(now)
            and self._stop_state.healthy
            and not self._stop_state.stopped
            and not self._stop_state.locked
        )

    def _request_stop(self, stopped: bool) -> None:
        self._stop_request_id += 1
        request = StopRequest()
        request.requester_id = self._requester_id
        request.request_id = self._stop_request_id
        request.stopped = stopped
        if not stopped:
            request.expected_boot_id = self._stop_state.boot_id
            request.expected_generation = self._stop_state.generation
        self._pending_stop_request = request
        self._publish_stop_request(time.monotonic())

    def _publish_stop_request(self, now: float) -> None:
        self._stop_request_pub.publish(self._pending_stop_request)
        self._last_stop_request_at = now

    def _on_stop_state(self, message: StopState) -> None:
        self._stop_state = message
        self._stop_state_at = time.monotonic()
        request = self._pending_stop_request
        if (
            request is None
            or message.last_requester_id != request.requester_id
            or message.last_request_id != request.request_id
        ):
            return
        if not message.last_request_accepted:
            self.get_logger().warning(message.last_request_reason)
            self._pending_stop_request = None
        elif request.stopped and message.applied:
            self._pending_stop_request = None
        elif (
            not request.stopped
            and message.healthy
            and not message.stopped
            and not message.locked
        ):
            self._pending_stop_request = None

    def _on_mode(self, message: ModeState) -> None:
        try:
            mode = Mode(message.mode)
        except ValueError:
            self.get_logger().warning(
                f'Rejected unknown Paddock mode {message.mode}'
            )
            return
        self._runtime_epoch = int(message.runtime_epoch)
        # "Runtime actually AUTONOMY and ready": a STABLE-but-not-ready runtime
        # is treated as not eligible for remote motion (fail closed). The
        # readiness reason is surfaced to the UI separately via ModeState.
        runtime_ready = (
            message.status == ModeState.STATUS_STABLE and bool(message.ready)
        )
        result = self._supervisor.set_runtime_mode(
            mode=mode,
            active_autonomy_map=message.active_autonomy_map,
            now=time.monotonic(),
            runtime_stable=runtime_ready,
        )
        self._apply(result)

    def _on_local_control(self, message: LocalControlState) -> None:
        now = time.monotonic()
        self._last_local_control_at = now
        identity = (message.process_epoch, message.takeover_epoch)
        if identity != self._local_control_identity:
            self._local_control_identity = identity
            # A process/takeover epoch edge must revoke remote grants even if
            # engagement and release occurred between two status samples.
            if not self._dualsense_active:
                self._dualsense_active = True
                self._apply(
                    self._supervisor.set_dualsense_active(True, now)
                )
        active = message.active
        if active == self._dualsense_active:
            return
        self._dualsense_active = active
        self._apply(self._supervisor.set_dualsense_active(active, now))

    def _on_supervision_timer(self) -> None:
        now = time.monotonic()
        if (
            self._pending_stop_request is not None
            and (
                self._last_stop_request_at is None
                or now - self._last_stop_request_at >= 0.05
            )
        ):
            self._publish_stop_request(now)
        if (
            not self._dualsense_active
            and (
                self._last_local_control_at is None
                or now - self._last_local_control_at
                > self._local_control_timeout
            )
        ):
            # Missing local status closes remote grants before the 150 ms mux
            # input can expire and expose a lower-priority command.
            self._dualsense_active = True
            self._apply(
                self._supervisor.set_dualsense_active(True, now)
            )
        self._apply(self._supervisor.tick(now))

    def _on_navigation_state(self, message: NavigationState) -> None:
        self._nav_state_at = time.monotonic()
        self._nav_action_active = (
            message.state == NavigationState.STATE_ACTIVE
        )

    def _navigation_state_fresh(self, now: float) -> bool:
        return (
            self._nav_state_at is not None
            and now - self._nav_state_at <= self._nav_state_timeout
        )

    def _sync_navigation(self, snapshot) -> None:
        """Translate accepted authority state into navigation-runtime intent."""
        state = snapshot.state
        goal = state.goal
        goal_key = (
            None if goal is None
            else (goal.map_name, goal.x, goal.y, goal.yaw)
        )
        if goal_key != self._nav_last_goal_key:
            if goal_key is not None:
                self._mission_revision += 1
                self._mission_id = uuid.uuid4().hex
                self._publish_nav_request(
                    NavigationRequest.OP_SELECT, state, goal
                )
            else:
                self._publish_nav_request(
                    NavigationRequest.OP_CANCEL, state, None
                )
            self._nav_last_goal_key = goal_key

        dispatch_active = state.navigation_active
        if dispatch_active and not self._nav_last_dispatch_active:
            self._publish_nav_request(
                NavigationRequest.OP_DISPATCH, state, goal
            )
        elif not dispatch_active and self._nav_last_dispatch_active:
            self._publish_nav_request(
                NavigationRequest.OP_CANCEL, state, None
            )
        self._nav_last_dispatch_active = dispatch_active

    def _publish_nav_request(self, operation, state, goal) -> None:
        message = NavigationRequest()
        message.stamp = self.get_clock().now().to_msg()
        message.lease_id = state.lease_id
        message.operation = int(operation)
        message.mission_id = self._mission_id
        message.mission_revision = self._mission_revision
        message.runtime_epoch = self._runtime_epoch
        message.map_id = state.active_autonomy_map
        message.mission_type = NavigationRequest.MISSION_SINGLE_GOAL
        if operation == NavigationRequest.OP_SELECT and goal is not None:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = message.stamp
            pose.pose.position.x = float(goal.x)
            pose.pose.position.y = float(goal.y)
            pose.pose.orientation.z = math.sin(float(goal.yaw) / 2.0)
            pose.pose.orientation.w = math.cos(float(goal.yaw) / 2.0)
            message.poses = [pose]
        self._nav_request_pub.publish(message)

    def _apply(self, result: SupervisorResult) -> None:
        now = time.monotonic()
        demand = result.manual_demand
        if demand is not None:
            message = ManualDemandMessage()
            message.stamp = self.get_clock().now().to_msg()
            message.active = True
            message.sequence = demand.sequence
            message.lease_generation = (
                result.snapshot.state.lease_generation
            )
            message.runtime_epoch = self._runtime_epoch
            message.signed_speed_mps = demand.signed_speed_mps
            message.steering_normalized = demand.steering_normalized
            self._manual_demand_pub.publish(message)
            self._manual_demand_asserted = True
        elif (
            self._manual_demand_asserted
            and not result.snapshot.state.manual_active
        ):
            message = ManualDemandMessage()
            message.stamp = self.get_clock().now().to_msg()
            message.active = False
            message.lease_generation = (
                result.snapshot.state.lease_generation
            )
            message.runtime_epoch = self._runtime_epoch
            self._manual_demand_pub.publish(message)
            self._manual_demand_asserted = False
        # Mission lifecycle must be compatible with motion: a real, current,
        # accepted/executing Nav2 action. DISPATCHING / CANCELING / terminal
        # states, or a stale navigation_state, forbid autonomous output even
        # if the supervisor's interlock is otherwise satisfied.
        nav_motion_ok = (
            self._nav_action_active and self._navigation_state_fresh(now)
        )
        permitted = (
            not result.snapshot.brake_intent
            and self._stop_clear(now)
            and nav_motion_ok
        )
        auto_output = self._auto_output_gate.update(
            now=now,
            permitted=permitted,
            command=result.autonomy_command,
        )
        if auto_output is not None:
            self._auto_pub.publish(_to_twist(auto_output))
        manual_permitted = (
            self._stop_clear(now)
            and result.snapshot.state.authority
            == CommandAuthorityState.AUTHORITY_PADDOCK_MANUAL
            and result.snapshot.manual_input_fresh
        )
        manual_output = self._manual_output_gate.update(
            now=now,
            permitted=manual_permitted,
            command=result.manual_command,
        )
        if manual_output is not None:
            self._paddock_pub.publish(_to_twist(manual_output))
        self._sync_navigation(result.snapshot)
        self._publish_state(result)

    def _publish_state(self, result: SupervisorResult) -> None:
        snapshot = result.snapshot
        state = snapshot.state
        stamp = self.get_clock().now().to_msg()

        now = time.monotonic()
        self._authority_pub.publish(_authority_message(
            result,
            stamp,
            self._stop_state,
            stop_fresh=self._stop_is_fresh(now),
            nav_action_active=(
                self._nav_action_active
                if self._navigation_state_fresh(now)
                else False
            ),
        ))

        lease = PaddockControlLease()
        lease.stamp = stamp
        lease.active = state.lease_active
        lease.client_id = state.lease_client_id
        lease.lease_id = state.lease_id
        lease.generation = state.lease_generation
        self._lease_pub.publish(lease)


def main(args=None) -> None:
    """Run the offline command-authority node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommandAuthorityNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
