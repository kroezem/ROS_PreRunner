"""Forward Foxglove simple goals to the Nav2 NavigateToPose action."""

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class FoxgloveGoalBridge(Node):
    """Forward only the latest Foxglove goal, serially, to Nav2."""

    CANCEL_RETRY_INTERVAL = 0.5
    MAX_CANCEL_ATTEMPTS = 3
    RESULT_RETRY_INTERVAL = 0.5
    SHUTDOWN_TIMEOUT = 1.0

    def __init__(self, action_client=None):
        super().__init__('foxglove_goal_bridge')
        self._action_client = action_client or ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        self._pending_pose = None
        self._next_generation = 1
        self._send_generation = None
        self._send_pose = None
        self._send_future = None
        self._active_generation = None
        self._goal_handle = None
        self._result_future = None
        self._next_result_attempt = 0.0
        self._cancel_future = None
        self._cancel_attempts = 0
        self._cancel_accepted = False
        self._next_cancel_attempt = 0.0
        self._cancel_exhausted_logged = False
        self._shutting_down = False
        self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self._goal_callback,
            10,
        )
        self.create_timer(0.5, self._pump)

    def _goal_callback(self, pose):
        if self._shutting_down:
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
        self._pending_pose = pose
        self._pump()

    def _pump(self):
        if self._shutting_down:
            if self._goal_handle is not None:
                self._ensure_result_request()
                self._request_cancel()
            return
        if self._goal_handle is not None:
            self._ensure_result_request()
            if self._pending_pose is not None:
                self._request_cancel()
            return
        if self._pending_pose is None or self._send_generation is not None:
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warning(
                'NavigateToPose action server is not ready; latest goal pending'
            )
            return

        pose = self._pending_pose
        self._pending_pose = None
        generation = self._next_generation
        self._next_generation += 1
        self._send_generation = generation
        self._send_pose = pose

        goal = NavigateToPose.Goal()
        goal.pose = pose
        try:
            future = self._action_client.send_goal_async(goal)
        except Exception as error:
            if self._send_generation == generation:
                self._clear_send_state()
                self._restore_pose_if_latest(pose)
            self.get_logger().error(
                f'NavigateToPose goal request raised synchronously: {error}'
            )
            return

        self._send_future = future
        future.add_done_callback(
            lambda completed, sent_generation=generation,
            sent_future=future, sent_pose=pose:
            self._goal_response_callback(
                completed,
                sent_generation,
                sent_future,
                sent_pose,
            )
        )

    def _goal_response_callback(
        self,
        future,
        generation,
        expected_future,
        pose,
    ):
        if (
            self._send_generation != generation
            or self._send_future is not expected_future
            or future is not expected_future
        ):
            self.get_logger().warning(
                f'Ignoring stale NavigateToPose send callback '
                f'for generation {generation}'
            )
            return

        self._clear_send_state()
        try:
            goal_handle = future.result()
        except Exception as error:
            self._restore_pose_if_latest(pose)
            self.get_logger().error(
                f'NavigateToPose goal request failed: {error}'
            )
            self._pump()
            return

        if not goal_handle.accepted:
            self._restore_pose_if_latest(pose)
            self.get_logger().warning('NavigateToPose goal rejected')
            self._pump()
            return

        self._active_generation = generation
        self._goal_handle = goal_handle
        self._reset_cancel_state()
        self.get_logger().info('NavigateToPose goal accepted')
        self._ensure_result_request()
        if self._shutting_down or self._pending_pose is not None:
            self._request_cancel(force=self._shutting_down)

    def _ensure_result_request(self):
        if (
            self._goal_handle is None
            or self._result_future is not None
            or time.monotonic() < self._next_result_attempt
        ):
            return
        generation = self._active_generation
        goal_handle = self._goal_handle
        try:
            future = goal_handle.get_result_async()
        except Exception as error:
            if self._active_matches(generation, goal_handle):
                self._next_result_attempt = (
                    time.monotonic() + self.RESULT_RETRY_INTERVAL
                )
            self.get_logger().error(
                'NavigateToPose result request raised synchronously; '
                f'the accepted goal remains active and will be retried: {error}'
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
        if not force:
            if self._cancel_attempts >= self.MAX_CANCEL_ATTEMPTS:
                if not self._cancel_exhausted_logged:
                    self.get_logger().error(
                        'NavigateToPose cancellation attempts exhausted; '
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
            'Canceling active NavigateToPose goal before replacement '
            f'(attempt {self._cancel_attempts})'
        )
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose cancel request raised synchronously: {error}'
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
                f'Ignoring stale NavigateToPose cancel callback '
                f'for generation {generation}'
            )
            return

        self._cancel_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose cancel request failed: {error}'
            )
            self._schedule_cancel_retry(generation, goal_handle)
            return

        if response.goals_canceling:
            self._cancel_accepted = True
            self.get_logger().info('NavigateToPose cancellation accepted')
        else:
            self.get_logger().warning(
                'NavigateToPose cancellation was not accepted'
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
                f'Ignoring stale NavigateToPose result callback '
                f'for generation {generation}'
            )
            return
        try:
            response = future.result()
            result = response.result
            status_name = STATUS_NAMES.get(
                response.status,
                f'UNKNOWN({response.status})',
            )
            self.get_logger().info(
                'NavigateToPose result: '
                f'status={status_name} '
                f'error_code={result.error_code} '
                f'error_msg={result.error_msg!r}'
            )
        except Exception as error:
            self.get_logger().error(
                'NavigateToPose result request failed; the accepted goal '
                f'remains active and will be retried: {error}'
            )
            if self._active_matches(generation, goal_handle):
                self._result_future = None
                self._next_result_attempt = (
                    time.monotonic() + self.RESULT_RETRY_INTERVAL
                )
                self._pump()
            return

        if self._active_matches(generation, goal_handle):
            self._clear_active_state()
            self._pump()

    def _active_matches(self, generation, goal_handle):
        return (
            self._active_generation == generation
            and self._goal_handle is goal_handle
        )

    def _restore_pose_if_latest(self, pose):
        if not self._shutting_down and self._pending_pose is None:
            self._pending_pose = pose

    def _clear_send_state(self):
        self._send_generation = None
        self._send_pose = None
        self._send_future = None

    def _reset_cancel_state(self):
        self._cancel_future = None
        self._cancel_attempts = 0
        self._cancel_accepted = False
        self._next_cancel_attempt = 0.0
        self._cancel_exhausted_logged = False

    def _clear_active_state(self):
        self._active_generation = None
        self._goal_handle = None
        self._result_future = None
        self._next_result_attempt = 0.0
        self._reset_cancel_state()

    def shutdown(self, timeout_sec=None):
        """Cancel an active goal and destroy the node within a fixed bound."""
        self._shutting_down = True
        self._pending_pose = None
        timeout = (
            self.SHUTDOWN_TIMEOUT if timeout_sec is None else timeout_sec
        )
        deadline = time.monotonic() + max(0.0, timeout)

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
