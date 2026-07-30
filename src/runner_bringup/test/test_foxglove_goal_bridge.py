"""Focused tests for Foxglove simple-goal and route actions."""

import json
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy

from runner_bringup.foxglove_goal_bridge import (
    _Request,
    FoxgloveGoalBridge,
    NavigationMode,
    QueueState,
)
from runner_interfaces.msg import KeyboardState
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class FakeFuture:
    """Small controllable future for action-client tests."""

    def __init__(self):
        self._callbacks = []
        self._result = None
        self._exception = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def resolve(self, result):
        self._result = result
        for callback in list(self._callbacks):
            callback(self)

    def fail(self, error):
        self._exception = error
        for callback in list(self._callbacks):
            callback(self)


class FakeCancelResponse:
    """Cancellation response containing accepted goals, or none."""

    def __init__(self, accepted=True):
        self.goals_canceling = [object()] if accepted else []


class FakeResultResponse:
    """NavigateToPose result service response."""

    def __init__(
        self,
        status=GoalStatus.STATUS_SUCCEEDED,
        error_code=0,
        error_msg='',
    ):
        self.status = status
        self.result = NavigateToPose.Result()
        self.result.error_code = error_code
        self.result.error_msg = error_msg


class FakeGoalHandle:
    """Controllable accepted goal handle."""

    accepted = True

    def __init__(
        self,
        *,
        get_result_error=None,
        cancel_errors=None,
    ):
        self.cancel_calls = 0
        self.cancel_futures = []
        self.result_futures = []
        self.get_result_calls = 0
        self.get_result_error = get_result_error
        self.cancel_errors = list(cancel_errors or [])

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_errors:
            error = self.cancel_errors.pop(0)
            if error is not None:
                raise error
        future = FakeFuture()
        self.cancel_futures.append(future)
        return future

    def get_result_async(self):
        self.get_result_calls += 1
        if self.get_result_error is not None:
            error = self.get_result_error
            self.get_result_error = None
            raise error
        future = FakeFuture()
        self.result_futures.append(future)
        return future

    @property
    def result_future(self):
        return self.result_futures[-1]


class RejectedGoalHandle:
    """Rejected action goal response."""

    accepted = False


class FakeActionClient:
    """Record goals sent by the bridge."""

    def __init__(self):
        self.ready = True
        self.goals = []
        self.send_futures = []
        self.send_errors = []
        self.feedback_callbacks = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        if self.send_errors:
            error = self.send_errors.pop(0)
            if error is not None:
                raise error
        self.goals.append(goal)
        self.feedback_callbacks.append(feedback_callback)
        future = FakeFuture()
        self.send_futures.append(future)
        return future

    def send_feedback(self, index, poses_remaining):
        feedback = SimpleNamespace(
            number_of_poses_remaining=poses_remaining
        )
        self.feedback_callbacks[index](
            SimpleNamespace(feedback=feedback)
        )


class FakePublisher:
    """Record messages published by a bridge test."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def bridge(tmp_path):
    """Create a bridge with a fake action client."""
    rclpy.init()
    action_client = FakeActionClient()
    route_action_client = FakeActionClient()
    node = FoxgloveGoalBridge(
        action_client=action_client,
        route_action_client=route_action_client,
        route_file=tmp_path / 'route.json',
    )
    node.CANCEL_RETRY_INTERVAL = 0.0
    node.RESULT_RETRY_INTERVAL = 0.0
    yield node, action_client
    if not node._shutting_down:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def make_pose(x, y=0.0):
    """Create a map-frame test pose."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def sent_x(action_client):
    return [goal.pose.pose.position.x for goal in action_client.goals]


def accept_first(action_client, handle=None):
    handle = handle or FakeGoalHandle()
    action_client.send_futures[0].resolve(handle)
    return handle


def finish(handle, status=GoalStatus.STATUS_CANCELED):
    handle.result_future.resolve(FakeResultResponse(status))


def route_goal_xs(route_action_client):
    """Return x coordinates for every sent route."""
    return [
        [pose.pose.position.x for pose in goal.poses]
        for goal in route_action_client.goals
    ]


def enter_route(node):
    """Select persistent route mode through the public control callback."""
    node._route_control_callback(String(data='set_route_mode'))
    assert node.navigation_mode == NavigationMode.ROUTE


def test_single_pose_forwarding(bridge):
    node, action_client = bridge
    pose = make_pose(1.25, -0.5)

    node._goal_callback(pose)

    assert len(action_client.goals) == 1
    assert action_client.goals[0].pose == pose


def test_one_replacement_waits_for_terminal_result(bridge):
    node, action_client = bridge
    first_handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    first_handle.cancel_futures[0].resolve(FakeCancelResponse())

    assert sent_x(action_client) == [1.0]
    finish(first_handle)
    assert sent_x(action_client) == [1.0, 2.0]


def test_three_rapid_poses_while_first_send_response_pending(bridge):
    node, action_client = bridge

    node._goal_callback(make_pose(1.0))
    node._goal_callback(make_pose(2.0))
    node._goal_callback(make_pose(3.0))
    first_handle = FakeGoalHandle()
    action_client.send_futures[0].resolve(first_handle)
    first_handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(first_handle)

    assert sent_x(action_client) == [1.0, 3.0]


def test_three_rapid_poses_while_first_goal_active(bridge):
    node, action_client = bridge
    first_handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    node._goal_callback(make_pose(3.0))
    first_handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(first_handle)

    assert sent_x(action_client) == [1.0, 3.0]


def test_newer_pending_pose_replaces_older_pending_pose(bridge):
    node, action_client = bridge
    first_handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    node._goal_callback(make_pose(4.0))

    assert node._pending_request.poses[0].pose.position.x == 4.0
    first_handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(first_handle)
    assert sent_x(action_client) == [1.0, 4.0]


def test_synchronous_send_exception_is_retried_by_timer(bridge):
    node, action_client = bridge
    action_client.send_errors = [RuntimeError('sync send')]

    node._goal_callback(make_pose(1.0))

    assert node._pending_request.poses[0].pose.position.x == 1.0
    assert node._send_future is None
    node._pump()
    assert sent_x(action_client) == [1.0]


def test_asynchronous_send_future_exception_preserves_latest(bridge):
    node, action_client = bridge

    node._goal_callback(make_pose(1.0))
    node._goal_callback(make_pose(2.0))
    action_client.send_futures[0].fail(RuntimeError('async send'))

    assert sent_x(action_client) == [1.0, 2.0]


def test_rejected_goal_preserves_latest(bridge):
    node, action_client = bridge

    node._goal_callback(make_pose(1.0))
    node._goal_callback(make_pose(2.0))
    action_client.send_futures[0].resolve(RejectedGoalHandle())

    assert sent_x(action_client) == [1.0, 2.0]


def test_synchronous_get_result_exception_keeps_active_until_result(bridge):
    node, action_client = bridge
    handle = FakeGoalHandle(get_result_error=RuntimeError('result setup'))

    node._goal_callback(make_pose(1.0))
    node._goal_callback(make_pose(2.0))
    action_client.send_futures[0].resolve(handle)

    assert node._goal_handle is handle
    assert sent_x(action_client) == [1.0]
    node._pump()
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle)
    assert sent_x(action_client) == [1.0, 2.0]


def test_asynchronous_result_exception_keeps_active_until_result(bridge):
    node, action_client = bridge
    handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    handle.result_future.fail(RuntimeError('result future'))

    assert node._goal_handle is handle
    assert sent_x(action_client) == [1.0]
    node._pump()
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle)
    assert sent_x(action_client) == [1.0, 2.0]


def test_synchronous_cancel_exception_retries_without_wedging(bridge):
    node, action_client = bridge
    handle = FakeGoalHandle(cancel_errors=[RuntimeError('sync cancel')])
    accept_first_after_pose(node, action_client, 1.0, handle)

    node._goal_callback(make_pose(2.0))
    node._pump()

    assert handle.cancel_calls == 2
    assert sent_x(action_client) == [1.0]
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle)
    assert sent_x(action_client) == [1.0, 2.0]


def test_cancel_future_exception_retries_without_wedging(bridge):
    node, action_client = bridge
    handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    handle.cancel_futures[0].fail(RuntimeError('cancel future'))
    node._pump()

    assert handle.cancel_calls == 2
    handle.cancel_futures[1].resolve(FakeCancelResponse())
    finish(handle)
    assert sent_x(action_client) == [1.0, 2.0]


def test_cancellation_rejection_stops_after_bounded_attempts(bridge):
    node, action_client = bridge
    handle = accept_first_after_pose(node, action_client, 1.0)

    node._goal_callback(make_pose(2.0))
    for attempt in range(node.MAX_CANCEL_ATTEMPTS):
        handle.cancel_futures[attempt].resolve(FakeCancelResponse(False))
        node._pump()

    assert handle.cancel_calls == node.MAX_CANCEL_ATTEMPTS
    assert sent_x(action_client) == [1.0]
    finish(handle, GoalStatus.STATUS_SUCCEEDED)
    assert sent_x(action_client) == [1.0, 2.0]


def test_stale_send_callback_cannot_clear_newer_send_state(bridge):
    node, action_client = bridge
    node._goal_callback(make_pose(1.0))
    stale_future = action_client.send_futures[0]
    stale_generation = node._send_generation
    node._clear_send_state()
    node._replace_request(
        _Request('NavigateToPose', (make_pose(2.0),)),
        'test replacement',
    )
    current_future = node._send_future
    current_generation = node._send_generation

    node._goal_response_callback(
        stale_future,
        stale_generation,
        stale_future,
        _Request('NavigateToPose', (make_pose(1.0),)),
    )

    assert node._send_future is current_future
    assert node._send_generation == current_generation


def test_stale_result_callback_cannot_clear_newer_active_goal(bridge):
    node, action_client = bridge
    old_handle = accept_first_after_pose(node, action_client, 1.0)
    old_generation = node._active_generation
    node._clear_active_state()
    node._replace_request(
        _Request('NavigateToPose', (make_pose(2.0),)),
        'test replacement',
    )
    new_handle = FakeGoalHandle()
    action_client.send_futures[1].resolve(new_handle)
    new_generation = node._active_generation

    node._result_callback(
        old_handle.result_future,
        old_generation,
        old_handle,
        old_handle.result_future,
    )

    assert node._goal_handle is new_handle
    assert node._active_generation == new_generation


def test_shutdown_requests_cancel_and_exits_within_bound(bridge):
    node, action_client = bridge
    handle = accept_first_after_pose(node, action_client, 1.0)
    node._pending_request = _Request(
        'NavigateToPose', (make_pose(2.0),)
    )
    node._pending_generation = node.generation

    started = time.monotonic()
    node.shutdown(timeout_sec=0.05)
    elapsed = time.monotonic() - started

    assert handle.cancel_calls == 1
    # ROS entity teardown can add scheduler-dependent DDS cleanup latency;
    # this still proves the unresolved cancellation cannot hang shutdown.
    assert elapsed < 1.0
    assert sent_x(action_client) == [1.0]


def test_shutdown_retries_synchronous_cancel_failure(bridge):
    node, action_client = bridge
    handle = FakeGoalHandle(cancel_errors=[RuntimeError('sync cancel')])
    accept_first_after_pose(node, action_client, 1.0, handle)

    node.shutdown(timeout_sec=0.05)

    assert handle.cancel_calls == 2
    assert sent_x(action_client) == [1.0]


def test_shutdown_after_context_is_invalid_does_not_create_executor(bridge):
    """Launch SIGINT may invalidate the context before node cleanup runs."""
    node, _ = bridge
    rclpy.shutdown()

    node.shutdown()

    assert node._shutting_down is True


def test_route_accumulation_and_start_dispatch(bridge):
    node, _ = bridge
    route_client = node._route_action_client
    enter_route(node)

    node._waypoint_callback(make_pose(1.0))
    node._waypoint_callback(make_pose(2.0))
    node._route_control_callback(String(data='start'))

    assert [pose.pose.position.x for pose in node.route] == [1.0, 2.0]
    assert route_goal_xs(route_client) == [[1.0, 2.0]]


def test_route_control_clear_and_loop_commands_persist(bridge):
    node, _ = bridge
    enter_route(node)
    node._waypoint_callback(make_pose(1.0))

    node._route_control_callback(String(data='loop_on'))
    assert node.loop_enabled
    document = node._route_file.read_text()
    assert '"loop_enabled": true' in document

    node._route_control_callback(String(data='loop_off'))
    assert not node.loop_enabled
    node._route_control_callback(String(data='loop_toggle'))
    assert node.loop_enabled
    node._route_control_callback(String(data='remove_last'))
    assert node.route == ()
    node._waypoint_callback(make_pose(1.0))
    node._route_control_callback(String(data='clear'))
    assert node.route == ()
    assert '"poses": []' in node._route_file.read_text()


def test_route_markers_have_stable_ids_labels_and_vehicle_scale(bridge):
    node, _ = bridge
    enter_route(node)
    publisher = FakePublisher()
    node._marker_pub = publisher

    node._waypoint_callback(make_pose(1.0, 2.0))
    node._waypoint_callback(make_pose(3.0, 4.0))

    markers = publisher.messages[-1].markers
    assert markers[0].action == Marker.DELETEALL
    assert len(markers) == 5
    spheres = markers[1::2]
    labels = markers[2::2]
    assert [marker.id for marker in spheres] == [0, 2]
    assert [marker.id for marker in labels] == [1, 3]
    assert all(
        marker.ns == FoxgloveGoalBridge.ROUTE_MARKER_NAMESPACE
        for marker in markers[1:]
    )
    assert all(
        marker.header.frame_id == 'map'
        for marker in markers
    )
    assert all(marker.type == Marker.SPHERE for marker in spheres)
    assert all(
        marker.scale.x == pytest.approx(0.06)
        and marker.scale.y == pytest.approx(0.06)
        for marker in spheres
    )
    assert all(
        marker.type == Marker.TEXT_VIEW_FACING
        for marker in labels
    )
    assert [marker.text for marker in labels] == ['1', '2']
    assert all(
        marker.scale.z == pytest.approx(0.055)
        for marker in labels
    )


def test_route_marker_deletion_prevents_stale_waypoints(bridge):
    node, _ = bridge
    enter_route(node)
    publisher = FakePublisher()
    node._marker_pub = publisher
    node._waypoint_callback(make_pose(1.0))
    node._waypoint_callback(make_pose(2.0))

    node._route_control_callback(String(data='remove_last'))
    after_remove = publisher.messages[-1].markers
    assert after_remove[0].action == Marker.DELETEALL
    assert [marker.id for marker in after_remove[1:]] == [0, 1]

    node._route_control_callback(String(data='clear'))
    after_clear = publisher.messages[-1].markers
    assert len(after_clear) == 1
    assert after_clear[0].action == Marker.DELETEALL


def test_remove_last_rejects_empty_route_cleanly(bridge):
    node, _ = bridge
    enter_route(node)
    warnings = []
    node.get_logger().warning = warnings.append

    node._route_control_callback(String(data='remove_last'))

    assert node.route == ()
    assert warnings == ['Cannot remove last waypoint: route is empty']


def test_persistence_round_trip(tmp_path):
    route_file = tmp_path / 'route.json'
    rclpy.init()
    first = FoxgloveGoalBridge(
        action_client=FakeActionClient(),
        route_action_client=FakeActionClient(),
        route_file=route_file,
    )
    enter_route(first)
    first._waypoint_callback(make_pose(1.0, 2.0))
    first._waypoint_callback(make_pose(3.0, 4.0))
    first._route_control_callback(String(data='loop_on'))
    first.destroy_node()

    second = FoxgloveGoalBridge(
        action_client=FakeActionClient(),
        route_action_client=FakeActionClient(),
        route_file=route_file,
    )
    try:
        assert [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in second.route
        ] == [(1.0, 2.0), (3.0, 4.0)]
        assert second.loop_enabled
    finally:
        second.destroy_node()
        rclpy.shutdown()


def test_successful_loop_route_redispatches(bridge):
    node, _ = bridge
    route_client = node._route_action_client
    enter_route(node)
    node._waypoint_callback(make_pose(1.0))
    node._waypoint_callback(make_pose(2.0))
    node._route_control_callback(String(data='loop_on'))
    node._route_control_callback(String(data='start'))
    handle = FakeGoalHandle()
    route_client.send_futures[0].resolve(handle)

    finish(handle, GoalStatus.STATUS_SUCCEEDED)

    assert route_goal_xs(route_client) == [
        [1.0, 2.0],
        [1.0, 2.0],
    ]


def test_autonomy_state_reports_route_progress_and_nav2_error(bridge):
    node, _ = bridge
    route_client = node._route_action_client
    enter_route(node)
    state_pub = FakePublisher()
    node._autonomy_state_pub = state_pub
    node._waypoint_callback(make_pose(1.0))
    node._waypoint_callback(make_pose(2.0))
    node._route_control_callback(String(data='start'))
    handle = FakeGoalHandle()
    route_client.send_futures[0].resolve(handle)

    node._publish_autonomy_state()
    accepted = json.loads(state_pub.messages[-1].data)
    assert accepted['goal_state'] == 'accepted'
    assert accepted['route_length'] == 2
    assert accepted['current_waypoint_index'] == 0
    assert accepted['loop_mode'] is False
    assert accepted['global_obstacles'] == 'unknown'
    assert accepted['time_in_state_seconds'] >= 0.0

    route_client.send_feedback(0, poses_remaining=1)
    node._publish_autonomy_state()
    executing = json.loads(state_pub.messages[-1].data)
    assert executing['goal_state'] == 'executing'
    assert executing['current_waypoint_index'] == 1

    handle.result_future.resolve(FakeResultResponse(
        GoalStatus.STATUS_ABORTED,
        error_code=208,
        error_msg='planner failed',
    ))
    node._publish_autonomy_state()
    aborted = json.loads(state_pub.messages[-1].data)
    assert aborted['goal_state'] == 'aborted'
    assert aborted['last_error_code'] == 208
    assert aborted['last_error_meaning'] == 'NO_VALID_PATH'


@pytest.mark.parametrize(
    ('diagnostic', 'expected'),
    [
        (KeyboardState.GLOBAL_OBSTACLES_UNKNOWN, 'unknown'),
        (KeyboardState.GLOBAL_OBSTACLES_DISABLED, 'disabled'),
        (KeyboardState.GLOBAL_OBSTACLES_ENABLED, 'enabled'),
    ],
)
def test_autonomy_state_reports_global_obstacle_diagnostic(
    bridge,
    diagnostic,
    expected,
):
    node, _ = bridge
    state_pub = FakePublisher()
    node._autonomy_state_pub = state_pub

    node._keyboard_state_callback(KeyboardState(
        global_obstacles_state=diagnostic
    ))
    node._publish_autonomy_state()

    document = json.loads(state_pub.messages[-1].data)
    assert document['global_obstacles'] == expected
    assert document['goal_state'] == 'none'


def test_stop_prevents_loop_redispatch(bridge):
    node, _ = bridge
    route_client = node._route_action_client
    enter_route(node)
    node._waypoint_callback(make_pose(1.0))
    node._route_control_callback(String(data='loop_on'))
    node._route_control_callback(String(data='start'))
    handle = FakeGoalHandle()
    route_client.send_futures[0].resolve(handle)

    node._route_control_callback(String(data='stop'))
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle)

    assert len(route_client.goals) == 1
    assert not node._route_run_enabled


def test_malformed_waypoints_and_commands_are_rejected(bridge):
    node, _ = bridge
    enter_route(node)
    warnings = []
    node.get_logger().warning = warnings.append
    invalid = make_pose(1.0)
    invalid.header.frame_id = ''
    non_map = make_pose(2.0)
    non_map.header.frame_id = 'odom'

    node._waypoint_callback(invalid)
    node._waypoint_callback(non_map)
    node._route_control_callback(String(data=' START '))
    node._route_control_callback(String(data='unknown'))

    assert node.route == ()
    assert any('malformed' in warning for warning in warnings)
    assert any('outside the map frame' in warning for warning in warnings)
    assert sum('unknown route command' in warning for warning in warnings) == 2


def test_malformed_persisted_route_is_ignored(tmp_path):
    route_file = tmp_path / 'route.json'
    route_file.write_text('{"poses": "not-a-list"}')
    rclpy.init()
    node = FoxgloveGoalBridge(
        action_client=FakeActionClient(),
        route_action_client=FakeActionClient(),
        route_file=route_file,
    )
    try:
        assert node.route == ()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_bridge_defaults_to_empty_waypoint_mode(bridge):
    node, _ = bridge

    assert node.navigation_mode == NavigationMode.WAYPOINT
    assert node.waypoint_queue == ()
    assert node.queue_state == QueueState.IDLE
    assert node._queue_retry_count == 0


def test_waypoint_fifo_auto_starts_preserves_pose_and_runs_sequentially(
    bridge,
):
    node, action_client = bridge
    first = make_pose(1.0)
    first.pose.orientation.x = 0.1
    first.pose.orientation.y = -0.2
    first.pose.orientation.z = 0.3
    first.pose.orientation.w = 0.9
    second = make_pose(2.0)

    node._waypoint_callback(first)
    node._waypoint_callback(second)

    assert node.waypoint_queue == (first, second)
    assert node.queue_state == QueueState.RUNNING
    assert len(action_client.goals) == 1
    assert action_client.goals[0].pose == first
    assert action_client.goals[0].pose.pose.orientation == (
        first.pose.orientation
    )

    first_handle = accept_first(action_client)
    finish(first_handle, GoalStatus.STATUS_SUCCEEDED)

    assert node.waypoint_queue == (second,)
    assert sent_x(action_client) == [1.0, 2.0]
    second_handle = FakeGoalHandle()
    action_client.send_futures[1].resolve(second_handle)
    finish(second_handle, GoalStatus.STATUS_SUCCEEDED)

    assert node.waypoint_queue == ()
    assert node.queue_state == QueueState.IDLE


def test_waypoint_failure_retries_once_then_pauses_with_front(bridge):
    node, action_client = bridge
    pose = make_pose(4.0)
    node._waypoint_callback(pose)
    first_generation = node.generation
    first_handle = accept_first(action_client)

    first_handle.result_future.resolve(FakeResultResponse(
        GoalStatus.STATUS_ABORTED,
        error_code=208,
        error_msg='no path',
    ))

    assert sent_x(action_client) == [4.0, 4.0]
    assert node.generation > first_generation
    assert node.waypoint_queue == (pose,)
    assert node._queue_retry_count == 1
    retry_handle = FakeGoalHandle()
    action_client.send_futures[1].resolve(retry_handle)
    retry_handle.result_future.resolve(FakeResultResponse(
        GoalStatus.STATUS_ABORTED,
        error_code=208,
        error_msg='still no path',
    ))

    assert sent_x(action_client) == [4.0, 4.0]
    assert node.waypoint_queue == (pose,)
    assert node.queue_state == QueueState.PAUSED
    assert node._queue_retry_count == 1


def test_waypoint_stop_retains_front_and_resume_resets_retry(bridge):
    node, action_client = bridge
    pose = make_pose(5.0)
    node._waypoint_callback(pose)
    handle = accept_first(action_client)

    node._route_control_callback(String(data='stop'))
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)

    assert node.waypoint_queue == (pose,)
    assert node.queue_state == QueueState.PAUSED
    assert node._queue_retry_count == 0
    assert sent_x(action_client) == [5.0]

    node._queue_retry_count = 1
    node._route_control_callback(String(data='start'))

    assert sent_x(action_client) == [5.0, 5.0]
    assert node.queue_state == QueueState.RUNNING
    assert node._queue_retry_count == 0
    generation = node.generation
    node._route_control_callback(String(data='start'))
    assert node.generation == generation
    assert sent_x(action_client) == [5.0, 5.0]


def test_waypoint_clear_cancels_and_stale_result_cannot_restart(bridge):
    node, action_client = bridge
    node._waypoint_callback(make_pose(6.0))
    node._waypoint_callback(make_pose(7.0))
    handle = accept_first(action_client)

    node._route_control_callback(String(data='clear'))
    generation = node.generation
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)

    assert node.generation == generation
    assert node.waypoint_queue == ()
    assert node.queue_state == QueueState.IDLE
    assert sent_x(action_client) == [6.0]
    node._route_control_callback(String(data='clear'))
    assert node.generation == generation


def test_action_server_unavailability_retains_queue_until_pump(bridge):
    node, action_client = bridge
    action_client.ready = False

    node._waypoint_callback(make_pose(8.0))
    node._waypoint_callback(make_pose(9.0))

    assert sent_x(action_client) == []
    assert len(node.waypoint_queue) == 2
    assert node._pending_request.purpose == 'queue'

    action_client.ready = True
    node._pump()
    assert sent_x(action_client) == [8.0]
    assert len(node.waypoint_queue) == 2


def test_waypoint_queue_limit_rejects_only_excess(bridge):
    node, action_client = bridge
    action_client.ready = False
    warnings = []
    node.get_logger().warning = warnings.append

    for index in range(node.MAX_WAYPOINT_QUEUE + 1):
        node._waypoint_callback(make_pose(float(index)))

    assert len(node.waypoint_queue) == node.MAX_WAYPOINT_QUEUE
    assert [pose.pose.position.x for pose in node.waypoint_queue] == list(
        map(float, range(node.MAX_WAYPOINT_QUEUE))
    )
    assert sum('queue is full' in warning for warning in warnings) == 1


def test_waypoint_queue_is_never_written_to_route_file(bridge):
    node, _ = bridge
    node._waypoint_callback(make_pose(10.0))

    assert not node._route_file.exists()


def test_waypoint_queue_markers_show_orientation_state_and_delete_all(
    bridge,
):
    node, _ = bridge
    marker_pub = FakePublisher()
    node._queue_marker_pub = marker_pub
    pose = make_pose(11.0)
    pose.pose.orientation.z = 0.6
    pose.pose.orientation.w = 0.8

    node._waypoint_callback(pose)

    markers = marker_pub.messages[-1].markers
    assert markers[0].action == Marker.DELETEALL
    assert markers[1].type == Marker.ARROW
    assert markers[1].id == 0
    assert markers[1].pose.orientation == pose.pose.orientation
    assert markers[2].text == 'ACTIVE'
    node._route_control_callback(String(data='clear'))
    assert len(marker_pub.messages[-1].markers) == 1
    assert marker_pub.messages[-1].markers[0].action == Marker.DELETEALL


def test_same_mode_commands_are_generation_preserving_noops(bridge):
    node, action_client = bridge
    node._waypoint_callback(make_pose(12.0))
    handle = accept_first(action_client)
    node._queue_retry_count = 1
    generation = node.generation

    for _ in range(5):
        node._route_control_callback(String(data='set_waypoint_mode'))

    assert node.generation == generation
    assert node.waypoint_queue[0].pose.position.x == 12.0
    assert node._goal_handle is handle
    assert handle.cancel_calls == 0
    assert node._queue_retry_count == 1


def test_route_to_waypoint_transition_cancels_and_clears_once(bridge):
    node, _ = bridge
    route_client = node._route_action_client
    enter_route(node)
    node._waypoint_callback(make_pose(13.0))
    node._route_control_callback(String(data='loop_on'))
    node._route_control_callback(String(data='start'))
    handle = FakeGoalHandle()
    route_client.send_futures[0].resolve(handle)

    node._route_control_callback(String(data='set_waypoint_mode'))
    generation = node.generation
    assert node.navigation_mode == NavigationMode.WAYPOINT
    assert node.route == ()
    assert not node.loop_enabled
    assert handle.cancel_calls == 1
    node._route_control_callback(String(data='set_waypoint_mode'))
    assert node.generation == generation
    assert handle.cancel_calls == 1

    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)
    assert node._goal_handle is None
    assert route_goal_xs(route_client) == [[13.0]]


def test_waypoint_to_route_transition_cancels_queue_and_drops_stale(
    bridge,
):
    node, action_client = bridge
    node._waypoint_callback(make_pose(14.0))
    node._waypoint_callback(make_pose(15.0))
    handle = accept_first(action_client)

    node._route_control_callback(String(data='set_route_mode'))
    generation = node.generation
    assert node.navigation_mode == NavigationMode.ROUTE
    assert node.waypoint_queue == ()
    assert node.queue_state == QueueState.IDLE
    assert handle.cancel_calls == 1
    node._route_control_callback(String(data='set_route_mode'))
    assert node.generation == generation
    assert handle.cancel_calls == 1

    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)
    assert node._goal_handle is None
    assert sent_x(action_client) == [14.0]


def test_delayed_acceptance_after_clear_is_canceled_without_restart(bridge):
    node, action_client = bridge
    node._waypoint_callback(make_pose(16.0))
    node._route_control_callback(String(data='clear'))
    handle = FakeGoalHandle()

    action_client.send_futures[0].resolve(handle)

    assert handle.cancel_calls == 1
    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)
    assert node.waypoint_queue == ()
    assert len(action_client.goals) == 1


def test_delayed_cancel_completion_after_replacement_uses_latest_generation(
    bridge,
):
    node, action_client = bridge
    handle = accept_first_after_pose(node, action_client, 17.0)
    node._goal_callback(make_pose(18.0))
    old_cancel_generation = node.generation
    node._goal_callback(make_pose(19.0))

    handle.cancel_futures[0].resolve(FakeCancelResponse())
    finish(handle, GoalStatus.STATUS_CANCELED)

    assert node.generation > old_cancel_generation
    assert sent_x(action_client) == [17.0, 19.0]


def test_route_mode_preserves_exact_waypoint_quaternion(bridge):
    node, _ = bridge
    enter_route(node)
    pose = make_pose(20.0)
    pose.pose.orientation.x = -0.3
    pose.pose.orientation.y = 0.1
    pose.pose.orientation.z = 0.4
    pose.pose.orientation.w = 0.85

    node._waypoint_callback(pose)

    assert node.route[0].pose.orientation == pose.pose.orientation
    persisted = json.loads(node._route_file.read_text())
    assert persisted['poses'][0]['orientation'] == {
        'x': -0.3,
        'y': 0.1,
        'z': 0.4,
        'w': 0.85,
    }


def accept_first_after_pose(
    node,
    action_client,
    x,
    handle=None,
):
    node._goal_callback(make_pose(x))
    return accept_first(action_client, handle)
