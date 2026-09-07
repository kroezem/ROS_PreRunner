"""Focused lifecycle and generation-invalidation tests for the runtime."""

from action_msgs.msg import GoalStatus
import pytest

from runner_bringup.navigation_runtime import (
    CancelActiveGoal,
    MISSION_SINGLE_GOAL,
    MissionPose,
    MissionRuntime,
    MissionState,
    SendGoal,
)


def _pose(x=1.0, y=2.0, yaw_w=1.0):
    return MissionPose(
        frame_id='map',
        position=(x, y, 0.0),
        orientation=(0.0, 0.0, 0.0, yaw_w),
    )


def _autonomy_runtime(epoch=3, map_id='studio'):
    runtime = MissionRuntime(boot_id='test-boot')
    runtime.observe_runtime(
        runtime_epoch=epoch, map_id=map_id, autonomy_ok=True, now=0.0
    )
    return runtime


def _select(runtime, revision=1, epoch=3, map_id='studio', mission_id='m1'):
    return runtime.select(
        mission_id=mission_id,
        mission_revision=revision,
        runtime_epoch=epoch,
        map_id=map_id,
        mission_type=MISSION_SINGLE_GOAL,
        poses=[_pose()],
        now=0.0,
    )


def test_pose_validation_rejects_nonfinite_and_zero_rotation():
    assert _pose().is_valid()
    assert not MissionPose('map', (float('nan'), 0.0, 0.0), (0, 0, 0, 1)).is_valid()
    assert not MissionPose('map', (0.0, 0.0, 0.0), (0, 0, 0, 0)).is_valid()
    assert not MissionPose('odom', (0.0, 0.0, 0.0), (0, 0, 0, 1)).is_valid()


def test_select_requires_autonomy_and_matching_epoch():
    idle = MissionRuntime()
    _select(idle)
    assert idle.mission is None and not idle.mission_valid

    runtime = _autonomy_runtime()
    _select(runtime, epoch=99)
    assert runtime.mission is None

    _select(runtime, epoch=3)
    assert runtime.mission_valid
    assert runtime.state == MissionState.IDLE


def test_dispatch_increments_generation_and_emits_send_goal():
    runtime = _autonomy_runtime()
    _select(runtime)
    commands = runtime.dispatch(now=0.0)

    send = [c for c in commands if isinstance(c, SendGoal)]
    assert send and send[0].generation == 1
    assert runtime.action_generation == 1
    assert runtime.state == MissionState.DISPATCHING
    assert runtime.inflight


def test_happy_path_reaches_active_then_succeeded():
    runtime = _autonomy_runtime()
    _select(runtime)
    runtime.dispatch(now=0.0)
    runtime.on_goal_response(1, accepted=True, goal_uuid='abcd')
    assert runtime.state == MissionState.ACTIVE
    assert runtime.goal_uuid == 'abcd'

    runtime.on_result(1, GoalStatus.STATUS_SUCCEEDED, 0, '')
    assert runtime.state == MissionState.SUCCEEDED
    assert not runtime.inflight
    # Logical mission is retained as a continuation candidate.
    assert runtime.mission_valid


def test_rejected_goal_is_failed_not_active():
    runtime = _autonomy_runtime()
    _select(runtime)
    runtime.dispatch(now=0.0)
    runtime.on_goal_response(1, accepted=False, goal_uuid='')
    assert runtime.state == MissionState.FAILED
    assert not runtime.inflight


def test_stale_result_from_older_generation_cannot_overwrite_current_state():
    runtime = _autonomy_runtime()
    _select(runtime)
    runtime.dispatch(now=0.0)
    runtime.on_goal_response(1, accepted=True, goal_uuid='g1')

    # A new mission supersedes and a fresh dispatch bumps the generation.
    _select(runtime, revision=2, mission_id='m2')
    runtime.on_result(1, GoalStatus.STATUS_CANCELED, 0, '')  # cancel of g1
    runtime.dispatch(now=1.0)
    assert runtime.action_generation == 2
    runtime.on_goal_response(2, accepted=True, goal_uuid='g2')
    assert runtime.state == MissionState.ACTIVE

    # Delayed SUCCEEDED result for the retired generation 1 must be ignored.
    runtime.on_result(1, GoalStatus.STATUS_SUCCEEDED, 0, '')
    assert runtime.state == MissionState.ACTIVE
    assert runtime.action_generation == 2

    # Delayed acceptance for generation 1 is also ignored.
    runtime.on_goal_response(1, accepted=True, goal_uuid='late')
    assert runtime.goal_uuid == 'g2'


def test_explicit_cancel_requests_cancel_and_retains_mission():
    runtime = _autonomy_runtime()
    _select(runtime)
    runtime.dispatch(now=0.0)
    runtime.on_goal_response(1, accepted=True, goal_uuid='g1')

    commands = runtime.cancel('operator cancel', now=1.0)
    assert any(isinstance(c, CancelActiveGoal) for c in commands)
    assert runtime.state == MissionState.CANCELING

    runtime.on_result(1, GoalStatus.STATUS_CANCELED, 0, '')
    assert runtime.state == MissionState.CANCELED
    assert runtime.mission_valid  # continuation candidate


def test_cancel_while_dispatching_before_acceptance_stays_canceling():
    runtime = _autonomy_runtime()
    _select(runtime)
    runtime.dispatch(now=0.0)
    # No goal handle yet: cancel cannot target a live goal, so it waits.
    commands = runtime.cancel('operator cancel', now=0.5)
    assert not commands
    assert runtime.state == MissionState.CANCELING
    assert runtime.inflight
    # The node finalizes an undelivered goal as CANCELED for this generation.
    runtime.on_result(1, GoalStatus.STATUS_CANCELED, 0, 'before dispatch')
    assert runtime.state == MissionState.CANCELED
    # A late acceptance for the retired generation is ignored.
    runtime.on_goal_response(1, accepted=True, goal_uuid='late')
    assert runtime.state == MissionState.CANCELED


def test_runtime_epoch_change_invalidates_mission_and_cancels_inflight():
    runtime = _autonomy_runtime(epoch=3)
    _select(runtime, epoch=3)
    runtime.dispatch(now=0.0)
    runtime.on_goal_response(1, accepted=True, goal_uuid='g1')

    commands = runtime.observe_runtime(
        runtime_epoch=4, map_id='studio', autonomy_ok=True, now=2.0
    )
    assert any(isinstance(c, CancelActiveGoal) for c in commands)
    assert not runtime.mission_valid
    assert runtime.mission is None
    assert runtime.state == MissionState.CANCELING


def test_leaving_autonomy_invalidates_mission():
    runtime = _autonomy_runtime()
    _select(runtime)
    assert runtime.mission_valid

    runtime.observe_runtime(
        runtime_epoch=3, map_id='studio', autonomy_ok=False, now=1.0
    )
    assert not runtime.mission_valid
    assert runtime.mission is None


def test_stale_mission_revision_cannot_rebind():
    runtime = _autonomy_runtime()
    _select(runtime, revision=5, mission_id='m5')
    _select(runtime, revision=3, mission_id='m3')
    assert runtime.mission.mission_id == 'm5'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
