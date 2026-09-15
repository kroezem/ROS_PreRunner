"""Focused static acceptance tests for the Stage 3 Nav2 composite."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
LAUNCH_PATH = PACKAGE / 'launch' / 'nav2.launch.py'
LOCALIZE_LAUNCH_PATH = PACKAGE / 'launch' / 'localize.launch.py'
PARAMS_PATH = PACKAGE / 'config' / 'nav2_params.yaml'
LOCALIZER_PARAMS_PATH = (
    PACKAGE / 'config' / 'localizer_params_online_async.yaml'
)
MAPPER_PARAMS_PATH = PACKAGE / 'config' / 'mapper_params_online_async.yaml'
SPEED_ENVELOPE_PATH = (
    PACKAGE.parent / 'runner_drive_adapter' / 'config' / 'speed_envelope.yaml'
)
BT_PATH = (
    PACKAGE / 'behavior_trees' / 'navigate_to_pose_forward_only.xml'
)
ROUTE_BT_PATH = (
    PACKAGE / 'behavior_trees'
    / 'navigate_through_poses_forward_only.xml'
)


def _params():
    params = yaml.safe_load(PARAMS_PATH.read_text())
    envelope = yaml.safe_load(SPEED_ENVELOPE_PATH.read_text())
    controller = params['controller_server']['ros__parameters']
    speed = envelope['controller_server']['ros__parameters']
    for key, value in speed.items():
        if key == 'FollowPath':
            controller[key].update(value)
        else:
            controller[key] = value
    return params


def test_controller_is_launched_once_and_lifecycle_managed():
    """One controller is launched and managed by the existing manager."""
    launch = LAUNCH_PATH.read_text()
    lifecycle_names = re.search(
        r"'node_names': \[(.*?)\],\n\s+'use_sim_time'",
        launch,
        re.DOTALL,
    ).group(1)

    assert launch.count("package='nav2_controller'") == 1
    assert launch.count("executable='controller_server'") == 1
    assert lifecycle_names.count("'controller_server',") == 1


def test_controller_output_has_only_the_physical_nav_topic():
    """The controller output is remapped away from normalized /cmd_vel."""
    launch = LAUNCH_PATH.read_text()

    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in launch
    assert "('/cmd_vel_nav', '/cmd_vel')" not in launch


def test_rpp_allows_reversing_and_uses_measured_speed_limits():
    """RPP uses the Jazzy class and Runner's measured forward domain."""
    controller = _params()['controller_server']['ros__parameters']
    rpp = controller['FollowPath']

    assert controller['controller_plugins'] == ['FollowPath']
    assert (
        rpp['plugin']
        == 'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert rpp['desired_linear_vel'] == 0.45
    assert rpp['min_approach_linear_velocity'] == 0.25
    assert rpp['allow_reversing'] is True
    assert controller['failure_tolerance'] == 2.0
    assert rpp['use_rotate_to_heading'] is False
    assert rpp['lookahead_dist'] == 0.40
    assert rpp['use_velocity_scaled_lookahead_dist'] is True
    assert rpp['min_lookahead_dist'] == 0.30
    assert rpp['max_lookahead_dist'] == 0.80
    assert rpp['lookahead_time'] == 1.0
    assert rpp['regulated_linear_scaling_min_radius'] == 0.75
    assert rpp['regulated_linear_scaling_min_speed'] == 0.30
    assert rpp['path_speed_profile_fallback'] == 0.25
    assert rpp['use_cost_regulated_linear_velocity_scaling'] is False
    assert rpp['inflation_cost_scaling_factor'] == 10.0
    assert rpp['cost_scaling_dist'] == 0.45
    assert rpp['use_collision_detection'] is True
    assert rpp['max_allowed_time_to_collision_up_to_carrot'] == 0.15
    assert controller['enable_stamped_cmd_vel'] is False


def test_planner_reserves_curvature_headroom_for_path_tracking():
    """Smac plans below the physical limit so RPP can correct tracking."""
    planner = _params()['planner_server']['ros__parameters']['GridBased']

    assert planner['minimum_turning_radius'] == 0.60
    planned_curvature = 1.0 / planner['minimum_turning_radius']
    physical_curvature = 2.1236
    assert planned_curvature == pytest.approx(1.6667, abs=0.0001)
    assert planned_curvature / physical_curvature < 0.79


def test_smac_smoothing_is_disabled_to_preserve_feasible_curvature():
    """The returned path retains the search path's curvature constraint."""
    planner = _params()['planner_server']['ros__parameters']['GridBased']

    assert planner['smooth_path'] is False
    assert 'smoother' not in planner


def test_planner_is_one_reverse_capable_hybrid_astar():
    """Smac directly returns both forward-only and reversing routes."""
    planner = _params()['planner_server']['ros__parameters']

    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == (
        'nav2_smac_planner::SmacPlannerHybrid'
    )
    assert planner['GridBased']['motion_model_for_search'] == 'REEDS_SHEPP'
    assert planner['GridBased']['reverse_penalty'] == 8.0
    assert planner['GridBased']['change_penalty'] == 20.0
    assert planner['GridBased']['cost_penalty'] == 2.0
    assert planner['GridBased']['analytic_expansion_max_length'] == 1.0


def test_goal_checker_requires_position_and_loose_final_heading():
    """Goal completion checks pose without latching an early XY crossing."""
    checker = _params()['controller_server']['ros__parameters'][
        'goal_checker'
    ]

    assert checker['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert checker['xy_goal_tolerance'] == 0.10
    assert checker['yaw_goal_tolerance'] == 0.5
    assert checker['stateful'] is False


def test_map_name_is_required_and_complete_bundle_is_validated():
    """Localization and navigation have no implicit or partial map."""
    for path in (LAUNCH_PATH, LOCALIZE_LAUNCH_PATH):
        launch = path.read_text()
        declaration = re.search(
            r"DeclareLaunchArgument\(\s*'map_name',(.*?)\n\s*\),",
            launch,
            re.DOTALL,
        ).group(1)

        assert 'DEFAULT_MAP_NAME' not in launch
        assert 'default_value' not in declaration
        assert "f'{map_file_name}.posegraph'" in launch
        assert "f'{map_file_name}.data'" in launch
        assert "static_yaml = f'{map_file_name}.yaml'" in launch
        assert "startswith('image:')" in launch


def test_local_costmap_uses_raw_scan_and_ratified_geometry():
    """The rolling controller costmap uses raw scan and exact footprint."""
    local = _params()['local_costmap']['local_costmap']['ros__parameters']
    obstacle = local['obstacle_layer']

    assert local['rolling_window'] is True
    assert local['width'] == 4
    assert local['height'] == 4
    assert local['resolution'] == 0.025
    assert local['update_frequency'] == 10.0
    # The higher threshold makes every update publish; it cannot publish more
    # often than the update loop (see the measured rationale in the yaml).
    assert local['publish_frequency'] == 2 * local['update_frequency']
    assert local['footprint'] == (
        '[[0.230, 0.0825], [0.230, -0.0825], '
        '[-0.060, -0.0825], [-0.060, 0.0825]]'
    )
    assert local['footprint_padding'] == 0.0
    assert obstacle['observation_sources'] == 'scan'
    assert obstacle['scan']['topic'] == '/scan'
    assert obstacle['scan']['data_type'] == 'LaserScan'
    assert obstacle['scan']['marking'] is True
    assert obstacle['scan']['clearing'] is True
    assert obstacle['combination_method'] == 1
    assert obstacle['scan']['min_obstacle_height'] == 0.0
    assert obstacle['scan']['max_obstacle_height'] == 2.0
    assert obstacle['scan']['obstacle_min_range'] == 0.05
    assert obstacle['scan']['obstacle_max_range'] == 5.0
    assert obstacle['scan']['raytrace_min_range'] == 0.0
    assert obstacle['scan']['raytrace_max_range'] == 6.0
    assert obstacle['scan']['expected_update_rate'] == 0.0
    assert obstacle['scan']['observation_persistence'] == 0.0
    assert obstacle['scan']['inf_is_valid'] is True
    assert local['inflation_layer']['inflation_radius'] == 0.45
    assert local['inflation_layer']['cost_scaling_factor'] == 10.0
    assert '/scan_slam' not in yaml.safe_dump(
        {'local_costmap': _params()['local_costmap']}
    )


def test_costmaps_share_live_evidence_semantics_and_static_authority():
    """Live evidence clears in both maps without erasing static occupancy."""
    local = _params()['local_costmap']['local_costmap']['ros__parameters']
    global_params = _params()['global_costmap']['global_costmap'][
        'ros__parameters'
    ]
    obstacle = global_params['obstacle_layer']
    inflation = global_params['inflation_layer']

    assert global_params['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]
    assert global_params['update_frequency'] == 3.0
    assert global_params['footprint'] == (
        '[[0.230, 0.0825], [0.230, -0.0825], '
        '[-0.060, -0.0825], [-0.060, 0.0825]]'
    )
    assert global_params['footprint_padding'] == 0.0
    assert obstacle['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
    assert obstacle['enabled'] is True
    assert obstacle['combination_method'] == 1
    assert obstacle['observation_sources'] == 'scan'
    assert obstacle['scan'] == {
        'topic': '/scan',
        'data_type': 'LaserScan',
        'clearing': True,
        'marking': True,
        'min_obstacle_height': 0.0,
        'max_obstacle_height': 2.0,
        'obstacle_min_range': 0.05,
        'obstacle_max_range': 5.0,
        'raytrace_min_range': 0.0,
        'raytrace_max_range': 6.0,
        'expected_update_rate': 0.0,
        'observation_persistence': 0.0,
        'inf_is_valid': True,
    }
    assert inflation['inflation_radius'] == 0.50
    assert inflation['cost_scaling_factor'] == 10.0

    semantic_fields = (
        'topic', 'data_type', 'clearing', 'marking',
        'min_obstacle_height', 'max_obstacle_height',
        'obstacle_min_range', 'obstacle_max_range',
        'raytrace_min_range', 'raytrace_max_range',
        'expected_update_rate', 'observation_persistence', 'inf_is_valid',
    )
    for field in semantic_fields:
        assert local['obstacle_layer']['scan'][field] == obstacle['scan'][field]


def test_costmap_geometry_satisfies_stage_a1_derived_invariants():
    """Both inflations cover the footprint and I11 covers Insane stopping."""
    params = _params()
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_params = params['global_costmap']['global_costmap'][
        'ros__parameters'
    ]
    circumscribed_radius = (0.230 ** 2 + 0.0825 ** 2) ** 0.5
    for costmap in (local, global_params):
        assert costmap['inflation_layer']['inflation_radius'] >= (
            circumscribed_radius
        )

    ceiling = 1.50
    braking_deceleration = 1.6 * ceiling + 0.27
    command_latency = 0.20
    stopping_distance = (
        ceiling ** 2 / (2.0 * braking_deceleration)
        + ceiling * command_latency
    )
    required_radius = stopping_distance + 0.80 + circumscribed_radius
    assert local['width'] / 2.0 >= required_radius
    assert local['height'] / 2.0 >= required_radius


def test_low_risk_nav2_wakeup_rates_are_reduced_without_timeout_changes():
    """Only the audited Nav2 wake rates change; bond timeout stays fixed."""
    params = _params()
    managed_nodes = (
        'map_server',
        'planner_server',
        'controller_server',
        'bt_navigator',
    )

    for node in managed_nodes:
        assert params[node]['ros__parameters']['bond_heartbeat_period'] == 1.0
    assert params['bt_navigator']['ros__parameters']['bt_loop_duration'] == 20
    assert "'bond_timeout': 4.0" in LAUNCH_PATH.read_text()


def test_slam_transform_publish_rate_is_ten_hz_with_existing_timeout():
    """Mapping and localization publish map to odom at the scan-match rate."""
    for path in (LOCALIZER_PARAMS_PATH, MAPPER_PARAMS_PATH):
        slam = yaml.safe_load(path.read_text())['slam_toolbox'][
            'ros__parameters'
        ]

        assert slam['transform_publish_period'] == 0.10
        assert slam['transform_timeout'] == 0.2


def test_behavior_tree_clears_global_costmap_once_on_planning_failure():
    """Controller patience is followed by one bounded forced replan."""
    root = ET.parse(BT_PATH).getroot()
    tags = [element.tag for element in root.iter()]
    startup = root.find('./BehaviorTree/Sequence')
    fallback = root.find('.//Fallback')
    pipeline = root.find('.//PipelineSequence')
    rate = pipeline.find('./RateController')
    recovery = rate.find('./RecoveryNode')
    controller_recovery = startup.find('./RecoveryNode')
    force_replan = controller_recovery.find('./Sequence')
    candidate = fallback.find(
        "./Sequence[@name='GenerateValidateAndCommitCandidate']"
    )
    planner = candidate.find('./ComputePathToPose')
    clear = recovery.find('./ClearEntireCostmap')
    follow = pipeline.find('./FollowPath')

    assert tags.count('ComputePathToPose') == 1
    assert tags.count('IsPathValid') == 0
    assert tags.count('FollowPath') == 1
    assert tags.count('PipelineSequence') == 1
    assert tags.count('Fallback') == 3
    assert tags.count('ReactiveFallback') == 0
    assert tags.count('GlobalUpdatedGoal') == 1
    assert tags.count('RateController') == 1
    assert tags.count('RecoveryNode') == 2
    assert tags.count('ClearEntireCostmap') == 1
    assert tags.count('UnsetBlackboard') == 2
    assert tags.count('PersistentPathValid') == 1
    assert tags.count('CandidatePathValid') == 1
    assert tags.count('GeneratePathSpeedProfile') == 1
    assert tags.count('CertifyCandidatePath') == 0
    assert tags.count('PathExists') == 1
    assert tags.count('ReportPathCommitment') == 2
    assert startup.attrib == {'name': 'StartWithFreshPath'}
    assert [child.tag for child in startup] == [
        'UnsetBlackboard', 'SetBlackboard', 'RecoveryNode',
    ]
    assert startup.find('./UnsetBlackboard').attrib == {'key': 'path'}
    assert startup.find('./SetBlackboard').attrib == {
        'value': 'initial_plan', 'output_key': 'replan_reason',
    }
    assert controller_recovery.attrib == {
        'number_of_retries': '1',
        'name': 'ReplanAfterControllerPatience',
    }
    assert [child.tag for child in controller_recovery] == [
        'PipelineSequence',
        'Sequence',
    ]
    assert [child.tag for child in force_replan] == [
        'WouldAControllerRecoveryHelp', 'SetBlackboard', 'UnsetBlackboard',
    ]
    assert force_replan.find('./WouldAControllerRecoveryHelp').attrib == {
        'error_code': '{follow_path_error_code}',
    }
    assert force_replan.find('./SetBlackboard').attrib == {
        'value': 'recovery_replan', 'output_key': 'replan_reason',
    }
    assert rate.attrib == {'hz': '3.0'}
    assert [child.tag for child in pipeline] == [
        'RateController',
        'FollowPath',
    ]
    assert [child.tag for child in rate] == ['RecoveryNode']
    assert recovery.attrib == {
        'number_of_retries': '1',
        'name': 'ClearGlobalCostmapOnPlanningFailure',
    }
    assert [child.tag for child in recovery] == [
        'Fallback',
        'ClearEntireCostmap',
    ]
    assert [child.tag for child in fallback] == [
        'ReactiveSequence', 'Sequence',
    ]
    assert fallback.attrib == {'name': 'ReplanWhenCommitmentAllows'}
    path_check = fallback.find(
        "./ReactiveSequence[@name='RetainCommittedPath']"
    )
    assert [child.tag for child in path_check] == [
        'PathExists', 'Fallback', 'PersistentPathValid',
    ]
    assert path_check.find('.//GlobalUpdatedGoal') is not None
    assert path_check.find(
        ".//SetBlackboard[@value='goal_update']"
    ).attrib['output_key'] == 'replan_reason'
    persistence = path_check.find('./PersistentPathValid')
    assert persistence.attrib == {
        'path': '{path}',
        'corridor_length': '1.25',
        'required_observations': '3',
        'max_progress_search_distance': '2.0',
        'robot_frame': 'base_link',
        'transform_tolerance': '0.3',
        'replan_reason': '{replan_reason}',
    }
    assert planner.attrib == {
        'goal': '{goal}',
        'path': '{candidate_path}',
        'planner_id': 'GridBased',
        'error_code_id': '{compute_path_error_code}',
    }
    assert follow.attrib == {
        'path': '{path}',
        'controller_id': 'FollowPath',
        'goal_checker_id': 'goal_checker',
        'error_code_id': '{follow_path_error_code}',
    }
    assert clear.attrib == {
        'name': 'ClearGlobalCostmap',
        'service_name': 'global_costmap/clear_entirely_global_costmap',
    }
    assert candidate.find(
        ".//CandidatePathValid[@path='{candidate_path}']"
    ) is not None
    assert candidate.find(
        ".//SetBlackboard[@value='{candidate_path}'][@output_key='path']"
    ) is not None
    assert rate.find('.//ComputePathToPose') is not None
    assert rate.find('.//FollowPath') is None
    assert 'Spin' not in tags
    assert 'BackUp' not in tags
    assert 'DriveOnHeading' not in tags
    assert 'Rotate' not in tags
    assert 'RotateToHeading' not in tags


def test_route_behavior_tree_clears_global_costmap_once_on_plan_failure():
    """Route control uses the same bounded, non-motion recovery."""
    root = ET.parse(ROUTE_BT_PATH).getroot()
    tags = [element.tag for element in root.iter()]
    startup = root.find('./BehaviorTree/Sequence')
    fallback = root.find('.//Fallback')
    pipeline = root.find('.//PipelineSequence')
    rate = pipeline.find('./RateController')
    recovery = rate.find('./RecoveryNode')
    controller_recovery = startup.find('./RecoveryNode')
    force_replan = controller_recovery.find('./Sequence')
    replan = fallback.find(
        "./Sequence[@name='GenerateValidateAndCommitCandidate']"
    )
    planner = replan.find('./ComputePathThroughPoses')
    clear = recovery.find('./ClearEntireCostmap')
    follow = pipeline.find('./FollowPath')

    assert tags.count('ComputePathThroughPoses') == 1
    assert tags.count('IsPathValid') == 0
    assert tags.count('RemovePassedGoals') == 1
    assert tags.count('FollowPath') == 1
    assert tags.count('PipelineSequence') == 1
    assert tags.count('Fallback') == 3
    assert tags.count('ReactiveFallback') == 0
    assert tags.count('GlobalUpdatedGoal') == 1
    assert tags.count('RateController') == 1
    assert tags.count('RecoveryNode') == 2
    assert tags.count('ClearEntireCostmap') == 1
    assert tags.count('UnsetBlackboard') == 2
    assert tags.count('PersistentPathValid') == 1
    assert tags.count('CandidatePathValid') == 1
    assert tags.count('GeneratePathSpeedProfile') == 1
    assert tags.count('CertifyCandidatePath') == 0
    assert tags.count('PathExists') == 1
    assert tags.count('ReportPathCommitment') == 2
    assert startup.attrib == {'name': 'StartWithFreshPath'}
    assert [child.tag for child in startup] == [
        'UnsetBlackboard', 'SetBlackboard', 'RecoveryNode',
    ]
    assert startup.find('./UnsetBlackboard').attrib == {'key': 'path'}
    assert startup.find('./SetBlackboard').attrib == {
        'value': 'initial_plan', 'output_key': 'replan_reason',
    }
    assert controller_recovery.attrib == {
        'number_of_retries': '1',
        'name': 'ReplanAfterControllerPatience',
    }
    assert [child.tag for child in controller_recovery] == [
        'PipelineSequence',
        'Sequence',
    ]
    assert [child.tag for child in force_replan] == [
        'WouldAControllerRecoveryHelp', 'SetBlackboard', 'UnsetBlackboard',
    ]
    assert force_replan.find('./WouldAControllerRecoveryHelp').attrib == {
        'error_code': '{follow_path_error_code}',
    }
    assert force_replan.find('./SetBlackboard').attrib == {
        'value': 'recovery_replan', 'output_key': 'replan_reason',
    }
    assert rate.attrib == {'hz': '3.0'}
    assert [child.tag for child in pipeline] == [
        'RateController',
        'FollowPath',
    ]
    assert [child.tag for child in rate] == ['RecoveryNode']
    assert recovery.attrib == {
        'number_of_retries': '1',
        'name': 'ClearGlobalCostmapOnPlanningFailure',
    }
    assert [child.tag for child in recovery] == [
        'Fallback',
        'ClearEntireCostmap',
    ]
    assert [child.tag for child in fallback] == [
        'ReactiveSequence', 'Sequence',
    ]
    assert fallback.attrib == {'name': 'ReplanRouteWhenCommitmentAllows'}
    path_check = fallback.find(
        "./ReactiveSequence[@name='RetainCommittedPath']"
    )
    assert [child.tag for child in path_check] == [
        'PathExists', 'Fallback', 'PersistentPathValid',
    ]
    assert path_check.find('.//GlobalUpdatedGoal') is not None
    assert path_check.find(
        ".//SetBlackboard[@value='goal_update']"
    ).attrib['output_key'] == 'replan_reason'
    assert [child.tag for child in replan] == [
        'RemovePassedGoals',
        'ComputePathThroughPoses',
        'Fallback',
    ]
    assert planner.attrib == {
        'goals': '{goals}',
        'path': '{candidate_path}',
        'planner_id': 'GridBased',
        'error_code_id': '{compute_path_error_code}',
    }
    assert follow.attrib == {
        'path': '{path}',
        'controller_id': 'FollowPath',
        'goal_checker_id': 'goal_checker',
        'error_code_id': '{follow_path_error_code}',
    }
    assert clear.attrib == {
        'name': 'ClearGlobalCostmap',
        'service_name': 'global_costmap/clear_entirely_global_costmap',
    }
    assert replan.find(
        ".//CandidatePathValid[@path='{candidate_path}']"
    ) is not None
    assert replan.find(
        ".//SetBlackboard[@value='{candidate_path}'][@output_key='path']"
    ) is not None
    assert rate.find('.//ComputePathThroughPoses') is not None
    assert rate.find('.//FollowPath') is None
    assert 'ComputePathToPose' not in tags
    assert 'Spin' not in tags
    assert 'BackUp' not in tags
    assert 'DriveOnHeading' not in tags
    assert 'Rotate' not in tags
    assert 'RotateToHeading' not in tags


@pytest.mark.parametrize('tree_path', (BT_PATH, ROUTE_BT_PATH))
def test_candidate_is_validated_before_it_can_replace_committed_path(
    tree_path,
):
    """A stale candidate fails closed; only a current-valid candidate commits."""
    root = ET.parse(tree_path).getroot()
    tags = [element.tag for element in root.iter()]
    validation = root.find(
        ".//Fallback[@name='ValidateCandidateAgainstCurrentGlobalCostmap']"
    )
    accept, reject = validation.findall('./Sequence')

    assert tags.count('CandidatePathValid') == 1
    assert tags.count('GeneratePathSpeedProfile') == 1
    assert 'CertifyCandidatePath' not in tags
    assert [child.tag for child in accept] == [
        'CandidatePathValid', 'GeneratePathSpeedProfile', 'SetBlackboard',
        'ReportPathCommitment',
    ]
    assert accept.find('./CandidatePathValid').attrib == {
        'path': '{candidate_path}',
        'global_frame': 'map',
        'robot_frame': 'base_link',
        'transform_tolerance': '0.3',
        'rejection_reason': '{candidate_rejection_reason}',
    }
    assert accept.find('./SetBlackboard').attrib == {
        'value': '{candidate_path}', 'output_key': 'path',
    }
    assert accept.find('./GeneratePathSpeedProfile').attrib == {
        'path': '{candidate_path}',
    }
    assert [child.tag for child in reject] == [
        'ReportPathCommitment', 'AlwaysFailure',
    ]
    assert reject.find('./ReportPathCommitment').attrib == {
        'event': 'candidate_rejected',
        'reason': '{candidate_rejection_reason}',
    }


def test_both_navigators_are_configured_with_explicit_trees():
    """Nav2 exposes both action servers with Runner's trees."""
    navigator = _params()['bt_navigator']['ros__parameters']
    launch = LAUNCH_PATH.read_text()

    assert navigator['navigators'] == [
        'navigate_to_pose',
        'navigate_through_poses',
    ]
    assert (
        navigator['navigate_through_poses']['plugin']
        == 'nav2_bt_navigator::NavigateThroughPosesNavigator'
    )
    assert 'default_nav_to_pose_bt_xml' in launch
    assert 'default_nav_through_poses_bt_xml' in launch
    assert navigator['plugin_lib_names'] == [
        'runner_nav2_behavior_tree_nodes',
    ]


def test_bt_navigator_owns_the_complete_d2_speed_policy():
    """Both BT engines consume one public policy owned by bt_navigator."""
    navigator = _params()['bt_navigator']['ros__parameters']
    expected = {
        'speed_policy.creep_speed': 0.25,
        'speed_policy.clearance_half_speed': 0.15,
        'speed_policy.curvature_window': 0.40,
        'speed_policy.max_lateral_acceleration': 0.35,
        'speed_policy.footprint_radius': 0.2444,
        'speed_policy.braking_linear': 1.6,
        'speed_policy.braking_constant': 0.27,
        'speed_policy.recovery_acceleration': 1.0,
    }
    assert {name: navigator[name] for name in expected} == expected

    source = (
        PACKAGE.parent / 'runner_nav2_behavior_tree' / 'src'
        / 'commitment_nodes.cpp'
    ).read_text()
    assert '"/bt_navigator/get_parameters"' in source
    assert 'declare_parameter' not in source


def test_stage2_topic_ownership_and_no_collision_monitor_remain():
    """Stage 3 does not bypass the adapter/mux or add Collision Monitor."""
    source_tree = PACKAGE.parent
    adapter = (
        source_tree
        / 'runner_drive_adapter'
        / 'runner_drive_adapter'
        / 'drive_adapter_node.py'
    ).read_text()
    mux = (PACKAGE / 'config' / 'twist_mux.yaml').read_text()
    stage3_text = '\n'.join(
        (
            LAUNCH_PATH.read_text(),
            PARAMS_PATH.read_text(),
            BT_PATH.read_text(),
        )
    )

    # v1.3 autonomy cutover: the adapter writes the *raw* converted command;
    # the command authority is the sole supervised writer of /cmd_vel_auto,
    # which is the one autonomy input on the existing mux (priority 50).
    assert "create_publisher(Twist, '/cmd_vel_auto_raw', 10)" in adapter
    assert "create_publisher(Twist, '/cmd_vel_auto', 10)" not in adapter
    assert 'topic: /cmd_vel_auto\n' in mux
    assert "('/cmd_vel_out', '/cmd_vel')" not in LAUNCH_PATH.read_text()
    assert 'collision_monitor' not in stage3_text.lower()
