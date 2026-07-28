"""Focused static acceptance tests for the Stage 3 Nav2 composite."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]
WORKSPACE = PACKAGE.parents[1]
LAUNCH_PATH = PACKAGE / 'launch' / 'nav2.launch.py'
PARAMS_PATH = PACKAGE / 'config' / 'nav2_params.yaml'
BT_PATH = (
    PACKAGE / 'behavior_trees' / 'navigate_to_pose_forward_only.xml'
)
ROUTE_BT_PATH = (
    PACKAGE / 'behavior_trees'
    / 'navigate_through_poses_forward_only.xml'
)


def _params():
    return yaml.safe_load(PARAMS_PATH.read_text())


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


def test_rpp_is_forward_only_and_uses_measured_speed_limits():
    """RPP uses the Jazzy class and Runner's measured forward domain."""
    controller = _params()['controller_server']['ros__parameters']
    rpp = controller['FollowPath']

    assert controller['controller_plugins'] == ['FollowPath']
    assert (
        rpp['plugin']
        == 'nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController'
    )
    assert rpp['desired_linear_vel'] == 0.290
    assert rpp['min_approach_linear_velocity'] == 0.126
    assert rpp['allow_reversing'] is False
    assert rpp['use_rotate_to_heading'] is False
    assert rpp['lookahead_dist'] == 0.40
    assert rpp['use_velocity_scaled_lookahead_dist'] is False
    assert rpp['regulated_linear_scaling_min_radius'] == 0.80
    assert rpp['regulated_linear_scaling_min_speed'] == 0.126
    assert rpp['use_collision_detection'] is True
    assert rpp['max_allowed_time_to_collision_up_to_carrot'] == 0.5
    assert controller['enable_stamped_cmd_vel'] is False


def test_planner_uses_measured_base_link_turning_radius():
    """Smac uses the measured base-link radius, not outer-wheel sweep."""
    planner = _params()['planner_server']['ros__parameters']['GridBased']

    assert planner['minimum_turning_radius'] == 0.470


def test_goal_checker_requires_position_and_loose_final_heading():
    """Goal completion checks pose without latching an early XY crossing."""
    checker = _params()['controller_server']['ros__parameters'][
        'goal_checker'
    ]

    assert checker['plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert checker['xy_goal_tolerance'] == 0.10
    assert checker['yaw_goal_tolerance'] == 0.5
    assert checker['stateful'] is False


def test_default_static_map_matches_global_costmap_resolution():
    """Smac configures against the ratified static-map resolution."""
    map_name = re.search(
        r"DEFAULT_MAP_NAME = '([^']+)'", LAUNCH_PATH.read_text()
    ).group(1)
    map_yaml = yaml.safe_load(
        (WORKSPACE / 'maps' / f'{map_name}.yaml').read_text()
    )
    global_costmap = _params()['global_costmap']['global_costmap'][
        'ros__parameters'
    ]

    assert map_yaml['resolution'] == global_costmap['resolution']


def test_local_costmap_uses_raw_scan_and_ratified_geometry():
    """The rolling controller costmap uses raw scan and exact footprint."""
    local = _params()['local_costmap']['local_costmap']['ros__parameters']
    obstacle = local['obstacle_layer']

    assert local['rolling_window'] is True
    assert local['width'] == 2
    assert local['height'] == 2
    assert local['resolution'] == 0.025
    assert local['update_frequency'] == 10.0
    assert local['publish_frequency'] == 5.0
    assert local['footprint'] == (
        '[[0.235, 0.100], [0.235, -0.100], '
        '[-0.060, -0.100], [-0.060, 0.100]]'
    )
    assert obstacle['observation_sources'] == 'scan'
    assert obstacle['scan']['topic'] == '/scan'
    assert obstacle['scan']['data_type'] == 'LaserScan'
    assert obstacle['scan']['marking'] is True
    assert obstacle['scan']['clearing'] is True
    assert obstacle['scan']['min_obstacle_height'] == 0.0
    assert obstacle['scan']['max_obstacle_height'] == 2.0
    assert obstacle['scan']['obstacle_min_range'] == 0.05
    assert obstacle['scan']['obstacle_max_range'] == 1.0
    assert obstacle['scan']['raytrace_min_range'] == 0.0
    assert obstacle['scan']['raytrace_max_range'] == 1.2
    assert obstacle['scan']['expected_update_rate'] == 0.0
    assert obstacle['scan']['observation_persistence'] == 0.0
    assert obstacle['scan']['inf_is_valid'] is True
    assert local['inflation_layer']['inflation_radius'] == 0.15
    assert local['inflation_layer']['cost_scaling_factor'] == 10.0
    assert '/scan_slam' not in yaml.safe_dump(
        {'local_costmap': _params()['local_costmap']}
    )


def test_global_costmap_preserves_inflation_radius_with_faster_cost_decay():
    """Global cost shaping widens planning space without reducing clearance."""
    global_params = _params()['global_costmap']['global_costmap'][
        'ros__parameters'
    ]
    inflation = global_params['inflation_layer']

    assert inflation['inflation_radius'] == 0.30
    assert inflation['cost_scaling_factor'] == 10.0


def test_behavior_tree_is_minimal_and_forward_only():
    """The explicit tree checks path validity before replanning."""
    root = ET.parse(BT_PATH).getroot()
    tags = [element.tag for element in root.iter()]
    fallback = root.find('.//ReactiveFallback')

    assert tags.count('ComputePathToPose') == 1
    assert tags.count('IsPathValid') == 1
    assert tags.count('FollowPath') == 1
    assert tags.count('PipelineSequence') == 1
    assert tags.count('ReactiveFallback') == 1
    assert tags.count('GlobalUpdatedGoal') == 1
    assert [child.tag for child in fallback] == [
        'ReactiveSequence',
        'ComputePathToPose',
    ]
    path_check = fallback.find(
        "./ReactiveSequence[@name='CheckIfNewPathNeeded']"
    )
    assert [child.tag for child in path_check] == [
        'Inverter',
        'IsPathValid',
    ]
    assert path_check.find('./Inverter/GlobalUpdatedGoal') is not None
    assert 'RateController' not in tags
    assert 'Spin' not in tags
    assert 'BackUp' not in tags
    assert 'DriveOnHeading' not in tags
    assert 'Rotate' not in tags
    assert 'RotateToHeading' not in tags
    assert 'RecoveryNode' not in tags


def test_route_behavior_tree_replans_without_motion_recovery():
    """Route navigation replans its through-poses path only when invalid."""
    root = ET.parse(ROUTE_BT_PATH).getroot()
    tags = [element.tag for element in root.iter()]
    fallback = root.find('.//ReactiveFallback')
    replan = fallback.findall('./ReactiveSequence')[1]

    assert tags.count('ComputePathThroughPoses') == 1
    assert tags.count('IsPathValid') == 1
    assert tags.count('RemovePassedGoals') == 1
    assert tags.count('FollowPath') == 1
    assert tags.count('PipelineSequence') == 1
    assert tags.count('ReactiveFallback') == 1
    assert tags.count('GlobalUpdatedGoal') == 1
    assert [child.tag for child in fallback] == [
        'ReactiveSequence',
        'ReactiveSequence',
    ]
    assert 'RateController' not in tags
    path_check = fallback.find(
        "./ReactiveSequence[@name='CheckIfNewPathNeeded']"
    )
    assert [child.tag for child in path_check] == [
        'Inverter',
        'IsPathValid',
    ]
    assert path_check.find('./Inverter/GlobalUpdatedGoal') is not None
    assert [child.tag for child in replan] == [
        'RemovePassedGoals',
        'ComputePathThroughPoses',
    ]
    assert 'ComputePathToPose' not in tags
    assert 'Spin' not in tags
    assert 'BackUp' not in tags
    assert 'DriveOnHeading' not in tags
    assert 'Rotate' not in tags
    assert 'RotateToHeading' not in tags
    assert 'RecoveryNode' not in tags


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

    assert "create_publisher(Twist, '/cmd_vel_auto', 10)" in adapter
    assert 'topic: /cmd_vel_auto' in mux
    assert "('/cmd_vel_out', '/cmd_vel')" not in LAUNCH_PATH.read_text()
    assert 'collision_monitor' not in stage3_text.lower()
