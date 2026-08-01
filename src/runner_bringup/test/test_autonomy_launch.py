"""Focused checks for the complete Stage 4 launch integration."""

import ast
from pathlib import Path


PACKAGE = Path(__file__).parents[1]
AUTONOMY_LAUNCH = PACKAGE / 'launch' / 'autonomy.launch.py'
NAV2_LAUNCH = PACKAGE / 'launch' / 'nav2.launch.py'
BENCH_LAUNCH = PACKAGE / 'launch' / 'autonomy_bench.launch.py'
TASKS = PACKAGE.parents[1] / '.vscode' / 'tasks.json'


def _node_packages(path):
    tree = ast.parse(path.read_text())
    packages = []
    for call in (
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
    ):
        package = next(
            keyword.value
            for keyword in call.keywords
            if keyword.arg == 'package'
        )
        packages.append(ast.literal_eval(package))
    return packages


def test_composite_includes_nav2_once_and_passes_map_name():
    """The established Nav2 stack is included once with its map argument."""
    source = AUTONOMY_LAUNCH.read_text()

    assert source.count("'nav2.launch.py'") == 1
    assert source.count('IncludeLaunchDescription(') == 1
    assert "DeclareLaunchArgument(\n            'map_name'," in source
    assert "launch_arguments={'map_name': map_name}.items()" in source


def test_composite_adds_only_missing_command_chain_nodes():
    """Nav2 supplies launched sensors; composite adds the command chain."""
    packages = _node_packages(AUTONOMY_LAUNCH)

    assert packages == [
        'joy',
        'runner_teleop',
        'runner_teleop',
        'runner_drive_adapter',
        'twist_mux',
    ]
    assert 'runner_encoder' not in packages
    assert 'robot_localization' not in packages
    assert not any(package.startswith('nav2_') for package in packages)


def test_encoder_hardware_owner_is_absent_from_all_composites():
    """The persistent encoder service is the only GPIO 22 owner."""
    launch_files = list((PACKAGE / 'launch').rglob('*.launch.py'))

    for launch_file in launch_files:
        source = launch_file.read_text()
        assert "package='runner_encoder'" not in source
        assert "executable='encoder_node'" not in source


def test_command_chain_parameters_match_the_stage2_bench():
    """The integration does not alter Stage 2 behavior or output ownership."""
    autonomy = AUTONOMY_LAUNCH.read_text()
    bench = BENCH_LAUNCH.read_text()
    required_fragments = (
        "'autorepeat_rate': 20.0, 'deadzone': 0.05",
        "'deadman_button': 0",
        "'controller_timeout': 0.15",
        "'keyboard_state_timeout': 0.15",
        "'fixed_throttle_initial_setpoint': 0.30",
        "executable='keyboard_bridge'",
        "'input_timeout': 0.15",
        "'speed_cap': 0.50",
        "package='runner_drive_adapter'",
        'parameters=[adapter_parameters]',
        "package='twist_mux'",
        'parameters=[mux_parameters]',
        "remappings=[('/cmd_vel_out', '/cmd_vel')]",
    )

    for fragment in required_fragments:
        assert fragment in autonomy
        assert fragment in bench

    assert "package='runner_motor'" not in autonomy
    assert "package='runner_motor'" not in bench
    assert 'esc_mode' not in autonomy
    assert 'esc_mode' not in bench


def test_composite_preserves_controller_and_mux_topic_ownership():
    """Only the controller and mux retain their established output remaps."""
    autonomy = AUTONOMY_LAUNCH.read_text()
    nav2 = NAV2_LAUNCH.read_text()

    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in nav2
    assert "remappings=[('/cmd_vel_out', '/cmd_vel')]" in autonomy
    assert "('/cmd_vel_nav', '/cmd_vel')" not in autonomy


def test_vscode_adds_autonomy_without_changing_existing_tasks():
    """The driving composite has its own task; bench and Nav2 remain."""
    tasks = TASKS.read_text()

    assert '"label": "Runner: Nav2"' in tasks
    assert '"label": "Runner: Autonomy Bench"' in tasks
    assert '"label": "Runner: Autonomy"' in tasks
    assert (
        'ros2 launch runner_bringup autonomy.launch.py '
        'map_name:=${input:runnerMap}'
    ) in tasks
