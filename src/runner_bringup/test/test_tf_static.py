"""Regression checks for the measured D-76 sensor transforms."""

import ast
from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1] / 'launch' / 'include' / 'tf_static.launch.py'
)


def _static_transforms():
    tree = ast.parse(LAUNCH.read_text())
    transforms = {}
    for call in (
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
    ):
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        if (
            ast.literal_eval(keywords['executable'])
            != 'static_transform_publisher'
        ):
            continue
        arguments = ast.literal_eval(keywords['arguments'])
        values = dict(zip(arguments[::2], arguments[1::2]))
        transforms[values['--child-frame-id']] = values
    return transforms


def test_d76_static_transforms_are_exact():
    """The post-rotation sled measurements replace the corrupt old values."""
    transforms = _static_transforms()

    assert transforms == {
        'base_laser': {
            '--x': '0.0733',
            '--y': '0.0',
            '--z': '0.1135',
            '--roll': '0',
            '--pitch': '0',
            '--yaw': '3.141592653589793',
            '--frame-id': 'base_link',
            '--child-frame-id': 'base_laser',
        },
        'imu_link': {
            '--x': '0.1233',
            '--y': '-0.0025',
            '--z': '0.1060',
            '--roll': '0',
            '--pitch': '0',
            '--yaw': '0',
            '--frame-id': 'base_link',
            '--child-frame-id': 'imu_link',
        },
    }
