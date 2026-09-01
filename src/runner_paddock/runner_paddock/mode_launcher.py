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

"""Exec the fixed launch composite selected by a systemd mode unit."""

import os
from pathlib import Path
import sys

from runner_paddock.mode_runtime import AUTONOMY_MAP_FILE
from runner_paddock.mode_runtime import MAP_DIRECTORY
from runner_paddock.mode_runtime import validate_map_bundle


def main() -> None:
    """Validate fixed arguments, then replace this process with ros2 launch."""
    if len(sys.argv) != 2 or sys.argv[1] not in ('mapping', 'autonomy'):
        raise SystemExit('usage: mode_launcher {mapping|autonomy}')
    if sys.argv[1] == 'mapping':
        arguments = ('ros2', 'launch', 'runner_bringup', 'map.launch.py')
    else:
        map_name = Path(AUTONOMY_MAP_FILE).read_text(encoding='utf-8').strip()
        validate_map_bundle(map_name, map_directory=MAP_DIRECTORY)
        arguments = (
            'ros2', 'launch', 'runner_bringup', 'autonomy.launch.py',
            f'map_name:={map_name}',
        )
    os.execvp(arguments[0], arguments)
