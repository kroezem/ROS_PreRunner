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

"""Static contracts for Stage 4 systemd ownership boundaries."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SERVICES = ROOT / 'services'


def source(name):
    """Read one tracked unit."""
    return (SERVICES / name).read_text(encoding='utf-8')


def test_mode_units_conflict_and_use_cgroup_teardown_without_boot_install():
    mapping = source('runner-mode-mapping.service')
    autonomy = source('runner-mode-autonomy.service')

    assert 'Conflicts=runner-mode-autonomy.service' in mapping
    assert 'Conflicts=runner-mode-mapping.service' in autonomy
    for unit in (mapping, autonomy):
        assert 'KillMode=control-group' in unit
        assert 'Restart=no' in unit
        assert '\n[Install]\n' not in unit
        assert 'pkill' not in unit


def test_fixed_units_launch_exactly_one_complete_composite():
    mapping = source('runner-mode-mapping.service')
    autonomy = source('runner-mode-autonomy.service')

    assert 'mode_launcher mapping' in mapping
    assert 'mode_launcher autonomy' in autonomy
    assert mapping.count('ExecStart=') == 1
    assert autonomy.count('ExecStart=') == 1


def test_only_supervisor_is_boot_enabled_and_controls_fixed_system_units():
    supervisor = source('runner-mode-supervisor.service')

    assert '[Install]' in supervisor
    assert 'WantedBy=multi-user.target' in supervisor
    assert 'mode_supervisor' in supervisor
    assert 'User=root' in supervisor


def test_command_authority_remains_persistent_but_offline_from_mux():
    authority = source('runner-command-authority.service')
    mux = (ROOT / 'src/runner_bringup/config/twist_mux.yaml').read_text()

    assert 'command_authority' in authority
    assert 'WantedBy=multi-user.target' in authority
    assert '/cmd_vel_paddock' not in mux


def test_install_docs_do_not_disable_authoritative_static_links():
    readme = (SERVICES / 'README.md').read_text(encoding='utf-8')

    assert 'systemctl disable runner-mode-mapping' not in readme
    assert 'lack of an `[Install]` section' in readme
