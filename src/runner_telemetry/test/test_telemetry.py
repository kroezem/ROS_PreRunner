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

"""Tests for Raspberry Pi telemetry parsing and warning transitions."""

import subprocess
from types import SimpleNamespace

import pytest

from runner_telemetry.telemetry import (
    calculate_cpu_utilization,
    decode_throttled,
    parse_loadavg,
    parse_proc_stat,
    parse_temperature_millidegrees,
    parse_throttled,
    read_soc_temperature,
    run_vcgencmd,
    WarningTransitions,
)


PROC_STAT_FIRST = """\
cpu  100 10 30 400 20 5 5 0 0 0
cpu0 50 5 15 200 10 2 3 0 0 0
cpu1 50 5 15 200 10 3 2 0 0 0
intr 123
"""
PROC_STAT_SECOND = """\
cpu  130 10 50 450 20 5 5 0 0 0
cpu0 60 5 25 220 10 2 3 0 0 0
cpu1 70 5 25 230 10 3 2 0 0 0
intr 456
"""


def test_proc_stat_delta_reports_total_and_each_core():
    first = parse_proc_stat(PROC_STAT_FIRST)
    second = parse_proc_stat(PROC_STAT_SECOND)

    utilization = calculate_cpu_utilization(first, second)

    assert utilization.total_percent == pytest.approx(50.0)
    assert utilization.core_ids == (0, 1)
    assert utilization.per_core_percent == pytest.approx((50.0, 50.0))


@pytest.mark.parametrize(
    'content',
    [
        '',
        'intr 123\n',
        'cpu 1 2 bad 4\ncpu0 1 2 3 4\n',
        'cpu 1 2 3 4\n',
    ],
)
def test_malformed_proc_stat_is_rejected(content):
    with pytest.raises(ValueError):
        parse_proc_stat(content)


def test_proc_stat_rejects_counter_regression():
    first = parse_proc_stat(PROC_STAT_SECOND)
    second = parse_proc_stat(PROC_STAT_FIRST)

    with pytest.raises(ValueError, match='counter delta'):
        calculate_cpu_utilization(first, second)


def test_loadavg_includes_runnable_queue_count():
    load = parse_loadavg('10.88 7.25 3.50 5/412 12345\n')

    assert load.one_minute == pytest.approx(10.88)
    assert load.five_minutes == pytest.approx(7.25)
    assert load.fifteen_minutes == pytest.approx(3.50)
    assert load.runnable_processes == 5


@pytest.mark.parametrize(
    'content',
    [
        '',
        '1.0 2.0',
        'x 2.0 3.0 1/20 5',
        '1 2 3 bad 5',
        '1 2 3 21/20 5',
    ],
)
def test_malformed_loadavg_is_rejected(content):
    with pytest.raises(ValueError):
        parse_loadavg(content)


@pytest.mark.parametrize(
    ('content', 'expected'),
    [
        ('50700\n', 50.7),
        ('  42345  ', 42.345),
        ('-1000', -1.0),
    ],
)
def test_temperature_millidegrees(content, expected):
    assert parse_temperature_millidegrees(content) == pytest.approx(expected)


@pytest.mark.parametrize('content', ['', '51.2', 'temp=51000', '51C'])
def test_malformed_temperature_content(content):
    with pytest.raises(ValueError, match='malformed thermal sysfs'):
        parse_temperature_millidegrees(content)


def test_temperature_source_confirms_zone_type_and_resolves_path(tmp_path):
    zone = tmp_path / 'zone'
    zone.mkdir()
    temp = zone / 'temp'
    zone_type = zone / 'type'
    temp.write_text('51875\n', encoding='ascii')
    zone_type.write_text('cpu-thermal\n', encoding='ascii')

    value, resolved = read_soc_temperature(temp, zone_type)

    assert value == pytest.approx(51.875)
    assert resolved == str(temp.resolve())


def test_temperature_source_rejects_non_soc_zone(tmp_path):
    temp = tmp_path / 'temp'
    zone_type = tmp_path / 'type'
    temp.write_text('50000\n', encoding='ascii')
    zone_type.write_text('nvme\n', encoding='ascii')

    with pytest.raises(ValueError, match='expected thermal zone type'):
        read_soc_temperature(temp, zone_type)


@pytest.mark.parametrize('kind', ['missing', 'unreadable'])
def test_missing_or_unreadable_temperature_source(tmp_path, kind):
    zone_type = tmp_path / 'type'
    zone_type.write_text('cpu-thermal\n', encoding='ascii')
    temp = tmp_path / 'temp'
    if kind == 'unreadable':
        temp.mkdir()

    with pytest.raises(OSError):
        read_soc_temperature(temp, zone_type)


@pytest.mark.parametrize(
    ('content', 'expected'),
    [
        ('throttled=0x0', 0),
        ('throttled=0xA000A', 0xA000A),
        ('throttled=0XA000A', 0xA000A),
        ('a000a\n', 0xA000A),
        ('  throttled = 0x50005  \n', 0x50005),
    ],
)
def test_valid_throttling_hexadecimal(content, expected):
    assert parse_throttled(content) == expected


@pytest.mark.parametrize(
    'content',
    ['', 'throttled=', 'throttled=xyz', '0x12 trailing', '-1', '100000000'],
)
def test_malformed_throttling_output(content):
    with pytest.raises(ValueError):
        parse_throttled(content)


def test_zero_mask_decodes_clear():
    state = decode_throttled(0)
    assert not any(
        value for name, value in vars(state).items() if name != 'raw'
    )
    assert state.current_conditions() == ()
    assert state.sticky_conditions() == ()


@pytest.mark.parametrize(
    ('bit', 'field'),
    [
        (0, 'current_undervoltage'),
        (1, 'current_frequency_capped'),
        (2, 'current_throttled'),
        (3, 'current_soft_temperature_limit'),
        (16, 'sticky_undervoltage'),
        (17, 'sticky_frequency_capped'),
        (18, 'sticky_throttled'),
        (19, 'sticky_soft_temperature_limit'),
    ],
)
def test_each_supported_bit(bit, field):
    state = decode_throttled(1 << bit)
    assert getattr(state, field)
    assert sum(
        value for name, value in vars(state).items()
        if name != 'raw'
    ) == 1


def test_multiple_simultaneous_bits():
    state = decode_throttled((1 << 0) | (1 << 2) | (1 << 17) | (1 << 19))
    assert state.current_conditions() == ('undervoltage', 'throttled')
    assert state.sticky_conditions() == (
        'frequency capped',
        'soft temperature limit',
    )


def test_vcgencmd_command_failure_is_reported_as_source_error():
    def failed_runner(*args, **kwargs):
        raise subprocess.CalledProcessError(1, args[0], stderr='failed')

    with pytest.raises(OSError, match='vcgencmd get_throttled failed'):
        run_vcgencmd(runner=failed_runner)


def test_vcgencmd_success_is_parsed():
    def successful_runner(*args, **kwargs):
        return SimpleNamespace(stdout='throttled=0x50005\n')

    assert run_vcgencmd(runner=successful_runner) == 0x50005


def test_active_transition_does_not_repeat_and_warns_after_reentry():
    tracker = WarningTransitions()
    active = decode_throttled((1 << 0) | (1 << 2))

    first, _ = tracker.observe(True, active)
    repeated, _ = tracker.observe(True, active)
    tracker.observe(True, decode_throttled(0))
    reentry, _ = tracker.observe(True, active)

    assert first == ('undervoltage', 'throttled')
    assert repeated == ()
    assert reentry == ('undervoltage', 'throttled')


def test_first_valid_sample_reports_sticky_once():
    tracker = WarningTransitions()
    sticky = decode_throttled((1 << 16) | (1 << 18))

    _, first = tracker.observe(True, sticky)
    _, repeated = tracker.observe(True, sticky)

    assert first == ('undervoltage', 'throttled')
    assert repeated == ()


def test_sticky_warning_is_deferred_across_invalid_samples():
    tracker = WarningTransitions()
    sticky = decode_throttled(1 << 19)

    assert tracker.observe(False, None) == ((), ())
    _, first_valid = tracker.observe(True, sticky)

    assert first_valid == ('soft temperature limit',)


def test_no_sticky_warning_when_first_valid_sample_is_clear():
    tracker = WarningTransitions()

    assert tracker.observe(True, decode_throttled(0)) == ((), ())
    _, later_sticky = tracker.observe(True, decode_throttled(1 << 16))
    assert later_sticky == ()
