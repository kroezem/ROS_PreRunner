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

"""Parsing and transition logic for Raspberry Pi telemetry."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
import subprocess


CURRENT_BITS = {
    0: 'undervoltage',
    1: 'frequency capped',
    2: 'throttled',
    3: 'soft temperature limit',
}
STICKY_BITS = {
    16: 'undervoltage',
    17: 'frequency capped',
    18: 'throttled',
    19: 'soft temperature limit',
}
CURRENT_MASK = sum(1 << bit for bit in CURRENT_BITS)
THROTTLED_PATTERN = re.compile(
    r'(?:throttled\s*=\s*)?(?:0[xX])?([0-9a-fA-F]+)',
)


@dataclass(frozen=True)
class ThrottledState:
    """Decoded Raspberry Pi firmware throttling state."""

    raw: int
    current_undervoltage: bool
    current_frequency_capped: bool
    current_throttled: bool
    current_soft_temperature_limit: bool
    sticky_undervoltage: bool
    sticky_frequency_capped: bool
    sticky_throttled: bool
    sticky_soft_temperature_limit: bool

    def current_conditions(self) -> tuple[str, ...]:
        """Return all active current conditions."""
        return tuple(
            name for bit, name in CURRENT_BITS.items()
            if self.raw & (1 << bit)
        )

    def sticky_conditions(self) -> tuple[str, ...]:
        """Return all recorded historical conditions."""
        return tuple(
            name for bit, name in STICKY_BITS.items()
            if self.raw & (1 << bit)
        )


@dataclass(frozen=True)
class CpuTimes:
    """Aggregate scheduler counters from one /proc/stat CPU row."""

    total: int
    idle: int


@dataclass(frozen=True)
class CpuSample:
    """Total and numerically ordered per-core scheduler counters."""

    total: CpuTimes
    cores: tuple[tuple[int, CpuTimes], ...]


@dataclass(frozen=True)
class CpuUtilization:
    """Total and per-core busy percentages over a sampling interval."""

    total_percent: float
    core_ids: tuple[int, ...]
    per_core_percent: tuple[float, ...]


@dataclass(frozen=True)
class LoadAverage:
    """Load averages and current runnable scheduler-entity count."""

    one_minute: float
    five_minutes: float
    fifteen_minutes: float
    runnable_processes: int


def parse_temperature_millidegrees(content: str) -> float:
    """Parse a thermal sysfs millidegree value as degrees Celsius."""
    stripped = content.strip()
    if not re.fullmatch(r'[+-]?\d+', stripped):
        raise ValueError(f'malformed thermal sysfs value: {content!r}')
    return int(stripped) / 1000.0


def read_soc_temperature(
    temp_path: Path,
    type_path: Path,
) -> tuple[float, str]:
    """Read a cpu-thermal zone and return Celsius and its resolved path."""
    zone_type = type_path.read_text(encoding='ascii').strip()
    if zone_type != 'cpu-thermal':
        raise ValueError(
            f'expected thermal zone type cpu-thermal, got {zone_type!r}',
        )
    temperature = parse_temperature_millidegrees(
        temp_path.read_text(encoding='ascii'),
    )
    return temperature, str(temp_path.resolve())


def parse_throttled(content: str) -> int:
    """Parse firmware or sysfs get_throttled hexadecimal output."""
    stripped = content.strip()
    match = THROTTLED_PATTERN.fullmatch(stripped)
    if match is None:
        raise ValueError(f'malformed get_throttled value: {content!r}')
    value = int(match.group(1), 16)
    if value > 0xFFFFFFFF:
        raise ValueError(f'get_throttled value exceeds uint32: {content!r}')
    return value


def read_throttled_sysfs(path: Path) -> int:
    """Read the direct Raspberry Pi firmware sysfs value."""
    return parse_throttled(path.read_text(encoding='ascii'))


def run_vcgencmd(
    command: str = '/usr/bin/vcgencmd',
    runner=subprocess.run,
) -> int:
    """Read get_throttled through vcgencmd for kernels without sysfs."""
    try:
        completed = runner(
            [command, 'get_throttled'],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise OSError(f'vcgencmd get_throttled failed: {error}') from error
    return parse_throttled(completed.stdout)


def parse_proc_stat(content: str) -> CpuSample:
    """Parse total and per-core CPU counters from /proc/stat."""
    rows = {}
    for line in content.splitlines():
        fields = line.split()
        if not fields or not re.fullmatch(r'cpu\d*', fields[0]):
            continue
        if len(fields) < 5 or not all(
            re.fullmatch(r'\d+', value) for value in fields[1:]
        ):
            raise ValueError(f'malformed /proc/stat CPU row: {line!r}')
        label = fields[0]
        if label in rows:
            raise ValueError(f'duplicate /proc/stat CPU row: {label}')
        counters = tuple(int(value) for value in fields[1:])
        # guest and guest_nice are already included in user and nice.
        total = sum(counters[:8])
        idle = counters[3] + (counters[4] if len(counters) > 4 else 0)
        rows[label] = CpuTimes(total=total, idle=idle)

    if 'cpu' not in rows:
        raise ValueError('/proc/stat does not contain an aggregate CPU row')
    cores = tuple(
        sorted(
            (
                (int(label[3:]), times)
                for label, times in rows.items()
                if label != 'cpu'
            ),
            key=lambda item: item[0],
        )
    )
    if not cores:
        raise ValueError('/proc/stat does not contain per-core CPU rows')
    return CpuSample(total=rows['cpu'], cores=cores)


def calculate_cpu_utilization(
    previous: CpuSample,
    current: CpuSample,
) -> CpuUtilization:
    """Calculate busy percentages from two /proc/stat counter samples."""
    previous_cores = dict(previous.cores)
    current_cores = dict(current.cores)
    if previous_cores.keys() != current_cores.keys():
        raise ValueError('per-core /proc/stat rows changed between samples')

    def percentage(old: CpuTimes, new: CpuTimes) -> float:
        total_delta = new.total - old.total
        idle_delta = new.idle - old.idle
        if total_delta <= 0 or idle_delta < 0 or idle_delta > total_delta:
            raise ValueError('invalid /proc/stat counter delta')
        busy = total_delta - idle_delta
        return min(100.0, max(0.0, 100.0 * busy / total_delta))

    core_ids = tuple(current_cores)
    return CpuUtilization(
        total_percent=percentage(previous.total, current.total),
        core_ids=core_ids,
        per_core_percent=tuple(
            percentage(previous_cores[index], current_cores[index])
            for index in core_ids
        ),
    )


def parse_loadavg(content: str) -> LoadAverage:
    """Parse load averages and runnable count from /proc/loadavg."""
    fields = content.split()
    if len(fields) < 4:
        raise ValueError(f'malformed /proc/loadavg: {content!r}')
    try:
        averages = tuple(float(value) for value in fields[:3])
    except ValueError:
        raise ValueError(f'malformed /proc/loadavg: {content!r}') from None
    if not all(math.isfinite(value) and value >= 0.0 for value in averages):
        raise ValueError(f'malformed /proc/loadavg: {content!r}')
    runnable_match = re.fullmatch(r'(\d+)/(\d+)', fields[3])
    if runnable_match is None:
        raise ValueError(f'malformed /proc/loadavg: {content!r}')
    runnable = int(runnable_match.group(1))
    total = int(runnable_match.group(2))
    if runnable > total:
        raise ValueError(f'malformed /proc/loadavg: {content!r}')
    return LoadAverage(*averages, runnable_processes=runnable)


def decode_throttled(raw: int) -> ThrottledState:
    """Decode all documented current and sticky firmware bits."""
    return ThrottledState(
        raw=raw,
        current_undervoltage=bool(raw & (1 << 0)),
        current_frequency_capped=bool(raw & (1 << 1)),
        current_throttled=bool(raw & (1 << 2)),
        current_soft_temperature_limit=bool(raw & (1 << 3)),
        sticky_undervoltage=bool(raw & (1 << 16)),
        sticky_frequency_capped=bool(raw & (1 << 17)),
        sticky_throttled=bool(raw & (1 << 18)),
        sticky_soft_temperature_limit=bool(raw & (1 << 19)),
    )


class WarningTransitions:
    """Track current activation edges and the first valid sticky sample."""

    def __init__(self):
        self._previous_current = 0
        self._sticky_checked = False

    def observe(
        self,
        valid: bool,
        state: ThrottledState | None,
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """Return current-edge and one-time startup-sticky warning fields."""
        if not valid or state is None:
            return (), ()

        current = state.raw & CURRENT_MASK
        activated = current & ~self._previous_current
        self._previous_current = current
        current_warning = state.current_conditions() if activated else ()

        sticky_warning = ()
        if not self._sticky_checked:
            self._sticky_checked = True
            sticky_warning = state.sticky_conditions()
        return current_warning, sticky_warning
