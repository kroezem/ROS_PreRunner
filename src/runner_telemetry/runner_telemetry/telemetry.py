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
