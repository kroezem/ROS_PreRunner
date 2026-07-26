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

"""Thread-safe state for the single-channel wheel encoder."""

from dataclasses import dataclass
import math
import threading
from typing import Optional


@dataclass(frozen=True)
class EncoderMeasurement:
    """One coherent encoder publication-cycle snapshot."""

    edge_count: int
    stationary: bool
    active_direction: int
    pending_direction: int

    def edge_rate(self, window_sec: float) -> float:
        """Return the unsigned fixed-window GPIO edge rate."""
        return self.edge_count / window_sec


class EncoderState:
    """Latch direction only when pulses resume after a confirmed stop."""

    def __init__(self, stationary_timeout_sec: float):
        """Initialize state with a validated pulse-absence timeout."""
        if (
            not math.isfinite(stationary_timeout_sec)
            or stationary_timeout_sec <= 0.0
        ):
            raise ValueError(
                'stationary_timeout_sec must be finite and greater than zero'
            )
        stationary_timeout_ns = int(stationary_timeout_sec * 1e9)
        if stationary_timeout_ns <= 0:
            raise ValueError(
                'stationary_timeout_sec must be at least one nanosecond'
            )

        self._stationary_timeout_ns = stationary_timeout_ns
        self._lock = threading.Lock()
        self._edge_count = 0
        self._last_edge_ns: Optional[int] = None
        self._stationary = True
        self._active_direction = 0
        self._latest_nonzero_direction = 0

    def update_direction(self, direction: int) -> bool:
        """Accept zero as no evidence and remember only valid nonzero values."""
        if direction not in (-1, 0, 1):
            return False

        if direction == 0:
            return True

        with self._lock:
            self._latest_nonzero_direction = direction
        return True

    def record_edge(self, monotonic_ns: int) -> None:
        """Record an edge and begin a new epoch after pulse-confirmed rest."""
        with self._lock:
            pulse_gap_confirmed_stop = (
                self._last_edge_ns is None
                or monotonic_ns - self._last_edge_ns
                >= self._stationary_timeout_ns
            )
            if self._stationary or pulse_gap_confirmed_stop:
                self._active_direction = self._latest_nonzero_direction

            self._stationary = False
            self._last_edge_ns = monotonic_ns
            self._edge_count += 1

    def take_measurement(self, monotonic_ns: int) -> EncoderMeasurement:
        """Take and reset one coherent fixed-window measurement."""
        with self._lock:
            self._stationary = (
                self._last_edge_ns is None
                or monotonic_ns - self._last_edge_ns
                >= self._stationary_timeout_ns
            )
            measurement = EncoderMeasurement(
                edge_count=self._edge_count,
                stationary=self._stationary,
                active_direction=self._active_direction,
                pending_direction=self._latest_nonzero_direction,
            )
            self._edge_count = 0
            return measurement
