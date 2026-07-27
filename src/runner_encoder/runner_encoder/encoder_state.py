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

from collections import deque
from dataclasses import dataclass
import math
import sys
import threading
from typing import Deque, Optional


DEFAULT_HISTORY_DEPTH = 4
MAX_LOOKBACK_NS = 500_000_000

# The GPIO debounce configuration rejects changes that are not stable for
# 100 us.  An interval shorter than that cannot be a valid accepted edge pair.
MIN_EDGE_INTERVAL_NS = 100_000


def validate_history_depth(history_depth: int) -> int:
    """Return a history depth that can safely size the timestamp deque."""
    if type(history_depth) is not int:
        raise ValueError('history_depth must be an integer')
    if history_depth < 1:
        raise ValueError('history_depth must be at least 1')
    if history_depth >= sys.maxsize:
        raise ValueError(
            f'history_depth must be no greater than {sys.maxsize - 1}'
        )
    return history_depth


@dataclass(frozen=True)
class EncoderMeasurement:
    """One coherent encoder publication-cycle snapshot."""

    edge_rate: float
    stationary: bool
    active_direction: int
    pending_direction: int


class EncoderState:
    """Latch direction only when pulses resume after a confirmed stop."""

    def __init__(
        self,
        stationary_timeout_sec: float,
        history_depth: int = DEFAULT_HISTORY_DEPTH,
    ):
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

        history_depth = validate_history_depth(history_depth)

        self._stationary_timeout_ns = stationary_timeout_ns
        self._lock = threading.Lock()
        self._edge_timestamps_ns: Deque[int] = deque(
            maxlen=history_depth + 1
        )
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
            if (
                self._last_edge_ns is not None
                and monotonic_ns - self._last_edge_ns
                < MIN_EDGE_INTERVAL_NS
            ):
                return

            pulse_gap_confirmed_stop = (
                self._last_edge_ns is None
                or monotonic_ns - self._last_edge_ns
                >= self._stationary_timeout_ns
            )
            if self._stationary or pulse_gap_confirmed_stop:
                self._active_direction = self._latest_nonzero_direction
                self._edge_timestamps_ns.clear()

            self._stationary = False
            self._last_edge_ns = monotonic_ns
            self._edge_timestamps_ns.append(monotonic_ns)
            self._prune_history(monotonic_ns)

    def take_measurement(self, monotonic_ns: int) -> EncoderMeasurement:
        """Take one coherent interval-rate measurement."""
        with self._lock:
            if (
                self._last_edge_ns is not None
                and monotonic_ns < self._last_edge_ns
            ):
                # Both clocks should be CLOCK_MONOTONIC.  If that invariant is
                # broken, discard the old epoch instead of producing a
                # negative age or retaining timestamps from another timeline.
                self._last_edge_ns = None
                self._edge_timestamps_ns.clear()
                self._stationary = True

            self._stationary = (
                self._last_edge_ns is None
                or monotonic_ns - self._last_edge_ns
                >= self._stationary_timeout_ns
            )
            if self._stationary:
                edge_rate = 0.0
                self._edge_timestamps_ns.clear()
            else:
                self._prune_history(monotonic_ns)
                edge_rate = self._estimate_edge_rate(monotonic_ns)

            measurement = EncoderMeasurement(
                edge_rate=edge_rate,
                stationary=self._stationary,
                active_direction=self._active_direction,
                pending_direction=self._latest_nonzero_direction,
            )
            return measurement

    def _prune_history(self, reference_ns: int) -> None:
        """Retain timestamps within both the depth and lookback bounds."""
        cutoff_ns = reference_ns - MAX_LOOKBACK_NS
        while (
            len(self._edge_timestamps_ns) > 1
            and self._edge_timestamps_ns[0] < cutoff_ns
        ):
            self._edge_timestamps_ns.popleft()

    def _estimate_edge_rate(self, monotonic_ns: int) -> float:
        """Estimate unsigned rate over retained intervals with stop decay."""
        if len(self._edge_timestamps_ns) < 2:
            return 0.0

        first_edge_ns = self._edge_timestamps_ns[0]
        previous_edge_ns = self._edge_timestamps_ns[-2]
        last_edge_ns = self._edge_timestamps_ns[-1]
        span_ns = last_edge_ns - first_edge_ns
        most_recent_interval_ns = last_edge_ns - previous_edge_ns
        if span_ns <= 0 or most_recent_interval_ns <= 0:
            return 0.0

        edge_rate = (
            (len(self._edge_timestamps_ns) - 1) * 1e9 / span_ns
        )
        dt_since_last_ns = monotonic_ns - last_edge_ns
        if dt_since_last_ns > most_recent_interval_ns:
            edge_rate = min(edge_rate, 1e9 / dt_since_last_ns)
        return edge_rate
