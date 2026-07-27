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

"""Hardware-free tests for the libgpiod encoder edge reader."""

from queue import Empty, Queue
import threading
from types import SimpleNamespace

import pytest

from runner_encoder import encoder_node
from runner_encoder.encoder_node import (
    GPIO_CHIP_LABEL,
    GPIO_CONSUMER,
    GpioEventReader,
    open_gpio_chip_by_label,
)


class FakeLine:
    def __init__(self, *, request_error=None):
        self.request_error = request_error
        self.request_args = None
        self.requested = False
        self.released = False
        self.events = Queue()
        self._pending_event = None

    def request(self, **kwargs):
        self.request_args = kwargs
        if self.request_error is not None:
            raise self.request_error
        self.requested = True

    def event_wait(self, sec, nsec):
        try:
            self._pending_event = self.events.get(timeout=0.01)
        except Empty:
            return False
        return True

    def event_read(self):
        event = self._pending_event
        self._pending_event = None
        return event

    def release(self):
        self.released = True
        self.requested = False


class FakeChip:
    def __init__(self, path, label, line=None):
        self.path = path
        self._label = label
        self.line = line or FakeLine()
        self.closed = False
        self.requested_offset = None

    def label(self):
        return self._label

    def get_line(self, offset):
        self.requested_offset = offset
        return self.line

    def close(self):
        self.closed = True


class FakeChipFactory:
    OPEN_BY_PATH = 4

    def __init__(self, chips):
        self.chips = chips
        self.calls = []

    def __call__(self, path, open_mode):
        self.calls.append((path, open_mode))
        return self.chips[path]


def fake_gpiod(chips):
    return SimpleNamespace(
        Chip=FakeChipFactory(chips),
        LINE_REQ_EV_BOTH_EDGES=6,
    )


def test_rp1_chip_resolution_returns_only_matching_open_chip():
    nonmatch = FakeChip('/dev/gpiochip0', 'brcmstb')
    match = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL)
    module = fake_gpiod({
        nonmatch.path: nonmatch,
        match.path: match,
    })

    path, chip = open_gpio_chip_by_label(
        GPIO_CHIP_LABEL,
        gpiod_module=module,
        chip_paths=[nonmatch.path, match.path],
    )

    assert path == match.path
    assert chip is match
    assert nonmatch.closed
    assert not match.closed
    chip.close()


def test_no_matching_gpio_chip_fails_clearly_and_closes_chips():
    chip = FakeChip('/dev/gpiochip0', 'brcmstb')
    module = fake_gpiod({chip.path: chip})

    with pytest.raises(
        RuntimeError,
        match="Expected exactly one GPIO chip labeled 'pinctrl-rp1'; found 0",
    ):
        open_gpio_chip_by_label(
            GPIO_CHIP_LABEL,
            gpiod_module=module,
            chip_paths=[chip.path],
        )

    assert chip.closed


def test_multiple_matching_gpio_chips_fail_clearly_and_close_chips():
    first = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL)
    second = FakeChip('/dev/gpiochip5', GPIO_CHIP_LABEL)
    module = fake_gpiod({
        first.path: first,
        second.path: second,
    })

    with pytest.raises(
        RuntimeError,
        match="Expected exactly one GPIO chip labeled 'pinctrl-rp1'; found 2",
    ):
        open_gpio_chip_by_label(
            GPIO_CHIP_LABEL,
            gpiod_module=module,
            chip_paths=[first.path, second.path],
        )

    assert first.closed
    assert second.closed


def test_reader_requests_line_22_for_both_edges_and_forwards_timestamp():
    line = FakeLine()
    chip = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL, line)
    module = fake_gpiod({chip.path: chip})
    recorded = []
    recorded_event = threading.Event()

    def record_edge(timestamp_ns):
        recorded.append(timestamp_ns)
        recorded_event.set()

    reader = GpioEventReader(
        22,
        record_edge,
        pytest.fail,
        gpiod_module=module,
        chip_paths=[chip.path],
    )
    line.events.put(SimpleNamespace(sec=123, nsec=456_789))

    assert recorded_event.wait(timeout=1.0)
    reader.close()

    assert chip.requested_offset == 22
    assert line.request_args == {
        'consumer': GPIO_CONSUMER,
        'type': module.LINE_REQ_EV_BOTH_EDGES,
    }
    assert recorded == [123_000_456_789]
    assert reader.event_count == 1


def test_shutdown_stops_reader_releases_line_and_closes_chip():
    line = FakeLine()
    chip = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL, line)
    module = fake_gpiod({chip.path: chip})
    reader = GpioEventReader(
        22,
        lambda timestamp_ns: None,
        pytest.fail,
        gpiod_module=module,
        chip_paths=[chip.path],
    )

    reader.close()

    assert line.released
    assert chip.closed
    assert reader._thread is None


def test_line_request_failure_closes_chip_without_releasing_unowned_line():
    line = FakeLine(request_error=RuntimeError('request failed'))
    chip = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL, line)
    module = fake_gpiod({chip.path: chip})

    with pytest.raises(RuntimeError, match='request failed'):
        GpioEventReader(
            22,
            lambda timestamp_ns: None,
            pytest.fail,
            gpiod_module=module,
            chip_paths=[chip.path],
        )

    assert not line.released
    assert chip.closed


def test_thread_start_failure_releases_requested_line_and_closes_chip(
    monkeypatch,
):
    line = FakeLine()
    chip = FakeChip('/dev/gpiochip4', GPIO_CHIP_LABEL, line)
    module = fake_gpiod({chip.path: chip})

    class FailingThread:
        def __init__(self, **kwargs):
            pass

        def start(self):
            raise RuntimeError('thread failed')

    monkeypatch.setattr(encoder_node.threading, 'Thread', FailingThread)

    with pytest.raises(RuntimeError, match='thread failed'):
        GpioEventReader(
            22,
            lambda timestamp_ns: None,
            pytest.fail,
            gpiod_module=module,
            chip_paths=[chip.path],
        )

    assert line.released
    assert chip.closed
