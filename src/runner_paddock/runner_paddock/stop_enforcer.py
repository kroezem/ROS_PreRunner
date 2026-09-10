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

"""Persistent global STOP using the existing mux; no actuator ownership."""
import concurrent.futures
import json
import os
from pathlib import Path
import time
import uuid

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import (
    EncoderState,
    LocalControlState,
    StopRequest,
    StopState,
)
from std_msgs.msg import Bool

LOCK_TIMEOUT = 0.15
DRAIN_TIME = 0.40
STATE_HEARTBEAT_PERIOD = 0.05


def _state_key(message):
    """Return STOP status content excluding its publication timestamp."""
    return tuple(
        getattr(message, slot)
        for slot in message.__slots__
        if slot != '_stamp'
    )


def persist(path, generation, stopped):
    """Atomic durable replacement runs outside the ROS enforcement executor."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.pending')
    with temporary.open('w') as stream:
        json.dump({'version': 1, 'generation': generation, 'stopped': stopped}, stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    descriptor = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return generation, stopped


def restore(path):
    value = json.loads(Path(path).read_text())
    if (value.get('version') != 1 or type(value.get('stopped')) is not bool
            or type(value.get('generation')) is not int
            or not 0 <= value['generation'] < 2**63):
        raise ValueError('invalid STOP record')
    return value['generation'], value['stopped']


class StopEnforcer(Node):
    def __init__(self):
        super().__init__('runner_stop_enforcer')
        self.path = self.declare_parameter(
            'state_file', '/home/matti/.local/state/runner/stop.json').value
        self.boot = str(uuid.uuid4())
        self.fault = ''
        try:
            self.generation, self.stopped = restore(self.path)
        except (OSError, ValueError, TypeError):
            self.generation, self.stopped = 0, True
            self.fault = 'STOP_STATE_UNKNOWN'
        self.durable = not self.fault
        self.locked = True
        self.clear_at = None
        self.zero_seen = False
        self.final_at = None
        self.encoder_at = None
        self.stationary = False
        self.local_at = None
        self.local_released = False
        self.local_neutral = False
        self.started = time.monotonic()
        self.worker = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.write = None
        self.last_requester_id = ''
        self.last_request_id = 0
        self.last_request_accepted = False
        self.last_request_reason = ''
        self.request_results = {}
        self.last_state_publish_at = None
        self.last_state_key = None
        # Lifespan prevents a backlogged old clear heartbeat from becoming fresh.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         lifespan=Duration(seconds=0.10))
        request_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        self.lock_pub = self.create_publisher(Bool, '/paddock/stop_lock', qos)
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel_stop', qos)
        self.state_pub = self.create_publisher(StopState, '/paddock/stop_state', 1)
        self.create_subscription(EncoderState, '/wheel/encoder_state', self.on_encoder, 1)
        self.create_subscription(
            LocalControlState,
            '/teleop/control_state',
            self.on_local_control,
            1,
        )
        self.create_subscription(Twist, '/cmd_vel', self.on_final, 1)
        self.create_subscription(
            StopRequest,
            '/paddock/internal/stop_request',
            self.request,
            request_qos,
        )
        self.create_timer(0.02, self.tick)
        # A valid persisted clear survives web/authority failure; on executor
        # startup retain lock through transport drain before restoring clear.
        if not self.stopped and self.durable:
            self.clear_at = self.started + DRAIN_TIME

    def on_encoder(self, message):
        self.encoder_at = time.monotonic()
        self.stationary = message.stationary

    def on_local_control(self, message):
        self.local_at = time.monotonic()
        self.local_released = message.released
        self.local_neutral = message.neutral

    def on_final(self, message):
        if self.locked and message.linear.x == 0 and message.angular.z == 0:
            self.zero_seen = True
            self.final_at = time.monotonic()

    def clear_reason(self):
        now = time.monotonic()
        if self.encoder_at is None or now - self.encoder_at > 0.20:
            return 'ENCODER_STALE'
        if not self.stationary:
            return 'NOT_STATIONARY'
        if self.local_at is None or now - self.local_at > 0.20:
            return 'LOCAL_STATUS_STALE'
        if not self.local_released or not self.local_neutral:
            return 'LOCAL_NOT_NEUTRAL'
        if self.fault or not self.durable or self.write:
            return self.fault or 'PERSISTENCE_PENDING'
        return ''

    def _ack(self, request, accepted, reason, *, store=True):
        result = (accepted, reason)
        if store:
            self.request_results[request.requester_id] = (
                request.request_id, result
            )
        self.last_requester_id = request.requester_id
        self.last_request_id = request.request_id
        self.last_request_accepted = accepted
        self.last_request_reason = reason

    def request(self, request):
        """Apply one boot-qualified request; retries are idempotent."""
        previous = self.request_results.get(request.requester_id)
        if previous is not None:
            previous_id, result = previous
            if request.request_id == previous_id:
                self._ack(request, *result)
                return
            if request.request_id < previous_id:
                self._ack(request, False, 'STALE_REQUEST_ID', store=False)
                return
        if request.expected_boot_id and request.expected_boot_id != self.boot:
            self._ack(request, False, 'STALE_EXECUTOR_BOOT')
            return
        if request.stopped:
            # Assert before disk work. Every request invalidates older clears.
            self.generation += 1
            self.stopped = True
            self.locked = True
            self.clear_at = None
            self.zero_seen = False
            self.durable = False
            self.lock_pub.publish(Bool(data=True))
            self.zero_pub.publish(Twist())
            self.write = self.worker.submit(persist, self.path, self.generation, True)
            self._ack(request, True, 'STOP_REQUESTED')
        elif request.expected_generation != self.generation:
            self._ack(request, False, 'STALE_STOP_GENERATION')
        elif self.clear_at is not None:
            self._ack(request, False, 'CLEAR_PENDING')
        else:
            reason = self.clear_reason()
            if not reason:
                self.generation += 1
                self.durable = False
                self.write = self.worker.submit(persist, self.path, self.generation, False)
                self._ack(request, True, 'CLEAR_REQUESTED')
            else:
                self._ack(request, False, reason)

    def tick(self):
        now = time.monotonic()
        if self.write and self.write.done():
            future, self.write = self.write, None
            try:
                generation, stopped = future.result()
                if generation == self.generation:
                    self.durable = True
                    self.fault = ''
                    if not stopped:
                        self.clear_at = now + DRAIN_TIME
            except Exception as error:
                self.fault = 'PERSISTENCE_FAILED:' + type(error).__name__
                self.locked = self.stopped = True
        if self.clear_at is not None:
            # After deliberate clear, require neutral throughout drain. Restored
            # known-clear startup does not depend on Paddock or a connected pad.
            restoring_clear = not self.stopped
            reason = '' if restoring_clear else self.clear_reason()
            if reason:
                self.generation += 1
                self.stopped = self.locked = True
                self.durable = False
                self.clear_at = None
                self.write = self.worker.submit(persist, self.path, self.generation, True)
            elif now >= self.clear_at:
                self.stopped = self.locked = False
                self.clear_at = None
        self.lock_pub.publish(Bool(data=self.locked))
        # During clear drain stop velocity must age out before lock release.
        if self.locked and self.clear_at is None:
            self.zero_pub.publish(Twist())
        message = StopState()
        message.stamp = self.get_clock().now().to_msg()
        message.boot_id = self.boot
        message.generation = self.generation
        message.stopped = self.stopped
        message.locked = self.locked
        message.healthy = not bool(self.fault)
        final_fresh = self.final_at is not None and now - self.final_at <= 0.10
        message.applied = (self.stopped and self.locked and self.durable
                           and self.zero_seen and final_fresh)
        message.clear_pending = self.clear_at is not None
        message.reason = self.fault or (
            'STOP_APPLIED' if message.applied else
            'CLEAR' if not self.locked else 'STOP_PENDING'
        )
        message.last_requester_id = self.last_requester_id
        message.last_request_id = self.last_request_id
        message.last_request_accepted = self.last_request_accepted
        message.last_request_reason = self.last_request_reason
        # The 50 Hz lock/zero enforcement above is safety-critical and remains
        # unchanged. Status changes publish immediately, with a 20 Hz liveness
        # heartbeat kept well inside the authority's 150 ms freshness bound.
        key = _state_key(message)
        due = (
            self.last_state_publish_at is None
            or now - self.last_state_publish_at >= STATE_HEARTBEAT_PERIOD
        )
        if key == self.last_state_key and not due:
            return
        self.state_pub.publish(message)
        self.last_state_key = key
        self.last_state_publish_at = now


def main():
    rclpy.init()
    node = StopEnforcer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        node.worker.shutdown(wait=True)
        if rclpy.ok():
            rclpy.shutdown()
