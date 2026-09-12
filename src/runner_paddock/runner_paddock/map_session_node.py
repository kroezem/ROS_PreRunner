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

"""
Persistent Paddock map-session executor.

Owns ``/paddock/map_request`` -> ``/paddock/map_state``. It never owns process
lifecycle: NEW MAP is forwarded to the mode supervisor as an OP_NEW_MAP mode
request. SAVE MAP is a transactional capture (SLAM serialize + a same-session
occupancy raster) that only enters the selectable catalog after full validation.
"""

import math
import os
from pathlib import Path
import threading
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import (
    MapCatalogEntry,
    MapRequest,
    MapState,
    ModeRequest,
    ModeState,
    PaddockControlLease,
    StopState,
)
from runner_paddock.map_session import (
    delete_bundle,
    DeleteState,
    MapError,
    MapSaveTransaction,
    MapSessionModel,
    OccupancyRaster,
    safe_basename,
    SaveOutcome,
    SaveState,
    SessionPhase,
    validate_bundle,
    validate_delete_candidate,
)
from slam_toolbox.srv import SerializePoseGraph


MAP_REQUEST_TOPIC = '/paddock/map_request'
MAP_STATE_TOPIC = '/paddock/map_state'
MODE_STATE_TOPIC = '/paddock/mode_state'
MODE_REQUEST_TOPIC = '/paddock/mode_request'
LEASE_TOPIC = '/paddock/control_lease'
STOP_STATE_TOPIC = '/paddock/stop_state'
MAP_TOPIC = '/map'
SERIALIZE_SERVICE = '/slam_toolbox/serialize_map'

MAP_DIRECTORY = Path(os.environ.get(
    'PADDOCK_MAP_DIRECTORY', '/home/matti/runner_ws/maps'
))
SELECTED_MAP_FILE = Path(os.environ.get(
    'PADDOCK_SELECTED_MAP_FILE',
    str(Path.home() / '.local/state/runner/selected_map'),
))
STOP_STATE_TIMEOUT_SEC = 1.0
SESSION_EVIDENCE_GRACE_SEC = 2.0
SAVE_RASTER_TIMEOUT_SEC = 15.0


class MapSessionNode(Node):
    """Serialize map-session operations and publish truthful map state."""

    def __init__(self) -> None:
        super().__init__('runner_map_executor')
        self._model = MapSessionModel(map_dir=MAP_DIRECTORY)
        self._lock = threading.Lock()
        self._lease_active = False
        self._lease_id = ''
        self._mode = 0
        self._mode_status = 0
        self._mode_ready = False
        self._active_autonomy_map = ''
        self._last_request_id = 0
        self._new_map_request_seq = 0
        self._mode_accepted_request_id = 0
        self._reset_state = MapState.RESET_IDLE
        self._reset_request_id = 0
        self._reset_detail = ''
        self._pending_reset: tuple[int, int, str] | None = None
        self._stop_state: StopState | None = None
        self._stop_state_at: float | None = None
        self._last_map: OccupancyGrid | None = None
        self._last_map_at: float | None = None

        applied = self._load_selection()
        self._model.set_selection(applied, applied, '')

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_pub = self.create_publisher(
            MapState, MAP_STATE_TOPIC, state_qos
        )
        self._mode_request_pub = self.create_publisher(
            ModeRequest, MODE_REQUEST_TOPIC, 10
        )

        group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            ModeState, MODE_STATE_TOPIC, self._on_mode_state, state_qos,
            callback_group=group,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map, map_qos,
            callback_group=group,
        )
        self.create_subscription(
            PaddockControlLease, LEASE_TOPIC, self._on_lease, 10,
            callback_group=group,
        )
        self.create_subscription(
            StopState, STOP_STATE_TOPIC, self._on_stop_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=group,
        )
        # Requests run in their own group: a slow save cannot block observations.
        self._request_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            MapRequest, MAP_REQUEST_TOPIC, self._on_map_request, 10,
            callback_group=self._request_group,
        )
        self._serialize_client = self.create_client(
            SerializePoseGraph, SERIALIZE_SERVICE,
            callback_group=group,
        )
        self.create_timer(0.5, self._on_timer, callback_group=group)
        self.get_logger().info(
            f'map executor ready; catalog root {MAP_DIRECTORY}, '
            f'selected map {applied or "(none)"}'
        )

    # ---- persistence -------------------------------------------------------

    def _load_selection(self) -> str:
        try:
            name = SELECTED_MAP_FILE.read_text(encoding='utf-8').strip()
        except OSError:
            return ''
        try:
            safe_basename(name)
            validate_bundle(MAP_DIRECTORY, name)
        except MapError:
            return ''
        return name

    def _store_selection(self, name: str) -> None:
        SELECTED_MAP_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = SELECTED_MAP_FILE.with_suffix('.tmp')
        tmp.write_text(f'{name}\n', encoding='utf-8')
        os.replace(tmp, SELECTED_MAP_FILE)

    # ---- observations -----------------------------------------------------

    def _on_lease(self, message: PaddockControlLease) -> None:
        self._lease_active = bool(message.active)
        self._lease_id = message.lease_id if message.active else ''

    def _on_stop_state(self, message: StopState) -> None:
        self._stop_state = message
        self._stop_state_at = time.monotonic()

    def _on_mode_state(self, message: ModeState) -> None:
        now = time.monotonic()
        self._mode = int(message.mode)
        self._mode_status = int(message.status)
        self._mode_ready = bool(message.ready)
        self._mode_accepted_request_id = int(message.accepted_request_id)
        self._active_autonomy_map = message.active_autonomy_map
        mapping_active = (
            int(message.mode) == ModeState.MODE_MAPPING
            and int(message.status) == ModeState.STATUS_STABLE
        )
        with self._lock:
            self._model.observe_runtime(
                mapping_active=mapping_active,
                session_id=message.mapping_session_id,
                runtime_epoch=int(message.runtime_epoch),
                now=now,
            )
            pending = self._pending_reset
            if pending is not None:
                mode_request_id, previous_epoch, previous_session = pending
                if int(message.accepted_request_id) >= mode_request_id:
                    if int(message.status) == ModeState.STATUS_TRANSITIONING:
                        self._reset_detail = (
                            message.detail or 'Resetting mapping session'
                        )
                    elif int(message.status) == ModeState.STATUS_FAULT:
                        self._reset_state = MapState.RESET_FAILED
                        self._reset_detail = (
                            message.detail or 'Mapping reset failed'
                        )
                        self._pending_reset = None
                    elif (
                        int(message.mode) == ModeState.MODE_MAPPING
                        and int(message.runtime_epoch) > previous_epoch
                        and message.mapping_session_id
                        and message.mapping_session_id != previous_session
                        and bool(message.ready)
                    ):
                        self._reset_state = MapState.RESET_SUCCEEDED
                        self._reset_detail = (
                            'Fresh mapping session ready: '
                            f'{message.mapping_session_id}'
                        )
                        self._pending_reset = None
                    elif message.detail.startswith('NEW MAP rejected:'):
                        self._reset_state = MapState.RESET_FAILED
                        self._reset_detail = message.detail
                        self._pending_reset = None
        self._publish_state()

    def _on_map(self, message: OccupancyGrid) -> None:
        now = time.monotonic()
        self._last_map = message
        self._last_map_at = now
        with self._lock:
            session = self._model.session
            produced_after = (
                session.active
                and now - session.created >= SESSION_EVIDENCE_GRACE_SEC
            )
            self._model.observe_map_evidence(
                produced_after=produced_after, now=now
            )
        self._publish_state()

    def _on_timer(self) -> None:
        with self._lock:
            self._model.refresh(time.monotonic())
        self._publish_state()

    # ---- requests -------------------------------------------------------

    def _stop_inhibited(self) -> bool:
        return (
            self._stop_state is not None
            and self._stop_state_at is not None
            and time.monotonic() - self._stop_state_at <= STOP_STATE_TIMEOUT_SEC
            and (self._stop_state.locked or self._stop_state.stopped)
        )

    def _authorized(self, message: MapRequest) -> bool:
        return (
            self._lease_active
            and bool(message.lease_id)
            and message.lease_id == self._lease_id
        )

    def _on_map_request(self, message: MapRequest) -> None:
        if not self._authorized(message):
            reason = 'request does not hold the active control lease'
            self._record_select_rejection(message, reason)
            self._record_new_map_rejection(message, reason)
            self.get_logger().warning(f'rejected map request: {reason}')
            return
        if message.request_id <= self._last_request_id:
            reason = f'stale/replayed map request {message.request_id}'
            self._record_select_rejection(message, reason)
            self._record_new_map_rejection(message, reason)
            self.get_logger().warning(
                f'rejected {reason}'
            )
            return
        self._last_request_id = int(message.request_id)
        operation = int(message.operation)
        try:
            if operation == MapRequest.OP_NEW_MAP:
                self._handle_new_map(message)
            elif operation == MapRequest.OP_SAVE_MAP:
                self._handle_save_map(message)
            elif operation == MapRequest.OP_SELECT_MAP:
                self._handle_select_map(message)
            elif operation == MapRequest.OP_DELETE_MAP:
                self._handle_delete_map(message)
            else:
                self.get_logger().warning(
                    f'rejected unknown map operation {operation}'
                )
        finally:
            self._publish_state()

    def _record_select_rejection(
        self, message: MapRequest, reason: str
    ) -> None:
        """Expose an executor-level SELECT rejection in authoritative state."""
        if int(message.operation) != MapRequest.OP_SELECT_MAP:
            return
        with self._lock:
            self._model.set_selection(
                message.name, self._model.selected_applied, reason
            )
        self._publish_state()

    def _record_new_map_rejection(
        self, message: MapRequest, reason: str
    ) -> None:
        if int(message.operation) != MapRequest.OP_NEW_MAP:
            return
        self._record_reset_rejection(
            int(message.request_id), f'NEW MAP rejected: {reason}'
        )
        self._publish_state()

    def _record_reset_rejection(self, request_id: int, reason: str) -> None:
        with self._lock:
            self._reset_state = MapState.RESET_FAILED
            self._reset_request_id = int(request_id)
            self._reset_detail = reason
            self._pending_reset = None
        self.get_logger().warning(reason)

    def _next_mode_request_id(self) -> int:
        """Return an ID newer than web and executor mode requests."""
        self._new_map_request_seq = max(
            self._new_map_request_seq + 1,
            self._mode_accepted_request_id + 1,
            time.time_ns(),
        )
        return self._new_map_request_seq

    def _handle_new_map(self, message: MapRequest) -> None:
        with self._lock:
            eligible = (
                self._mode == ModeState.MODE_MAPPING
                and self._mode_status == ModeState.STATUS_STABLE
                and self._model.session.active
            )
        if not eligible:
            self._record_reset_rejection(
                int(message.request_id),
                'NEW MAP rejected: requires an active stable MAPPING session',
            )
            return
        mode_request_id = self._next_mode_request_id()
        with self._lock:
            self._reset_state = MapState.RESET_RUNNING
            self._reset_request_id = int(message.request_id)
            self._reset_detail = 'Revoking motion and waiting for encoder stationary'
            self._pending_reset = (
                mode_request_id,
                self._model.session.runtime_epoch,
                self._model.session.session_id,
            )
        request = ModeRequest()
        request.stamp = self.get_clock().now().to_msg()
        request.request_id = mode_request_id
        request.lease_id = message.lease_id
        request.requested_mode = ModeRequest.MODE_MAPPING
        request.operation = ModeRequest.OP_NEW_MAP
        self._mode_request_pub.publish(request)
        self.get_logger().info(
            'forwarded NEW MAP to mode supervisor as OP_NEW_MAP mode request'
        )

    def _handle_select_map(self, message: MapRequest) -> None:
        previous = self._model.selected_applied
        try:
            name = safe_basename(message.name)
            if self._mode_status != ModeState.STATUS_STABLE:
                raise MapError('runtime transition in progress')
            if (
                self._mode == ModeState.MODE_AUTONOMY
                and name != self._model.selected_applied
            ):
                raise MapError('cannot hot-swap the map under a live runtime')
            validate_bundle(MAP_DIRECTORY, name)
            self._store_selection(name)
            with self._lock:
                self._model.set_selection(name, name, '')
            self.get_logger().info(f'selected map {name}')
        except (MapError, OSError) as error:
            reason = (
                str(error) if isinstance(error, MapError)
                else f'could not persist selected map: {error}'
            )
            with self._lock:
                self._model.set_selection(
                    message.name, previous, reason
                )
            self.get_logger().warning(f'map selection rejected: {reason}')

    def _handle_delete_map(self, message: MapRequest) -> None:
        name = message.name
        try:
            name = safe_basename(name)
            validate_delete_candidate(
                MAP_DIRECTORY, name,
                selected=self._model.selected_applied,
                active_autonomy_map=self._active_autonomy_map,
            )
            info = delete_bundle(MAP_DIRECTORY, name)
        except (MapError, OSError) as error:
            detail = str(error)
            with self._lock:
                self._model.record_delete(
                    int(message.request_id), name,
                    DeleteState.FAILED, detail,
                )
            self.get_logger().warning(f'DELETE MAP {name} rejected: {detail}')
            return
        detail = f'deleted saved bundle {info.name}'
        with self._lock:
            self._model.record_delete(
                int(message.request_id), info.name,
                DeleteState.SUCCEEDED, detail,
            )
        self.get_logger().info(f'DELETE MAP {info.name}: {detail}')

    def _handle_save_map(self, message: MapRequest) -> None:
        cached = self._model.cached_save(int(message.request_id))
        if cached is not None:
            with self._lock:
                self._model.record_save(int(message.request_id), cached)
            return
        try:
            name = safe_basename(message.name)
            session_id = self._model.session.session_id
            if not self._model.session.active:
                raise MapError('no active mapping session')
            if message.session_id != session_id:
                raise MapError('request session does not match current session')
            if self._model.session.phase in (
                SessionPhase.STARTING, SessionPhase.NONE
            ):
                raise MapError('current mapping session is not ready')
            if not self._mode_ready:
                raise MapError('mapping runtime is not ready')
            if not self._stop_inhibited():
                raise MapError('SAVE MAP requires STOP asserted/inhibited')
            if self._model.save_state == SaveState.RUNNING:
                raise MapError('a save is already in flight')
        except MapError as error:
            outcome = SaveOutcome(SaveState.FAILED, str(error))
            with self._lock:
                self._model.record_save(int(message.request_id), outcome)
            self.get_logger().warning(f'SAVE MAP rejected: {error}')
            return

        with self._lock:
            self._model.begin_save(int(message.request_id))
        self._publish_state()
        outcome = self._run_save(name, session_id)
        with self._lock:
            self._model.record_save(int(message.request_id), outcome)
        level = (
            self.get_logger().info
            if outcome.state == SaveState.SUCCEEDED
            else self.get_logger().error
        )
        level(f'SAVE MAP {name}: {outcome.detail}')

    def _run_save(self, name: str, session_id: str) -> SaveOutcome:
        txn = MapSaveTransaction(MAP_DIRECTORY, name, session_id)
        try:
            txn.prepare()
        except MapError as error:
            return SaveOutcome(SaveState.FAILED, str(error))

        if not self._serialize_client.wait_for_service(timeout_sec=5.0):
            return SaveOutcome(
                SaveState.FAILED,
                f'{SERIALIZE_SERVICE} unavailable; is slam_toolbox mapping?',
            )
        request = SerializePoseGraph.Request()
        request.filename = txn.serialize_target()
        future = self._serialize_client.call_async(request)
        deadline = time.monotonic() + 120.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return SaveOutcome(SaveState.FAILED, 'SerializePoseGraph timed out')
        response = future.result()
        if response is None or response.result != 0:
            return SaveOutcome(
                SaveState.FAILED,
                f'SerializePoseGraph returned result='
                f'{getattr(response, "result", "none")}',
            )
        try:
            txn.check_serialized()
        except MapError as error:
            return SaveOutcome(SaveState.FAILED, str(error))

        raster = self._capture_raster()
        if raster is None:
            return SaveOutcome(
                SaveState.FAILED,
                'no fresh same-session /map grid to raster',
            )
        try:
            txn.write_raster(raster)
            info = txn.validate()
            if session_id != self._model.session.session_id:
                raise MapError('session changed during save')
            outcome = txn.publish()
        except MapError as error:
            return SaveOutcome(SaveState.FAILED, str(error))
        self.get_logger().info(
            f'saved {name} rev {info.revision or outcome.revision} '
            f'({info.width}x{info.height} @ {info.resolution:.3f} m)'
        )
        return outcome

    def _capture_raster(self) -> OccupancyRaster | None:
        """First /map grid received after this call, else the latest if fresh."""
        start = time.monotonic()
        deadline = start + SAVE_RASTER_TIMEOUT_SEC
        while time.monotonic() < deadline:
            message = self._last_map
            received = self._last_map_at
            if message is not None and received is not None and received >= start:
                return self._to_raster(message)
            time.sleep(0.1)
        message = self._last_map
        received = self._last_map_at
        if (
            message is not None
            and received is not None
            and time.monotonic() - received <= SAVE_RASTER_TIMEOUT_SEC
        ):
            return self._to_raster(message)
        return None

    @staticmethod
    def _to_raster(message: OccupancyGrid) -> OccupancyRaster | None:
        info = message.info
        if not math.isfinite(info.resolution) or info.resolution <= 0.0:
            return None
        orientation = info.origin.orientation
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        return OccupancyRaster(
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=float(yaw),
            data=tuple(int(value) for value in message.data),
            stamp_sec=int(message.header.stamp.sec),
            stamp_nanosec=int(message.header.stamp.nanosec),
        )

    # ---- publication ----------------------------------------------------

    def _publish_state(self) -> None:
        with self._lock:
            model = self._model
            session = model.session
            now = time.monotonic()
            message = MapState()
            message.stamp = self.get_clock().now().to_msg()
            message.session_id = session.session_id
            message.runtime_epoch = session.runtime_epoch
            message.mapping_active = session.active
            message.session_phase = int(session.phase)
            message.session_ready = bool(
                session.ready(now) and self._mode_ready
            )
            message.unsaved = bool(session.active and session.unsaved)
            message.saved_name = session.saved_name
            message.saved_revision = session.saved_revision
            message.save_state = int(model.save_state)
            message.save_request_id = model.save_request_id
            message.save_detail = model.save_detail
            message.selected_map_requested = model.selected_requested
            message.selected_map_applied = model.selected_applied
            message.selected_map_reason = model.selected_reason
            message.delete_state = int(model.delete_state)
            message.delete_request_id = model.delete_request_id
            message.delete_name = model.delete_name
            message.delete_detail = model.delete_detail
            message.reset_state = self._reset_state
            message.reset_request_id = self._reset_request_id
            message.reset_detail = self._reset_detail
            message.catalog = [
                self._catalog_entry(entry) for entry in model.catalog()
            ]
        self._state_pub.publish(message)

    @staticmethod
    def _catalog_entry(entry) -> MapCatalogEntry:
        row = MapCatalogEntry()
        row.name = entry.name
        row.revision = entry.revision
        row.complete = entry.complete
        row.selected = entry.selected
        row.session_id = entry.session_id
        row.reason = entry.reason
        row.resolution = entry.resolution
        row.width = entry.width
        row.height = entry.height
        return row


def main(args=None) -> None:
    """Run the persistent map-session executor."""
    rclpy.init(args=args)
    node = None
    try:
        node = MapSessionNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
