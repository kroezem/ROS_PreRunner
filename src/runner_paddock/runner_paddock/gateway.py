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
ROS-free operator-intent gateway for the Paddock browser.

One browser session at a time holds the single control lease; every other
connection is a read-only observer. The gateway translates validated browser
actions into ordered, typed intents that the ROS layer publishes verbatim on
``/paddock/control_event`` (RUN / STOP / CLEAR STOP / goal / lease / heartbeat),
``/paddock/mode_request`` (runtime selection) and ``/paddock/map_request``
(NEW / SAVE / SELECT / DELETE map). It manufactures nothing on its own: an
intent is produced only in direct response to a fresh browser message, so if
the browser goes silent the Pi-side lease simply expires and RUN is revoked.

State safety: a reconnecting browser starts with no lease, no RUN and no goal;
the gateway never replays a prior dispatch or STOP clear.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Optional


# Mirror of runner_interfaces/PaddockControlEvent event constants.
class ControlEvent(IntEnum):
    """Ordered control events accepted from the current lease holder."""

    RUN_PRESSED = 0
    RUN_RELEASED = 1
    STOP = 2
    MANUAL_ACTIVE = 3
    MANUAL_INACTIVE = 4
    GOAL_SELECTED = 5
    LEASE_ACQUIRED = 6
    LEASE_RELEASED = 7
    LEASE_LOST = 8
    HEARTBEAT = 9
    CLEAR_STOP = 10


# Mirror of runner_interfaces/ModeRequest.
MODE_IDLE = 0
MODE_MAPPING = 1
MODE_AUTONOMY = 2
MODE_OP_SELECT_RUNTIME = 0
MODE_OP_NEW_MAP = 1

# Mirror of runner_interfaces/MapRequest.
MAP_OP_NEW_MAP = 0
MAP_OP_SAVE_MAP = 1
MAP_OP_SELECT_MAP = 2
MAP_OP_DELETE_MAP = 3
RECORDING_OP_START = 0
RECORDING_OP_STOP = 1
RECORDING_OP_DELETE = 2

_MODE_NAMES = {'idle': MODE_IDLE, 'mapping': MODE_MAPPING, 'autonomy': MODE_AUTONOMY}


@dataclass(frozen=True)
class ControlEventIntent:
    """One ``/paddock/control_event`` message to publish."""

    event: ControlEvent
    sequence: int
    client_id: str
    lease_id: str
    goal_frame: str = 'map'
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    manual_speed_mps: float = 0.0
    manual_steering: float = 0.0


@dataclass(frozen=True)
class ModeRequestIntent:
    """One ``/paddock/mode_request`` message to publish."""

    requested_mode: int
    operation: int
    lease_id: str
    autonomy_map: str = ''


@dataclass(frozen=True)
class MapRequestIntent:
    """One ``/paddock/map_request`` message to publish."""

    operation: int
    lease_id: str
    name: str = ''
    session_id: str = ''


@dataclass(frozen=True)
class ConfigRequestIntent:
    """One revision-checked supported operational configuration request."""

    lease_id: str
    expected_revision: int
    field: str
    value: float


@dataclass(frozen=True)
class ClearCostmapsIntent:
    """Request Nav2's existing global and local full-clear services."""


@dataclass(frozen=True)
class ObstacleProcessingIntent:
    """Set one Nav2 costmap obstacle plugin's live enabled parameter."""

    costmap: str
    enabled: bool


@dataclass(frozen=True)
class AutonomyTuningIntent:
    """Apply one complete preset or manually edited live tuning snapshot."""

    preset: str = ''
    values: dict = field(default_factory=dict)


@dataclass(frozen=True)
class InitialPoseIntent:
    """One validated map-frame seed for slam_toolbox localization."""

    x: float
    y: float
    yaw: float
    frame: str = 'map'


@dataclass(frozen=True)
class RecordingRequestIntent:
    """One persistent recording-executor request."""

    operation: int
    lease_id: str
    name: str = ''
    profile: str = ''


@dataclass(frozen=True)
class GatewayResult:
    """Outcome of one browser action."""

    accepted: bool
    reason: str
    intents: tuple = ()
    # Client-visible lease role after applying this action.
    role: str = 'observer'


@dataclass
class OperatorGateway:
    """Single-lease operator intent state machine, ROS- and clock-free."""

    _sequence: int = 0
    _owner_conn: Optional[str] = None
    _client_id: str = ''
    _lease_id: str = ''
    _run_pressed: bool = False
    _manual_active: bool = False
    _uuid_factory: object = field(default=None)

    def __post_init__(self) -> None:
        if self._uuid_factory is None:
            import uuid

            self._uuid_factory = lambda: uuid.uuid4().hex

    # -- queries ---------------------------------------------------------------

    @property
    def lease_held(self) -> bool:
        """Return whether any browser connection currently holds the lease."""
        return self._owner_conn is not None

    def role_for(self, conn_id: str) -> str:
        """Return 'controller' for the lease owner, else 'observer'."""
        return 'controller' if conn_id == self._owner_conn else 'observer'

    def public_state(self) -> dict:
        """Small serialisable view for the browser status panel."""
        return {
            'lease_held': self.lease_held,
            'client_id': self._client_id,
            'lease_id': self._lease_id,
            'sequence': self._sequence,
            'run_pressed': self._run_pressed,
        }

    # -- lifecycle -----------------------------------------------------------

    def on_disconnect(self, conn_id: str) -> GatewayResult:
        """Release the lease if this connection held it; never leaves it stuck."""
        if conn_id != self._owner_conn:
            return GatewayResult(True, 'observer disconnected')
        intent = self._control(ControlEvent.LEASE_RELEASED)
        self._owner_conn = None
        self._client_id = ''
        self._lease_id = ''
        self._run_pressed = False
        self._manual_active = False
        return GatewayResult(True, 'lease released on disconnect', (intent,))

    # -- actions -----------------------------------------------------------

    def handle(self, conn_id: str, action: dict) -> GatewayResult:
        """Validate and translate one browser action message."""
        if not isinstance(action, dict):
            return self._reject(conn_id, 'action must be an object')
        name = action.get('action')
        handler = getattr(self, f'_do_{name}', None) if isinstance(name, str) else None
        if handler is None:
            return self._reject(conn_id, f'unknown action {name!r}')
        return handler(conn_id, action)

    def _do_acquire(self, conn_id: str, _action: dict) -> GatewayResult:
        if self._owner_conn == conn_id:
            return GatewayResult(True, 'already holding the lease', (), 'controller')
        if self._owner_conn is not None:
            return GatewayResult(
                False, 'another session holds the control lease', (), 'observer'
            )
        self._owner_conn = conn_id
        self._client_id = self._uuid_factory()
        self._lease_id = self._uuid_factory()
        self._run_pressed = False
        self._manual_active = False
        return GatewayResult(
            True,
            'control lease acquired',
            (self._control(ControlEvent.LEASE_ACQUIRED),),
            'controller',
        )

    def _do_takeover(self, conn_id: str, _action: dict) -> GatewayResult:
        """Revoke the current owner, then atomically grant a fresh lease."""
        if self._owner_conn == conn_id:
            return GatewayResult(
                True, 'already holding the lease', (), 'controller'
            )
        if self._owner_conn is None:
            return self._do_acquire(conn_id, _action)

        lost = self._control(ControlEvent.LEASE_LOST)
        self._owner_conn = None
        self._client_id = ''
        self._lease_id = ''
        self._run_pressed = False
        self._manual_active = False

        self._owner_conn = conn_id
        self._client_id = self._uuid_factory()
        self._lease_id = self._uuid_factory()
        acquired = self._control(ControlEvent.LEASE_ACQUIRED)
        return GatewayResult(
            True,
            'control lease taken over',
            (lost, acquired),
            'controller',
        )

    def _do_release(self, conn_id: str, _action: dict) -> GatewayResult:
        if conn_id != self._owner_conn:
            return self._reject(conn_id, 'not the lease owner')
        return self.on_disconnect(conn_id)

    def _do_heartbeat(self, conn_id: str, _action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True, 'heartbeat', (self._control(ControlEvent.HEARTBEAT),), 'controller'
        )

    def _do_run(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        held = bool(action.get('held'))
        if held and not self._run_pressed:
            self._run_pressed = True
            return GatewayResult(
                True, 'RUN pressed',
                (self._control(ControlEvent.RUN_PRESSED),), 'controller',
            )
        if held and self._run_pressed:
            # Keep the lease fresh while the button is held; the authority
            # keeps run_held latched until an explicit RUN_RELEASED.
            return GatewayResult(
                True, 'RUN held',
                (self._control(ControlEvent.HEARTBEAT),), 'controller',
            )
        if not held and self._run_pressed:
            self._run_pressed = False
            return GatewayResult(
                True, 'RUN released',
                (self._control(ControlEvent.RUN_RELEASED),), 'controller',
            )
        return GatewayResult(True, 'RUN idle', (), 'controller')

    def _do_stop(self, conn_id: str, _action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        self._run_pressed = False
        self._manual_active = False
        return GatewayResult(
            True, 'STOP requested',
            (self._control(ControlEvent.STOP),), 'controller',
        )

    def _do_clear_stop(self, conn_id: str, _action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True, 'CLEAR STOP requested',
            (self._control(ControlEvent.CLEAR_STOP),), 'controller',
        )

    def _do_manual(self, conn_id: str, action: dict) -> GatewayResult:
        """Validate one fresh joystick sample or explicit release."""
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        active = bool(action.get('active'))
        if not active:
            if not self._manual_active:
                return GatewayResult(True, 'manual idle', (), 'controller')
            self._manual_active = False
            return GatewayResult(
                True,
                'manual released',
                (self._control(ControlEvent.MANUAL_INACTIVE),),
                'controller',
            )
        try:
            speed = float(action['speed_mps'])
            steering = float(action['steering'])
        except (KeyError, TypeError, ValueError):
            return self._reject(conn_id, 'manual needs finite speed and steering')
        if not all(math.isfinite(value) for value in (speed, steering)):
            return self._reject(conn_id, 'manual needs finite speed and steering')
        self._manual_active = True
        self._run_pressed = False
        return GatewayResult(
            True,
            'manual demand',
            (self._control(
                ControlEvent.MANUAL_ACTIVE,
                manual_speed_mps=speed,
                manual_steering=steering,
            ),),
            'controller',
        )

    def _do_clear_obstacles(
        self, conn_id: str, _action: dict
    ) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True,
            'Nav2 costmap clear requested',
            (ClearCostmapsIntent(),),
            'controller',
        )

    def _do_set_obstacle_processing(
        self, conn_id: str, action: dict
    ) -> GatewayResult:
        """Validate an explicit global/local live obstacle-layer state."""
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        costmap = str(action.get('costmap', '')).strip().lower()
        if costmap not in ('global', 'local'):
            return self._reject(
                conn_id, 'obstacle processing costmap must be global or local'
            )
        enabled = action.get('enabled')
        if not isinstance(enabled, bool):
            return self._reject(
                conn_id, 'obstacle processing enabled must be boolean'
            )
        return GatewayResult(
            True,
            f'{costmap} obstacle processing '
            f'{"ON" if enabled else "OFF"} requested',
            (ObstacleProcessingIntent(costmap, enabled),),
            'controller',
        )

    def _do_set_autonomy_tuning(
        self, conn_id: str, action: dict
    ) -> GatewayResult:
        """Validate the shape of a lease-scoped live tuning request."""
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        preset = str(action.get('preset', '')).strip().lower()
        values = action.get('values', {})
        if preset:
            if preset not in ('timid', 'confident') or values:
                return self._reject(conn_id, 'invalid autonomy preset request')
        elif not isinstance(values, dict):
            return self._reject(conn_id, 'autonomy tuning values must be an object')
        return GatewayResult(
            True,
            f'autonomy tuning {preset or "custom"} requested',
            (AutonomyTuningIntent(preset=preset, values=values),),
            'controller',
        )

    def _do_set_config(self, conn_id: str, action: dict) -> GatewayResult:
        """Validate one allowlisted operational configuration request."""
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        field = str(action.get('field', ''))
        if field != 'manual_max_speed_mps':
            return self._reject(conn_id, 'unsupported configuration field')
        try:
            value = float(action['value'])
            revision = int(action['expected_revision'])
        except (KeyError, TypeError, ValueError):
            return self._reject(conn_id, 'configuration value/revision invalid')
        if not math.isfinite(value) or revision < 0:
            return self._reject(conn_id, 'configuration value/revision invalid')
        return GatewayResult(
            True,
            f'{field} configuration requested',
            (ConfigRequestIntent(
                lease_id=self._lease_id,
                expected_revision=revision,
                field=field,
                value=value,
            ),),
            'controller',
        )

    def _do_start_recording(
        self, conn_id: str, action: dict
    ) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True,
            'recording start requested',
            (RecordingRequestIntent(
                RECORDING_OP_START,
                self._lease_id,
                name=str(action.get('name', '')).strip(),
                profile=str(action.get('profile', 'runner_debug')).strip(),
            ),),
            'controller',
        )

    def _do_stop_recording(
        self, conn_id: str, _action: dict
    ) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True,
            'recording stop requested',
            (RecordingRequestIntent(
                RECORDING_OP_STOP, self._lease_id
            ),),
            'controller',
        )

    def _do_delete_recording(
        self, conn_id: str, action: dict
    ) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        name = str(action.get('name', '')).strip()
        if not name:
            return self._reject(conn_id, 'delete requires a recording name')
        return GatewayResult(
            True,
            f'recording delete {name} requested',
            (RecordingRequestIntent(
                RECORDING_OP_DELETE, self._lease_id, name=name
            ),),
            'controller',
        )

    def _do_select_mode(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        mode = _MODE_NAMES.get(str(action.get('mode', '')).lower())
        if mode is None:
            return self._reject(conn_id, 'mode must be idle, mapping or autonomy')
        return GatewayResult(
            True, f'runtime {action.get("mode")} requested',
            (ModeRequestIntent(mode, MODE_OP_SELECT_RUNTIME, self._lease_id),),
            'controller',
        )

    def _do_new_map(self, conn_id: str, _action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        return GatewayResult(
            True, 'NEW MAP requested',
            (MapRequestIntent(MAP_OP_NEW_MAP, self._lease_id),), 'controller',
        )

    def _do_save_map(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        name = str(action.get('name', '')).strip()
        if not name:
            return self._reject(conn_id, 'save requires a map basename')
        return GatewayResult(
            True, f'SAVE MAP {name} requested',
            (MapRequestIntent(MAP_OP_SAVE_MAP, self._lease_id, name=name),),
            'controller',
        )

    def _do_select_map(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        name = str(action.get('name', '')).strip()
        if not name:
            return self._reject(conn_id, 'select requires a map basename')
        return GatewayResult(
            True, f'SELECT MAP {name} requested',
            (MapRequestIntent(MAP_OP_SELECT_MAP, self._lease_id, name=name),),
            'controller',
        )

    def _do_delete_map(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        name = str(action.get('name', '')).strip()
        if not name:
            return self._reject(conn_id, 'delete requires a map basename')
        return GatewayResult(
            True, f'DELETE MAP {name} requested',
            (MapRequestIntent(MAP_OP_DELETE_MAP, self._lease_id, name=name),),
            'controller',
        )

    def _do_select_goal(self, conn_id: str, action: dict) -> GatewayResult:
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        try:
            x = float(action['x'])
            y = float(action['y'])
            yaw = float(action.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            return self._reject(conn_id, 'goal needs finite x, y and yaw')
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return self._reject(conn_id, 'goal needs finite x, y and yaw')
        frame = str(action.get('frame', 'map')) or 'map'
        if frame != 'map':
            return self._reject(conn_id, 'goal frame must be map')
        return GatewayResult(
            True, f'goal ({x:.2f}, {y:.2f}, {yaw:.2f}) selected',
            (self._control(
                ControlEvent.GOAL_SELECTED,
                goal_frame=frame, goal_x=x, goal_y=y, goal_yaw=yaw,
            ),),
            'controller',
        )

    def _do_set_initial_pose(
        self, conn_id: str, action: dict
    ) -> GatewayResult:
        """Validate a map-frame localization seed for backend application."""
        owned = self._require_owner(conn_id)
        if owned is not None:
            return owned
        try:
            x = float(action['x'])
            y = float(action['y'])
            yaw = float(action.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            return self._reject(
                conn_id, 'initial pose needs finite x, y and yaw'
            )
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return self._reject(
                conn_id, 'initial pose needs finite x, y and yaw'
            )
        frame = str(action.get('frame', 'map')) or 'map'
        if frame != 'map':
            return self._reject(conn_id, 'initial pose frame must be map')
        self._run_pressed = False
        self._manual_active = False
        return GatewayResult(
            True,
            f'initial pose ({x:.2f}, {y:.2f}, {yaw:.2f}) requested',
            (
                self._control(ControlEvent.STOP),
                InitialPoseIntent(x=x, y=y, yaw=yaw, frame=frame),
            ),
            'controller',
        )

    # -- helpers ---------------------------------------------------------------

    def _require_owner(self, conn_id: str) -> Optional[GatewayResult]:
        if conn_id != self._owner_conn:
            return self._reject(conn_id, 'this session does not hold the lease')
        return None

    def _reject(self, conn_id: str, reason: str) -> GatewayResult:
        return GatewayResult(False, reason, (), self.role_for(conn_id))

    def _control(self, event: ControlEvent, **fields) -> ControlEventIntent:
        self._sequence += 1
        return ControlEventIntent(
            event=event,
            sequence=self._sequence,
            client_id=self._client_id,
            lease_id=self._lease_id,
            **fields,
        )
