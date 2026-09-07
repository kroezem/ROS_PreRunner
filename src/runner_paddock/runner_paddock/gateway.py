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
(NEW / SAVE / SELECT map). It manufactures nothing on its own: an intent is
produced only in direct response to a fresh browser message, so if the browser
goes silent the Pi-side lease simply expires and RUN is revoked.

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
        return GatewayResult(
            True,
            'control lease acquired',
            (self._control(ControlEvent.LEASE_ACQUIRED),),
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
        return GatewayResult(
            True, f'goal ({x:.2f}, {y:.2f}, {yaw:.2f}) selected',
            (self._control(
                ControlEvent.GOAL_SELECTED,
                goal_frame=frame, goal_x=x, goal_y=y, goal_yaw=yaw,
            ),),
            'controller',
        )

    # -- helpers ---------------------------------------------------------------

    def _require_owner(self, conn_id: str) -> Optional[GatewayResult]:
        if conn_id != self._owner_conn:
            return self._reject(conn_id, 'this session does not hold the lease')
        return None

    def _reject(self, conn_id: str, reason: str) -> GatewayResult:
        return GatewayResult(False, reason, (), self.role_for(conn_id))

    def _control(self, event: ControlEvent, **goal) -> ControlEventIntent:
        self._sequence += 1
        return ControlEventIntent(
            event=event,
            sequence=self._sequence,
            client_id=self._client_id,
            lease_id=self._lease_id,
            **goal,
        )
