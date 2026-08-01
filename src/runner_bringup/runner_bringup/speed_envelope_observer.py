"""Publish non-authoritative reconciliation of the speed-envelope origin."""

from dataclasses import dataclass
import math
from pathlib import Path
import time

from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from runner_interfaces.msg import SpeedEnvelopeEntry, SpeedEnvelopeStatus
import yaml


STATUS_TOPIC = '/speed_envelope/status'
PUBLICATION_PERIOD_SEC = 1.0
DEFAULT_REQUEST_TIMEOUT_SEC = 0.25


@dataclass(frozen=True)
class OriginValue:
    """One flattened parameter from the committed origin."""

    key: str
    node_name: str
    parameter_name: str
    value: bool | int | float


@dataclass
class Observation:
    """Latest independent read attempt for one origin value."""

    token: int = 0
    deadline: float = 0.0
    in_flight: bool = False
    available: bool = False
    value: bool | int | float | None = None
    detail: str = 'not_requested'


def _flatten(prefix, values):
    for name, value in values.items():
        parameter_name = f'{prefix}.{name}' if prefix else name
        if isinstance(value, dict):
            yield from _flatten(parameter_name, value)
        elif isinstance(value, (bool, int, float)):
            yield parameter_name, value
        else:
            raise ValueError(
                f'unsupported origin value for {parameter_name}: {value!r}'
            )


def load_origin(path: Path) -> list[OriginValue]:
    """Load and flatten every node parameter in the shared origin."""
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or not document:
        raise ValueError('speed-envelope origin must contain node mappings')

    result = []
    for node_name, node_block in document.items():
        try:
            parameters = node_block['ros__parameters']
        except (KeyError, TypeError) as error:
            raise ValueError(
                f'{node_name} has no ros__parameters mapping'
            ) from error
        for parameter_name, value in _flatten('', parameters):
            result.append(OriginValue(
                key=f'{node_name}.{parameter_name}',
                node_name=f'/{node_name}',
                parameter_name=parameter_name,
                value=value,
            ))
    return result


def parameter_value(message):
    """Convert one parameter-service value to a supported Python scalar."""
    if message.type == ParameterType.PARAMETER_BOOL:
        return message.bool_value
    if message.type == ParameterType.PARAMETER_INTEGER:
        return message.integer_value
    if message.type == ParameterType.PARAMETER_DOUBLE:
        return message.double_value
    raise ValueError(f'unsupported observed parameter type {message.type}')


def values_equal(origin, observed):
    """Compare like-typed scalar values without hiding type divergence."""
    if isinstance(origin, bool):
        return isinstance(observed, bool) and origin == observed
    if isinstance(origin, int):
        return (
            isinstance(observed, int)
            and not isinstance(observed, bool)
            and origin == observed
        )
    return (
        isinstance(observed, float)
        and math.isclose(origin, observed, rel_tol=1e-12, abs_tol=1e-12)
    )


class ReconciliationStore:
    """Track independent asynchronous read attempts without blocking."""

    def __init__(self, origins):
        self.origins = list(origins)
        self.observations = {
            origin.key: Observation() for origin in self.origins
        }

    def begin(self, key, now, timeout):
        observation = self.observations[key]
        observation.token += 1
        observation.deadline = now + timeout
        observation.in_flight = True
        observation.available = False
        observation.value = None
        observation.detail = 'pending'
        return observation.token

    def resolve(self, key, token, value=None, detail='available'):
        observation = self.observations[key]
        if token != observation.token or not observation.in_flight:
            return False
        observation.in_flight = False
        observation.available = detail == 'available'
        observation.value = value if observation.available else None
        observation.detail = detail
        return True

    def expire(self, now):
        for observation in self.observations.values():
            if observation.in_flight and now >= observation.deadline:
                observation.in_flight = False
                observation.available = False
                observation.value = None
                observation.detail = 'timeout'


def build_entry(origin, observation):
    """Build one typed origin/observation entry."""
    message = SpeedEnvelopeEntry()
    message.key = origin.key
    message.node_name = origin.node_name
    message.parameter_name = origin.parameter_name
    if isinstance(origin.value, bool):
        message.value_type = SpeedEnvelopeEntry.TYPE_BOOL
        message.origin_bool = origin.value
    elif isinstance(origin.value, int):
        message.value_type = SpeedEnvelopeEntry.TYPE_INTEGER
        message.origin_integer = origin.value
    else:
        message.value_type = SpeedEnvelopeEntry.TYPE_DOUBLE
        message.origin_double = origin.value

    message.available = observation.available
    message.detail = observation.detail
    if not observation.available:
        message.divergence = SpeedEnvelopeEntry.DIVERGENCE_UNKNOWN
        return message

    observed = observation.value
    if isinstance(observed, bool):
        message.observed_bool = observed
    elif isinstance(observed, int):
        message.observed_integer = observed
    elif isinstance(observed, float):
        message.observed_double = observed
    message.divergence = (
        SpeedEnvelopeEntry.DIVERGENCE_MATCH
        if values_equal(origin.value, observed)
        else SpeedEnvelopeEntry.DIVERGENCE_DIFFERENT
    )
    return message


class SpeedEnvelopeObserver(Node):
    """Reconcile origin values through nonblocking parameter-service calls."""

    def __init__(self):
        super().__init__('speed_envelope_observer')
        self.declare_parameter('origin_file', '')
        self.declare_parameter(
            'request_timeout_sec', DEFAULT_REQUEST_TIMEOUT_SEC
        )
        origin_file = Path(self.get_parameter('origin_file').value)
        timeout = self.get_parameter('request_timeout_sec').value
        if not origin_file.is_file():
            raise ValueError(f'origin_file does not exist: {origin_file}')
        if not isinstance(timeout, float) or not 0.0 < timeout < 1.0:
            raise ValueError('request_timeout_sec must be a float within (0, 1)')

        self._origin_file = origin_file
        self._request_timeout = timeout
        self._store = ReconciliationStore(load_origin(origin_file))
        self._parameter_clients = {
            node_name: AsyncParameterClient(self, node_name)
            for node_name in {
                origin.node_name for origin in self._store.origins
            }
        }
        self._publisher = self.create_publisher(
            SpeedEnvelopeStatus, STATUS_TOPIC, 10
        )
        self.create_timer(PUBLICATION_PERIOD_SEC, self._tick)
        self.get_logger().info(
            f'observing {len(self._store.origins)} speed-envelope values '
            f'from {origin_file}'
        )

    def _tick(self):
        now = time.monotonic()
        self._store.expire(now)
        self._publish()
        for origin in self._store.origins:
            observation = self._store.observations[origin.key]
            if observation.in_flight:
                continue
            client = self._parameter_clients[origin.node_name]
            if not client.services_are_ready():
                token = self._store.begin(
                    origin.key, now, self._request_timeout
                )
                self._store.resolve(
                    origin.key, token, detail='service_unavailable'
                )
                continue
            token = self._store.begin(
                origin.key, now, self._request_timeout
            )
            future = client.get_parameters([origin.parameter_name])
            future.add_done_callback(
                lambda completed, item=origin, request_token=token:
                self._complete(item, request_token, completed)
            )

    def _complete(self, origin, token, future):
        try:
            response = future.result()
            if len(response.values) != 1:
                raise ValueError('parameter service returned no value')
            value = parameter_value(response.values[0])
        except Exception as error:  # service errors remain per-key diagnostics
            self._store.resolve(
                origin.key,
                token,
                detail=f'error:{type(error).__name__}',
            )
            return
        self._store.resolve(origin.key, token, value=value)

    def _publish(self):
        message = SpeedEnvelopeStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.origin_file = str(self._origin_file)
        message.entries = [
            build_entry(origin, self._store.observations[origin.key])
            for origin in self._store.origins
        ]
        message.all_available = all(
            entry.available for entry in message.entries
        )
        message.any_divergence = any(
            entry.divergence == SpeedEnvelopeEntry.DIVERGENCE_DIFFERENT
            for entry in message.entries
        )
        self._publisher.publish(message)


def main():
    """Run the speed-envelope observer."""
    rclpy.init()
    node = None
    try:
        node = SpeedEnvelopeObserver()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except ValueError as error:
        rclpy.logging.get_logger('speed_envelope_observer').error(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
