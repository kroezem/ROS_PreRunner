"""Focused tests for speed-envelope origin and per-key reconciliation."""

from pathlib import Path

from runner_bringup.speed_envelope_observer import (
    build_entry,
    load_origin,
    OriginValue,
    ReconciliationStore,
)
from runner_interfaces.msg import SpeedEnvelopeEntry
import yaml


WORKSPACE = Path(__file__).parents[2]
ORIGIN = (
    WORKSPACE / 'runner_drive_adapter' / 'config' / 'speed_envelope.yaml'
)


def test_origin_has_only_expected_consumers_and_flattened_keys():
    """The shared file is the complete narrow two-consumer origin."""
    origins = load_origin(ORIGIN)

    assert len(origins) == 27
    assert {origin.node_name for origin in origins} == {
        '/controller_server',
        '/drive_adapter',
    }
    assert len({origin.key for origin in origins}) == len(origins)
    assert any(
        origin.parameter_name == 'FollowPath.desired_linear_vel'
        and origin.value == 0.45
        for origin in origins
    )
    assert any(
        origin.parameter_name == 'output_max' and origin.value == 0.12
        for origin in origins
    )


def test_origin_values_are_not_duplicated_in_consumer_specific_files():
    """Launched consumer files retain only parameters outside the origin."""
    origins = load_origin(ORIGIN)
    nav2 = yaml.safe_load(
        (WORKSPACE / 'runner_bringup/config/nav2_params.yaml').read_text()
    )['controller_server']['ros__parameters']
    adapter = yaml.safe_load(
        (WORKSPACE / 'runner_drive_adapter/config/drive_adapter.yaml').read_text()
    )['drive_adapter']['ros__parameters']

    for origin in origins:
        parts = origin.parameter_name.split('.')
        consumer = nav2 if origin.node_name == '/controller_server' else adapter
        if len(parts) == 1:
            assert parts[0] not in consumer
        else:
            assert parts[1] not in consumer[parts[0]]


def test_one_timeout_does_not_suppress_an_available_key():
    """One expired request remains unknown while another key stays usable."""
    available = OriginValue('node.available', '/node', 'available', 0.45)
    missing = OriginValue('node.missing', '/node', 'missing', 0.15)
    store = ReconciliationStore([available, missing])

    available_token = store.begin(available.key, now=10.0, timeout=0.25)
    store.begin(missing.key, now=10.0, timeout=0.25)
    assert store.resolve(available.key, available_token, value=0.45)
    store.expire(now=10.251)

    entries = [
        build_entry(origin, store.observations[origin.key])
        for origin in store.origins
    ]
    assert len(entries) == 2
    assert entries[0].available is True
    assert entries[0].divergence == SpeedEnvelopeEntry.DIVERGENCE_MATCH
    assert entries[1].available is False
    assert entries[1].divergence == SpeedEnvelopeEntry.DIVERGENCE_UNKNOWN
    assert entries[1].detail == 'timeout'


def test_stale_completion_cannot_overwrite_timeout_or_newer_attempt():
    """A late service reply is ignored after its per-key deadline."""
    origin = OriginValue('node.value', '/node', 'value', 1.0)
    store = ReconciliationStore([origin])
    old_token = store.begin(origin.key, now=1.0, timeout=0.25)
    store.expire(now=1.3)
    new_token = store.begin(origin.key, now=2.0, timeout=0.25)

    assert not store.resolve(origin.key, old_token, value=9.0)
    assert store.resolve(origin.key, new_token, value=1.0)
    assert store.observations[origin.key].value == 1.0
