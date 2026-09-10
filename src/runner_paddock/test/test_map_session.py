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

"""Tests for the ROS-free map catalog, save transaction, and session model."""

import json

import pytest

from runner_paddock.map_session import (
    build_manifest,
    CORE_EXTENSIONS,
    delete_bundle,
    MapError,
    MapSaveTransaction,
    MapSessionModel,
    OccupancyRaster,
    safe_basename,
    SaveOutcome,
    SaveState,
    scan_catalog,
    SessionPhase,
    validate_bundle,
    validate_delete_candidate,
)


def raster(width=4, height=3):
    """Return a deterministic grid: one occupied cell, one free, rest unknown."""
    data = [-1] * (width * height)
    data[0] = 100
    data[1] = 0
    return OccupancyRaster(
        width=width,
        height=height,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        data=tuple(data),
    )


def write_bundle(directory, name='studio', *, manifest=True, session='s1'):
    """Write a complete four-file bundle, optionally with a manifest."""
    grid = raster(20, 16)
    (directory / f'{name}.posegraph').write_bytes(b'posegraph-bytes')
    (directory / f'{name}.data').write_bytes(b'data-bytes')
    (directory / f'{name}.pgm').write_bytes(grid.to_pgm_bytes())
    (directory / f'{name}.yaml').write_text(grid.to_yaml_text(f'{name}.pgm'))
    if manifest:
        document = build_manifest(session, name, directory, created=1.0)
        (directory / f'{name}.manifest.json').write_text(
            json.dumps(document, sort_keys=True)
        )


@pytest.mark.parametrize(
    'name', ['', '../evil', 'a/b', 'a\\b', '..', '.', 'x' * 65, '-lead']
)
def test_safe_basename_rejects_unsafe_names(name):
    with pytest.raises(MapError):
        safe_basename(name)


def test_safe_basename_accepts_bounded_basename():
    assert safe_basename('studio_2.v1-final') == 'studio_2.v1-final'


def test_pgm_and_yaml_round_trip_is_plausible(tmp_path):
    write_bundle(tmp_path, 'room')
    info = validate_bundle(tmp_path, 'room')
    assert (info.width, info.height) == (20, 16)
    assert info.resolution == pytest.approx(0.05)
    assert info.session_id == 's1'
    assert info.revision  # 12-hex manifest revision


def test_catalog_hides_incomplete_and_reports_reason(tmp_path):
    write_bundle(tmp_path, 'good')
    write_bundle(tmp_path, 'broken', manifest=False)
    (tmp_path / 'broken.pgm').unlink()

    catalog = {entry.name: entry for entry in scan_catalog(tmp_path)}

    assert catalog['good'].complete
    assert not catalog['broken'].complete
    assert 'raster' in catalog['broken'].reason


def test_model_catalog_reuses_validation_until_files_change(
    tmp_path, monkeypatch
):
    calls = 0
    original = scan_catalog

    def counted(path, selected=''):
        nonlocal calls
        calls += 1
        return original(path, selected)

    monkeypatch.setattr('runner_paddock.map_session.scan_catalog', counted)
    model = MapSessionModel(map_dir=tmp_path)
    assert model.catalog() == []
    assert model.catalog() == []
    assert calls == 1

    (tmp_path / 'new.yaml').write_text('image: new.pgm\n')
    model.catalog()
    assert calls == 2


def test_catalog_flags_manifest_hash_mismatch(tmp_path):
    write_bundle(tmp_path, 'tampered')
    (tmp_path / 'tampered.data').write_bytes(b'a-different-payload')

    entry = next(
        item for item in scan_catalog(tmp_path) if item.name == 'tampered'
    )
    assert not entry.complete
    assert 'hash mismatch' in entry.reason


def test_manifest_revision_is_deterministic_for_identical_artifacts(tmp_path):
    (tmp_path / 'a.posegraph').write_bytes(b'p')
    (tmp_path / 'a.data').write_bytes(b'd')
    (tmp_path / 'a.pgm').write_bytes(b'i')
    (tmp_path / 'a.yaml').write_text('image: a.pgm\n')

    first = build_manifest('sX', 'a', tmp_path, created=1.0)
    second = build_manifest('sY', 'a', tmp_path, created=999.0)

    assert first['revision'] == second['revision']
    assert len(first['revision']) == 12


def test_save_transaction_publishes_atomically(tmp_path):
    txn = MapSaveTransaction(tmp_path, 'field', 'sess-7')
    txn.prepare()
    txn.staged_path('posegraph').write_bytes(b'posegraph')
    txn.staged_path('data').write_bytes(b'data')
    txn.check_serialized()
    txn.write_raster(raster(30, 20))
    txn.validate()

    # Nothing selectable before publish.
    assert scan_catalog(tmp_path) == []

    outcome = txn.publish()

    assert outcome.state == SaveState.SUCCEEDED
    catalog = scan_catalog(tmp_path)
    assert [entry.name for entry in catalog] == ['field']
    assert catalog[0].complete
    manifest = json.loads((tmp_path / 'field.manifest.json').read_text())
    assert manifest['session_id'] == 'sess-7'
    for extension in CORE_EXTENSIONS:
        assert (tmp_path / f'field.{extension}').stat().st_size > 0


def test_save_transaction_refuses_to_clobber_existing_bundle(tmp_path):
    write_bundle(tmp_path, 'field')
    txn = MapSaveTransaction(tmp_path, 'field', 'sess-9')
    with pytest.raises(MapError, match='already exists'):
        txn.prepare()


def test_save_transaction_rejects_empty_serialized_artifacts(tmp_path):
    txn = MapSaveTransaction(tmp_path, 'field', 'sess-1')
    txn.prepare()
    txn.staged_path('posegraph').write_bytes(b'')
    txn.staged_path('data').write_bytes(b'data')
    with pytest.raises(MapError, match='non-empty .posegraph'):
        txn.check_serialized()


def test_delete_bundle_removes_only_a_complete_named_bundle(tmp_path):
    write_bundle(tmp_path, 'keep')
    write_bundle(tmp_path, 'remove')

    deleted = delete_bundle(tmp_path, 'remove')

    assert deleted.name == 'remove'
    assert [entry.name for entry in scan_catalog(tmp_path)] == ['keep']
    assert not list(tmp_path.glob('remove.*'))


def test_delete_bundle_rejects_invalid_or_incomplete_bundle(tmp_path):
    write_bundle(tmp_path, 'broken')
    (tmp_path / 'broken.data').unlink()

    with pytest.raises(MapError, match='missing or empty .data'):
        delete_bundle(tmp_path, 'broken')

    assert (tmp_path / 'broken.posegraph').exists()


def test_delete_policy_rejects_selected_and_active_autonomy_maps(tmp_path):
    write_bundle(tmp_path, 'selected')
    write_bundle(tmp_path, 'active')

    with pytest.raises(MapError, match='selected map'):
        validate_delete_candidate(
            tmp_path, 'selected', selected='selected', active_autonomy_map=''
        )
    with pytest.raises(MapError, match='used by AUTONOMY'):
        validate_delete_candidate(
            tmp_path, 'active', selected='', active_autonomy_map='active'
        )

    assert validate_bundle(tmp_path, 'selected')
    assert validate_bundle(tmp_path, 'active')


def test_session_id_change_discards_prior_session_state(tmp_path):
    model = MapSessionModel(map_dir=tmp_path)
    model.observe_runtime(
        mapping_active=True, session_id='epoch-1', runtime_epoch=1, now=10.0
    )
    model.observe_map_evidence(produced_after=True, now=13.0)
    assert model.session.ready(13.5)
    assert model.session.phase == SessionPhase.READY

    model.observe_runtime(
        mapping_active=True, session_id='epoch-2', runtime_epoch=2, now=20.0
    )

    # Old-session evidence cannot satisfy the new session.
    assert model.session.session_id == 'epoch-2'
    assert model.session.phase == SessionPhase.STARTING
    assert not model.session.ready(20.5)
    assert model.session.unsaved


def test_stale_evidence_drops_session_ready(tmp_path):
    model = MapSessionModel(map_dir=tmp_path)
    model.observe_runtime(
        mapping_active=True, session_id='e1', runtime_epoch=1, now=0.0
    )
    model.observe_map_evidence(produced_after=True, now=1.0)
    assert model.session.ready(2.0)

    model.refresh(now=100.0)
    assert model.session.phase == SessionPhase.STARTING
    assert not model.session.ready(100.0)


def test_save_result_cache_is_idempotent(tmp_path):
    model = MapSessionModel(map_dir=tmp_path)
    outcome = SaveOutcome(SaveState.SUCCEEDED, 'done', name='m', revision='r')
    model.record_save(5, outcome)
    assert model.cached_save(5) is outcome
    assert model.cached_save(6) is None
