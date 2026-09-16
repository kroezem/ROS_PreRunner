"""Focused tests for the legacy flat-bundle -> directory-per-map migration."""

import json

import migrate_maps
from runner_paddock.map_session import validate_bundle


def write_legacy_bundle(map_root, name='studio', *, manifest=None):
    (map_root / f'{name}.posegraph').write_bytes(b'posegraph-bytes')
    (map_root / f'{name}.data').write_bytes(b'data-bytes')
    (map_root / f'{name}.pgm').write_bytes(b'P5\n2 2\n255\n\x00\x00\x00\x00')
    (map_root / f'{name}.yaml').write_text(
        f'image: {name}.pgm\n'
        'mode: trinary\n'
        'resolution: 0.050000\n'
        'origin: [-1.0, -2.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
    )
    if manifest is not None:
        (map_root / f'{name}.manifest.json').write_text(json.dumps(manifest))


def test_migrate_publishes_valid_directory_and_archives_legacy_files(tmp_path):
    write_legacy_bundle(tmp_path, 'studio')

    changed = migrate_maps.migrate(tmp_path, dry_run=False)

    assert changed == 2  # one publish, one archive
    info = validate_bundle(tmp_path, 'studio')
    assert (info.width, info.height) == (2, 2)
    assert (tmp_path / 'studio' / 'map.yaml').read_text().splitlines()[0] == (
        'image: occupancy.pgm'
    )
    # Every other line is preserved verbatim.
    assert 'resolution: 0.050000' in (tmp_path / 'studio' / 'map.yaml').read_text()
    for legacy_name in ('studio.posegraph', 'studio.data', 'studio.pgm', 'studio.yaml'):
        assert not (tmp_path / legacy_name).exists()
        assert (tmp_path / '.legacy-migrated' / legacy_name).is_file()


def test_migrate_preserves_legacy_manifest_provenance(tmp_path):
    write_legacy_bundle(tmp_path, 'studio', manifest={
        'version': 1, 'name': 'studio', 'session_id': 'orig-session',
        'created': 1000.0, 'created_iso': '1970-01-01T00:16:40Z',
        'revision': 'deadbeef0000', 'artifacts': {},
    })

    migrate_maps.migrate(tmp_path, dry_run=False)

    manifest = json.loads((tmp_path / 'studio' / 'manifest.json').read_text())
    assert manifest['session_id'] == 'orig-session'
    assert manifest['created'] == 1000.0
    # Hashes are always recomputed against the new canonical filenames.
    assert set(manifest['artifacts']) == {
        'map.yaml', 'occupancy.pgm', 'posegraph.posegraph', 'posegraph.data',
    }


def test_migrate_is_idempotent(tmp_path):
    write_legacy_bundle(tmp_path, 'studio')
    first = migrate_maps.migrate(tmp_path, dry_run=False)
    second = migrate_maps.migrate(tmp_path, dry_run=False)

    assert first == 2
    assert second == 0


def test_migrate_skips_incomplete_legacy_bundle(tmp_path):
    write_legacy_bundle(tmp_path, 'broken')
    (tmp_path / 'broken.pgm').unlink()

    changed = migrate_maps.migrate(tmp_path, dry_run=False)

    assert changed == 0
    assert not (tmp_path / 'broken').exists()
    assert (tmp_path / 'broken.posegraph').exists()
    assert (tmp_path / 'broken.data').exists()
    assert (tmp_path / 'broken.yaml').exists()


def test_dry_run_writes_nothing(tmp_path):
    write_legacy_bundle(tmp_path, 'studio')

    changed = migrate_maps.migrate(tmp_path, dry_run=True)

    assert changed == 2  # would-publish preview + would-archive preview
    assert not (tmp_path / 'studio').exists()
    assert (tmp_path / 'studio.posegraph').exists()
    assert not (tmp_path / '.legacy-migrated').exists()


def test_migrate_never_touches_legacy_files_for_an_invalid_existing_directory(tmp_path):
    write_legacy_bundle(tmp_path, 'studio')
    (tmp_path / 'studio').mkdir()
    (tmp_path / 'studio' / 'sentinel').write_text('do not touch')

    changed = migrate_maps.migrate(tmp_path, dry_run=False)

    # An existing 'studio' directory that does not itself validate is a
    # conflict, not a signal that migration is done: the legacy flat files
    # are the only confirmed-good copy, so they are never archived over it.
    assert (tmp_path / 'studio' / 'sentinel').is_file()
    assert changed == 0
    assert (tmp_path / 'studio.posegraph').is_file()
    assert not (tmp_path / '.legacy-migrated').exists()


def test_migrate_archives_leftover_legacy_files_once_directory_is_valid(tmp_path):
    write_legacy_bundle(tmp_path, 'studio')
    migrate_maps.migrate(tmp_path, dry_run=False)
    # Simulate a partially-completed prior run: the directory published, but
    # the legacy-file archive step never ran (e.g. interrupted).
    (tmp_path / '.legacy-migrated' / 'studio.pgm').replace(
        tmp_path / 'studio.pgm'
    )

    changed = migrate_maps.migrate(tmp_path, dry_run=False)

    assert changed == 1
    assert (tmp_path / '.legacy-migrated' / 'studio.pgm').is_file()
    assert not (tmp_path / 'studio.pgm').exists()
