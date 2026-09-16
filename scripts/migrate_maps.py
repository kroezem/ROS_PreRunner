#!/usr/bin/env python3
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
One-time, idempotent migration from flat map bundles to directory-per-map.

For each legacy flat bundle ``<map_id>.posegraph`` / ``.data`` / ``.yaml`` (and
its referenced occupancy raster) found directly under the map root, this
builds and validates the equivalent ``<map_id>/`` directory bundle in a
private staging area, publishes it with one atomic rename only once it is
confirmed complete, and only then moves the legacy flat files aside into
``.legacy-migrated/`` -- never deleting them. A legacy manifest's session id
and creation time are preserved; artifact hashes are always recomputed
against the new canonical filenames.

Never run automatically at Paddock startup. Run by hand:

    python3 scripts/migrate_maps.py maps/            # migrate
    python3 scripts/migrate_maps.py maps/ --dry-run   # preview only

Re-running after a full migration is a no-op: an already-published map
directory is left untouched, and legacy files already moved aside are not
revisited.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

try:
    from runner_paddock.map_session import (
        build_manifest,
        MANIFEST_NAME,
        MAP_YAML_NAME,
        MapError,
        OCCUPANCY_NAME,
        POSEGRAPH_DATA_NAME,
        POSEGRAPH_POSEGRAPH_NAME,
        resolve_image_path,
        parse_map_yaml,
        safe_basename,
        validate_bundle,
    )
except ImportError as error:  # pragma: no cover - operator-facing guidance
    sys.exit(
        'Could not import runner_paddock.map_session '
        f'({error}). Source the built workspace first, e.g.:\n'
        '  source /opt/ros/jazzy/setup.bash\n'
        '  source "$HOME/runner_ws/install/setup.bash"'
    )

LEGACY_BACKUP_DIRNAME = '.legacy-migrated'
STAGING_DIRNAME = '.staging'
LEGACY_CORE_EXTENSIONS = ('posegraph', 'data', 'yaml')


# Recognized legacy flat-file suffixes, longest first so '.manifest.json'
# matches before the bare '.json' it would otherwise be confused with.
_LEGACY_SUFFIXES = ('.manifest.json', '.posegraph', '.data', '.yaml', '.pgm')


def _discover_legacy_ids(map_root: Path) -> list[str]:
    """
    Return every map id with at least one legacy flat file directly under
    the root -- not only a ``.yaml``, so a run interrupted after archiving
    some but not all of one map's legacy files still finds the rest.
    """
    ids: set[str] = set()
    for entry in map_root.iterdir():
        if not entry.is_file():
            continue
        for suffix in _LEGACY_SUFFIXES:
            if entry.name.endswith(suffix) and len(entry.name) > len(suffix):
                try:
                    ids.add(safe_basename(entry.name[: -len(suffix)]))
                except MapError:
                    pass
                break
    return sorted(ids)


def _legacy_paths(map_root: Path, map_id: str) -> dict[str, Path]:
    return {
        extension: map_root / f'{map_id}.{extension}'
        for extension in LEGACY_CORE_EXTENSIONS
    }


def _legacy_image_path(map_root: Path, map_id: str) -> Path | None:
    yaml_path = map_root / f'{map_id}.yaml'
    if not yaml_path.is_file():
        return None
    try:
        meta = parse_map_yaml(yaml_path)
    except MapError:
        return None
    return resolve_image_path(yaml_path, meta['image'])


def _legacy_manifest(map_root: Path, map_id: str) -> dict | None:
    path = map_root / f'{map_id}.manifest.json'
    if not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    return document if isinstance(document, dict) else None


def _rewrite_image_line(text: str, new_image_name: str) -> str:
    """Rewrite only the ``image:`` line; every other line is byte-preserved."""
    lines = text.splitlines(keepends=True)
    rewritten = False
    for index, line in enumerate(lines):
        if line.strip().startswith('image:'):
            newline = '\n' if line.endswith('\n') else ''
            lines[index] = f'image: {new_image_name}{newline}'
            rewritten = True
            break
    if not rewritten:
        raise MapError('legacy YAML has no image: line to rewrite')
    return ''.join(lines)


def _build_new_bundle(
    map_root: Path, map_id: str, *, dry_run: bool
) -> tuple[bool, bool, str]:
    """
    Stage, validate, and publish one map id's directory bundle.

    Returns (built_changed, safe_to_archive_legacy, message). Legacy flat
    files are only ever safe to archive once a *validated* directory bundle
    exists for this id -- either published just now, or already present and
    independently confirmed valid. A directory that exists but fails
    validation is left alone and its legacy siblings are never touched, so a
    conflict never destroys the only good copy of a map.
    """
    final_dir = map_root / map_id
    if final_dir.exists():
        try:
            validate_bundle(map_root, map_id)
        except MapError as error:
            return False, False, (
                f'{map_id}: a directory already exists at {final_dir} but '
                f'does not validate ({error}); legacy files left untouched'
            )
        return False, True, f'{map_id}: directory bundle already published, skipped'

    legacy = _legacy_paths(map_root, map_id)
    missing = [
        str(path) for path in legacy.values() if not path.is_file()
        or path.stat().st_size == 0
    ]
    image_path = _legacy_image_path(map_root, map_id)
    if image_path is None or not image_path.is_file() or image_path.stat().st_size == 0:
        missing.append(str(image_path) if image_path else '<unresolvable image>')
    if missing:
        return False, False, (
            f'{map_id}: incomplete legacy bundle, not migrated -- missing: '
            + ', '.join(missing)
        )

    if dry_run:
        return True, False, f'{map_id}: would build and publish {final_dir}'

    staging_root = map_root / STAGING_DIRNAME
    staging_dir = staging_root / map_id
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True)
    try:
        shutil.copy2(legacy['posegraph'], staging_dir / POSEGRAPH_POSEGRAPH_NAME)
        shutil.copy2(legacy['data'], staging_dir / POSEGRAPH_DATA_NAME)
        shutil.copy2(image_path, staging_dir / OCCUPANCY_NAME)
        rewritten_yaml = _rewrite_image_line(
            legacy['yaml'].read_text(encoding='utf-8'), OCCUPANCY_NAME
        )
        (staging_dir / MAP_YAML_NAME).write_text(rewritten_yaml, encoding='utf-8')

        old_manifest = _legacy_manifest(map_root, map_id)
        created = (
            float(old_manifest['created'])
            if old_manifest and isinstance(old_manifest.get('created'), (int, float))
            else legacy['posegraph'].stat().st_mtime
        )
        session_id = (
            str(old_manifest.get('session_id', ''))
            if old_manifest else ''
        )
        manifest = build_manifest(session_id, map_id, staging_dir, created=created)
        (staging_dir / MANIFEST_NAME).write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8'
        )

        validate_bundle(staging_root, map_id)
    except (MapError, OSError) as error:
        shutil.rmtree(staging_dir, ignore_errors=True)
        return False, False, (
            f'{map_id}: migration build failed, nothing published: {error}'
        )

    staging_dir.replace(final_dir)
    return True, True, f'{map_id}: published {final_dir}'


def _archive_legacy_files(
    map_root: Path, map_id: str, *, dry_run: bool
) -> tuple[bool, str]:
    """Move any remaining legacy flat files for ``map_id`` into the backup dir."""
    legacy = dict(_legacy_paths(map_root, map_id))
    manifest_path = map_root / f'{map_id}.manifest.json'
    if manifest_path.is_file():
        legacy['manifest.json'] = manifest_path
    # Prefer the yaml-declared image path (still resolvable on a first pass),
    # but every legacy raster in this codebase is '<map_id>.pgm' -- fall back
    # to that literal name so a rerun after a partial archive (yaml already
    # moved, image left behind) still finds it.
    image_path = _legacy_image_path(map_root, map_id) or (map_root / f'{map_id}.pgm')
    if image_path.parent == map_root and image_path.is_file():
        legacy[image_path.suffix.lstrip('.') or 'image'] = image_path

    present = {key: path for key, path in legacy.items() if path.is_file()}
    if not present:
        return False, f'{map_id}: no legacy flat files remain'

    if dry_run:
        names = ', '.join(path.name for path in present.values())
        return True, f'{map_id}: would archive legacy files: {names}'

    backup_dir = map_root / LEGACY_BACKUP_DIRNAME
    backup_dir.mkdir(exist_ok=True)
    moved = []
    for path in present.values():
        destination = backup_dir / path.name
        if destination.exists():
            continue
        path.replace(destination)
        moved.append(path.name)
    return bool(moved), f'{map_id}: archived legacy files to {backup_dir}: {", ".join(moved)}'


def migrate(map_root: Path, *, dry_run: bool) -> int:
    """Migrate every discoverable legacy bundle; return the changed count."""
    if not map_root.is_dir():
        print(f'{map_root}: no such directory', file=sys.stderr)
        return 0
    changed = 0
    for map_id in _discover_legacy_ids(map_root):
        built_changed, safe_to_archive, built_message = _build_new_bundle(
            map_root, map_id, dry_run=dry_run
        )
        print(built_message)
        if built_changed:
            changed += 1
        # Archiving legacy files is only attempted once a validated
        # directory bundle exists (or, in a dry run, would exist). Every
        # other outcome -- incomplete legacy bundle, failed build, or an
        # existing directory that does not validate -- leaves the legacy
        # files exactly where they are.
        if not safe_to_archive and not (dry_run and built_changed):
            continue
        archived_changed, archived_message = _archive_legacy_files(
            map_root, map_id, dry_run=dry_run
        )
        print(archived_message)
        if archived_changed:
            changed += 1
    if not changed:
        print('nothing to migrate')
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_root', type=Path, help='map root directory (e.g. maps/)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='report what would change without writing anything',
    )
    args = parser.parse_args()
    migrate(args.map_root.resolve(), dry_run=args.dry_run)


if __name__ == '__main__':
    main()
