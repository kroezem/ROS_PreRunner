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
ROS-independent mapping-session model, map catalog, and save transaction.

This module owns every decision that does not need ROS: safe names, complete-
bundle validation, occupancy-raster writing/plausibility, manifest hashing, the
staging -> publish transaction, and the mapping-session lifecycle. The ROS node
supplies clocks, SLAM service calls, and the latest ``/map`` grid.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tempfile
import time
from typing import Optional

import yaml


# Core artifacts of a complete slam_toolbox localization bundle.
CORE_EXTENSIONS = ('posegraph', 'data', 'yaml', 'pgm')
MANIFEST_SUFFIX = '.manifest.json'
STAGING_DIRNAME = '.staging'
MAP_BASENAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
MAX_BASENAME_LEN = 64


class SessionPhase(IntEnum):
    """Values mirror runner_interfaces/MapState.SESSION_* constants."""

    NONE = 0
    STARTING = 1
    READY = 2
    SAVING = 3
    SAVED = 4
    FAILED = 5


class SaveState(IntEnum):
    """Values mirror runner_interfaces/MapState.SAVE_* constants."""

    IDLE = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3


class DeleteState(IntEnum):
    """Values mirror runner_interfaces/MapState.DELETE_* constants."""

    IDLE = 0
    SUCCEEDED = 1
    FAILED = 2


class MapError(ValueError):
    """A rejected name, save, or selection with an actionable reason."""


def safe_basename(name: str) -> str:
    """Return ``name`` when it is a bounded, separator-free map basename."""
    if not name or len(name) > MAX_BASENAME_LEN:
        raise MapError('map name must be 1-64 characters')
    if '..' in name or '/' in name or '\\' in name or os.sep in name:
        raise MapError('map name must not contain path separators or ".."')
    if Path(name).name != name:
        raise MapError('map name must be a bare basename')
    if not MAP_BASENAME.fullmatch(name):
        raise MapError(
            'map name must start with a letter or number and use only '
            'letters, numbers, dot, underscore, or hyphen'
        )
    return name


def _read_pgm_header(path: Path) -> tuple[int, int, int]:
    """Return (width, height, maxval) for a binary P5 PGM, else raise."""
    with path.open('rb') as stream:
        head = stream.read(64)
    if not head.startswith(b'P5'):
        raise MapError(f'{path.name} is not a binary PGM (missing P5 magic)')
    tokens: list[bytes] = []
    index = 2
    while len(tokens) < 3 and index < len(head):
        while index < len(head) and head[index:index + 1].isspace():
            index += 1
        if index < len(head) and head[index:index + 1] == b'#':
            while index < len(head) and head[index:index + 1] not in (b'\n',):
                index += 1
            continue
        start = index
        while index < len(head) and not head[index:index + 1].isspace():
            index += 1
        if index > start:
            tokens.append(head[start:index])
    if len(tokens) < 3:
        raise MapError(f'{path.name} PGM header is truncated')
    try:
        width, height, maxval = (int(token) for token in tokens[:3])
    except ValueError as error:
        raise MapError(f'{path.name} PGM header is not numeric') from error
    return width, height, maxval


def parse_map_yaml(path: Path) -> dict:
    """Parse a map YAML and return validated image/resolution/origin fields."""
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise MapError(f'{path.name} does not parse as YAML: {error}') from error
    if not isinstance(document, dict):
        raise MapError(f'{path.name} is not a YAML mapping')
    image = document.get('image')
    if not isinstance(image, str) or not image:
        raise MapError(f'{path.name} has no image entry')
    resolution = document.get('resolution')
    if not isinstance(resolution, (int, float)) or not resolution > 0.0:
        raise MapError(f'{path.name} resolution is missing or not positive')
    origin = document.get('origin')
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(isinstance(value, (int, float)) for value in origin)
    ):
        raise MapError(f'{path.name} origin must be a list of three numbers')
    return {
        'image': image,
        'resolution': float(resolution),
        'origin': [float(value) for value in origin],
    }


@dataclass(frozen=True)
class BundleInfo:
    """Validated metadata for one complete map bundle."""

    name: str
    revision: str
    session_id: str
    resolution: float
    width: int
    height: int


def _bundle_paths(map_dir: Path, name: str) -> dict[str, Path]:
    return {
        extension: map_dir / f'{name}.{extension}'
        for extension in CORE_EXTENSIONS
    }


def read_manifest(map_dir: Path, name: str) -> Optional[dict]:
    """Return a parsed manifest for ``name`` or None when it is absent/bad."""
    path = map_dir / f'{name}{MANIFEST_SUFFIX}'
    if not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    if not isinstance(document, dict):
        return None
    return document


def validate_bundle(map_dir: Path, name: str) -> BundleInfo:
    """Raise :class:`MapError` unless ``name`` is a complete, usable bundle."""
    safe_basename(name)
    paths = _bundle_paths(map_dir, name)
    for extension in ('posegraph', 'data'):
        path = paths[extension]
        if not path.is_file() or path.stat().st_size == 0:
            raise MapError(f'{name}: missing or empty .{extension}')
    yaml_path = paths['yaml']
    if not yaml_path.is_file() or yaml_path.stat().st_size == 0:
        raise MapError(f'{name}: missing or empty .yaml')
    meta = parse_map_yaml(yaml_path)
    image_path = Path(meta['image'])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise MapError(f'{name}: YAML references missing raster {meta["image"]}')
    width, height, maxval = _read_pgm_header(image_path)
    if width <= 0 or height <= 0:
        raise MapError(f'{name}: raster dimensions are not positive')
    if not 0 < maxval <= 255:
        raise MapError(f'{name}: raster maxval {maxval} is implausible')
    manifest = read_manifest(map_dir, name)
    revision = ''
    session_id = ''
    if manifest is not None:
        revision = str(manifest.get('revision', ''))
        session_id = str(manifest.get('session_id', ''))
        recorded = manifest.get('artifacts', {})
        if isinstance(recorded, dict) and recorded:
            for extension, digest in recorded.items():
                path = map_dir / f'{name}.{extension}'
                if not path.is_file() or _sha256(path) != digest:
                    raise MapError(
                        f'{name}: manifest hash mismatch for .{extension}'
                    )
    return BundleInfo(
        name=name,
        revision=revision,
        session_id=session_id,
        resolution=meta['resolution'],
        width=width,
        height=height,
    )


def delete_bundle(map_dir: Path, name: str) -> BundleInfo:
    """
    Remove one validated bundle after staging every artifact atomically.

    Each rename is atomic within ``map_dir``. If staging fails, already moved
    files are restored before the rejection is returned. Once every artifact
    is staged, the bundle has left catalog truth and best-effort cleanup of the
    private tombstone cannot expose a partial saved map.
    """
    info = validate_bundle(map_dir, name)
    paths = list(_bundle_paths(map_dir, name).values())
    manifest = map_dir / f'{name}{MANIFEST_SUFFIX}'
    if manifest.exists():
        paths.append(manifest)
    tombstone = Path(tempfile.mkdtemp(prefix=f'.delete-{name}-', dir=map_dir))
    moved: list[tuple[Path, Path]] = []
    try:
        for source in paths:
            destination = tombstone / source.name
            os.replace(source, destination)
            moved.append((source, destination))
    except OSError as error:
        rollback_errors = []
        for source, destination in reversed(moved):
            try:
                os.replace(destination, source)
            except OSError as rollback_error:
                rollback_errors.append(str(rollback_error))
        shutil.rmtree(tombstone, ignore_errors=True)
        detail = f'could not stage complete bundle deletion: {error}'
        if rollback_errors:
            detail += f'; rollback incomplete: {"; ".join(rollback_errors)}'
        raise MapError(detail) from error
    shutil.rmtree(tombstone, ignore_errors=True)
    return info


def validate_delete_candidate(
    map_dir: Path, name: str, *, selected: str, active_autonomy_map: str
) -> BundleInfo:
    """Reject a bundle that selection or the live runtime still references."""
    safe_basename(name)
    if name == selected:
        raise MapError(
            'cannot delete the selected map; select another map first'
        )
    if name == active_autonomy_map:
        raise MapError('cannot delete the map used by AUTONOMY')
    return validate_bundle(map_dir, name)


@dataclass(frozen=True)
class CatalogEntry:
    """One catalog row; ``complete`` gates whether the bundle is selectable."""

    name: str
    revision: str = ''
    complete: bool = False
    selected: bool = False
    session_id: str = ''
    reason: str = ''
    resolution: float = 0.0
    width: int = 0
    height: int = 0


def scan_catalog(map_dir: Path, selected: str = '') -> list[CatalogEntry]:
    """
    List every discovered bundle, validating completeness cheaply.

    Discovery is by any core artifact under the top-level map root; the staging
    directory is never scanned, so a half-written bundle can never appear.
    """
    if not map_dir.is_dir():
        return []
    names: set[str] = set()
    for entry in sorted(map_dir.iterdir()):
        if not entry.is_file():
            continue
        suffix = entry.suffix.lstrip('.')
        if suffix in CORE_EXTENSIONS:
            stem = entry.name[: -(len(suffix) + 1)]
            if MAP_BASENAME.fullmatch(stem):
                names.add(stem)
    catalog: list[CatalogEntry] = []
    for name in sorted(names):
        try:
            info = validate_bundle(map_dir, name)
        except MapError as error:
            catalog.append(CatalogEntry(
                name=name,
                complete=False,
                selected=(name == selected),
                reason=str(error),
            ))
            continue
        catalog.append(CatalogEntry(
            name=name,
            revision=info.revision,
            complete=True,
            selected=(name == selected),
            session_id=info.session_id,
            resolution=info.resolution,
            width=info.width,
            height=info.height,
        ))
    return catalog


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class OccupancyRaster:
    """A consistent ``/map`` snapshot captured for a save."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]
    stamp_sec: int = 0
    stamp_nanosec: int = 0

    def to_pgm_bytes(
        self, occupied_thresh: float = 0.65, free_thresh: float = 0.196
    ) -> bytes:
        """Render trinary PGM bytes with nav2 map_saver row order (top-down)."""
        occ = int(occupied_thresh * 100)
        free = int(free_thresh * 100)
        rows = bytearray()
        for row in range(self.height - 1, -1, -1):
            base = row * self.width
            line = bytearray(self.width)
            for column in range(self.width):
                value = self.data[base + column]
                if value < 0:
                    line[column] = 205
                elif value >= occ:
                    line[column] = 0
                elif 0 <= value <= free:
                    line[column] = 254
                else:
                    line[column] = 205
            rows += line
        header = f'P5\n{self.width} {self.height}\n255\n'.encode('ascii')
        return header + bytes(rows)

    def to_yaml_text(self, image_name: str) -> str:
        """Render a map_server-compatible YAML for this raster."""
        return (
            f'image: {image_name}\n'
            'mode: trinary\n'
            f'resolution: {self.resolution:.6f}\n'
            f'origin: [{self.origin_x:.6f}, {self.origin_y:.6f}, '
            f'{self.origin_yaw:.6f}]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n'
        )


@dataclass
class SaveOutcome:
    """Result of one save attempt."""

    state: SaveState
    detail: str
    name: str = ''
    revision: str = ''
    session_id: str = ''


def build_manifest(
    session_id: str, name: str, map_dir: Path, *, created: float
) -> dict:
    """Build the manifest dict from the four on-disk staged artifacts."""
    artifacts = {
        extension: _sha256(map_dir / f'{name}.{extension}')
        for extension in CORE_EXTENSIONS
    }
    joined = ''.join(artifacts[extension] for extension in CORE_EXTENSIONS)
    revision = hashlib.sha256(joined.encode('ascii')).hexdigest()[:12]
    return {
        'version': 1,
        'name': name,
        'session_id': session_id,
        'created': created,
        'created_iso': time.strftime(
            '%Y-%m-%dT%H:%M:%SZ', time.gmtime(created)
        ),
        'revision': revision,
        'artifacts': artifacts,
    }


class MapSaveTransaction:
    """
    Stage the four artifacts, validate, then publish atomically.

    The node performs SLAM serialization (into the staging directory) and hands
    this object the captured occupancy raster. Nothing enters the selectable
    catalog until :meth:`publish` has moved a fully validated bundle.
    """

    def __init__(self, map_dir: Path, name: str, session_id: str) -> None:
        self.map_dir = map_dir
        self.name = safe_basename(name)
        self.session_id = session_id
        self.staging = map_dir / STAGING_DIRNAME
        self.created = time.time()

    def staged_path(self, extension: str) -> Path:
        return self.staging / f'{self.name}.{extension}'

    def prepare(self) -> None:
        """Create a clean staging area, refusing to clobber a live bundle."""
        for extension in CORE_EXTENSIONS:
            live = self.map_dir / f'{self.name}.{extension}'
            if live.exists():
                raise MapError(
                    f'{self.name}: a bundle with this name already exists'
                )
        if (self.map_dir / f'{self.name}{MANIFEST_SUFFIX}').exists():
            raise MapError(f'{self.name}: a bundle with this name already exists')
        self.staging.mkdir(parents=True, exist_ok=True)
        for path in self.staging.glob(f'{self.name}.*'):
            path.unlink()

    def serialize_target(self) -> str:
        """Absolute path stem passed to slam_toolbox SerializePoseGraph."""
        return str(self.staging / self.name)

    def check_serialized(self) -> None:
        """Verify the SLAM step produced non-empty posegraph/data."""
        for extension in ('posegraph', 'data'):
            path = self.staged_path(extension)
            if not path.is_file() or path.stat().st_size == 0:
                raise MapError(
                    f'{self.name}: SerializePoseGraph did not create '
                    f'a non-empty .{extension}'
                )

    def write_raster(self, raster: OccupancyRaster) -> None:
        """Write the occupancy PGM and its YAML from the captured grid."""
        if raster.width <= 0 or raster.height <= 0:
            raise MapError(f'{self.name}: captured /map grid is empty')
        if len(raster.data) != raster.width * raster.height:
            raise MapError(f'{self.name}: captured /map grid is inconsistent')
        self.staged_path('pgm').write_bytes(raster.to_pgm_bytes())
        self.staged_path('yaml').write_text(
            raster.to_yaml_text(f'{self.name}.pgm'), encoding='utf-8'
        )

    def validate(self) -> BundleInfo:
        """Full complete-bundle validation against the staged files."""
        info = validate_bundle(self.staging, self.name)
        if info.width <= 1 or info.height <= 1:
            raise MapError(f'{self.name}: staged raster is degenerate')
        return info

    def publish(self) -> SaveOutcome:
        """Write the manifest, then atomically move the bundle into the root."""
        manifest = build_manifest(
            self.session_id, self.name, self.staging, created=self.created
        )
        manifest_staged = self.staging / f'{self.name}{MANIFEST_SUFFIX}'
        manifest_staged.write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8'
        )
        # Move core artifacts first, manifest last: a reader that sees the
        # manifest has already seen every artifact it names.
        for extension in CORE_EXTENSIONS:
            os.replace(
                self.staged_path(extension),
                self.map_dir / f'{self.name}.{extension}',
            )
        os.replace(manifest_staged, self.map_dir / f'{self.name}{MANIFEST_SUFFIX}')
        return SaveOutcome(
            state=SaveState.SUCCEEDED,
            detail='bundle published to catalog',
            name=self.name,
            revision=manifest['revision'],
            session_id=self.session_id,
        )


@dataclass
class MappingSession:
    """The lifecycle of one mapping SLAM session."""

    session_id: str = ''
    runtime_epoch: int = 0
    created: float = 0.0
    phase: SessionPhase = SessionPhase.NONE
    unsaved: bool = False
    saved_name: str = ''
    saved_revision: str = ''
    # Monotonic time the current-session map evidence was last observed.
    evidence_at: Optional[float] = None

    @property
    def active(self) -> bool:
        return bool(self.session_id) and self.phase != SessionPhase.NONE

    def ready(self, now: float, evidence_timeout: float = 8.0) -> bool:
        """Current-session evidence must be recent, not merely ever-seen."""
        return (
            self.active
            and self.phase in (SessionPhase.READY, SessionPhase.SAVED)
            and self.evidence_at is not None
            and now - self.evidence_at <= evidence_timeout
        )


@dataclass
class MapSessionModel:
    """
    Reduce runtime/evidence/save inputs into one publishable snapshot.

    ROS-free: the node feeds it observations and reads back the fields it needs
    to publish ``/paddock/map_state``.
    """

    map_dir: Path
    session: MappingSession = field(default_factory=MappingSession)
    save_state: SaveState = SaveState.IDLE
    save_request_id: int = 0
    save_detail: str = ''
    selected_requested: str = ''
    selected_applied: str = ''
    selected_reason: str = ''
    delete_state: DeleteState = DeleteState.IDLE
    delete_request_id: int = 0
    delete_name: str = ''
    delete_detail: str = ''
    # request_id -> SaveOutcome, for idempotent retries within one process.
    _save_results: dict = field(default_factory=dict)
    _catalog_key: tuple | None = field(default=None, init=False, repr=False)
    _catalog_entries: list[CatalogEntry] = field(
        default_factory=list, init=False, repr=False
    )

    def observe_runtime(
        self,
        *,
        mapping_active: bool,
        session_id: str,
        runtime_epoch: int,
        now: float,
    ) -> None:
        """Follow ModeState; a new session id discards all prior session state."""
        if not mapping_active or not session_id:
            if self.session.active:
                self.session = MappingSession()
            return
        if session_id != self.session.session_id:
            # Fresh session: nothing keyed to the old id survives.
            self.session = MappingSession(
                session_id=session_id,
                runtime_epoch=runtime_epoch,
                created=now,
                phase=SessionPhase.STARTING,
                unsaved=True,
            )
            if self.save_state == SaveState.RUNNING:
                self.save_state = SaveState.FAILED
                self.save_detail = 'session changed during save'

    def observe_map_evidence(self, *, produced_after: bool, now: float) -> None:
        """Record that ``/map`` updated for the current session."""
        if not self.session.active or not produced_after:
            return
        self.session.evidence_at = now
        if self.session.phase == SessionPhase.STARTING:
            self.session.phase = SessionPhase.READY

    def refresh(self, now: float) -> None:
        """Recompute derived phase/catalog-independent fields."""
        if (
            self.session.active
            and self.session.phase in (SessionPhase.READY, SessionPhase.SAVED)
            and not self.session.ready(now)
        ):
            # Evidence went stale; no longer claim the session is ready.
            self.session.phase = SessionPhase.STARTING

    def cached_save(self, request_id: int) -> Optional[SaveOutcome]:
        return self._save_results.get(request_id)

    def record_save(self, request_id: int, outcome: SaveOutcome) -> None:
        self._save_results[request_id] = outcome
        self.save_request_id = request_id
        self.save_state = outcome.state
        self.save_detail = outcome.detail
        if outcome.state == SaveState.SUCCEEDED:
            self.session.unsaved = False
            self.session.saved_name = outcome.name
            self.session.saved_revision = outcome.revision
            self.session.phase = SessionPhase.SAVED
        elif outcome.state == SaveState.FAILED and self.session.active:
            if self.session.phase == SessionPhase.SAVING:
                self.session.phase = SessionPhase.READY

    def begin_save(self, request_id: int) -> None:
        self.save_request_id = request_id
        self.save_state = SaveState.RUNNING
        self.save_detail = 'serializing SLAM posegraph'
        if self.session.active:
            self.session.phase = SessionPhase.SAVING

    def set_selection(self, requested: str, applied: str, reason: str) -> None:
        self.selected_requested = requested
        self.selected_applied = applied
        self.selected_reason = reason

    def record_delete(
        self, request_id: int, name: str, state: DeleteState, detail: str
    ) -> None:
        """Record the authoritative result of one delete request."""
        self.delete_request_id = request_id
        self.delete_name = name
        self.delete_state = state
        self.delete_detail = detail

    def catalog(self) -> list[CatalogEntry]:
        rows = []
        try:
            paths = tuple(self.map_dir.iterdir())
        except OSError:
            paths = ()
        for path in paths:
            try:
                stat = path.stat()
            except OSError:
                continue
            rows.append((path.name, stat.st_mtime_ns, stat.st_size))
        key = (self.selected_applied, tuple(sorted(rows)))
        if key != self._catalog_key:
            self._catalog_entries = scan_catalog(
                self.map_dir, selected=self.selected_applied
            )
            self._catalog_key = key
        return list(self._catalog_entries)
