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

This module owns every decision that does not need ROS: safe map ids, complete-
bundle validation, occupancy-raster writing/plausibility, manifest hashing, the
staging -> publish transaction, and the mapping-session lifecycle. The ROS node
supplies clocks, SLAM service calls, and the latest ``/map`` grid.

Storage layout: one directory per map, named by its map id, containing a fixed
set of canonical filenames::

    maps/<map_id>/
      map.yaml
      occupancy.pgm
      posegraph.posegraph
      posegraph.data
      semantics.png        (optional; owned by runner_paddock.semantics)
      manifest.json

The directory name is the map's sole identity. ``map.yaml``'s ``image:`` field
is written to reference ``occupancy.pgm`` relatively; every reader resolves it
relative to ``map.yaml``'s own directory rather than assuming the filename.
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


# Canonical fixed filenames inside one map directory.
MAP_YAML_NAME = 'map.yaml'
OCCUPANCY_NAME = 'occupancy.pgm'
# slam_toolbox's SerializePoseGraph appends '.posegraph'/'.data' to whatever
# stem it is given; this stem is not required to match the map id.
POSEGRAPH_STEM = 'posegraph'
POSEGRAPH_POSEGRAPH_NAME = f'{POSEGRAPH_STEM}.posegraph'
POSEGRAPH_DATA_NAME = f'{POSEGRAPH_STEM}.data'
SEMANTICS_NAME = 'semantics.png'
MANIFEST_NAME = 'manifest.json'

# Required for a bundle to be considered complete/selectable.
CORE_FILENAMES = (
    MAP_YAML_NAME, OCCUPANCY_NAME, POSEGRAPH_POSEGRAPH_NAME, POSEGRAPH_DATA_NAME,
)
# Every filename the manifest may hash, core plus optional.
CANONICAL_FILENAMES = CORE_FILENAMES + (SEMANTICS_NAME,)

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
    """A rejected map id, save, or selection with an actionable reason."""


def safe_basename(map_id: str) -> str:
    """Return ``map_id`` when it is a bounded, single-component directory name."""
    if not map_id or len(map_id) > MAX_BASENAME_LEN:
        raise MapError('map id must be 1-64 characters')
    if '..' in map_id or '/' in map_id or '\\' in map_id or os.sep in map_id:
        raise MapError('map id must not contain path separators or ".."')
    if Path(map_id).name != map_id:
        raise MapError('map id must be a bare single path component')
    if not MAP_BASENAME.fullmatch(map_id):
        raise MapError(
            'map id must start with a letter or number and use only '
            'letters, numbers, dot, underscore, or hyphen'
        )
    return map_id


def map_directory(map_dir: Path, map_id: str) -> Path:
    """Return the directory owning one map's artifacts, validating the id."""
    return map_dir / safe_basename(map_id)


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


def resolve_image_path(yaml_path: Path, image_field: str) -> Path:
    """Resolve a YAML ``image:`` value relative to the YAML's own directory."""
    image_path = Path(image_field)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return image_path


@dataclass(frozen=True)
class BundleInfo:
    """Validated metadata for one complete map bundle."""

    name: str
    revision: str
    session_id: str
    resolution: float
    width: int
    height: int


def read_manifest(map_dir: Path, map_id: str) -> Optional[dict]:
    """Return a parsed manifest for ``map_id`` or None when it is absent/bad."""
    path = map_directory(map_dir, map_id) / MANIFEST_NAME
    if not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    if not isinstance(document, dict):
        return None
    return document


def validate_bundle(map_dir: Path, map_id: str) -> BundleInfo:
    """Raise :class:`MapError` unless ``map_id`` is a complete, usable bundle."""
    safe_basename(map_id)
    base = map_directory(map_dir, map_id)
    if not base.is_dir():
        raise MapError(f'{map_id}: no such map directory')
    for filename in (POSEGRAPH_POSEGRAPH_NAME, POSEGRAPH_DATA_NAME):
        path = base / filename
        if not path.is_file() or path.stat().st_size == 0:
            raise MapError(f'{map_id}: missing or empty {filename}')
    yaml_path = base / MAP_YAML_NAME
    if not yaml_path.is_file() or yaml_path.stat().st_size == 0:
        raise MapError(f'{map_id}: missing or empty {MAP_YAML_NAME}')
    meta = parse_map_yaml(yaml_path)
    image_path = resolve_image_path(yaml_path, meta['image'])
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise MapError(f'{map_id}: {MAP_YAML_NAME} references missing raster {meta["image"]}')
    width, height, maxval = _read_pgm_header(image_path)
    if width <= 0 or height <= 0:
        raise MapError(f'{map_id}: raster dimensions are not positive')
    if not 0 < maxval <= 255:
        raise MapError(f'{map_id}: raster maxval {maxval} is implausible')
    manifest = read_manifest(map_dir, map_id)
    revision = ''
    session_id = ''
    if manifest is not None:
        revision = str(manifest.get('revision', ''))
        session_id = str(manifest.get('session_id', ''))
        recorded = manifest.get('artifacts', {})
        if isinstance(recorded, dict) and recorded:
            for filename, digest in recorded.items():
                if (
                    not isinstance(filename, str)
                    or '/' in filename or '\\' in filename or '..' in filename
                    or Path(filename).name != filename
                ):
                    raise MapError(
                        f'{map_id}: manifest names an invalid artifact filename'
                    )
                path = base / filename
                if not path.is_file() or _sha256(path) != digest:
                    raise MapError(
                        f'{map_id}: manifest hash mismatch for {filename}'
                    )
    return BundleInfo(
        name=map_id,
        revision=revision,
        session_id=session_id,
        resolution=meta['resolution'],
        width=width,
        height=height,
    )


def delete_bundle(map_dir: Path, map_id: str) -> BundleInfo:
    """
    Remove one validated map directory via a single atomic rename-away.

    A directory rename is one atomic filesystem operation: it either fully
    succeeds (the map directory now lives under a private tombstone, ready
    for best-effort cleanup) or fully fails (the live directory is
    untouched). There is no partial-move state to roll back.
    """
    info = validate_bundle(map_dir, map_id)
    base = map_directory(map_dir, map_id)
    tombstone = Path(tempfile.mkdtemp(prefix=f'.delete-{map_id}-', dir=map_dir))
    destination = tombstone / map_id
    try:
        os.replace(base, destination)
    except OSError as error:
        shutil.rmtree(tombstone, ignore_errors=True)
        raise MapError(
            f'{map_id}: could not stage directory deletion: {error}'
        ) from error
    shutil.rmtree(tombstone, ignore_errors=True)
    return info


def validate_delete_candidate(
    map_dir: Path, map_id: str, *, selected: str, active_autonomy_map: str
) -> BundleInfo:
    """Reject a bundle that selection or the live runtime still references."""
    safe_basename(map_id)
    if map_id == selected:
        raise MapError(
            'cannot delete the selected map; select another map first'
        )
    if map_id == active_autonomy_map:
        raise MapError('cannot delete the map used by AUTONOMY')
    return validate_bundle(map_dir, map_id)


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
    List every discovered map directory, validating completeness cheaply.

    Discovery is by top-level subdirectory name under the map root; a name
    that does not pass :func:`safe_basename` (including every dot-prefixed
    staging/tombstone directory) is never treated as a map. A half-published
    bundle can never appear, since staging/tombstones always fail that check.
    """
    if not map_dir.is_dir():
        return []
    names: list[str] = []
    for entry in sorted(map_dir.iterdir()):
        if not entry.is_dir():
            continue
        if not MAP_BASENAME.fullmatch(entry.name):
            continue
        names.append(entry.name)
    catalog: list[CatalogEntry] = []
    for name in names:
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
    session_id: str, map_id: str, base_dir: Path, *, created: float
) -> dict:
    """Build the manifest dict, hashing whichever canonical files are present."""
    artifacts = {
        filename: _sha256(base_dir / filename)
        for filename in CANONICAL_FILENAMES
        if (base_dir / filename).is_file()
    }
    joined = ''.join(
        artifacts[filename] for filename in CANONICAL_FILENAMES
        if filename in artifacts
    )
    revision = hashlib.sha256(joined.encode('ascii')).hexdigest()[:12]
    return {
        'version': 1,
        'name': map_id,
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
    Stage a complete map directory, validate it, then publish atomically.

    The node performs SLAM serialization (into the staging directory) and hands
    this object the captured occupancy raster. Nothing enters the selectable
    catalog until :meth:`publish` has moved the fully validated, staged
    directory into place with one atomic rename.
    """

    def __init__(self, map_dir: Path, map_id: str, session_id: str) -> None:
        self.map_dir = map_dir
        self.name = safe_basename(map_id)
        self.session_id = session_id
        self.staging_root = map_dir / STAGING_DIRNAME
        self.staging = self.staging_root / self.name
        self.created = time.time()

    def staged_path(self, filename: str) -> Path:
        return self.staging / filename

    def prepare(self) -> None:
        """Create a clean staging directory, refusing to clobber a live bundle."""
        if map_directory(self.map_dir, self.name).exists():
            raise MapError(
                f'{self.name}: a bundle with this name already exists'
            )
        if self.staging.exists():
            shutil.rmtree(self.staging)
        self.staging.mkdir(parents=True, exist_ok=True)

    def serialize_target(self) -> str:
        """Absolute path stem passed to slam_toolbox SerializePoseGraph."""
        return str(self.staging / POSEGRAPH_STEM)

    def check_serialized(self) -> None:
        """Verify the SLAM step produced non-empty posegraph/data."""
        for filename in (POSEGRAPH_POSEGRAPH_NAME, POSEGRAPH_DATA_NAME):
            path = self.staged_path(filename)
            if not path.is_file() or path.stat().st_size == 0:
                raise MapError(
                    f'{self.name}: SerializePoseGraph did not create '
                    f'a non-empty {filename}'
                )

    def write_raster(self, raster: OccupancyRaster) -> None:
        """Write the occupancy PGM and its YAML from the captured grid."""
        if raster.width <= 0 or raster.height <= 0:
            raise MapError(f'{self.name}: captured /map grid is empty')
        if len(raster.data) != raster.width * raster.height:
            raise MapError(f'{self.name}: captured /map grid is inconsistent')
        self.staged_path(OCCUPANCY_NAME).write_bytes(raster.to_pgm_bytes())
        self.staged_path(MAP_YAML_NAME).write_text(
            raster.to_yaml_text(OCCUPANCY_NAME), encoding='utf-8'
        )

    def validate(self) -> BundleInfo:
        """Full complete-bundle validation against the staged directory."""
        info = validate_bundle(self.staging_root, self.name)
        if info.width <= 1 or info.height <= 1:
            raise MapError(f'{self.name}: staged raster is degenerate')
        return info

    def publish(self) -> SaveOutcome:
        """Write the manifest, then atomically rename the directory into place."""
        manifest = build_manifest(
            self.session_id, self.name, self.staging, created=self.created
        )
        (self.staging / MANIFEST_NAME).write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding='utf-8'
        )
        final_dir = map_directory(self.map_dir, self.name)
        os.replace(self.staging, final_dir)
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
