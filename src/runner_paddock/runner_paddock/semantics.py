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
Optional semantic class raster, aligned 1:1 with a map's occupancy raster.

Authoritative class values (the pixel byte *value* is authority -- never
inferred from display color):

    0 = UNCLASSIFIED (no effect on autonomy)
    1 = CAUTION
    2 = LETHAL

Every other byte value is rejected today; the format has headroom (0-255)
for future classes, but nothing beyond 0/1/2 is accepted yet.

Storage: ``<map_dir>/semantics.png``, an 8-bit grayscale (non-palette) PNG
written/read with the same top-down row order as ``occupancy.pgm`` (see
``OccupancyRaster.to_pgm_bytes``), so the two rasters are byte-row-aligned on
disk. Dimensions must exactly equal the occupancy raster's, re-read fresh
from disk on every write and read; resolution and origin are never
duplicated here, only ever sourced from ``map.yaml``.

This module is storage/validation only: nothing here is read by SLAM, Nav2,
or any autonomy component, and it starts no ROS node, topic, or service.
Callers are responsible for serializing concurrent access to one map (the
Paddock web layer does this with its existing operator-action lock); this
module performs one bounded, atomic file transaction per call and takes no
lock of its own.
"""

import json
import os
from pathlib import Path
import tempfile
import time

from runner_paddock.map_session import build_manifest
from runner_paddock.map_session import map_directory
from runner_paddock.map_session import MANIFEST_NAME
from runner_paddock.map_session import MapError
from runner_paddock.map_session import read_manifest
from runner_paddock.map_session import SEMANTICS_NAME
from runner_paddock.map_session import validate_bundle
from runner_paddock.png_codec import decode_grayscale_png
from runner_paddock.png_codec import encode_grayscale_png
from runner_paddock.png_codec import PngError


CLASS_UNCLASSIFIED = 0
CLASS_CAUTION = 1
CLASS_LETHAL = 2
VALID_CLASS_VALUES = frozenset({CLASS_UNCLASSIFIED, CLASS_CAUTION, CLASS_LETHAL})


class SemanticsError(MapError):
    """A semantics raster that is missing-but-required, corrupt, or invalid."""


def semantics_path(map_dir: Path, map_id: str) -> Path:
    return map_directory(map_dir, map_id) / SEMANTICS_NAME


def _flip_rows(pixels: bytes, width: int, height: int) -> bytes:
    """
    Convert between ROS grid row-major order (row 0 = origin/bottom) and
    on-disk image order (row 0 = top), matching the flip already applied by
    :meth:`OccupancyRaster.to_pgm_bytes`. The same operation is its own
    inverse.
    """
    out = bytearray(len(pixels))
    for row in range(height):
        source = row * width
        destination = (height - 1 - row) * width
        out[destination:destination + width] = pixels[source:source + width]
    return bytes(out)


def _reject_unsupported_values(data: bytes) -> None:
    invalid = sorted({value for value in data if value not in VALID_CLASS_VALUES})
    if invalid:
        raise SemanticsError(f'unsupported semantic class value(s): {invalid}')


def encode_semantics_png(width: int, height: int, grid_data: bytes) -> bytes:
    """Encode a grid-row-major (row 0 = origin) class buffer to PNG bytes."""
    if width <= 0 or height <= 0:
        raise SemanticsError('semantic raster dimensions must be positive')
    if len(grid_data) != width * height:
        raise SemanticsError(
            f'expected {width * height} class bytes for a {width}x{height} '
            f'raster, got {len(grid_data)}'
        )
    _reject_unsupported_values(grid_data)
    try:
        return encode_grayscale_png(width, height, _flip_rows(grid_data, width, height))
    except PngError as error:
        raise SemanticsError(str(error)) from error


def decode_semantics_png(
    data: bytes, *, expected_width: int, expected_height: int
) -> bytes:
    """Decode PNG bytes to a grid-row-major class buffer, or raise :class:`SemanticsError`."""
    try:
        width, height, image_order = decode_grayscale_png(data)
    except PngError as error:
        raise SemanticsError(f'semantics.png does not decode: {error}') from error
    if (width, height) != (expected_width, expected_height):
        raise SemanticsError(
            f'semantics.png is {width}x{height}, occupancy raster is '
            f'{expected_width}x{expected_height}'
        )
    grid_data = _flip_rows(image_order, width, height)
    _reject_unsupported_values(grid_data)
    return grid_data


def _atomic_write(path: Path, data: bytes) -> None:
    with tempfile.NamedTemporaryFile(
        dir=path.parent, prefix=f'.{path.name}.', delete=False,
    ) as stream:
        temporary = Path(stream.name)
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        os.replace(temporary, path)
    except OSError:
        temporary.unlink(missing_ok=True)
        raise


def read_semantics(
    map_dir: Path, map_id: str, *, expected_width: int, expected_height: int
) -> bytes | None:
    """
    Return the grid-row-major class buffer, or ``None`` when the file is
    absent (the caller must then synthesize an all-UNCLASSIFIED layer).

    Raises :class:`SemanticsError` when the file exists but is corrupt,
    wrong-sized, or contains an unsupported value -- that is a distinct
    outcome from "absent" and callers must report it, not silently apply it
    or silently treat it as if the map had no semantic layer.
    """
    path = semantics_path(map_dir, map_id)
    if not path.is_file():
        return None
    data = path.read_bytes()
    if not data:
        raise SemanticsError(f'{path}: semantics.png exists but is empty')
    return decode_semantics_png(
        data, expected_width=expected_width, expected_height=expected_height
    )


def write_semantics(map_dir: Path, map_id: str, grid_data: bytes) -> None:
    """
    Validate and atomically persist one semantic raster for ``map_id``.

    Dimensions are taken from re-validating the map's current occupancy
    raster on disk, never from the caller, so a save can never silently
    apply against stale or mismatched dimensions -- a length mismatch raises
    :class:`SemanticsError` instead. Writes ``semantics.png`` first, then
    recomputes and republishes ``manifest.json`` (whose ``artifacts`` hash
    set is a plain function of "which canonical files exist now"), so a
    reader that observes the new manifest has already observed the new
    ``semantics.png``. Both are tempfile-then-``os.replace`` writes into the
    already-published map directory: only two of the bundle's files change,
    so this is not routed through the create-only staging transaction that
    ``MapSaveTransaction`` uses for a brand-new bundle.
    """
    info = validate_bundle(map_dir, map_id)
    blob = encode_semantics_png(info.width, info.height, grid_data)
    base = map_directory(map_dir, map_id)

    _atomic_write(base / SEMANTICS_NAME, blob)

    previous_manifest = read_manifest(map_dir, map_id) or {}
    session_id = str(previous_manifest.get('session_id', ''))
    created = previous_manifest.get('created')
    if not isinstance(created, (int, float)):
        created = time.time()
    manifest = build_manifest(session_id, map_id, base, created=created)
    _atomic_write(
        base / MANIFEST_NAME,
        json.dumps(manifest, indent=2, sort_keys=True).encode('utf-8'),
    )
