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

"""Tests for the optional semantic raster: storage, alignment, validation."""

import json

import pytest

from runner_paddock.map_session import (
    delete_bundle,
    MANIFEST_NAME,
    MapSaveTransaction,
    OccupancyRaster,
    POSEGRAPH_DATA_NAME,
    POSEGRAPH_POSEGRAPH_NAME,
    scan_catalog,
    validate_bundle,
)
from runner_paddock.png_codec import decode_grayscale_png
from runner_paddock.semantics import (
    CLASS_CAUTION,
    CLASS_LETHAL,
    CLASS_UNCLASSIFIED,
    decode_semantics_png,
    encode_semantics_png,
    read_semantics,
    semantics_path,
    SemanticsError,
    write_semantics,
)


def publish_bundle(map_dir, name='studio', *, width=5, height=4, occ_data=None):
    """Publish a minimal complete map bundle via the real save transaction."""
    txn = MapSaveTransaction(map_dir, name, 'sess')
    txn.prepare()
    txn.staged_path(POSEGRAPH_POSEGRAPH_NAME).write_bytes(b'p')
    txn.staged_path(POSEGRAPH_DATA_NAME).write_bytes(b'd')
    data = occ_data if occ_data is not None else tuple([-1] * (width * height))
    raster = OccupancyRaster(
        width=width, height=height, resolution=0.05,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0, data=data,
    )
    txn.write_raster(raster)
    txn.validate()
    return txn.publish()


def test_encode_decode_round_trips_grid_order_values(tmp_path):
    width, height = 4, 3
    grid_data = bytes([
        0, 1, 2, 0,
        1, 1, 0, 2,
        2, 0, 0, 1,
    ])

    blob = encode_semantics_png(width, height, grid_data)
    decoded = decode_semantics_png(blob, expected_width=width, expected_height=height)

    assert decoded == grid_data


def test_row_alignment_matches_occupancy_pgm_convention():
    """Semantic cell (r, c) must land at the same on-disk image row as an
    occupied occupancy cell (r, c) -- verified against the real PGM writer,
    not just against semantics.py's own convention."""
    width, height = 5, 4
    occupancy_data = [-1] * (width * height)
    occupancy_data[0] = 100  # grid row 0 (origin), column 0: occupied
    raster = OccupancyRaster(
        width=width, height=height, resolution=0.05,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0, data=tuple(occupancy_data),
    )
    pgm_pixels = raster.to_pgm_bytes().split(b'\n255\n', 1)[1]
    occupied_image_row = next(
        row for row in range(height) if pgm_pixels[row * width] == 0
    )

    semantic_grid = bytearray(width * height)
    semantic_grid[0] = CLASS_LETHAL  # grid row 0, column 0
    blob = encode_semantics_png(width, height, bytes(semantic_grid))
    _, _, image_pixels = decode_grayscale_png(blob)
    lethal_image_row = next(
        row for row in range(height) if image_pixels[row * width] == CLASS_LETHAL
    )

    assert lethal_image_row == occupied_image_row


def test_encode_rejects_unsupported_class_value():
    with pytest.raises(SemanticsError, match='unsupported'):
        encode_semantics_png(2, 2, bytes([0, 1, 2, 9]))


def test_decode_rejects_wrong_dimensions():
    blob = encode_semantics_png(3, 2, bytes(6))
    with pytest.raises(SemanticsError, match='occupancy raster is'):
        decode_semantics_png(blob, expected_width=4, expected_height=2)


def test_write_and_read_semantics_round_trip(tmp_path):
    publish_bundle(tmp_path, 'studio', width=5, height=4)
    grid_data = bytes([CLASS_UNCLASSIFIED, CLASS_CAUTION, CLASS_LETHAL, 0, 0] * 4)

    write_semantics(tmp_path, 'studio', grid_data)
    read_back = read_semantics(tmp_path, 'studio', expected_width=5, expected_height=4)

    assert read_back == grid_data


def test_missing_semantics_reads_as_none(tmp_path):
    publish_bundle(tmp_path, 'studio', width=3, height=3)

    result = read_semantics(tmp_path, 'studio', expected_width=3, expected_height=3)

    assert result is None


def test_write_semantics_rejects_length_mismatched_with_current_occupancy(tmp_path):
    publish_bundle(tmp_path, 'studio', width=5, height=4)

    with pytest.raises(SemanticsError, match='expected 20 class bytes'):
        write_semantics(tmp_path, 'studio', bytes(6))


def test_write_semantics_updates_manifest_with_semantics_hash(tmp_path):
    publish_bundle(tmp_path, 'studio', width=3, height=3)

    write_semantics(tmp_path, 'studio', bytes(9))

    manifest = json.loads((tmp_path / 'studio' / MANIFEST_NAME).read_text())
    assert 'semantics.png' in manifest['artifacts']
    assert manifest['session_id'] == 'sess'  # preserved from the original save


def test_corrupt_semantics_file_is_reported_and_never_silently_applied(tmp_path):
    publish_bundle(tmp_path, 'studio', width=3, height=3)
    write_semantics(tmp_path, 'studio', bytes(9))
    semantics_path(tmp_path, 'studio').write_bytes(b'not a png')

    with pytest.raises(SemanticsError, match='does not decode'):
        read_semantics(tmp_path, 'studio', expected_width=3, expected_height=3)


def test_corrupt_semantics_file_does_not_invalidate_the_core_bundle(tmp_path):
    """A broken optional layer must never take the occupancy map down with it."""
    publish_bundle(tmp_path, 'studio', width=3, height=3)
    write_semantics(tmp_path, 'studio', bytes(9))
    semantics_path(tmp_path, 'studio').write_bytes(b'not a png')

    info = validate_bundle(tmp_path, 'studio')

    assert (info.width, info.height) == (3, 3)
    catalog = {entry.name: entry for entry in scan_catalog(tmp_path)}
    assert catalog['studio'].complete


def test_delete_bundle_removes_semantics_with_the_rest_of_the_bundle(tmp_path):
    publish_bundle(tmp_path, 'studio', width=3, height=3)
    write_semantics(tmp_path, 'studio', bytes(9))
    assert semantics_path(tmp_path, 'studio').is_file()

    delete_bundle(tmp_path, 'studio')

    assert not (tmp_path / 'studio').exists()


def test_recreating_a_deleted_map_never_reattaches_old_semantics(tmp_path):
    publish_bundle(tmp_path, 'first', width=3, height=3)
    write_semantics(tmp_path, 'first', bytes([CLASS_LETHAL] * 9))
    delete_bundle(tmp_path, 'first')

    publish_bundle(tmp_path, 'first', width=3, height=3)

    assert read_semantics(tmp_path, 'first', expected_width=3, expected_height=3) is None
