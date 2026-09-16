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

"""Tests for the dependency-free 8-bit grayscale PNG codec."""

import struct
import zlib

import pytest

from runner_paddock.png_codec import (
    _chunk,
    _SIGNATURE,
    decode_grayscale_png,
    encode_grayscale_png,
    PngError,
)


@pytest.mark.parametrize('width,height', [(1, 1), (5, 3), (1, 40), (40, 1), (37, 29)])
def test_round_trip_is_exact(width, height):
    pixels = bytes((i * 37 + 5) % 3 for i in range(width * height))
    blob = encode_grayscale_png(width, height, pixels)

    decoded_width, decoded_height, decoded_pixels = decode_grayscale_png(blob)

    assert (decoded_width, decoded_height) == (width, height)
    assert decoded_pixels == pixels


def test_encode_rejects_mismatched_pixel_length():
    with pytest.raises(PngError, match='does not match width'):
        encode_grayscale_png(4, 4, bytes(10))


def test_decode_rejects_bad_signature():
    with pytest.raises(PngError, match='signature'):
        decode_grayscale_png(b'not a png at all')


def test_decode_detects_corrupted_chunk_via_crc():
    blob = bytearray(encode_grayscale_png(4, 4, bytes(16)))
    blob[40] ^= 0xFF

    with pytest.raises(PngError, match='CRC'):
        decode_grayscale_png(bytes(blob))


def test_decode_rejects_non_grayscale_color_type():
    # Hand-build a minimal RGB (color type 2) PNG.
    width, height = 1, 1
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    raw = bytes([0, 255, 0, 0])  # filter 0, one RGB pixel
    idat = zlib.compress(raw)
    blob = _SIGNATURE + _chunk(b'IHDR', ihdr) + _chunk(b'IDAT', idat) + _chunk(b'IEND', b'')

    with pytest.raises(PngError, match='color type'):
        decode_grayscale_png(blob)


def test_decode_handles_every_standard_scanline_filter_type():
    """A future producer's PNG library may choose different filters per row."""
    width, height = 4, 5
    rows = [[(r * width + c) % 7 for c in range(width)] for r in range(height)]

    def paeth(a, b, c):
        p = a + b - c
        pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
        if pa <= pb and pa <= pc:
            return a
        if pb <= pc:
            return b
        return c

    raw = bytearray()
    previous = [0] * width
    for row_index, row in enumerate(rows):
        filter_type = row_index % 5
        raw.append(filter_type)
        filtered = []
        for i, value in enumerate(row):
            a = row[i - 1] if i >= 1 else 0
            b = previous[i]
            c = previous[i - 1] if i >= 1 else 0
            if filter_type == 0:
                f = value
            elif filter_type == 1:
                f = (value - a) & 0xFF
            elif filter_type == 2:
                f = (value - b) & 0xFF
            elif filter_type == 3:
                f = (value - (a + b) // 2) & 0xFF
            else:
                f = (value - paeth(a, b, c)) & 0xFF
            filtered.append(f)
        raw += bytes(filtered)
        previous = row

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    idat = zlib.compress(bytes(raw))
    blob = _SIGNATURE + _chunk(b'IHDR', ihdr) + _chunk(b'IDAT', idat) + _chunk(b'IEND', b'')

    decoded_width, decoded_height, pixels = decode_grayscale_png(blob)

    assert (decoded_width, decoded_height) == (width, height)
    assert pixels == bytes(value for row in rows for value in row)
