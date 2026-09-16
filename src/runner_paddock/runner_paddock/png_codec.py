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
Minimal, dependency-free 8-bit grayscale PNG codec.

Uses only the standard library (``zlib``, ``struct``) -- no Pillow or other
imaging dependency. Supports exactly what ``runner_paddock.semantics``
needs: single-channel, 8-bit-per-pixel, non-interlaced grayscale (PNG color
type 0), where the pixel byte value is the authoritative payload.

Encoding always emits filter type 0 (None) per scanline. Decoding supports
every standard PNG filter type (0-4), validates each chunk's CRC32, and
ignores unrecognized ancillary chunks, since a future producer (e.g. an
offline classifier using a different PNG library) is not guaranteed to
write the same filter or chunk choices this module does.
"""

import struct
import zlib

_SIGNATURE = b'\x89PNG\r\n\x1a\n'
_GRAYSCALE_COLOR_TYPE = 0
_BIT_DEPTH = 8


class PngError(ValueError):
    """A PNG that fails to parse, is corrupt, or uses an unsupported feature."""


def _chunk(chunk_type: bytes, data: bytes) -> bytes:
    return (
        struct.pack('>I', len(data))
        + chunk_type
        + data
        + struct.pack('>I', zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
    )


def encode_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    """Encode raw 8-bit grayscale pixel bytes (row-major, top row first)."""
    if width <= 0 or height <= 0:
        raise PngError('width and height must be positive')
    if len(pixels) != width * height:
        raise PngError('pixel data length does not match width * height')
    ihdr = struct.pack(
        '>IIBBBBB', width, height, _BIT_DEPTH, _GRAYSCALE_COLOR_TYPE, 0, 0, 0
    )
    raw = bytearray()
    for row in range(height):
        raw.append(0)  # filter type 0 (None)
        raw += pixels[row * width:(row + 1) * width]
    idat = zlib.compress(bytes(raw), level=9)
    return (
        _SIGNATURE
        + _chunk(b'IHDR', ihdr)
        + _chunk(b'IDAT', idat)
        + _chunk(b'IEND', b'')
    )


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter(raw: bytes, width: int, height: int) -> bytes:
    """Invert every standard PNG scanline filter (bpp=1 for 8-bit grayscale)."""
    out = bytearray(width * height)
    pos = 0
    previous_row = bytes(width)
    for row in range(height):
        filter_type = raw[pos]
        pos += 1
        line = bytearray(raw[pos:pos + width])
        pos += width
        if filter_type == 0:
            pass
        elif filter_type == 1:
            for i in range(width):
                a = line[i - 1] if i >= 1 else 0
                line[i] = (line[i] + a) & 0xFF
        elif filter_type == 2:
            for i in range(width):
                line[i] = (line[i] + previous_row[i]) & 0xFF
        elif filter_type == 3:
            for i in range(width):
                a = line[i - 1] if i >= 1 else 0
                b = previous_row[i]
                line[i] = (line[i] + (a + b) // 2) & 0xFF
        elif filter_type == 4:
            for i in range(width):
                a = line[i - 1] if i >= 1 else 0
                b = previous_row[i]
                c = previous_row[i - 1] if i >= 1 else 0
                line[i] = (line[i] + _paeth(a, b, c)) & 0xFF
        else:
            raise PngError(f'unsupported PNG scanline filter type {filter_type}')
        out[row * width:(row + 1) * width] = line
        previous_row = bytes(line)
    return bytes(out)


def decode_grayscale_png(data: bytes) -> tuple[int, int, bytes]:
    """Decode 8-bit grayscale PNG bytes; returns (width, height, pixels)."""
    if not data.startswith(_SIGNATURE):
        raise PngError('not a PNG file (bad signature)')
    pos = len(_SIGNATURE)
    width = height = None
    idat = bytearray()
    seen_ihdr = False
    seen_iend = False
    while pos < len(data) and not seen_iend:
        if pos + 8 > len(data):
            raise PngError('truncated PNG chunk header')
        length = struct.unpack('>I', data[pos:pos + 4])[0]
        chunk_type = data[pos + 4:pos + 8]
        chunk_start = pos + 8
        chunk_end = chunk_start + length
        if length < 0 or chunk_end + 4 > len(data):
            raise PngError(f'truncated PNG {chunk_type!r} chunk')
        chunk_data = data[chunk_start:chunk_end]
        crc_expected = struct.unpack('>I', data[chunk_end:chunk_end + 4])[0]
        crc_actual = zlib.crc32(chunk_type + chunk_data) & 0xFFFFFFFF
        if crc_actual != crc_expected:
            raise PngError(
                f'PNG {chunk_type!r} chunk fails CRC check (corrupt file)'
            )
        if chunk_type == b'IHDR':
            if len(chunk_data) != 13:
                raise PngError('malformed IHDR chunk')
            (
                width, height, bit_depth, color_type, compression,
                filter_method, interlace,
            ) = struct.unpack('>IIBBBBB', chunk_data)
            if bit_depth != _BIT_DEPTH or color_type != _GRAYSCALE_COLOR_TYPE:
                raise PngError(
                    f'unsupported PNG format (bit depth {bit_depth}, color '
                    f'type {color_type}); only 8-bit grayscale is supported'
                )
            if compression != 0 or filter_method != 0:
                raise PngError('unsupported PNG compression/filter method')
            if interlace != 0:
                raise PngError('interlaced PNG is not supported')
            seen_ihdr = True
        elif chunk_type == b'IDAT':
            if not seen_ihdr:
                raise PngError('PNG IDAT chunk appears before IHDR')
            idat += chunk_data
        elif chunk_type == b'IEND':
            seen_iend = True
        pos = chunk_end + 4
    if not seen_ihdr:
        raise PngError('PNG has no IHDR chunk')
    if not seen_iend:
        raise PngError('PNG has no IEND chunk')
    if not idat:
        raise PngError('PNG has no IDAT data')
    try:
        raw = zlib.decompress(bytes(idat))
    except zlib.error as error:
        raise PngError(f'PNG pixel data does not inflate: {error}') from error
    expected_length = height * (1 + width)
    if len(raw) != expected_length:
        raise PngError('PNG pixel data length does not match declared dimensions')
    pixels = _unfilter(raw, width, height)
    return width, height, pixels
