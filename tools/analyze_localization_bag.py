#!/usr/bin/env python3
"""Report localization update metrics from one or more ROS 2 MCAP bags."""

import argparse
import bisect
import ctypes
import math
import statistics
import struct
import zlib
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRANSLATION_EPSILON_M = 1e-5
YAW_EPSILON_RAD = 1e-5

HEALTH_TOPICS = (
    '/scan',
    '/scan_slam',
    '/scan_rf2o',
    '/odom_rf2o',
    '/odometry/filtered',
    '/imu/data',
    '/tf',
    '/initialpose',
    '/pose',
)

SCAN_TOPICS = ('/scan', '/scan_slam')
MCAP_MAGIC = b'\x89MCAP0\r\n'

OP_FOOTER = 0x02
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06


class McapParseError(RuntimeError):
    pass


class McapBufferReader:
    def __init__(self, data):
        self.data = memoryview(data)
        self.offset = 0

    def take(self, size):
        end = self.offset + size
        if size < 0 or end > len(self.data):
            raise McapParseError('record ended before all fields were decoded')
        value = self.data[self.offset:end]
        self.offset = end
        return value

    def uint16(self):
        return struct.unpack('<H', self.take(2))[0]

    def uint32(self):
        return struct.unpack('<I', self.take(4))[0]

    def uint64(self):
        return struct.unpack('<Q', self.take(8))[0]

    def string(self):
        return bytes(self.take(self.uint32())).decode('utf-8')

    def bytes64(self):
        return bytes(self.take(self.uint64()))

    def remaining_bytes(self):
        return bytes(self.take(len(self.data) - self.offset))


def normalize_frame(frame_id):
    return frame_id.lstrip('/')


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate_position(positions, receive_ns):
    if not positions:
        return None

    receive_times_ns = [position['receive_ns'] for position in positions]
    index = bisect.bisect_left(receive_times_ns, receive_ns)
    if index < len(positions) and receive_times_ns[index] == receive_ns:
        return positions[index]['x'], positions[index]['y']
    if index == 0 or index == len(positions):
        return None

    previous = positions[index - 1]
    current = positions[index]
    interval_ns = current['receive_ns'] - previous['receive_ns']
    if interval_ns <= 0:
        return None

    fraction = (receive_ns - previous['receive_ns']) / interval_ns
    return (
        previous['x'] + fraction * (current['x'] - previous['x']),
        previous['y'] + fraction * (current['y'] - previous['y']),
    )


def transform_point(transform, point):
    point_x, point_y = point
    cosine = math.cos(transform['yaw'])
    sine = math.sin(transform['yaw'])
    return (
        transform['x'] + cosine * point_x - sine * point_y,
        transform['y'] + sine * point_x + cosine * point_y,
    )


def path_length(positions):
    return sum(
        math.hypot(
            current['x'] - previous['x'],
            current['y'] - previous['y'],
        )
        for previous, current in zip(positions, positions[1:])
    )


def positions_in_window(positions, start_ns, end_ns):
    if not positions:
        return []

    selected = [
        position
        for position in positions
        if start_ns <= position['receive_ns'] <= end_ns
    ]
    for boundary_ns in (start_ns, end_ns):
        point = interpolate_position(positions, boundary_ns)
        if point is not None:
            selected.append({
                'receive_ns': boundary_ns,
                'x': point[0],
                'y': point[1],
            })

    selected.sort(key=lambda position: position['receive_ns'])
    return [
        position
        for index, position in enumerate(selected)
        if (
            index == 0
            or position['receive_ns'] != selected[index - 1]['receive_ns']
        )
    ]


def percentile(values, percentile_value):
    if not values:
        return None

    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    rank = (len(ordered) - 1) * percentile_value / 100.0
    lower_index = math.floor(rank)
    upper_index = math.ceil(rank)
    if lower_index == upper_index:
        return ordered[lower_index]

    fraction = rank - lower_index
    return (
        ordered[lower_index] * (1.0 - fraction)
        + ordered[upper_index] * fraction
    )


def distribution(values):
    return {
        'median': statistics.median(values) if values else None,
        'p90': percentile(values, 90),
        'p95': percentile(values, 95),
        'maximum': max(values) if values else None,
    }


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def summarize_scans(records):
    header_stamps_ns = [record['header_stamp_ns'] for record in records]
    header_gaps_s = [
        (current - previous) / 1e9
        for previous, current in zip(
            header_stamps_ns,
            header_stamps_ns[1:],
        )
    ]
    return {
        'count': len(records),
        'range_lengths': Counter(
            record['range_length'] for record in records
        ),
        'intensity_lengths': Counter(
            record['intensity_length'] for record in records
        ),
        'angle_min': Counter(record['angle_min'] for record in records),
        'angle_max': Counter(record['angle_max'] for record in records),
        'angle_increment': Counter(
            record['angle_increment'] for record in records
        ),
        'first_header_stamp_ns': (
            header_stamps_ns[0] if header_stamps_ns else None
        ),
        'last_header_stamp_ns': (
            header_stamps_ns[-1] if header_stamps_ns else None
        ),
        'non_increasing_header_stamps': sum(
            gap <= 0.0 for gap in header_gaps_s
        ),
        'header_gap_distribution': distribution(header_gaps_s),
    }


def compare_scan_headers(source_records, output_records):
    source_by_stamp = {
        record['header_stamp_ns']: record for record in source_records
    }
    output_by_stamp = {
        record['header_stamp_ns']: record for record in output_records
    }
    matched_stamps = source_by_stamp.keys() & output_by_stamp.keys()
    return {
        'matched': len(matched_stamps),
        'missing_output': len(source_by_stamp.keys() - output_by_stamp.keys()),
        'unexpected_output': len(
            output_by_stamp.keys() - source_by_stamp.keys()
        ),
        'frame_mismatches': sum(
            source_by_stamp[stamp]['frame_id']
            != output_by_stamp[stamp]['frame_id']
            for stamp in matched_stamps
        ),
    }


def format_value(value, digits=3, suffix=''):
    if value is None:
        return 'n/a'
    return f'{value:.{digits}f}{suffix}'


def format_counter(counter):
    return '{' + ', '.join(
        f'{key!r}: {count}'
        for key, count in sorted(counter.items())
    ) + '}'


def format_stamp_ns(stamp_ns):
    if stamp_ns is None:
        return 'n/a'
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return f'{seconds}.{nanoseconds:09d}'


def mcap_files_for_path(bag_path):
    if bag_path.is_file():
        if bag_path.suffix.lower() != '.mcap':
            raise RuntimeError(f'not an MCAP file: {bag_path}')
        return [bag_path]
    if bag_path.is_dir():
        mcap_paths = sorted(bag_path.glob('*.mcap'))
        if mcap_paths:
            return mcap_paths
        raise RuntimeError(f'bag directory contains no MCAP files: {bag_path}')
    raise RuntimeError(f'bag path not found: {bag_path}')


def mcap_has_closing_magic(mcap_path):
    if mcap_path.stat().st_size < len(MCAP_MAGIC):
        return False
    with mcap_path.open('rb') as stream:
        stream.seek(-len(MCAP_MAGIC), 2)
        return stream.read() == MCAP_MAGIC


def decompress_zstd(data, uncompressed_size):
    try:
        library = ctypes.CDLL('libzstd.so.1')
    except OSError as error:
        raise McapParseError(
            f'zstd decompression library unavailable: {error}'
        ) from error
    library.ZSTD_decompress.argtypes = (
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_void_p,
        ctypes.c_size_t,
    )
    library.ZSTD_decompress.restype = ctypes.c_size_t
    library.ZSTD_isError.argtypes = (ctypes.c_size_t,)
    library.ZSTD_isError.restype = ctypes.c_uint
    library.ZSTD_getErrorName.argtypes = (ctypes.c_size_t,)
    library.ZSTD_getErrorName.restype = ctypes.c_char_p

    source = ctypes.create_string_buffer(data)
    destination = ctypes.create_string_buffer(uncompressed_size)
    result = library.ZSTD_decompress(
        destination,
        uncompressed_size,
        source,
        len(data),
    )
    if library.ZSTD_isError(result):
        error_name = library.ZSTD_getErrorName(result).decode()
        raise McapParseError(f'zstd decompression failed: {error_name}')
    if result != uncompressed_size:
        raise McapParseError(
            f'zstd produced {result} bytes, expected {uncompressed_size}'
        )
    return destination.raw[:result]


def decompress_lz4(data, uncompressed_size):
    try:
        library = ctypes.CDLL('liblz4.so.1')
    except OSError as error:
        raise McapParseError(
            f'lz4 decompression library unavailable: {error}'
        ) from error
    library.LZ4F_createDecompressionContext.argtypes = (
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_uint,
    )
    library.LZ4F_createDecompressionContext.restype = ctypes.c_size_t
    library.LZ4F_freeDecompressionContext.argtypes = (ctypes.c_void_p,)
    library.LZ4F_freeDecompressionContext.restype = ctypes.c_size_t
    library.LZ4F_decompress.argtypes = (
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_void_p,
    )
    library.LZ4F_decompress.restype = ctypes.c_size_t
    library.LZ4F_isError.argtypes = (ctypes.c_size_t,)
    library.LZ4F_isError.restype = ctypes.c_uint
    library.LZ4F_getErrorName.argtypes = (ctypes.c_size_t,)
    library.LZ4F_getErrorName.restype = ctypes.c_char_p

    context = ctypes.c_void_p()
    result = library.LZ4F_createDecompressionContext(
        ctypes.byref(context),
        100,
    )
    if library.LZ4F_isError(result):
        error_name = library.LZ4F_getErrorName(result).decode()
        raise McapParseError(
            f'lz4 decompression setup failed: {error_name}'
        )

    source = ctypes.create_string_buffer(data)
    destination = ctypes.create_string_buffer(uncompressed_size)
    source_offset = 0
    destination_offset = 0
    try:
        while True:
            source_size = ctypes.c_size_t(len(data) - source_offset)
            destination_size = ctypes.c_size_t(
                uncompressed_size - destination_offset
            )
            result = library.LZ4F_decompress(
                context,
                ctypes.byref(destination, destination_offset),
                ctypes.byref(destination_size),
                ctypes.byref(source, source_offset),
                ctypes.byref(source_size),
                None,
            )
            if library.LZ4F_isError(result):
                error_name = library.LZ4F_getErrorName(result).decode()
                raise McapParseError(
                    f'lz4 decompression failed: {error_name}'
                )
            source_offset += source_size.value
            destination_offset += destination_size.value
            if result == 0:
                break
            if source_size.value == 0 and destination_size.value == 0:
                raise McapParseError('lz4 decompression made no progress')
    finally:
        library.LZ4F_freeDecompressionContext(context)

    if destination_offset != uncompressed_size:
        raise McapParseError(
            'lz4 produced '
            f'{destination_offset} bytes, expected {uncompressed_size}'
        )
    return destination.raw[:destination_offset]


def decode_chunk(content):
    reader = McapBufferReader(content)
    reader.uint64()
    reader.uint64()
    uncompressed_size = reader.uint64()
    uncompressed_crc = reader.uint32()
    compression = reader.string()
    compressed_records = reader.bytes64()

    if compression == '':
        records = compressed_records
    elif compression == 'zstd':
        records = decompress_zstd(compressed_records, uncompressed_size)
    elif compression == 'lz4':
        records = decompress_lz4(compressed_records, uncompressed_size)
    else:
        raise McapParseError(
            f'unsupported MCAP chunk compression {compression!r}'
        )

    if len(records) != uncompressed_size:
        raise McapParseError(
            f'chunk contains {len(records)} bytes, '
            f'expected {uncompressed_size}'
        )
    if uncompressed_crc and zlib.crc32(records) != uncompressed_crc:
        raise McapParseError('chunk CRC does not match its contents')
    return records


def iter_buffer_records(data):
    offset = 0
    while offset < len(data):
        if len(data) - offset < 9:
            raise McapParseError('truncated record header inside chunk')
        opcode, content_length = struct.unpack_from('<BQ', data, offset)
        offset += 9
        end = offset + content_length
        if end > len(data):
            raise McapParseError('truncated record content inside chunk')
        yield opcode, memoryview(data)[offset:end]
        offset = end


def recover_mcap_file(mcap_path, retained_topics):
    schemas = {}
    channels = {}
    topic_types = {}
    messages = []
    message_count = 0
    bag_start_ns = None
    bag_end_ns = None

    def process_record(opcode, content):
        nonlocal message_count, bag_start_ns, bag_end_ns

        if opcode == OP_SCHEMA:
            reader = McapBufferReader(content)
            schema_id = reader.uint16()
            schemas[schema_id] = reader.string()
        elif opcode == OP_CHANNEL:
            reader = McapBufferReader(content)
            channel_id = reader.uint16()
            schema_id = reader.uint16()
            topic = reader.string()
            reader.string()
            channels[channel_id] = (topic, schema_id)
            if schema_id in schemas:
                topic_types[topic] = schemas[schema_id]
        elif opcode == OP_MESSAGE:
            reader = McapBufferReader(content)
            channel_id = reader.uint16()
            reader.uint32()
            receive_ns = reader.uint64()
            reader.uint64()
            serialized_data = reader.remaining_bytes()
            if channel_id not in channels:
                raise McapParseError(
                    f'message references unknown channel {channel_id}'
                )

            topic, schema_id = channels[channel_id]
            if schema_id in schemas:
                topic_types[topic] = schemas[schema_id]
            message_count += 1
            bag_start_ns = (
                receive_ns
                if bag_start_ns is None
                else min(bag_start_ns, receive_ns)
            )
            bag_end_ns = (
                receive_ns
                if bag_end_ns is None
                else max(bag_end_ns, receive_ns)
            )
            if topic in retained_topics:
                messages.append((topic, serialized_data, receive_ns))
        elif opcode == OP_CHUNK:
            records = decode_chunk(content)
            for nested_opcode, nested_content in iter_buffer_records(records):
                process_record(nested_opcode, nested_content)

    parse_error = None
    footer_seen = False
    with mcap_path.open('rb') as stream:
        if stream.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
            raise McapParseError('file does not start with MCAP magic')

        while True:
            record_offset = stream.tell()
            header = stream.read(9)
            if not header:
                break
            if len(header) < 9:
                parse_error = (
                    f'truncated record header at byte {record_offset}'
                )
                break

            opcode, content_length = struct.unpack('<BQ', header)
            content = stream.read(content_length)
            if len(content) != content_length:
                parse_error = (
                    f'truncated record at byte {record_offset}: '
                    f'expected {content_length} content bytes, '
                    f'found {len(content)}'
                )
                break
            try:
                process_record(opcode, content)
            except (McapParseError, UnicodeDecodeError) as error:
                parse_error = (
                    f'malformed record at byte {record_offset}: {error}'
                )
                break
            if opcode == OP_FOOTER:
                footer_seen = True
                break

    return {
        'topic_types': topic_types,
        'messages': messages,
        'message_count': message_count,
        'bag_start_ns': bag_start_ns,
        'bag_end_ns': bag_end_ns,
        'parse_error': parse_error,
        'footer_seen': footer_seen,
        'truncated': (
            parse_error is not None
            or not footer_seen
            or not mcap_has_closing_magic(mcap_path)
        ),
    }


def recover_mcap_bag(mcap_paths, retained_topics, open_error=None):
    topic_types = {}
    messages = []
    message_count = 0
    bag_start_ns = None
    bag_end_ns = None
    recovery_notes = []
    truncated = False

    for mcap_path in mcap_paths:
        recovered = recover_mcap_file(mcap_path, retained_topics)
        truncated = truncated or recovered['truncated']
        topic_types.update(recovered['topic_types'])
        messages.extend(recovered['messages'])
        message_count += recovered['message_count']
        if recovered['bag_start_ns'] is not None:
            bag_start_ns = (
                recovered['bag_start_ns']
                if bag_start_ns is None
                else min(bag_start_ns, recovered['bag_start_ns'])
            )
            bag_end_ns = (
                recovered['bag_end_ns']
                if bag_end_ns is None
                else max(bag_end_ns, recovered['bag_end_ns'])
            )
        if recovered['parse_error']:
            recovery_notes.append(
                f'{mcap_path.name}: {recovered["parse_error"]}'
            )
        elif not recovered['footer_seen']:
            recovery_notes.append(
                f'{mcap_path.name}: recording ended before footer'
            )

    messages.sort(key=lambda item: item[2])
    return {
        'topic_types': topic_types,
        'messages': messages,
        'message_count': message_count,
        'bag_start_ns': bag_start_ns,
        'bag_end_ns': bag_end_ns,
        'recovery_used': True,
        'truncated': truncated,
        'recovery_notes': recovery_notes,
        'open_error': str(open_error) if open_error is not None else None,
    }


def load_bag_messages(bag_path, retained_topics=None):
    retained_topics = (
        set(HEALTH_TOPICS)
        if retained_topics is None
        else set(retained_topics)
    )
    mcap_paths = mcap_files_for_path(bag_path)
    if any(not mcap_has_closing_magic(path) for path in mcap_paths):
        return recover_mcap_bag(mcap_paths, retained_topics)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    try:
        reader.open(storage_options, converter_options)
    except Exception as error:
        return recover_mcap_bag(
            mcap_paths,
            retained_topics,
            open_error=error,
        )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    messages = []
    message_count = 0
    bag_start_ns = None
    bag_end_ns = None
    try:
        while reader.has_next():
            topic, serialized_data, receive_ns = reader.read_next()
            message_count += 1
            bag_start_ns = (
                receive_ns
                if bag_start_ns is None
                else min(bag_start_ns, receive_ns)
            )
            bag_end_ns = (
                receive_ns
                if bag_end_ns is None
                else max(bag_end_ns, receive_ns)
            )
            if topic in retained_topics:
                messages.append((topic, serialized_data, receive_ns))
    except Exception as error:
        return recover_mcap_bag(
            mcap_paths,
            retained_topics,
            open_error=error,
        )

    messages.sort(key=lambda item: item[2])
    return {
        'topic_types': topic_types,
        'messages': messages,
        'message_count': message_count,
        'bag_start_ns': bag_start_ns,
        'bag_end_ns': bag_end_ns,
        'recovery_used': False,
        'truncated': False,
        'recovery_notes': [],
        'open_error': None,
    }


def resolve_analysis_window(loaded, start_time_s=0.0, end_time_s=None):
    """Resolve a bag-relative receive-time window using loaded bag bounds."""
    bag_start_ns = loaded['bag_start_ns']
    bag_end_ns = loaded['bag_end_ns']
    if bag_start_ns is None:
        raise RuntimeError('bag contains no recoverable messages')

    bag_duration_s = (bag_end_ns - bag_start_ns) / 1e9
    window_start_s = start_time_s
    window_end_s = (
        bag_duration_s
        if end_time_s is None
        else min(end_time_s, bag_duration_s)
    )
    if window_start_s >= bag_duration_s:
        raise RuntimeError(
            f'start time {window_start_s:g} s is outside the '
            f'{bag_duration_s:.3f} s bag'
        )
    if window_end_s <= window_start_s:
        raise RuntimeError(
            f'analysis window end {window_end_s:g} s must be after '
            f'start {window_start_s:g} s'
        )

    return {
        'bag_start_ns': bag_start_ns,
        'bag_end_ns': bag_end_ns,
        'bag_duration_s': bag_duration_s,
        'window_start_s': window_start_s,
        'window_end_s': window_end_s,
        'window_start_ns': bag_start_ns + round(window_start_s * 1e9),
        'window_end_ns': bag_start_ns + round(window_end_s * 1e9),
        'duration_s': window_end_s - window_start_s,
    }


def messages_in_receive_window(messages, start_ns, end_ns):
    """Return messages whose bag receive timestamps lie in an inclusive window."""
    return [
        message
        for message in messages
        if start_ns <= message[2] <= end_ns
    ]


def read_bag(
    bag_path,
    seed_success_max_delay_s,
    start_time_s=0.0,
    end_time_s=None,
):
    loaded = load_bag_messages(bag_path)
    topic_types = loaded['topic_types']
    decoded_topics = (
        '/tf',
        '/initialpose',
        '/pose',
        '/odometry/filtered',
        *SCAN_TOPICS,
    )
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in decoded_topics
        if topic in topic_types
    }

    window = resolve_analysis_window(loaded, start_time_s, end_time_s)
    bag_start_ns = window['bag_start_ns']
    bag_end_ns = window['bag_end_ns']
    bag_duration_s = window['bag_duration_s']
    window_start_s = window['window_start_s']
    window_end_s = window['window_end_s']
    window_start_ns = window['window_start_ns']
    window_end_ns = window['window_end_ns']
    duration_s = window['duration_s']

    counts = {topic: 0 for topic in HEALTH_TOPICS}
    scans = {topic: [] for topic in SCAN_TOPICS}
    all_publications = []
    initial_poses = []
    poses = []
    odometry_positions = []
    tf_odom_positions = []

    for topic, serialized_data, receive_ns in loaded['messages']:
        in_window = window_start_ns <= receive_ns <= window_end_ns

        if topic in counts and in_window:
            counts[topic] += 1

        if topic in SCAN_TOPICS and in_window:
            message = deserialize_message(serialized_data, message_types[topic])
            scans[topic].append({
                'receive_ns': receive_ns,
                'header_stamp_ns': stamp_to_ns(message.header.stamp),
                'frame_id': message.header.frame_id,
                'range_length': len(message.ranges),
                'intensity_length': len(message.intensities),
                'angle_min': message.angle_min,
                'angle_max': message.angle_max,
                'angle_increment': message.angle_increment,
            })

        if topic == '/tf':
            message = deserialize_message(serialized_data, message_types[topic])
            for transform in message.transforms:
                if (
                    normalize_frame(transform.header.frame_id) == 'map'
                    and normalize_frame(transform.child_frame_id) == 'odom'
                ):
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    all_publications.append({
                        'receive_ns': receive_ns,
                        'header_stamp': transform.header.stamp,
                        'x': translation.x,
                        'y': translation.y,
                        'yaw': quaternion_to_yaw(rotation),
                    })
                if (
                    normalize_frame(transform.header.frame_id) == 'odom'
                    and normalize_frame(transform.child_frame_id) == 'base_link'
                ):
                    translation = transform.transform.translation
                    tf_odom_positions.append({
                        'receive_ns': receive_ns,
                        'x': translation.x,
                        'y': translation.y,
                    })

        elif topic == '/initialpose':
            if not in_window:
                continue
            message = deserialize_message(serialized_data, message_types[topic])
            pose = message.pose.pose
            initial_poses.append({
                'receive_ns': receive_ns,
                'header_stamp': message.header.stamp,
                'frame_id': message.header.frame_id,
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': quaternion_to_yaw(pose.orientation),
            })

        elif topic == '/pose':
            if not in_window:
                continue
            message = deserialize_message(serialized_data, message_types[topic])
            pose = message.pose.pose
            poses.append({
                'receive_ns': receive_ns,
                'header_stamp': message.header.stamp,
                'frame_id': message.header.frame_id,
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': quaternion_to_yaw(pose.orientation),
            })

        elif topic == '/odometry/filtered':
            message = deserialize_message(serialized_data, message_types[topic])
            position = message.pose.pose.position
            odometry_positions.append({
                'receive_ns': receive_ns,
                'x': position.x,
                'y': position.y,
            })

    if '/odometry/filtered' in topic_types:
        odom_positions = odometry_positions
        odom_position_source = '/odometry/filtered'
    else:
        odom_positions = tf_odom_positions
        odom_position_source = (
            '/tf odom -> base_link' if tf_odom_positions else None
        )
    odom_positions.sort(key=lambda position: position['receive_ns'])

    all_publications.sort(key=lambda publication: publication['receive_ns'])
    all_corrections = []
    previous_publication = None

    for publication in all_publications:
        if previous_publication is not None:
            delta_x = publication['x'] - previous_publication['x']
            delta_y = publication['y'] - previous_publication['y']
            translation_delta = math.hypot(delta_x, delta_y)
            yaw_delta = wrap_angle(
                publication['yaw'] - previous_publication['yaw']
            )
            if (
                translation_delta > TRANSLATION_EPSILON_M
                or abs(yaw_delta) > YAW_EPSILON_RAD
            ):
                all_corrections.append({
                    'receive_ns': publication['receive_ns'],
                    'translation_delta': translation_delta,
                    'yaw_delta': yaw_delta,
                    'previous_transform': previous_publication,
                    'current_transform': publication,
                })
        previous_publication = publication

    publications = [
        publication
        for publication in all_publications
        if window_start_ns <= publication['receive_ns'] <= window_end_ns
    ]
    corrections = [
        correction
        for correction in all_corrections
        if window_start_ns <= correction['receive_ns'] <= window_end_ns
    ]

    for correction in corrections:
        position = interpolate_position(
            odom_positions,
            correction['receive_ns'],
        )
        if position is None:
            correction['pose_jump'] = None
            correction['lever_arm'] = None
            continue

        previous_position = transform_point(
            correction['previous_transform'],
            position,
        )
        current_position = transform_point(
            correction['current_transform'],
            position,
        )
        correction['pose_jump'] = math.hypot(
            current_position[0] - previous_position[0],
            current_position[1] - previous_position[1],
        )
        correction['lever_arm'] = math.hypot(*position)

    correction_times_s = [
        (correction['receive_ns'] - bag_start_ns) / 1e9
        for correction in corrections
    ]
    correction_gaps_s = [
        current - previous
        for previous, current in zip(
            correction_times_s,
            correction_times_s[1:],
        )
    ]
    translation_deltas_m = [
        correction['translation_delta'] for correction in corrections
    ]
    absolute_yaw_deltas_rad = [
        abs(correction['yaw_delta']) for correction in corrections
    ]
    pose_jumps_m = [
        correction['pose_jump']
        for correction in corrections
        if correction['pose_jump'] is not None
    ]
    lever_arms_m = [
        correction['lever_arm']
        for correction in corrections
        if correction['lever_arm'] is not None
    ]
    pose_jump_distribution = distribution(pose_jumps_m)
    translation_distribution = distribution(translation_deltas_m)
    corrections_with_pose_jumps = [
        correction
        for correction in corrections
        if correction['pose_jump'] is not None
    ]
    invariant_pose_jump_total_m = sum(
        correction['pose_jump']
        for correction in corrections_with_pose_jumps
    )
    inflation_ratio = (
        sum(
            correction['translation_delta']
            for correction in corrections_with_pose_jumps
        )
        / invariant_pose_jump_total_m
        if invariant_pose_jump_total_m
        else None
    )
    window_odom_positions = positions_in_window(
        odom_positions,
        window_start_ns,
        window_end_ns,
    )
    odom_path_length_m = (
        path_length(window_odom_positions) if window_odom_positions else None
    )
    pose_correction_per_m = (
        sum(pose_jumps_m) / odom_path_length_m
        if odom_path_length_m not in (None, 0.0) and pose_jumps_m
        else None
    )

    for index, initial_pose in enumerate(initial_poses):
        receive_ns = initial_pose['receive_ns']
        next_seed_ns = (
            initial_poses[index + 1]['receive_ns']
            if index + 1 < len(initial_poses)
            else None
        )
        previous_correction = next(
            (
                correction
                for correction in reversed(corrections)
                if correction['receive_ns'] <= receive_ns
            ),
            None,
        )
        next_correction = next(
            (
                correction
                for correction in corrections
                if correction['receive_ns'] > receive_ns
            ),
            None,
        )
        initial_pose['relative_receive_s'] = (
            receive_ns - bag_start_ns
        ) / 1e9
        initial_pose['since_previous_correction_s'] = (
            (receive_ns - previous_correction['receive_ns']) / 1e9
            if previous_correction is not None
            else None
        )
        initial_pose['until_next_correction_s'] = (
            (next_correction['receive_ns'] - receive_ns) / 1e9
            if next_correction is not None
            else None
        )
        initial_pose['next_translation_delta'] = (
            next_correction['translation_delta']
            if next_correction is not None
            else None
        )
        initial_pose['next_yaw_delta'] = (
            next_correction['yaw_delta']
            if next_correction is not None
            else None
        )
        first_subsequent_pose = next(
            (
                pose
                for pose in poses
                if (
                    pose['receive_ns'] > receive_ns
                    and (
                        next_seed_ns is None
                        or pose['receive_ns'] < next_seed_ns
                    )
                )
            ),
            None,
        )
        initial_pose['first_subsequent_pose'] = first_subsequent_pose
        initial_pose['pose_delay_s'] = (
            (first_subsequent_pose['receive_ns'] - receive_ns) / 1e9
            if first_subsequent_pose is not None
            else None
        )
        initial_pose['pose_translation_from_seed_m'] = (
            math.hypot(
                first_subsequent_pose['x'] - initial_pose['x'],
                first_subsequent_pose['y'] - initial_pose['y'],
            )
            if first_subsequent_pose is not None
            else None
        )
        initial_pose['pose_yaw_from_seed_rad'] = (
            wrap_angle(
                first_subsequent_pose['yaw'] - initial_pose['yaw']
            )
            if first_subsequent_pose is not None
            else None
        )
        initial_pose['seed_succeeded'] = (
            normalize_frame(initial_pose['frame_id']) == 'map'
            and initial_pose['pose_delay_s'] is not None
            and initial_pose['pose_delay_s'] <= seed_success_max_delay_s
        )

    scan_summaries = {
        topic: summarize_scans(records)
        for topic, records in scans.items()
    }
    return {
        'path': bag_path,
        'duration_s': duration_s,
        'bag_duration_s': bag_duration_s,
        'window_start_s': window_start_s,
        'window_end_s': window_end_s,
        'counts': counts,
        'publications': publications,
        'corrections': corrections,
        'correction_times_s': correction_times_s,
        'gap_distribution': distribution(correction_gaps_s),
        'translation_distribution': translation_distribution,
        'pose_jump_distribution': pose_jump_distribution,
        'yaw_distribution': distribution(absolute_yaw_deltas_rad),
        'lever_mean': (
            statistics.mean(lever_arms_m) if lever_arms_m else None
        ),
        'lever_max': max(lever_arms_m) if lever_arms_m else None,
        'inflation_ratio': inflation_ratio,
        'odom_position_source': odom_position_source,
        'odom_path_length_m': odom_path_length_m,
        'pose_correction_per_m': pose_correction_per_m,
        'initial_poses': initial_poses,
        'poses': poses,
        'scan_summaries': scan_summaries,
        'scan_header_comparison': compare_scan_headers(
            scans['/scan'],
            scans['/scan_slam'],
        ),
        'seed_success_max_delay_s': seed_success_max_delay_s,
        'bag_start_ns': bag_start_ns,
        'recovery_used': loaded['recovery_used'],
        'recovered_message_count': (
            loaded['message_count'] if loaded['recovery_used'] else None
        ),
        'truncated': loaded['truncated'],
        'recovery_notes': loaded['recovery_notes'],
        'open_error': loaded['open_error'],
    }


def print_distribution(label, values, unit):
    print(
        f'  {label}: '
        f'median={format_value(values["median"], suffix=unit)}, '
        f'p90={format_value(values["p90"], suffix=unit)}, '
        f'p95={format_value(values["p95"], suffix=unit)}, '
        f'max={format_value(values["maximum"], suffix=unit)}'
    )


def print_bag_report(result):
    duration_s = result['duration_s']
    publications = result['publications']
    corrections = result['corrections']

    print(f'\n=== {result["path"]} ===')
    print('Time basis: bag receive time for all rates, gaps, and associations')
    print(f'Bag duration: {result["bag_duration_s"]:.3f} s')
    print(
        'Analysis window: '
        f'{result["window_start_s"]:.3f} s to '
        f'{result["window_end_s"]:.3f} s bag-relative '
        f'(duration {duration_s:.3f} s)'
    )
    print('Count source: MCAP message records (metadata.yaml counts not used)')
    print(f'Recording truncated: {"yes" if result["truncated"] else "no"}')
    if result['recovery_used']:
        print(
            'Sequential recovery: '
            f'{result["recovered_message_count"]} messages recovered'
        )
        if result['open_error']:
            print(f'  rosbag2 reader error: {result["open_error"]}')
        for note in result['recovery_notes']:
            print(f'  stopped cleanly: {note}')
    print('\nTopic health:')
    print(f'  {"topic":<16} {"messages":>10} {"average rate":>15}')
    for topic in HEALTH_TOPICS:
        count = result['counts'][topic]
        rate = count / duration_s if duration_s > 0.0 else 0.0
        print(f'  {topic:<16} {count:>10d} {rate:>12.3f} Hz')

    print('\nScan geometry:')
    for topic in SCAN_TOPICS:
        summary = result['scan_summaries'][topic]
        print(f'  {topic}:')
        print(f'    messages: {summary["count"]}')
        print(
            '    len(ranges): '
            f'{format_counter(summary["range_lengths"])}'
        )
        print(
            '    len(intensities): '
            f'{format_counter(summary["intensity_lengths"])}'
        )
        print(
            '    angle_min: '
            f'{format_counter(summary["angle_min"])}'
        )
        print(
            '    angle_max: '
            f'{format_counter(summary["angle_max"])}'
        )
        print(
            '    angle_increment: '
            f'{format_counter(summary["angle_increment"])}'
        )
        print(
            '    header stamps: '
            f'first={format_stamp_ns(summary["first_header_stamp_ns"])}, '
            f'last={format_stamp_ns(summary["last_header_stamp_ns"])}, '
            'non-increasing='
            f'{summary["non_increasing_header_stamps"]}'
        )
        print_distribution(
            'header stamp gap',
            summary['header_gap_distribution'],
            ' s',
        )

    comparison = result['scan_header_comparison']
    print('  /scan -> /scan_slam header preservation:')
    print(f'    matching stamps: {comparison["matched"]}')
    print(f'    source stamps without output: {comparison["missing_output"]}')
    print(
        '    output stamps without source: '
        f'{comparison["unexpected_output"]}'
    )
    print(f'    frame mismatches: {comparison["frame_mismatches"]}')

    publication_rate = (
        len(publications) / duration_s if duration_s > 0.0 else 0.0
    )
    print('\nmap -> odom publications:')
    print(f'  count: {len(publications)}')
    print(f'  rate: {publication_rate:.3f} Hz')
    if publications:
        first_s = (
            publications[0]['receive_ns'] - result['bag_start_ns']
        ) / 1e9
        last_s = (
            publications[-1]['receive_ns'] - result['bag_start_ns']
        ) / 1e9
        print(f'  first bag-relative receive time: {first_s:.3f} s')
        print(f'  last bag-relative receive time: {last_s:.3f} s')
    else:
        print('  first bag-relative receive time: n/a')
        print('  last bag-relative receive time: n/a')

    correction_rate = (
        len(corrections) / duration_s if duration_s > 0.0 else 0.0
    )
    print('\nDistinct corrections:')
    print(
        '  definition: change from the previous published transform with '
        f'translation > {TRANSLATION_EPSILON_M:g} m or '
        f'|yaw| > {YAW_EPSILON_RAD:g} rad'
    )
    print('  initial map -> odom publication counted as correction: no')
    print(f'  count: {len(corrections)}')
    print(f'  rate over analysis window: {correction_rate:.3f} Hz')
    print_distribution(
        'inter-correction gap',
        result['gap_distribution'],
        ' s',
    )
    print_distribution(
        'raw map -> odom translation magnitude',
        result['translation_distribution'],
        ' m',
    )
    print_distribution(
        'pose jump at base_link',
        result['pose_jump_distribution'],
        ' m',
    )
    print(
        '  odom-frame position source: '
        f'{result["odom_position_source"] or "n/a"}'
    )
    print(
        '  lever arm (distance from odom origin): '
        f'mean={format_value(result["lever_mean"], suffix=" m")}, '
        f'max={format_value(result["lever_max"], suffix=" m")}'
    )
    print(
        '  raw/invariant inflation ratio (cumulative raw / invariant): '
        f'{format_value(result["inflation_ratio"], suffix="x")}'
    )
    print(
        '  odom-frame path length: '
        f'{format_value(result["odom_path_length_m"], suffix=" m")}'
    )
    print(
        '  total pose correction per metre travelled: '
        f'{format_value(result["pose_correction_per_m"], suffix=" m/m")}'
    )
    print_distribution(
        'absolute yaw magnitude',
        result['yaw_distribution'],
        ' rad',
    )

    print('\nInitial poses:')
    print(
        '  seed success definition: header.frame_id resolves to map and the '
        'first subsequent /pose arrives before the next seed and within '
        f'{result["seed_success_max_delay_s"]:.3f} s'
    )
    if not result['initial_poses']:
        print('  none')
    for index, initial_pose in enumerate(result['initial_poses'], start=1):
        stamp = initial_pose['header_stamp']
        print(f'  #{index}')
        print(
            '    bag-relative receive time: '
            f'{initial_pose["relative_receive_s"]:.3f} s'
        )
        print(f'    header stamp: {stamp.sec}.{stamp.nanosec:09d}')
        print(f'    header.frame_id: {initial_pose["frame_id"]!r}')
        print(
            f'    pose: x={initial_pose["x"]:.6f} m, '
            f'y={initial_pose["y"]:.6f} m, '
            f'yaw={initial_pose["yaw"]:.6f} rad'
        )
        print(
            '    time since previous distinct correction: '
            f'{format_value(initial_pose["since_previous_correction_s"], suffix=" s")}'
        )
        print(
            '    time until next distinct correction: '
            f'{format_value(initial_pose["until_next_correction_s"], suffix=" s")}'
        )
        print(
            '    next correction change: '
            f'translation={format_value(initial_pose["next_translation_delta"], 6, " m")}, '
            f'yaw={format_value(initial_pose["next_yaw_delta"], 6, " rad")}'
        )
        first_pose = initial_pose['first_subsequent_pose']
        print(
            '    first subsequent /pose delay: '
            f'{format_value(initial_pose["pose_delay_s"], 6, " s")}'
        )
        if first_pose is not None:
            pose_stamp = first_pose['header_stamp']
            pose_translation = format_value(
                initial_pose['pose_translation_from_seed_m'],
                6,
                ' m',
            )
            pose_yaw = format_value(
                initial_pose['pose_yaw_from_seed_rad'],
                6,
                ' rad',
            )
            print(
                '    first subsequent /pose header: '
                f'{pose_stamp.sec}.{pose_stamp.nanosec:09d}, '
                f'frame_id={first_pose["frame_id"]!r}'
            )
            print(
                '    first subsequent /pose delta from seed: '
                f'translation={pose_translation}, yaw={pose_yaw}'
            )
        print(
            '    seed succeeded: '
            f'{"yes" if initial_pose["seed_succeeded"] else "no"}'
        )


def print_comparison(results):
    print('\n=== Bag comparison ===')
    headers = (
        'bag',
        'dur_s',
        'corr',
        'corr_hz',
        'gap_med',
        'gap_max',
        'trans_med',
        'trans_p95',
        'trans_max',
        'pose_jump_med',
        'pose_jump_p95',
        'pose_jump_max',
        'lever_mean',
        'yaw_abs_max',
    )
    rows = []
    for result in results:
        rows.append((
            result['path'].name,
            f'{result["duration_s"]:.1f}',
            str(len(result['corrections'])),
            f'{len(result["corrections"]) / result["duration_s"]:.3f}',
            format_value(result['gap_distribution']['median']),
            format_value(result['gap_distribution']['maximum']),
            format_value(result['translation_distribution']['median']),
            format_value(result['translation_distribution']['p95']),
            format_value(result['translation_distribution']['maximum']),
            format_value(result['pose_jump_distribution']['median']),
            format_value(result['pose_jump_distribution']['p95']),
            format_value(result['pose_jump_distribution']['maximum']),
            format_value(result['lever_mean']),
            format_value(result['yaw_distribution']['maximum']),
        ))

    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    print(
        '  '.join(
            header.ljust(widths[index])
            for index, header in enumerate(headers)
        )
    )
    print(
        '  '.join('-' * width for width in widths)
    )
    for row in rows:
        print(
            '  '.join(
                value.ljust(widths[index])
                for index, value in enumerate(row)
            )
        )
    print(
        'Units: gaps in s, translation/pose jump/lever arm in m, '
        'yaw_abs_max in rad; '
        'initial transform excluded from corr.'
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Analyze slam_toolbox map -> odom corrections in MCAP bags.',
    )
    parser.add_argument(
        'bags',
        nargs='+',
        type=Path,
        metavar='BAG',
        help='ROS 2 MCAP bag directory or bare .mcap file',
    )
    parser.add_argument(
        '--start-time',
        type=float,
        default=0.0,
        metavar='SECONDS',
        help='analysis-window start in bag-relative seconds (default: 0)',
    )
    parser.add_argument(
        '--end-time',
        type=float,
        default=None,
        metavar='SECONDS',
        help='analysis-window end in bag-relative seconds (default: bag end)',
    )
    parser.add_argument(
        '--seed-success-max-delay',
        type=float,
        default=2.0,
        metavar='SECONDS',
        help=(
            'maximum first-subsequent-/pose delay counted as a successful '
            'map-frame seed (default: 2.0)'
        ),
    )
    args = parser.parse_args()
    if not math.isfinite(args.start_time) or args.start_time < 0.0:
        parser.error('--start-time must be a finite, non-negative value')
    if args.end_time is not None:
        if not math.isfinite(args.end_time) or args.end_time < 0.0:
            parser.error('--end-time must be a finite, non-negative value')
        if args.end_time <= args.start_time:
            parser.error('--end-time must be greater than --start-time')
    return args


def main():
    args = parse_args()
    results = []

    for bag_path in args.bags:
        try:
            result = read_bag(
                bag_path,
                args.seed_success_max_delay,
                args.start_time,
                args.end_time,
            )
        except Exception as error:
            raise SystemExit(f'error: failed to read {bag_path}: {error}') from error
        print_bag_report(result)
        results.append(result)

    if len(results) > 1:
        print_comparison(results)


if __name__ == '__main__':
    main()
