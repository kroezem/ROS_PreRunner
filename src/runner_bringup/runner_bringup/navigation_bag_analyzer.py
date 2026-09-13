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

"""Replay Stage-A1 navigation evidence from a rosbag2 recording."""

from argparse import ArgumentParser
from bisect import bisect_left
from collections import defaultdict
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


FOOTPRINT = ((0.230, 0.0825), (0.230, -0.0825),
             (-0.060, -0.0825), (-0.060, 0.0825))
LETHAL = 100
FIRST_PATH_METRES = 0.5
IMMEDIATE_SECONDS = 0.5
CUSP_PROXIMITY_SECONDS = 0.75
STORM_GAP_SECONDS = 5.0
STORM_POSE_METRES = 0.15


@dataclass
class Attempt:
    """One Nav2 action generation reconstructed from NavigationState."""

    generation: int
    mission_id: str
    start: float
    end: float | None = None
    outcome: str = 'INCOMPLETE'
    error_code: int = 0
    classification: str = 'incomplete'
    plan_count: int = 0
    distance_metres: float = 0.0
    replans_per_metre: float | None = None


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def _bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def _world_to_cell(grid, x, y):
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    return (
        math.floor((x - origin.x) / resolution),
        math.floor((y - origin.y) / resolution),
    )


def footprint_lethal_cells(grid, x, y, yaw):
    """Return lethal cells touched by the Nav2-style footprint perimeter."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    vertices = [
        _world_to_cell(
            grid,
            x + cosine * px - sine * py,
            y + sine * px + cosine * py,
        )
        for px, py in FOOTPRINT
    ]
    hits = set()
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        for cell_x, cell_y in _bresenham(*start, *end):
            if not (0 <= cell_x < grid.info.width
                    and 0 <= cell_y < grid.info.height):
                continue
            if grid.data[cell_y * grid.info.width + cell_x] >= LETHAL:
                hits.add((cell_x, cell_y))
    return hits


def path_static_overlap(path, static_map):
    """Count unique occupied static-map cells touched by a path footprint."""
    hits = set()
    # The first pose is the measured start state, not geometry chosen by the
    # planner. Count only the future path so a robot already beside a wall is
    # not mislabeled as a plan crossing static occupancy.
    for stamped_pose in path.poses[1:]:
        pose = stamped_pose.pose
        hits.update(footprint_lethal_cells(
            static_map, pose.position.x, pose.position.y,
            _yaw(pose.orientation),
        ))
    return len(hits)


def stopping_window_bound(
        speed=1.5, lookahead=0.8, command_latency=0.20):
    """Return I11 stopping and robot-centred square-window lower bounds."""
    deceleration = 1.6 * speed + 0.27
    braking = speed * speed / (2.0 * deceleration)
    stopping = braking + speed * command_latency
    margin = math.hypot(0.230, 0.0825)
    radius = stopping + lookahead + margin
    return {
        'speed_mps': speed,
        'deceleration_mps2': deceleration,
        'braking_distance_m': braking,
        'command_latency_distance_m': speed * command_latency,
        'stopping_distance_m': stopping,
        'max_lookahead_m': lookahead,
        'footprint_margin_m': margin,
        'required_radius_m': radius,
        'required_width_m': 2.0 * radius,
    }


def _nearest(items, stamp):
    if not items:
        return None
    stamps = [item[0] for item in items]
    index = bisect_left(stamps, stamp)
    choices = items[max(0, index - 1):min(len(items), index + 1)]
    return min(choices, key=lambda item: abs(item[0] - stamp))


def _pose_at(odometry, stamp):
    nearest = _nearest(odometry, stamp)
    if nearest is None:
        return None
    pose = nearest[1].pose.pose
    return pose.position.x, pose.position.y, _yaw(pose.orientation)


def _transform_at(transforms, stamp):
    nearest = _nearest(transforms, stamp)
    return None if nearest is None else nearest[1]


def _map_pose_to_odom(pose, transform):
    """Apply the inverse of map->odom to a map-frame pose."""
    tx, ty, transform_yaw = transform
    dx = pose.position.x - tx
    dy = pose.position.y - ty
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        _yaw(pose.orientation) - transform_yaw,
    )


def _first_path_poses(path, distance_limit=FIRST_PATH_METRES):
    if not path.poses:
        return []
    selected = [path.poses[0]]
    distance = 0.0
    for previous, current in zip(path.poses, path.poses[1:]):
        distance += math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        selected.append(current)
        if distance >= distance_limit:
            break
    return selected


def _flicker(grids):
    """Compare lethal state in common world cells of consecutive frames."""
    marks = clears = comparisons = frames_with_toggles = 0
    previous = None
    for _, grid in grids:
        resolution = grid.info.resolution
        origin = grid.info.origin.position
        current = {}
        for cell_y in range(grid.info.height):
            for cell_x in range(grid.info.width):
                key = (
                    round((origin.x / resolution) + cell_x),
                    round((origin.y / resolution) + cell_y),
                )
                current[key] = (
                    grid.data[cell_y * grid.info.width + cell_x] >= LETHAL
                )
        if previous is not None:
            frame_marks = frame_clears = 0
            for key in previous.keys() & current.keys():
                before = previous[key]
                after = current[key]
                frame_marks += int(not before and after)
                frame_clears += int(before and not after)
            comparisons += 1
            marks += frame_marks
            clears += frame_clears
            frames_with_toggles += int(frame_marks + frame_clears > 0)
        previous = current
    if not grids:
        return {'available': False, 'frames': 0}
    toggles = marks + clears
    return {
        'available': True,
        'frames': len(grids),
        'frame_comparisons': comparisons,
        'mark_toggles': marks,
        'clear_toggles': clears,
        'total_toggles': toggles,
        'frames_with_toggles': frames_with_toggles,
        'mean_toggles_per_frame': (
            toggles / comparisons if comparisons else 0.0
        ),
    }


def _distance(odometry, start, end):
    samples = [item for item in odometry if start <= item[0] <= end]
    total = 0.0
    for (_, first), (_, second) in zip(samples, samples[1:]):
        a = first.pose.pose.position
        b = second.pose.pose.position
        step = math.hypot(b.x - a.x, b.y - a.y)
        if step <= 0.5:  # Do not count localization discontinuities as travel.
            total += step
    return total


def _direction_flip_times(commands, start, end):
    flips = []
    last_sign = 0
    for stamp, message in commands:
        if not start <= stamp <= end:
            continue
        value = message.linear.x
        sign = 1 if value > 0.01 else (-1 if value < -0.01 else 0)
        if sign and last_sign and sign != last_sign:
            flips.append(stamp)
        if sign:
            last_sign = sign
    return flips


def _read_bag(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    path = Path(uri).resolve()
    if path.is_file():
        path = path.parent
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    wanted = {
        '/map', '/plan', '/local_costmap/costmap',
        '/global_costmap/costmap', '/paddock/navigation_state',
        '/odometry/filtered', '/cmd_vel_nav', '/tf', '/tf_static',
        '/rosout',
    }
    data = defaultdict(list)
    first_stamp = last_stamp = None
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        first_stamp = stamp_ns if first_stamp is None else first_stamp
        last_stamp = stamp_ns
        if topic not in wanted:
            continue
        message = deserialize_message(serialized, get_message(types[topic]))
        data[topic].append((stamp_ns / 1e9, message))
    return path, data, first_stamp / 1e9, last_stamp / 1e9, types


def analyze(uri):
    """Return all Stage-A1 acceptance metrics for one bag."""
    path, data, start_time, end_time, topic_types = _read_bag(uri)
    static_map = data['/map'][0][1] if data['/map'] else None
    plans = data['/plan']
    overlap_details = []
    if static_map is not None:
        for index, (stamp, plan) in enumerate(plans, 1):
            cells = path_static_overlap(plan, static_map)
            if cells:
                overlap_details.append({
                    'plan_index': index,
                    'time_sec': stamp - start_time,
                    'static_lethal_cells': cells,
                })

    attempts = {}
    for stamp, state in data['/paddock/navigation_state']:
        generation = state.action_generation
        if state.state == state.STATE_DISPATCHING and generation not in attempts:
            attempts[generation] = Attempt(
                generation, state.mission_id, stamp,
            )
        attempt = attempts.get(generation)
        if attempt is None or attempt.end is not None:
            continue
        if state.state in (state.STATE_SUCCEEDED, state.STATE_FAILED,
                           state.STATE_CANCELED):
            attempt.end = stamp
            attempt.error_code = state.error_code
            attempt.outcome = {
                state.STATE_SUCCEEDED: 'SUCCEEDED',
                state.STATE_FAILED: 'FAILED',
                state.STATE_CANCELED: 'CANCELED',
            }[state.state]

    odometry = data['/odometry/filtered']
    commands = data['/cmd_vel_nav']
    for attempt in attempts.values():
        if attempt.end is None:
            continue
        attempt.plan_count = sum(
            attempt.start <= stamp <= attempt.end for stamp, _ in plans
        )
        attempt.distance_metres = _distance(
            odometry, attempt.start, attempt.end,
        )
        replans = max(0, attempt.plan_count - 1)
        if attempt.distance_metres > 0.01:
            attempt.replans_per_metre = replans / attempt.distance_metres
        if attempt.error_code == 106:
            duration = attempt.end - attempt.start
            flips = _direction_flip_times(
                commands, attempt.start, attempt.end,
            )
            if duration <= IMMEDIATE_SECONDS:
                attempt.classification = 'immediate-first-tick'
            elif flips and attempt.end - flips[-1] <= CUSP_PROXIMITY_SECONDS:
                attempt.classification = 'direction-flip-proximate'
            else:
                attempt.classification = 'executing-collision'
        else:
            attempt.classification = attempt.outcome.lower()

    transforms = []
    for stamp, tf_message in data['/tf'] + data['/tf_static']:
        for transform in tf_message.transforms:
            if (transform.header.frame_id == 'map'
                    and transform.child_frame_id == 'odom'):
                translation = transform.transform.translation
                transforms.append((stamp, (
                    translation.x, translation.y,
                    _yaw(transform.transform.rotation),
                )))
    transforms.sort(key=lambda item: item[0])

    dispatch_checks = []
    local_grids = data['/local_costmap/costmap']
    for attempt in attempts.values():
        first_plan = next((item for item in plans
                           if item[0] >= attempt.start
                           and (attempt.end is None
                                or item[0] <= attempt.end)), None)
        if first_plan is None:
            dispatch_checks.append({
                'generation': attempt.generation, 'available': False,
                'reason': 'no plan recorded for dispatch',
            })
            continue
        stamp, plan = first_plan
        grid_item = _nearest(local_grids, stamp)
        transform = _transform_at(transforms, stamp)
        if grid_item is None or transform is None:
            dispatch_checks.append({
                'generation': attempt.generation, 'available': False,
                'reason': 'local costmap or map-to-odom TF unavailable',
            })
            continue
        hits = set()
        for stamped_pose in _first_path_poses(plan):
            x, y, yaw = _map_pose_to_odom(stamped_pose.pose, transform)
            hits.update(footprint_lethal_cells(grid_item[1], x, y, yaw))
        dispatch_checks.append({
            'generation': attempt.generation,
            'available': True,
            'plan_time_sec': stamp - start_time,
            'costmap_offset_sec': grid_item[0] - stamp,
            'lethal': bool(hits),
            'lethal_cells': len(hits),
        })

    completed = [item for item in attempts.values() if item.end is not None]
    storms = []
    current = []
    for previous, following in zip(completed, completed[1:]):
        previous_pose = _pose_at(odometry, previous.end)
        following_pose = _pose_at(odometry, following.start)
        repeats = (
            previous.error_code == 106
            and previous.mission_id == following.mission_id
            and following.start - previous.end <= STORM_GAP_SECONDS
            and previous_pose is not None and following_pose is not None
            and math.hypot(previous_pose[0] - following_pose[0],
                           previous_pose[1] - following_pose[1])
            <= STORM_POSE_METRES
        )
        if repeats:
            if not current:
                current = [previous.generation]
            current.append(following.generation)
        elif current:
            storms.append(current)
            current = []
    if current:
        storms.append(current)

    missed = []
    for stamp, log in data['/rosout']:
        if 'Control loop missed its desired rate' in log.msg:
            missed.append({'time_sec': stamp - start_time, 'message': log.msg})

    classes = defaultdict(int)
    outcomes = defaultdict(int)
    for attempt in completed:
        if attempt.error_code == 106:
            classes[attempt.classification] += 1
        outcomes[
            f'{attempt.outcome}-{attempt.error_code}'
            if attempt.error_code else attempt.outcome
        ] += 1
    total_distance = sum(item.distance_metres for item in completed)
    total_replans = sum(max(0, item.plan_count - 1) for item in completed)
    available_checks = [item for item in dispatch_checks
                        if item['available']]
    return {
        'schema_version': 1,
        'bag': str(path),
        'duration_sec': end_time - start_time,
        'topic_types': topic_types,
        'window_derivation': stopping_window_bound(),
        'static_plan_overlap': {
            'available': static_map is not None,
            'plans': len(plans),
            'overlapping_plans': len(overlap_details),
            'details': overlap_details,
        },
        'dispatch_execution_lethality': {
            'distance_m': FIRST_PATH_METRES,
            'available_dispatches': len(available_checks),
            'lethal_dispatches': sum(item['lethal']
                                     for item in available_checks),
            'details': dispatch_checks,
        },
        'attempts': {
            'total': len(completed),
            'outcomes': dict(sorted(outcomes.items())),
            'classes': dict(sorted(classes.items())),
            'details': [asdict(item) for item in completed],
        },
        'replanning': {
            'total_replans': total_replans,
            'distance_metres': total_distance,
            'replans_per_metre': (
                total_replans / total_distance if total_distance else None
            ),
        },
        'identical_redispatch_storms': {
            'count': len(storms),
            'generations': storms,
        },
        'lethal_flicker': {
            'planning': _flicker(data['/global_costmap/costmap']),
            'execution': _flicker(local_grids),
        },
        'control_loop_misses': {
            'count': len(missed),
            'events': missed,
        },
    }


def _summary(report):
    overlap = report['static_plan_overlap']
    dispatch = report['dispatch_execution_lethality']
    attempts = report['attempts']
    replanning = report['replanning']
    rate = replanning['replans_per_metre']
    rate_text = 'n/a' if rate is None else f'{rate:.3f}/m'
    lines = [
        f"Bag: {report['bag']}",
        f"Duration: {report['duration_sec']:.3f} s",
        'Static-plan overlap: '
        f"{overlap['overlapping_plans']}/{overlap['plans']}",
        'Dispatch first-0.5m execution lethality: '
        f"{dispatch['lethal_dispatches']}/"
        f"{dispatch['available_dispatches']} available",
        f"Attempts: {attempts['total']} {attempts['outcomes']}",
        f"106 classes: {attempts['classes']}",
        'Replanning: '
        f"{replanning['total_replans']} replans / "
        f"{replanning['distance_metres']:.3f} m = "
        f'{rate_text}',
        'Identical redispatch storms: '
        f"{report['identical_redispatch_storms']['count']} "
        f"{report['identical_redispatch_storms']['generations']}",
        'Planning lethal flicker: '
        f"{report['lethal_flicker']['planning']}",
        'Execution lethal flicker: '
        f"{report['lethal_flicker']['execution']}",
        'Control-loop misses: '
        f"{report['control_loop_misses']['count']}",
    ]
    return '\n'.join(lines)


def main(args=None):
    """CLI entry point."""
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory or contained MCAP')
    parser.add_argument(
        '--json', action='store_true', help='emit machine-readable JSON',
    )
    parsed = parser.parse_args(args)
    report = analyze(parsed.bag)
    if parsed.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(_summary(report))


if __name__ == '__main__':
    main()
