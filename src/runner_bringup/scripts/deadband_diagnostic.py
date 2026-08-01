#!/usr/bin/env python3
"""Record commanded ESC pulse widths without controlling hardware."""

import csv
from datetime import datetime
from pathlib import Path

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


NEUTRAL_US = 1500
THR_MAX_US = 2000
BRK_MAX_US = 1000

AXIS_BRAKE = 2
AXIS_THROTTLE = 5


def throttle_us(linear_x):
    cmd = max(-1.0, min(1.0, linear_x))
    if cmd > 0.0:
        return int(NEUTRAL_US + cmd * (THR_MAX_US - NEUTRAL_US))
    if cmd < -0.05:
        return int(NEUTRAL_US + cmd * (NEUTRAL_US - BRK_MAX_US))
    return NEUTRAL_US


class DeadbandDiagnostic(Node):
    def __init__(self, csv_file):
        super().__init__('deadband_diagnostic')
        self._csv_file = csv_file
        self._writer = csv.writer(csv_file)
        self._writer.writerow([
            'timestamp',
            'raw_throttle_axis',
            'raw_brake_axis',
            'linear_x',
            'thr_us',
        ])
        self._throttle_axis = None
        self._brake_axis = None
        self._linear_x = None
        self.min_thr_us = None
        self.max_thr_us = None
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.05, self._sample)

    def _on_joy(self, msg):
        if max(AXIS_BRAKE, AXIS_THROTTLE) >= len(msg.axes):
            return
        self._throttle_axis = msg.axes[AXIS_THROTTLE]
        self._brake_axis = msg.axes[AXIS_BRAKE]

    def _on_cmd(self, msg):
        self._linear_x = msg.linear.x

    def _sample(self):
        if (
            self._throttle_axis is None
            or self._brake_axis is None
            or self._linear_x is None
        ):
            return

        timestamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
        thr_us = throttle_us(self._linear_x)
        self.min_thr_us = (
            thr_us if self.min_thr_us is None else min(self.min_thr_us, thr_us)
        )
        self.max_thr_us = (
            thr_us if self.max_thr_us is None else max(self.max_thr_us, thr_us)
        )
        row = [
            timestamp,
            self._throttle_axis,
            self._brake_axis,
            self._linear_x,
            thr_us,
        ]
        self._writer.writerow(row)
        self._csv_file.flush()
        print(
            f'{timestamp} throttle={self._throttle_axis:+.6f} '
            f'brake={self._brake_axis:+.6f} '
            f'linear.x={self._linear_x:+.6f} thr_us={thr_us}',
            flush=True,
        )


def main():
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = Path.cwd() / f'deadband_{stamp}.csv'
    rclpy.init()
    with csv_path.open('w', newline='') as csv_file:
        node = DeadbandDiagnostic(csv_file)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if node.min_thr_us is None:
        print('thr_us range: no samples')
    else:
        print(f'thr_us range: {node.min_thr_us}..{node.max_thr_us}')
    print(f'CSV: {csv_path}')


if __name__ == '__main__':
    main()
