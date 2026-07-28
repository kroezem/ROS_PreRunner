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

"""ROS node publishing Raspberry Pi platform telemetry."""

import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_interfaces.msg import SystemTelemetry

from runner_telemetry.telemetry import (
    decode_throttled,
    read_soc_temperature,
    read_throttled_sysfs,
    run_vcgencmd,
    WarningTransitions,
)


THERMAL_TEMP_PATH = Path('/sys/class/thermal/thermal_zone0/temp')
THERMAL_TYPE_PATH = Path('/sys/class/thermal/thermal_zone0/type')
THROTTLED_SYSFS_PATH = Path(
    '/sys/devices/platform/soc/soc:firmware/get_throttled',
)
ERROR_THROTTLE_SECONDS = 10.0


class TelemetryNode(Node):
    """Publish typed Raspberry Pi health telemetry at one hertz."""

    def __init__(self):
        super().__init__('telemetry_node')
        self._publisher = self.create_publisher(
            SystemTelemetry,
            '/system/telemetry',
            10,
        )
        self._use_throttled_sysfs = THROTTLED_SYSFS_PATH.is_file()
        self._warnings = WarningTransitions()
        self._timer = self.create_timer(1.0, self._publish_sample)
        throttle_source = (
            str(THROTTLED_SYSFS_PATH)
            if self._use_throttled_sysfs
            else '/usr/bin/vcgencmd get_throttled'
        )
        self.get_logger().info(
            f'thermal source={THERMAL_TEMP_PATH}; '
            f'throttling source={throttle_source}',
        )

    def _read_throttled(self) -> int:
        if self._use_throttled_sysfs:
            return read_throttled_sysfs(THROTTLED_SYSFS_PATH)
        return run_vcgencmd()

    def _publish_sample(self) -> None:
        temperature = math.nan
        throttled = None
        valid = True
        try:
            temperature, _ = read_soc_temperature(
                THERMAL_TEMP_PATH,
                THERMAL_TYPE_PATH,
            )
        except (OSError, UnicodeError, ValueError) as error:
            valid = False
            self.get_logger().error(
                f'SoC temperature source failed: {error}',
                throttle_duration_sec=ERROR_THROTTLE_SECONDS,
            )

        try:
            raw = self._read_throttled()
            throttled = decode_throttled(raw)
        except (OSError, UnicodeError, ValueError) as error:
            valid = False
            self.get_logger().error(
                f'throttling mask source failed: {error}',
                throttle_duration_sec=ERROR_THROTTLE_SECONDS,
            )

        message = SystemTelemetry()
        message.stamp = self.get_clock().now().to_msg()
        message.pi_valid = valid
        message.soc_temperature_celsius = temperature
        if throttled is not None:
            message.throttled_raw = throttled.raw
            message.current_undervoltage = throttled.current_undervoltage
            message.current_frequency_capped = \
                throttled.current_frequency_capped
            message.current_throttled = throttled.current_throttled
            message.current_soft_temperature_limit = \
                throttled.current_soft_temperature_limit
            message.sticky_undervoltage = throttled.sticky_undervoltage
            message.sticky_frequency_capped = \
                throttled.sticky_frequency_capped
            message.sticky_throttled = throttled.sticky_throttled
            message.sticky_soft_temperature_limit = \
                throttled.sticky_soft_temperature_limit

        current_warning, sticky_warning = self._warnings.observe(
            valid,
            throttled,
        )
        if current_warning:
            self.get_logger().warning(
                'Pi current throttling condition activated: '
                f'{", ".join(current_warning)}; '
                f'raw=0x{throttled.raw:x}',
            )
        if sticky_warning:
            self.get_logger().warning(
                'Pi historical conditions occurred since boot and may not '
                f'currently be active: {", ".join(sticky_warning)}; '
                f'raw=0x{throttled.raw:x}',
            )
        self._publisher.publish(message)


def main(args=None):
    """Run the telemetry node."""
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
