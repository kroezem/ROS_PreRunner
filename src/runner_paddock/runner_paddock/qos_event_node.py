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

"""ROS node policy for processes that diagnose missing state themselves."""

from rclpy.event_handler import PublisherEventCallbacks
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.node import Node


class ExplicitQoSEventNode(Node):
    """Avoid implicit warning-only QoS event handlers unless requested."""

    def create_subscription(self, *args, **kwargs):
        """Create a subscription without rclpy's implicit event callbacks."""
        if kwargs.get('event_callbacks') is None:
            kwargs['event_callbacks'] = SubscriptionEventCallbacks(
                use_default_callbacks=False
            )
        return super().create_subscription(*args, **kwargs)

    def create_publisher(self, *args, **kwargs):
        """Create a publisher without rclpy's implicit event callbacks."""
        if kwargs.get('event_callbacks') is None:
            kwargs['event_callbacks'] = PublisherEventCallbacks(
                use_default_callbacks=False
            )
        return super().create_publisher(*args, **kwargs)
