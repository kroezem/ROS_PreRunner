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

"""Tests for the Paddock processes' explicit QoS event policy."""

from rclpy.context import Context
from rclpy.event_handler import SubscriptionEventCallbacks
from runner_paddock.qos_event_node import ExplicitQoSEventNode
from std_msgs.msg import Empty


def test_implicit_qos_event_handlers_are_not_installed():
    context = Context()
    context.init()
    node = ExplicitQoSEventNode('qos_event_policy_test', context=context)
    try:
        publisher = node.create_publisher(Empty, '/qos_event_policy', 1)
        subscription = node.create_subscription(
            Empty, '/qos_event_policy', lambda _message: None, 1
        )

        assert publisher.event_handlers == []
        assert subscription.event_handlers == []
        assert list(node.waitables) == []
    finally:
        node.destroy_node()
        context.shutdown()


def test_explicit_qos_event_handler_is_retained():
    context = Context()
    context.init()
    node = ExplicitQoSEventNode('explicit_qos_event_test', context=context)

    def callback(_event):
        pass

    try:
        subscription = node.create_subscription(
            Empty,
            '/explicit_qos_event',
            lambda _message: None,
            1,
            event_callbacks=SubscriptionEventCallbacks(
                incompatible_qos=callback,
                use_default_callbacks=False,
            ),
        )

        assert len(subscription.event_handlers) == 1
        assert subscription.event_handlers[0].callback is callback
        assert list(node.waitables) == subscription.event_handlers
    finally:
        node.destroy_node()
        context.shutdown()
