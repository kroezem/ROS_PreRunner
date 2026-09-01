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

"""Dedicated ROS executor thread lifecycle for the Paddock web process."""

import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor

from runner_paddock.ros_state_node import RosStateNode
from runner_paddock.state_cache import StateCache


class RosRuntime:
    """Own one ROS context, subscriber node, executor, and thread."""

    def __init__(self, cache: StateCache) -> None:
        self._cache = cache
        self._context: Context | None = None
        self._node: RosStateNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._spin_error: BaseException | None = None

    @property
    def thread_alive(self) -> bool:
        """Report whether the executor thread remains alive."""
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Initialize ROS and spin the read-only node on a dedicated thread."""
        if self._thread is not None:
            raise RuntimeError('ROS runtime is already started')
        context = Context()
        rclpy.init(context=context)
        try:
            node = RosStateNode(self._cache, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
        except BaseException:
            context.try_shutdown()
            raise

        self._context = context
        self._node = node
        self._executor = executor
        self._spin_error = None
        self._thread = threading.Thread(
            target=self._spin,
            name='paddock-ros-executor',
            daemon=False,
        )
        self._thread.start()

    def stop(self, timeout_sec: float = 5.0) -> None:
        """Stop ROS and verify that no executor thread is left behind."""
        thread = self._thread
        executor = self._executor
        node = self._node
        context = self._context
        if thread is None:
            return

        if executor is not None:
            executor.shutdown(timeout_sec=timeout_sec)
        thread.join(timeout_sec)
        if thread.is_alive():
            if context is not None:
                context.try_shutdown()
            thread.join(timeout_sec)
        if thread.is_alive():
            raise RuntimeError('ROS executor thread did not stop cleanly')

        if executor is not None and node is not None:
            executor.remove_node(node)
        if node is not None:
            node.destroy_node()
        if context is not None:
            context.try_shutdown()

        self._thread = None
        self._executor = None
        self._node = None
        self._context = None
        if self._spin_error is not None:
            error = self._spin_error
            self._spin_error = None
            raise RuntimeError('ROS executor stopped unexpectedly') from error

    def _spin(self) -> None:
        try:
            assert self._executor is not None
            self._executor.spin()
        except ExternalShutdownException:
            pass
        except BaseException as error:
            self._spin_error = error
