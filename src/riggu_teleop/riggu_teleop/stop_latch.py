#!/usr/bin/env python3
# Copyright 2026 Tarun Tom
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
Latching stop: the stop button toggles /estop (std_msgs/Bool), the twist_mux lock.

/estop is republished at `rate` Hz as a heartbeat: twist_mux engages the lock if it
stops arriving, so a dead stop_latch also stops the robot. ~/set (std_srvs/SetBool)
latches or releases it without the pad (e.g. from the laptop).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class EdgeToggle:
    """Toggle a state on each rising edge of a button."""

    def __init__(self, state=False):
        self.state = state
        self._pressed = False

    def update(self, pressed):
        """Feed the button level; return True if the state toggled."""
        rising = pressed and not self._pressed
        self._pressed = pressed
        if rising:
            self.state = not self.state
        return rising


class StopLatch(Node):
    """Stop button -> latched /estop, with heartbeat and a SetBool service."""

    def __init__(self):
        super().__init__('stop_latch')
        self.button = self.declare_parameter('button', 1).value
        rate = self.declare_parameter('rate', 10.0).value
        self.latch = EdgeToggle()
        self.pub = self.create_publisher(Bool, 'estop', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_service(SetBool, '~/set', self.on_set)
        self.create_timer(1.0 / rate, self.publish)
        self.publish()

    def on_joy(self, msg):
        """Toggle on a press of the stop button."""
        if self.button >= len(msg.buttons):
            self.get_logger().warn(
                f'stop button {self.button} not on this pad ({len(msg.buttons)} buttons)',
                throttle_duration_sec=10.0)
            return
        if self.latch.update(bool(msg.buttons[self.button])):
            self.report()
            self.publish()

    def on_set(self, request, response):
        """Latch (true) or release (false) the stop."""
        self.latch.state = request.data
        self.report()
        self.publish()
        response.success = True
        response.message = 'latched' if self.latch.state else 'released'
        return response

    def report(self):
        """Log the new state."""
        if self.latch.state:
            self.get_logger().warn('STOP latched: twist_mux output locked')
        else:
            self.get_logger().info('stop released')

    def publish(self):
        """Publish the current state (heartbeat)."""
        self.pub.publish(Bool(data=self.latch.state))


def main():
    """Run the node."""
    rclpy.init()
    try:
        rclpy.spin(StopLatch())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM: rclpy's signal handler has already shut the context down


if __name__ == '__main__':
    main()
