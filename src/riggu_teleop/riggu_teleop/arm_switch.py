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
Arm and disarm the drive from the pad.

Holding every `arm_buttons` for `arm_hold` s activates `controller` (on the real robot
this arms the DriveLink motors); `disarm_button` deactivates it at once. Bring-up never
arms by itself: this is the deliberate human action that does.
"""

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ArmCombo:
    """Button logic: 'arm' once per held combo, 'disarm' on a press."""

    def __init__(self, arm_buttons, hold, disarm_button):
        self.arm_buttons = list(arm_buttons)
        self.hold = hold
        self.disarm_button = disarm_button
        self._since = None
        self._fired = False
        self._disarm_pressed = False

    def buttons_used(self):
        """Highest button index the combo reads, plus one."""
        return max(self.arm_buttons + [self.disarm_button]) + 1

    def update(self, buttons, now):
        """Feed button levels at time `now` (s); return 'arm', 'disarm' or None."""
        disarm = bool(buttons[self.disarm_button])
        rising = disarm and not self._disarm_pressed
        self._disarm_pressed = disarm
        if rising:
            self._since, self._fired = None, False
            return 'disarm'
        if disarm or not all(buttons[i] for i in self.arm_buttons):
            self._since, self._fired = None, False
            return None
        if self._since is None:
            self._since = now
        if not self._fired and now - self._since >= self.hold:
            self._fired = True
            return 'arm'
        return None


class ArmSwitch(Node):
    """/joy combo -> controller_manager switch_controller."""

    def __init__(self):
        super().__init__('arm_switch')
        self.combo = ArmCombo(
            self.declare_parameter('arm_buttons', [4, 9]).value,
            self.declare_parameter('arm_hold', 1.0).value,
            self.declare_parameter('disarm_button', 8).value)
        self.controller = self.declare_parameter('controller', 'diff_drive_controller').value
        self.client = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

    def on_joy(self, msg):
        """Run the combo logic; switch the controller when it fires."""
        if len(msg.buttons) < self.combo.buttons_used():
            self.get_logger().warn(
                f'arm/disarm buttons not on this pad ({len(msg.buttons)} buttons)',
                throttle_duration_sec=10.0)
            return
        action = self.combo.update(msg.buttons, self.get_clock().now().nanoseconds * 1e-9)
        if action:
            self.switch(action)

    def switch(self, action):
        """Activate (arm) or deactivate (disarm) the drive controller."""
        if not self.client.service_is_ready():
            self.get_logger().error(f'{action}: controller_manager not available')
            return
        request = SwitchController.Request(
            strictness=SwitchController.Request.STRICT, timeout=Duration(sec=2))
        if action == 'arm':
            request.activate_controllers = [self.controller]
            self.get_logger().warn(f'ARMING: activating {self.controller}')
        else:
            request.deactivate_controllers = [self.controller]
            self.get_logger().warn(f'disarming: deactivating {self.controller}')
        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self.done(action, f))

    def done(self, action, future):
        """Log the switch result."""
        result = future.result()
        if result is not None and result.ok:
            self.get_logger().warn(f'{action}ed')
        else:
            detail = getattr(result, 'message', '') if result is not None else future.exception()
            self.get_logger().error(
                f'{action} failed (already {action}ed? see controller_manager log) {detail}')


def main():
    """Run the node."""
    rclpy.init()
    try:
        rclpy.spin(ArmSwitch())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM: rclpy's signal handler has already shut the context down


if __name__ == '__main__':
    main()
