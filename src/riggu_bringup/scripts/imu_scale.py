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
Gyro scale correction for the EKF: /imu -> /imu/corrected.

The BNO055 yaw rate reads ~3 % fast (measured 2026-11-28 against floor-mark spins), and
robot_localization has no scale parameter. This divides angular_velocity.z by
gyro_z_scale (robot.yaml imu.gyro_z_scale; bringup.launch.py passes 1.0 outside
mode:=real). Everything else passes through. Temporary: drop it once the DriveLink
imu.bno055 module applies the scale itself.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def corrected_rate(raw, scale):
    """Return the true yaw rate from a gyro that reads `scale` times it."""
    return raw / scale


class ImuScale(Node):
    """Republish /imu with the yaw rate corrected."""

    def __init__(self):
        super().__init__('imu_scale')
        self.scale = self.declare_parameter('gyro_z_scale', 1.0).value
        if self.scale <= 0.0:
            raise SystemExit('gyro_z_scale must be > 0')
        self.pub = self.create_publisher(Imu, '/imu/corrected', 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)

    def on_imu(self, msg):
        """Correct and republish one sample."""
        msg.angular_velocity.z = corrected_rate(msg.angular_velocity.z, self.scale)
        self.pub.publish(msg)


def main():
    """Run the node."""
    rclpy.init()
    try:
        rclpy.spin(ImuScale())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM: rclpy's signal handler has already shut the context down


if __name__ == '__main__':
    main()
