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
Wheel twist for the EKF: /joint_states wheel velocities -> /wheel/twist.

diff_drive_controller publishes /odom only while it is active (armed). The encoders
report through joint_state_broadcaster at all times, so this keeps the EKF's velocity
measured while disarmed (standing still, or pushed by hand) and its state bounded.
wheel_radius and wheel_separation come from riggu_description/config/robot.yaml
(passed by bringup.launch.py), the same values diff_drive_controller uses.
"""

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


def twist_from_wheels(left, right, radius, separation):
    """Body (vx, vyaw) of a diff drive from wheel angular velocities (rad/s)."""
    return radius * (left + right) / 2.0, radius * (right - left) / separation


class WheelTwist(Node):
    """Publish a TwistWithCovarianceStamped per /joint_states message."""

    def __init__(self):
        super().__init__('wheel_twist')
        self.radius = self.declare_parameter('wheel_radius', 0.0).value
        self.separation = self.declare_parameter('wheel_separation', 0.0).value
        if self.radius <= 0.0 or self.separation <= 0.0:
            raise SystemExit('wheel_radius and wheel_separation must be > 0')
        self.joints = (self.declare_parameter('left_wheel', 'left_wheel_joint').value,
                       self.declare_parameter('right_wheel', 'right_wheel_joint').value)
        self.frame_id = self.declare_parameter('frame_id', 'base_footprint').value
        # vx, vy, vz, vroll, vpitch, vyaw
        diagonal = self.declare_parameter('covariance_diagonal', [0.0] * 6).value
        self.covariance = [0.0] * 36
        for i, value in enumerate(diagonal):
            self.covariance[7 * i] = float(value)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/wheel/twist', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        """Publish the body twist if both wheels have a velocity in this message."""
        if not msg.velocity:
            return
        velocity = dict(zip(msg.name, msg.velocity))
        if not all(j in velocity for j in self.joints):
            return
        vx, vyaw = twist_from_wheels(velocity[self.joints[0]], velocity[self.joints[1]],
                                     self.radius, self.separation)
        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist.twist.linear.x = vx
        out.twist.twist.angular.z = vyaw
        out.twist.covariance = self.covariance
        self.pub.publish(out)


def main():
    """Run the node."""
    rclpy.init()
    try:
        rclpy.spin(WheelTwist())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # SIGINT/SIGTERM: rclpy's signal handler has already shut the context down


if __name__ == '__main__':
    main()
