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
Wheel radius / separation calibration from /joint_states wheel angles.

  straight D   drive straight, tape-measure D metres  -> wheel.radius
  turns N      N full turns in place (calibrate radius first) -> wheel.separation

Press Enter at the start and at the end; drive with teleop in another terminal.
Prints the odometry error with the current robot.yaml values and the corrected
value. It does not edit robot.yaml.
"""

import argparse
import math
import os
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
import yaml

JOINTS = ('left_wheel_joint', 'right_wheel_joint')


def radius_from_straight(d_left, d_right, distance):
    """Wheel radius that makes the mean wheel angle cover `distance`."""
    return abs(distance) / abs((d_left + d_right) / 2.0)


def separation_from_turns(d_left, d_right, radius, turns):
    """Wheel separation that makes the wheel-angle difference `turns` full turns."""
    return radius * abs(d_right - d_left) / (2.0 * math.pi * abs(turns))


def straight_error(d_left, d_right, radius, distance):
    """Relative odometry distance error with `radius`."""
    return (radius * abs((d_left + d_right) / 2.0) - abs(distance)) / abs(distance)


def turns_error(d_left, d_right, radius, separation, turns):
    """Relative odometry yaw error with `radius` and `separation`."""
    yaw = radius * abs(d_right - d_left) / separation
    return (yaw - 2.0 * math.pi * abs(turns)) / (2.0 * math.pi * abs(turns))


def load_wheel(path):
    """Return (radius, separation) from robot.yaml."""
    with open(path, encoding='utf-8') as f:
        wheel = yaml.safe_load(f)['wheel']
    return float(wheel['radius']['value']), float(wheel['separation']['value'])


def main():
    """Run the interactive calibration."""
    default_config = os.path.join(
        get_package_share_directory('riggu_description'), 'config', 'robot.yaml')
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('test', choices=['straight', 'turns'])
    parser.add_argument('amount', type=float, help='metres (straight) or full turns (turns)')
    parser.add_argument('--robot-config', default=default_config)
    args, ros_args = parser.parse_known_args()
    if args.amount == 0:
        parser.error('amount must be non-zero')
    radius, separation = load_wheel(args.robot_config)

    rclpy.init(args=ros_args)
    node = rclpy.create_node('calibrate_wheels')
    latest = {}

    def on_joints(msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                latest[name] = pos

    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def snapshot(prompt):
        input(prompt)
        missing = [j for j in JOINTS if j not in latest]
        if missing:
            raise SystemExit(f'no /joint_states for {missing}; is bringup running?')
        return [latest[j] for j in JOINTS]

    try:
        print(f'{args.robot_config}: wheel.radius {radius:.5f} m, '
              f'wheel.separation {separation:.4f} m')
        start = snapshot('Robot at the start mark. Enter to record... ')
        end = snapshot(f'Drive the {args.test} test now, then stop. Enter to record... ')
    except (KeyboardInterrupt, EOFError):
        return
    finally:
        # stop spinning before tearing down, or the spin thread raises
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    d_left, d_right = end[0] - start[0], end[1] - start[1]
    print(f'wheel angle change: left {d_left:+.3f} rad, right {d_right:+.3f} rad')
    if args.test == 'straight':
        if abs(d_left + d_right) < 1e-3:
            raise SystemExit('wheels did not turn forward or backward')
        print(f'odometry distance error (current radius): '
              f'{100 * straight_error(d_left, d_right, radius, args.amount):+.2f} %')
        print(f'calibrated wheel.radius: '
              f'{radius_from_straight(d_left, d_right, args.amount):.5f} m')
    else:
        if abs(d_right - d_left) < 1e-3:
            raise SystemExit('robot did not turn')
        print(f'odometry yaw error (current values): '
              f'{100 * turns_error(d_left, d_right, radius, separation, args.amount):+.2f} %')
        print(f'calibrated wheel.separation: '
              f'{separation_from_turns(d_left, d_right, radius, args.amount):.4f} m')


if __name__ == '__main__':
    main()
