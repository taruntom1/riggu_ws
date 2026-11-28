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
Closed-loop odometry check: wheel /odom vs EKF /odometry/filtered vs /imu yaw.

Press Enter at a floor mark, drive a closed path that ends back on the mark (a square,
N full spins in place), press Enter again. Per source it prints the distance from the
start and the yaw change, unwrapped, with its residual from the nearest whole turn: on a
closed path both are the source's error. It also prints the largest IMU-minus-wheel yaw
gap seen during the run (motor magnets disturbing BNO055 heading show up there).
"""

import argparse
import math
import threading

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

SOURCES = ('/odom', '/odometry/filtered', '/imu')


def yaw_of(q):
    """Yaw (rad) of a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Track:
    """Unwrapped yaw and the latest position of one source."""

    def __init__(self):
        self.yaw = None  # unwrapped
        self.xy = None

    def update(self, yaw, xy=None):
        self.yaw = yaw if self.yaw is None else self.yaw + wrap(yaw - self.yaw)
        self.xy = xy


def turn_residual(dyaw):
    """Yaw change minus the nearest whole number of turns (rad)."""
    return dyaw - 2.0 * math.pi * round(dyaw / (2.0 * math.pi))


def report(start, end):
    """Lines describing each source's change between two snapshots {topic: (yaw, xy)}."""
    lines = []
    for topic in SOURCES:
        if topic not in start or topic not in end:
            lines.append(f'{topic:20s} no data')
            continue
        (y0, p0), (y1, p1) = start[topic], end[topic]
        dyaw = y1 - y0
        dist = '' if p0 is None else f'distance from start {math.dist(p0, p1):.3f} m, '
        lines.append(f'{topic:20s} {dist}yaw change {math.degrees(dyaw):+9.2f} deg '
                     f'(residual {math.degrees(turn_residual(dyaw)):+6.2f} deg)')
    return lines


def main():
    """Run the interactive check."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    _, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('odom_check')
    tracks = {t: Track() for t in SOURCES}
    lock = threading.Lock()
    gap = {'base': None, 'max': 0.0, 'running': False}

    def on_odom(topic, msg):
        p = msg.pose.pose
        with lock:
            tracks[topic].update(yaw_of(p.orientation), (p.position.x, p.position.y))
            watch()

    def on_imu(msg):
        with lock:
            tracks['/imu'].update(yaw_of(msg.orientation))
            watch()

    def watch():
        imu, wheel = tracks['/imu'].yaw, tracks['/odom'].yaw
        if not gap['running'] or imu is None or wheel is None:
            return
        if gap['base'] is None:
            gap['base'] = imu - wheel
        gap['max'] = max(gap['max'], abs(imu - wheel - gap['base']))

    for topic in SOURCES[:2]:
        node.create_subscription(Odometry, topic, lambda m, t=topic: on_odom(t, m), 20)
    node.create_subscription(Imu, '/imu', on_imu, 50)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def snapshot():
        with lock:
            return {t: (k.yaw, k.xy) for t, k in tracks.items() if k.yaw is not None}

    try:
        input('Robot on the floor mark. Enter to record the start... ')
        start = snapshot()
        missing = [t for t in SOURCES if t not in start]
        if missing:
            print(f'warning: no data yet on {missing}')
        with lock:
            gap['running'] = True
        input('Drive the closed path back onto the mark, stop. Enter to record... ')
        end = snapshot()
        with lock:
            max_gap = gap['max'] if gap['base'] is not None else None
    except (KeyboardInterrupt, EOFError):
        return
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    print('\n'.join(report(start, end)))
    if max_gap is not None:
        print(f'max |IMU yaw - wheel yaw| drift during the run: {math.degrees(max_gap):.2f} deg')


if __name__ == '__main__':
    main()
