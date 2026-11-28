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
slam.launch.py on bringup mode:=mock with a synthetic /scan of a 4 x 3 m room.

slam_toolbox goes active, publishes a 5 cm /map with the room's walls occupied, and
owns map -> odom (the EKF keeps odom -> base_footprint).
"""

import math
import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# x_min, x_max, y_min, y_max around the laser (m); off the 5 cm grid, so no wall
# lands exactly on the edge of the map's bounding box
ROOM = (-1.53, 2.47, -1.42, 1.58)
BEAMS = 360


def room_range(angle):
    """Distance from the origin to the ROOM walls along angle."""
    x_min, x_max, y_min, y_max = ROOM
    c, s = math.cos(angle), math.sin(angle)
    hits = []
    if c > 1e-9:
        hits.append(x_max / c)
    if c < -1e-9:
        hits.append(x_min / c)
    if s > 1e-9:
        hits.append(y_max / s)
    if s < -1e-9:
        hits.append(y_min / s)
    return min(hits)


@pytest.mark.launch_test
def generate_test_description():
    """Mock bring-up plus slam.launch.py."""
    def include(pkg, name, args):
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', name]),
            launch_arguments=args.items())
    return LaunchDescription([
        include('riggu_bringup', 'bringup.launch.py', {'mode': 'mock'}),
        include('riggu_navigation', 'slam.launch.py', {}),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSlamMock(unittest.TestCase):
    """slam_toolbox against the mock robot standing in a box room."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_slam_mock')
        cls.map = None
        cls.tf_parents = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.node.create_subscription(OccupancyGrid, '/map',
                                     lambda m: setattr(cls, 'map', m), latched)

        def on_tf(msg):
            for t in msg.transforms:
                cls.tf_parents[t.child_frame_id] = t.header.frame_id
        cls.node.create_subscription(TFMessage, '/tf', on_tf, 50)

        cls.scan_pub = cls.node.create_publisher(LaserScan, '/scan', 10)
        cls.node.create_timer(0.1, cls.publish_scan)

    @classmethod
    def publish_scan(cls):
        scan = LaserScan()
        scan.header.stamp = cls.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (BEAMS - 1)
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = 0.05, 12.0
        # +-2 cm jitter: slam_toolbox sizes the grid to the farthest hits and drops a
        # hit on its edge, so a perfectly flat synthetic wall would vanish entirely
        scan.ranges = [room_range(scan.angle_min + i * scan.angle_increment)
                       + 0.02 * (i % 3 - 1) for i in range(BEAMS)]
        cls.scan_pub.publish(scan)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def occupied_near(self, wx, wy, band=0.15):
        """Return whether /map has an occupied cell within band of (wx, wy)."""
        info = self.map.info
        for k, value in enumerate(self.map.data):
            if value == 100:
                x = info.origin.position.x + (k % info.width + 0.5) * info.resolution
                y = info.origin.position.y + (k // info.width + 0.5) * info.resolution
                if abs(x - wx) < band and abs(y - wy) < band:
                    return True
        return False

    def describe_map(self):
        """Origin, size and occupied-cell bounds of /map, for failure messages."""
        info = self.map.info
        cells = [(k % info.width, k // info.width)
                 for k, v in enumerate(self.map.data) if v == 100]
        bounds = None
        if cells:
            xs = [info.origin.position.x + (i + 0.5) * info.resolution for i, _ in cells]
            ys = [info.origin.position.y + (j + 0.5) * info.resolution for _, j in cells]
            bounds = (round(min(xs), 2), round(max(xs), 2), round(min(ys), 2), round(max(ys), 2))
        free = sum(1 for v in self.map.data if v == 0)
        return (f'origin ({info.origin.position.x:.2f}, {info.origin.position.y:.2f}) '
                f'{info.width}x{info.height}, {len(cells)} occupied in x/y {bounds}, '
                f'{free} free')

    def test_slam(self):
        client = self.node.create_client(GetState, '/slam_toolbox/get_state')
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))

        def active():
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            return future.result() is not None and future.result().current_state.label == 'active'
        self.assertTrue(self.spin_until(active, 60.0), 'slam_toolbox not active')

        self.assertTrue(self.spin_until(lambda: self.map is not None, 60.0), 'no /map')
        self.assertAlmostEqual(self.map.info.resolution, 0.05, places=4)
        self.assertEqual(self.map.header.frame_id, 'map')

        # All four walls show up as occupied cells
        # (/map is republished every map_update_interval as scans come in).
        for wx, wy in ((ROOM[0], 0.0), (ROOM[1], 0.0), (0.0, ROOM[2]), (0.0, ROOM[3])):
            self.assertTrue(self.spin_until(lambda: self.occupied_near(wx, wy), 30.0),
                            f'wall at ({wx}, {wy}) not in /map: {self.describe_map()}')

        self.assertTrue(self.spin_until(lambda: self.tf_parents.get('odom') == 'map', 30.0),
                        'no map -> odom TF')
        self.assertEqual(self.tf_parents.get('base_footprint'), 'odom')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Every process exits cleanly."""

    def test_exit_codes(self, proc_info):
        for name in proc_info.process_names():
            allowed = [0, -2, -15]
            if 'slam_toolbox' in name:
                # Jazzy slam_toolbox 2.8 aborts in teardown on SIGINT (RCLError: timer
                # created after rcl_shutdown); mapping is unaffected.
                allowed.append(-6)
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=allowed, process=name)
