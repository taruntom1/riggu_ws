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
localization.launch.py on bringup mode:=mock with the sim reference map.

The test raycasts /scan from maps/house.yaml at TRUE_POSE (the mock robot never
moves), gives AMCL a rough initial pose and forces no-motion updates: AMCL must
converge on TRUE_POSE and publish map -> odom.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
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
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
import yaml

MAP_YAML = os.path.join(get_package_share_directory('riggu_navigation'), 'maps', 'house.yaml')
TRUE_POSE = (0.0, 0.0, 0.0)      # map frame: the sim spawn point (mapping started there)
GUESS = (0.30, -0.20, 0.17)      # rough initial pose: 36 cm, 10 deg off
BEAMS = 360
RANGE_MAX = 12.0


def load_map(path):
    """Return (occupied(i, j), resolution, origin, width, height) of a map_server map."""
    with open(path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(path), meta['image']), 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = (int(v) for v in line.split())
        f.readline()  # maxval
        pixels = f.read()
    occupied_below = (1.0 - meta['occupied_thresh']) * 255

    def occupied(i, j):
        # image row 0 is the top (max y)
        return pixels[(height - 1 - j) * width + i] < occupied_below
    return occupied, meta['resolution'], meta['origin'], width, height


def raycast(path, pose):
    """Return BEAMS ranges from pose (x, y, yaw) to the first occupied cell."""
    occupied, res, origin, width, height = load_map(path)
    ranges = []
    for k in range(BEAMS):
        angle = pose[2] - math.pi + k * 2 * math.pi / BEAMS
        c, s = math.cos(angle), math.sin(angle)
        r, hit = 0.05, math.inf
        while r < RANGE_MAX:
            i = int((pose[0] + r * c - origin[0]) / res)
            j = int((pose[1] + r * s - origin[1]) / res)
            if not (0 <= i < width and 0 <= j < height):
                break
            if occupied(i, j):
                hit = r
                break
            r += res / 2
        ranges.append(hit)
    return ranges


@pytest.mark.launch_test
def generate_test_description():
    """Mock bring-up plus localization.launch.py on the sim map."""
    def include(pkg, name, args):
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', name]),
            launch_arguments=args.items())
    return LaunchDescription([
        include('riggu_bringup', 'bringup.launch.py', {'mode': 'mock'}),
        include('riggu_navigation', 'localization.launch.py', {'map': MAP_YAML}),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(q):
    """Yaw of a geometry_msgs Quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TestLocalizationMock(unittest.TestCase):
    """map_server + AMCL against the mock robot standing at the sim spawn point."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_localization_mock')
        cls.map = None
        cls.pose = None
        cls.tf_parents = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        cls.node.create_subscription(OccupancyGrid, '/map',
                                     lambda m: setattr(cls, 'map', m), latched)
        cls.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                     lambda m: setattr(cls, 'pose', m), latched)

        def on_tf(msg):
            for t in msg.transforms:
                cls.tf_parents[t.child_frame_id] = t.header.frame_id
        cls.node.create_subscription(TFMessage, '/tf', on_tf, 50)

        cls.ranges = raycast(MAP_YAML, TRUE_POSE)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/scan', 10)
        cls.node.create_timer(0.1, cls.publish_scan)
        cls.initial_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

    @classmethod
    def publish_scan(cls):
        scan = LaserScan()
        scan.header.stamp = cls.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / BEAMS
        scan.angle_max = scan.angle_min + scan.angle_increment * (BEAMS - 1)
        scan.scan_time = 0.1
        scan.range_min, scan.range_max = 0.05, RANGE_MAX
        scan.ranges = cls.ranges
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

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        return future.result()

    def test_localization(self):
        state = self.node.create_client(GetState, '/amcl/get_state')
        self.assertTrue(state.wait_for_service(timeout_sec=60.0))

        def active():
            result = self.call(state, GetState.Request())
            return result is not None and result.current_state.label == 'active'
        self.assertTrue(self.spin_until(active, 60.0), 'amcl not active')

        self.assertTrue(self.spin_until(lambda: self.map is not None, 30.0), 'no /map')
        self.assertAlmostEqual(self.map.info.resolution, 0.05, places=4)

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = self.node.get_clock().now().to_msg()
        initial.pose.pose.position.x, initial.pose.pose.position.y = GUESS[:2]
        initial.pose.pose.orientation.z = math.sin(GUESS[2] / 2)
        initial.pose.pose.orientation.w = math.cos(GUESS[2] / 2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.25
        initial.pose.covariance[35] = 0.07
        self.initial_pub.publish(initial)
        self.assertTrue(self.spin_until(lambda: self.pose is not None, 30.0), 'no /amcl_pose')

        nomotion = self.node.create_client(Empty, '/request_nomotion_update')
        self.assertTrue(nomotion.wait_for_service(timeout_sec=10.0))

        def error():
            p = self.pose.pose.pose
            dyaw = yaw_of(p.orientation) - TRUE_POSE[2]
            return (math.hypot(p.position.x - TRUE_POSE[0], p.position.y - TRUE_POSE[1]),
                    abs(math.atan2(math.sin(dyaw), math.cos(dyaw))))
        for _ in range(40):
            self.call(nomotion, Empty.Request())
            self.spin_until(lambda: False, 0.3)
        dist, dyaw = error()
        self.assertLess(dist, 0.10, f'AMCL {dist:.3f} m off')
        self.assertLess(math.degrees(dyaw), 5.0, f'AMCL {math.degrees(dyaw):.1f} deg off')

        self.assertEqual(self.tf_parents.get('odom'), 'map', 'no map -> odom TF')
        self.assertEqual(self.tf_parents.get('base_footprint'), 'odom')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Every process exits cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
