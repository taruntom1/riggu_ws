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

"""bringup.launch.py mode:=mock: controllers come up and a velocity command moves /odom."""

import time
import unittest

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import Imu, JointState

CONTROLLERS = {'joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller'}


@pytest.mark.launch_test
def generate_test_description():
    """Launch the mock bring-up."""
    bringup = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('riggu_bringup'), 'launch', 'bringup.launch.py']),
        launch_arguments={'mode': 'mock'}.items())
    return LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestBringupMock(unittest.TestCase):
    """Checks against the running mock bring-up."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_bringup_mock')
        cls.latest = {}
        for topic, msg_type in (('/joint_states', JointState), ('/imu', Imu),
                                ('/odom', Odometry)):
            cls.node.create_subscription(
                msg_type, topic, lambda msg, t=topic: cls.latest.__setitem__(t, msg), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout):
        """Spin until predicate() is truthy; return its last value."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            result = predicate()
            if result:
                return result
        return predicate()

    def active_controllers(self):
        """Names of the active controllers, or an empty set."""
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return set()
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.result() is None:
                return set()
            return {c.name for c in future.result().controller if c.state == 'active'}
        finally:
            self.node.destroy_client(client)

    def test_1_controllers_active(self):
        active = self.spin_until(lambda: CONTROLLERS <= self.active_controllers(), 90)
        self.assertTrue(active, f'active: {self.active_controllers()}')

    def test_2_joint_states_and_imu(self):
        self.spin_until(lambda: {'/joint_states', '/imu'} <= self.latest.keys(), 20)
        joints = self.latest.get('/joint_states')
        self.assertIsNotNone(joints, 'no /joint_states')
        self.assertEqual(set(joints.name), {'left_wheel_joint', 'right_wheel_joint'})
        imu = self.latest.get('/imu')
        self.assertIsNotNone(imu, 'no /imu')
        self.assertEqual(imu.header.frame_id, 'imu_link')

    def test_3_cmd_vel_moves_odom(self):
        self.spin_until(lambda: '/odom' in self.latest, 20)
        start = self.latest.get('/odom')
        self.assertIsNotNone(start, 'no /odom')
        self.assertEqual(start.header.frame_id, 'odom')
        self.assertEqual(start.child_frame_id, 'base_footprint')

        pub = self.node.create_publisher(TwistStamped, '/cmd_vel_safe', 10)
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = 0.2
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        odom = self.latest['/odom']
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.2, delta=0.05)
        moved = odom.pose.pose.position.x - start.pose.pose.position.x
        self.assertGreater(moved, 0.2, f'moved {moved:.3f} m in 3 s at 0.2 m/s')
        self.assertAlmostEqual(odom.pose.pose.position.y, 0.0, delta=0.02)

        # cmd_vel_timeout (0.5 s) stops the robot once commands stop
        self.spin_until(lambda: abs(self.latest['/odom'].twist.twist.linear.x) < 1e-3, 3)
        self.assertLess(abs(self.latest['/odom'].twist.twist.linear.x), 1e-3)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Processes exit cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
