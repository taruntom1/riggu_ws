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
bringup.launch.py mode:=mock: controllers, then the cmd_vel chain end to end.

/cmd_vel_nav and a fake /joy go through twist_mux -> collision_monitor -> /cmd_vel_safe;
the pad nodes and the arm switch run (joy:=true arm:=true) on the test's /joy.
Button and axis indices come from riggu_teleop/config/joy_generic.yaml.
"""

import time
import unittest

from ament_index_python.packages import get_package_share_directory
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
from sensor_msgs.msg import Imu, JointState, Joy
import yaml

CONTROLLERS = {'joint_state_broadcaster', 'imu_sensor_broadcaster', 'diff_drive_controller'}
with open(f"{get_package_share_directory('riggu_teleop')}/config/joy_generic.yaml",
          encoding='utf-8') as _f:
    PAD = yaml.safe_load(_f)
TELEOP = PAD['teleop_twist_joy_node']['ros__parameters']
DEADMAN = TELEOP['enable_button']
LINEAR_AXIS = TELEOP['axis_linear']['x']
LINEAR_SCALE = TELEOP['scale_linear']['x']
STOP = PAD['stop_latch']['ros__parameters']['button']
ARM = PAD['arm_switch']['ros__parameters']


@pytest.mark.launch_test
def generate_test_description():
    """Launch the mock bring-up."""
    bringup = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('riggu_bringup'), 'launch', 'bringup.launch.py']),
        launch_arguments={'mode': 'mock', 'joy': 'true', 'arm': 'true'}.items())
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
        cls.nav = cls.node.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        cls.joy = cls.node.create_publisher(Joy, '/joy', 10)

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

    def vx(self):
        """Latest /odom forward velocity."""
        return self.latest['/odom'].twist.twist.linear.x

    def drive(self, seconds, nav=None, buttons=(), linear_axis=0.0):
        """Publish /cmd_vel_nav (if nav) and /joy at 20 Hz for `seconds`."""
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_footprint'
        joy = Joy(buttons=[1 if i in buttons else 0 for i in range(13)], axes=[0.0] * 8)
        joy.axes[LINEAR_AXIS] = linear_axis
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = self.node.get_clock().now().to_msg()
            if nav is not None:
                cmd.header.stamp = now
                cmd.twist.linear.x = nav
                self.nav.publish(cmd)
            joy.header.stamp = now
            self.joy.publish(joy)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_stopped(self):
        """Idle until /odom is at rest (timeouts brake the wheels)."""
        self.spin_until(lambda: abs(self.vx()) < 1e-3, 5)
        self.assertLess(abs(self.vx()), 1e-3)

    def test_3_nav_moves_odom_through_chain(self):
        self.spin_until(lambda: '/odom' in self.latest, 20)
        start = self.latest.get('/odom')
        self.assertIsNotNone(start, 'no /odom')
        self.assertEqual(start.header.frame_id, 'odom')
        self.assertEqual(start.child_frame_id, 'base_footprint')

        self.drive(3.0, nav=0.2)
        odom = self.latest['/odom']
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.2, delta=0.05)
        moved = odom.pose.pose.position.x - start.pose.pose.position.x
        self.assertGreater(moved, 0.2, f'moved {moved:.3f} m in 3 s at 0.2 m/s')
        self.assertAlmostEqual(odom.pose.pose.position.y, 0.0, delta=0.02)
        # twist_mux / cmd_vel_timeout stop the robot once commands stop
        self.wait_stopped()

    def test_4_joy_needs_deadman_and_beats_nav(self):
        self.drive(1.5, nav=0.2, linear_axis=-0.5)  # stick without deadman: nav drives
        self.assertAlmostEqual(self.vx(), 0.2, delta=0.05)
        self.drive(1.5, nav=0.2, buttons=[DEADMAN], linear_axis=-0.5)
        self.assertAlmostEqual(self.vx(), -0.5 * LINEAR_SCALE, delta=0.03)
        self.drive(1.5, nav=0.2)  # deadman released: nav again
        self.assertAlmostEqual(self.vx(), 0.2, delta=0.05)
        self.wait_stopped()

    def test_5_stop_latch_holds_until_pressed_again(self):
        self.drive(1.0, nav=0.2)
        self.assertAlmostEqual(self.vx(), 0.2, delta=0.05)
        self.drive(0.2, nav=0.2, buttons=[STOP])
        self.drive(1.5, nav=0.2)  # released the button: still latched
        self.assertLess(abs(self.vx()), 1e-3)
        self.drive(1.0, nav=0.2, buttons=[DEADMAN], linear_axis=1.0)  # joy is locked too
        self.assertLess(abs(self.vx()), 1e-3)
        self.drive(0.2, nav=0.2, buttons=[STOP])
        self.drive(1.5, nav=0.2)
        self.assertAlmostEqual(self.vx(), 0.2, delta=0.05)
        self.wait_stopped()

    def test_6_pad_disarms_and_arms(self):
        self.drive(0.3, buttons=[ARM['disarm_button']])
        inactive = self.spin_until(
            lambda: 'diff_drive_controller' not in self.active_controllers(), 5)
        self.assertTrue(inactive, 'disarm button did not deactivate the drive')
        self.drive(ARM['arm_hold'] * 0.5, buttons=ARM['arm_buttons'])  # too short
        self.assertNotIn('diff_drive_controller', self.active_controllers())
        self.drive(0.5, buttons=[])
        self.drive(ARM['arm_hold'] + 0.5, buttons=ARM['arm_buttons'])
        active = self.spin_until(
            lambda: 'diff_drive_controller' in self.active_controllers(), 5)
        self.assertTrue(active, 'arm combo did not activate the drive')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Processes exit cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
