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

"""bringup.launch.py: drive inactive on real, imu:=false, cmd_vel chain, footprint, EKF."""

import importlib.util
from pathlib import Path

from xml.dom import minidom

from launch import LaunchContext
from launch.substitutions import TextSubstitution
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bringup', PKG / 'launch' / 'bringup.launch.py')
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)
ROBOT_YAML = PKG.parent / 'riggu_description' / 'config' / 'robot.yaml'


def drive_args(mode):
    context = LaunchContext()
    context.launch_configurations.update({'mode': mode, 'robot_config': str(ROBOT_YAML)})
    (node,) = bringup.drive_spawner(context, 'controllers.yaml', 'false')
    return node._Node__arguments  # launch_ros keeps them private


@pytest.mark.parametrize('mode,inactive', [('real', True), ('sim', False), ('mock', False)])
def test_drive_spawner_inactive_only_on_real(mode, inactive):
    args = drive_args(mode)
    assert args[0] == 'diff_drive_controller'
    assert ('--inactive' in args) == inactive


def test_wheel_params_from_robot_yaml():
    args = ' '.join(drive_args('real'))
    assert '--controller-ros-args=-p wheel_radius:=' in args
    assert '--controller-ros-args=-r ~/cmd_vel:=/cmd_vel_safe' in args


def description(mode, imu):
    context = LaunchContext()
    context.launch_configurations.update({
        'mode': mode, 'imu': imu, 'robot_config': str(ROBOT_YAML),
        'drivelink_agent': 'tcp://127.0.0.1:47620', 'drivelink_device': 'base',
        'drivelink_token_file': '/etc/drivelink/token',
        'drivelink_left_label': 'pid_left', 'drivelink_right_label': 'pid_right'})
    return minidom.parseString(
        bringup.robot_description(context, TextSubstitution(text='controllers.yaml')))


def ros2_control_sensors(doc):
    return [s.getAttribute('name') for block in doc.getElementsByTagName('ros2_control')
            for s in block.getElementsByTagName('sensor')]


@pytest.mark.parametrize('mode', ['real', 'mock'])
def test_imu_optional(mode):
    with_imu, without = description(mode, 'true'), description(mode, 'false')
    assert ros2_control_sensors(with_imu) == ['imu']
    assert ros2_control_sensors(without) == []
    joints = [j.getAttribute('name') for b in without.getElementsByTagName('ros2_control')
              for j in b.getElementsByTagName('joint')]
    assert joints == ['left_wheel_joint', 'right_wheel_joint']
    # the imu_link frame stays in the model
    assert 'imu_link' in [link.getAttribute('name')
                          for link in without.getElementsByTagName('link')]


def test_real_maps_board_controller_labels():
    doc = description('real', 'false')
    instances = [p.firstChild.data for p in doc.getElementsByTagName('param')
                 if p.getAttribute('name') == 'instance']
    assert instances == ['pid_left', 'pid_right']


@pytest.mark.parametrize('imu,expected', [
    ('true', ['joint_state_broadcaster', 'imu_sensor_broadcaster']),
    ('false', ['joint_state_broadcaster'])])
def test_recover_never_arms(imu, expected):
    context = LaunchContext()
    context.launch_configurations.update({
        'mode': 'real', 'imu': imu, 'robot_config': str(ROBOT_YAML),
        'drivelink_agent': 'tcp://127.0.0.1:47620', 'drivelink_device': 'base',
        'drivelink_token_file': '/etc/drivelink/token',
        'drivelink_left_label': 'pid_left', 'drivelink_right_label': 'pid_right'})
    nodes = bringup.launch_setup(context, TextSubstitution(text='controllers.yaml'), 'false')
    (diagnostics,) = [n for n in nodes if n.node_executable == 'diagnostics']
    # launch_ros normalises keys to substitution tuples, list items to YAML substitutions
    params = {''.join(s.perform(context) for s in k): v
              for d in diagnostics._Node__parameters for k, v in d.items()}
    recovery = [yaml.safe_load(''.join(s.perform(context) for s in item))
                for item in params['recovery_controllers']]
    assert recovery == expected


def test_footprint_encloses_chassis_and_wheels():
    robot = yaml.safe_load(ROBOT_YAML.read_text())
    poly = bringup.footprint(robot, 0.0)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    wheel_y = robot['wheel']['separation']['value'] / 2 + robot['wheel']['width']['value'] / 2
    assert max(ys) == pytest.approx(-min(ys))
    assert max(ys) >= wheel_y and max(ys) >= robot['chassis']['width']['value'] / 2
    assert max(xs) >= robot['wheel']['radius']['value']  # axle at x = 0, near the front
    assert max(xs) - min(xs) == pytest.approx(robot['chassis']['length']['value'])
    grown = bringup.footprint(robot, 0.05)
    assert grown[0][0] == pytest.approx(poly[0][0] + 0.05)


def chain(mode, check_joy, joy='auto', arm='auto'):
    """Return the teleop include args and collision_monitor params for these args."""
    context = LaunchContext()
    context.launch_configurations.update({
        'mode': mode, 'joy': joy, 'arm': arm, 'collision_check_joy': check_joy,
        'robot_config': str(ROBOT_YAML),
        'collision_monitor_config': str(PKG / 'config' / 'collision_monitor.yaml')})
    teleop, monitor, _ = bringup.cmd_vel_chain(context, 'false')

    def text(subs):
        return subs if isinstance(subs, str) else ''.join(s.perform(context) for s in subs)
    include = {text(k): text(v) for k, v in teleop.launch_arguments}
    # launch_ros keeps values as YAML-encoded substitutions
    params = {text(k): v if isinstance(v, float) else yaml.safe_load(text(v))
              for d in monitor._Node__parameters if isinstance(d, dict) for k, v in d.items()}
    return include, params


@pytest.mark.parametrize('mode,pad', [('real', 'true'), ('sim', 'false'), ('mock', 'false')])
def test_chain_through_collision_monitor(mode, pad):
    include, params = chain(mode, 'true')
    assert include['cmd_vel_out'] == '/cmd_vel_mux' and include['nav_in'] == '/cmd_vel_nav'
    assert include['joy'] == pad and include['arm'] == pad
    assert params['cmd_vel_in_topic'] == '/cmd_vel_mux'
    assert params['cmd_vel_out_topic'] == '/cmd_vel_safe'
    assert ('scan.source_timeout' in params) == (mode == 'mock')


def test_joy_bypass_moves_monitor_to_nav():
    include, params = chain('real', 'false', joy='false', arm='false')
    assert include['cmd_vel_out'] == '/cmd_vel_safe'
    assert include['nav_in'] == '/cmd_vel_nav_checked'
    assert include['joy'] == 'false' and include['arm'] == 'false'
    assert params['cmd_vel_in_topic'] == '/cmd_vel_nav'
    assert params['cmd_vel_out_topic'] == '/cmd_vel_nav_checked'


def ekf(imu='true', orientation='true'):
    context = LaunchContext()
    context.launch_configurations.update({
        'imu': imu, 'use_imu_orientation': orientation,
        'ekf_config': str(PKG / 'config' / 'ekf.yaml'),
        'ekf_gyro_only_config': str(PKG / 'config' / 'ekf_gyro_only.yaml')})
    return bringup.ekf_params(context)


YAW, VX, VY, VYAW = 5, 6, 7, 11


def test_ekf_fuses_wheels_and_imu():
    params = ekf()
    assert params['two_d_mode'] and params['frequency'] == 30.0
    assert (params['world_frame'], params['base_link_frame']) == ('odom', 'base_footprint')
    assert params['odom0'] == '/odom' and params['imu0'] == '/imu'
    assert [i for i, on in enumerate(params['odom0_config']) if on] == [VX, VY, VYAW]
    assert [i for i, on in enumerate(params['imu0_config']) if on] == [YAW, VYAW]


def test_ekf_gyro_only_drops_absolute_yaw():
    params = ekf(orientation='false')
    assert [i for i, on in enumerate(params['imu0_config']) if on] == [VYAW]
    assert params['odom0_config'] == ekf()['odom0_config']


def test_ekf_without_imu_uses_wheels_only():
    params = ekf(imu='false')
    assert not [k for k in params if k.startswith('imu0')]
    assert params['odom0'] == '/odom'


def test_only_the_ekf_publishes_odom_tf():
    controllers = yaml.safe_load((PKG / 'config' / 'controllers.yaml').read_text())
    assert controllers['diff_drive_controller']['ros__parameters']['enable_odom_tf'] is False
    assert ekf()['publish_tf'] is True
