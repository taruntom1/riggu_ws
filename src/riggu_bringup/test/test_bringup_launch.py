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

"""bringup.launch.py: real loads the drive inactive; imu:=false drops the IMU sensor."""

import importlib.util
from pathlib import Path

from xml.dom import minidom

from launch import LaunchContext
from launch.substitutions import TextSubstitution
import pytest

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
