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
riggu base bring-up: robot_state_publisher, ros2_control, controller spawners.

mode:=real|sim|mock picks the ros2_control hardware (see riggu_description).
In sim, Gazebo runs the controller manager (gz_ros2_control), so no
ros2_control_node is started here; riggu_gazebo/sim.launch.py includes this.
On real, activating diff_drive_controller arms the DriveLink motors.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def drive_spawner(context, controllers, use_sim_time):
    """Spawn diff_drive_controller with wheel params and limits from robot.yaml."""
    with open(LaunchConfiguration('robot_config').perform(context), encoding='utf-8') as f:
        robot = yaml.safe_load(f)
    wheel, limits = robot['wheel'], robot['limits']
    params = {
        'wheel_radius': wheel['radius']['value'],
        'wheel_separation': wheel['separation']['value'],
        'linear.x.max_velocity': limits['max_linear_velocity']['value'],
        'linear.x.min_velocity': -limits['max_linear_velocity']['value'],
        'linear.x.max_acceleration': limits['max_linear_accel']['value'],
        'linear.x.min_acceleration': -limits['max_linear_accel']['value'],
        'angular.z.max_velocity': limits['max_angular_velocity']['value'],
        'angular.z.min_velocity': -limits['max_angular_velocity']['value'],
        'angular.z.max_acceleration': limits['max_angular_accel']['value'],
        'angular.z.min_acceleration': -limits['max_angular_accel']['value'],
    }
    ros_args = ['-r ~/cmd_vel:=/cmd_vel_safe', '-r ~/odom:=/odom']
    ros_args += [f'-p {name}:={float(value)}' for name, value in params.items()]
    return [spawner('diff_drive_controller', controllers, use_sim_time, ros_args)]


def spawner(name, controllers, use_sim_time, ros_args=()):
    """Return a spawner node for one controller."""
    args = [name, '--param-file', controllers, '--controller-manager-timeout', '120']
    # '=' form: argparse would read a bare '-p ...' value as the spawner's own -p
    args += [f'--controller-ros-args={arg}' for arg in ros_args]
    return Node(package='controller_manager', executable='spawner', name=f'spawner_{name}',
                arguments=args, parameters=[{'use_sim_time': use_sim_time}],
                output='screen')


def generate_launch_description():
    """Build the launch description."""
    description_share = FindPackageShare('riggu_description')
    controllers = PathJoinSubstitution(
        [FindPackageShare('riggu_bringup'), 'config', 'controllers.yaml'])
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([description_share, 'urdf', 'riggu.urdf.xacro']),
            ' mode:=', mode,
            ' robot_config:=', LaunchConfiguration('robot_config'),
            ' controllers_file:=', controllers,
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mock',
                              choices=['real', 'sim', 'mock'],
                              description='ros2_control hardware'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'robot_config',
            default_value=PathJoinSubstitution([description_share, 'config', 'robot.yaml']),
            description='Dimensions file (wheel params and limits for the controller)'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': use_sim_time}],
             output='screen'),

        # real and mock; in sim the controller manager lives in Gazebo
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers, {'use_sim_time': use_sim_time}],
             remappings=[('~/robot_description', '/robot_description')],
             condition=UnlessCondition(is_sim),
             output='screen'),

        spawner('joint_state_broadcaster', controllers, use_sim_time),
        spawner('imu_sensor_broadcaster', controllers, use_sim_time, ['-r ~/imu:=/imu']),
        OpaqueFunction(function=drive_spawner, args=[controllers, use_sim_time]),
    ])
